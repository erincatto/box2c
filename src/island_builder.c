// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "box2d/allocate.h"
#include "box2d/math.h"
#include "box2d/types.h"

#include "atomic.inl"
#include "island_builder.h"
#include "stack_allocator.h"

#include <assert.h>
#include <stdlib.h>
#include <string.h>

static int32_t b2GetLowestBodyIndex(const b2BodyLink* links, int32_t index)
{
	for (;;)
	{
		int32_t parent = atomic_load_long(&links[index].parent);
		if (parent == index)
		{
			return index;
		}

		index = parent;
	}
}

b2IslandBuilder b2CreateIslandBuilder(int32_t bodyCapacity)
{
	b2IslandBuilder builder = {0};

	builder.bodyCapacity = bodyCapacity;

	builder.links = b2Alloc(bodyCapacity * sizeof(b2BodyLink));
	for (int32_t i = 0; i < bodyCapacity; ++i)
	{
		atomic_store_long(&builder.links[i].parent, i);
	}

	return builder;
}

void b2DestroyIslandBuilder(b2IslandBuilder* builder)
{
	b2Free(builder->links);
}

// TODO_ERIN this shouldn't use the stack allocator. Instead it should grow capacity as needed.
void b2StartIslands(b2IslandBuilder* builder, int32_t bodyCapacity, int32_t jointCapacity, int32_t contactCapacity, b2StackAllocator* allocator)
{
	if (bodyCapacity > builder->bodyCapacity)
	{
		b2BodyLink* newLinks = b2Alloc(bodyCapacity * sizeof(b2BodyLink));
		memcpy(newLinks, builder->links, builder->bodyCapacity * sizeof(b2BodyLink));
		for (int32_t i = builder->bodyCapacity; i < bodyCapacity; ++i)
		{
			atomic_store_long(&newLinks[i].parent, i);
		}

		b2Free(builder->links);
		builder->links = newLinks;
		builder->bodyCapacity = bodyCapacity;
	}

	builder->jointCapacity = jointCapacity;
	builder->jointLinks = b2AllocateStackItem(allocator, jointCapacity * sizeof(int32_t), "joint links");

	builder->contactCapacity = contactCapacity;
	builder->contactLinks = b2AllocateStackItem(allocator, contactCapacity * sizeof(int32_t), "contact links");
	builder->contactElements = b2AllocateStackItem(allocator, contactCapacity * sizeof(b2ContactElement), "contact elements");

	builder->contactIslandEnds = NULL;
	builder->contactIslands = NULL;

	builder->jointIslandEnds = NULL;
	builder->jointIslands = NULL;
}

// body union
static void b2LinkBodies(b2IslandBuilder* builder, int32_t awakeIndexA, int32_t awakeIndexB)
{
	if (awakeIndexA == B2_NULL_INDEX || awakeIndexB == B2_NULL_INDEX)
	{
		return;
	}

	assert(awakeIndexA < builder->bodyCapacity);
	assert(awakeIndexB < builder->bodyCapacity);

	long rootA = awakeIndexA;
	long rootB = awakeIndexB;
	
	b2BodyLink* links = builder->links;

	for (;;)
	{
		rootA = b2GetLowestBodyIndex(links, rootA);
		rootB = b2GetLowestBodyIndex(links, rootB);

		if (rootA < rootB)
		{
			// Attempt to link the tree B to tree A. This will fail if another thread has changed the parent of B.
			bool success = atomic_compare_exchange_weak_long(&links[rootB].parent, &rootB, rootA);
			if (success == false)
			{
				// failed to connect trees, try again
				continue;
			}
		}
		else if (rootB < rootA)
		{
			// Attempt to link the tree A to tree B. This will fail if another thread has changed the parent of A.
			bool success = atomic_compare_exchange_weak_long(&links[rootA].parent, &rootA, rootB);
			if (success == false)
			{
				// failed to connect trees, try again
				continue;
			}
		}

		// Path compression
		int32_t lowest = B2_MIN(rootA, rootB);
		b2AtomicStoreMin(&links[awakeIndexA].parent, lowest);
		b2AtomicStoreMin(&links[awakeIndexB].parent, lowest);

		break;
	}
}

void b2LinkJoint(b2IslandBuilder* builder, int32_t jointIndex, int32_t awakeIndexA, int32_t awakeIndexB)
{
	assert(awakeIndexA != B2_NULL_INDEX || awakeIndexB != B2_NULL_INDEX);
	assert(jointIndex < builder->jointCapacity);

	b2LinkBodies(builder, awakeIndexA, awakeIndexB);

	// Max to ensure awake body (static bodies are never awake)
	builder->jointLinks[jointIndex] = B2_MAX(awakeIndexA, awakeIndexB);
}

void b2LinkContact(b2IslandBuilder* builder, int32_t contactIndex, b2Contact* contact, uint64_t sortKey, int32_t awakeIndexA, int32_t awakeIndexB)
{
	assert(awakeIndexA != B2_NULL_INDEX || awakeIndexB != B2_NULL_INDEX);
	assert(contactIndex < builder->contactCapacity);

	b2LinkBodies(builder, awakeIndexA, awakeIndexB);

	// Max to ensure awake body (static bodies are never awake)
	builder->contactLinks[contactIndex] = B2_MAX(awakeIndexA, awakeIndexB);
	builder->contactElements[contactIndex].contact = contact;
	builder->contactElements[contactIndex].sortKey = sortKey;
}

static void b2BuildBodyIslands(b2IslandBuilder* builder, const int32_t* awakeBodies, int32_t bodyCount, b2StackAllocator* allocator)
{
	builder->bodyCount = bodyCount;
	int32_t* bodyIslands = b2AllocateStackItem(allocator, bodyCount * sizeof(int32_t), "body islands");
	int32_t* baseIndices = b2AllocateStackItem(allocator, (bodyCount + 1) * sizeof(int32_t), "body island ends");
	baseIndices[0] = 0;
	int32_t islandCount = 0;

	// Calculate island index for all bodies
	// Example:
	// links   : [0 0 2 2 3 5 5]

	b2BodyLink* links = builder->links;
	for (int32_t i = 0; i < bodyCount; ++i)
	{
		b2BodyLink* link = links + i;
		int32_t parent = atomic_load_long(&link->parent);
		if (i != parent)
		{
			int32_t islandIndex = links[parent].islandIndex;
			link->islandIndex = islandIndex;
			baseIndices[islandIndex + 1] += 1;
		}
		else
		{
			link->islandIndex = islandCount;
			islandCount += 1;
			baseIndices[islandCount] = 1;
		}
	}

	// Example
	// islandCount = 3
	// islands : [0 0 1 1 1 2 2]
	// base    : [0 2 3 1]

	for (int32_t island = 1; island < islandCount; ++island)
	{
		baseIndices[island] += baseIndices[island - 1];
	}

	// Example
	// base    : [0 2 5 1]

	for (int32_t i = 0; i < bodyCount; ++i)
	{
		b2BodyLink* link = links + i;

		int32_t base = baseIndices[link->islandIndex];

		// This resolves the body index from b2World::activeBodies to b2World::bodies
		bodyIslands[base] = awakeBodies[i];

		baseIndices[link->islandIndex] += 1;

		atomic_store_long(&link->parent, i);
	}

	builder->bodyIslandEnds = baseIndices;

	builder->islandCount = islandCount;
	builder->bodyIslands = bodyIslands;
}

static void b2BuildJointIslands(b2IslandBuilder* builder, const int32_t* constraintToBody, int32_t constraintCount, int32_t** outConstraints, int32_t** outConstraintsEnd, b2StackAllocator* allocator)
{
	if (constraintCount == 0)
	{
		return;
	}

	const int32_t islandCount = builder->islandCount;
	const b2BodyLink* links = builder->links;

	int32_t* constraints = b2AllocateStackItem(allocator, constraintCount * sizeof(int32_t), "constraint islands");
	int32_t* constraintEnds = b2AllocateStackItem(allocator, (islandCount + 1) * sizeof(int32_t), "constraint island ends");

	// Counting sort constraints by island index with adjustment to keep end index array
	// https://en.wikipedia.org/wiki/Counting_sort

	// Over sized by 1
	memset(constraintEnds, 0, (islandCount + 1) * sizeof(int32_t));

	for (int32_t i = 0; i < constraintCount; ++i)
	{
		int32_t bodyIndex = constraintToBody[i];
		int32_t islandIndex = links[bodyIndex].islandIndex;
		assert(islandIndex < islandCount);

		// Keeping a 0 at index 0
		constraintEnds[islandIndex + 1] += 1;
	}

	assert(constraintEnds[0] == 0);

	for (int32_t i = 1; i < islandCount; ++i)
	{
		constraintEnds[i] += constraintEnds[i - 1];
	}

	// Note: this would need to be reverse to keep the sort stable
	for (int32_t i = 0; i < constraintCount; ++i)
	{
		int32_t bodyIndex = constraintToBody[i];
		int32_t islandIndex = links[bodyIndex].islandIndex;
		int32_t j = constraintEnds[islandIndex];
		constraints[j] = i;
		constraintEnds[islandIndex] = j + 1;
	}

	// Now constraintEnds holds the end index for each constraint island (and implicitly the count)
	// For example:
	// [3, 5, 6]
	// means the constraints have these islands:
	// [0 0 0 1 1 2] 

	*outConstraints = constraints;
	*outConstraintsEnd = constraintEnds;
}

static void b2BuildContactIslands(b2IslandBuilder* builder, b2StackAllocator* allocator)
{
	int32_t contactCount = builder->contactCount;
	if (contactCount == 0)
	{
		return;
	}

	const int32_t islandCount = builder->islandCount;
	const b2BodyLink* links = builder->links;
	const int32_t* contactLinks = builder->contactLinks;

	b2ContactElement* contactIslands = b2AllocateStackItem(allocator, contactCount * sizeof(b2ContactElement), "contact islands");
	int32_t* islandEnds = b2AllocateStackItem(allocator, (islandCount + 1) * sizeof(int32_t), "contact island ends");

	// Counting sort constraints by island index with adjustment to keep end index array
	// https://en.wikipedia.org/wiki/Counting_sort

	// Over sized by 1
	memset(islandEnds, 0, (islandCount + 1) * sizeof(int32_t));

	// Count the number of contacts in each island
	for (int32_t i = 0; i < contactCount; ++i)
	{
		int32_t bodyIndex = contactLinks[i];
		int32_t islandIndex = links[bodyIndex].islandIndex;
		assert(islandIndex < islandCount);

		// Keeping a 0 at index 0
		islandEnds[islandIndex + 1] += 1;
	}

	assert(islandEnds[0] == 0);

	for (int32_t i = 1; i < islandCount; ++i)
	{
		islandEnds[i] += islandEnds[i - 1];
	}

	// Note: this would need to be reverse to keep the sort stable
	for (int32_t i = 0; i < contactCount; ++i)
	{
		int32_t bodyIndex = contactLinks[i];
		int32_t islandIndex = links[bodyIndex].islandIndex;
		int32_t j = islandEnds[islandIndex];
		contactIslands[j] = builder->contactElements[i];
		islandEnds[islandIndex] = j + 1;
	}

	// Now constraintEnds holds the end index for each constraint island (and implicitly the count)
	// For example:
	// [3, 5, 6]
	// means the constraints have these islands:
	// [0 0 0 1 1 2]
	builder->contactIslandEnds = islandEnds;
	builder->contactIslands = contactIslands;
}

static int b2CompareIslands(const void* index1, const void* index2)
{
	const b2IslandIndex* island1 = index1;
	const b2IslandIndex* island2 = index2;
	return island2->constraintCount - island1->constraintCount;
}

static void b2SortIslands(b2IslandBuilder* builder, b2StackAllocator* allocator)
{
	int32_t contactCount = builder->contactCount;
	int32_t jointCount = builder->jointCount;
	int32_t islandCount = builder->islandCount;

	b2IslandIndex* sortedIslands = b2AllocateStackItem(allocator, islandCount * sizeof(b2IslandIndex), "sorted islands");

	// Initialize index
	for (int32_t i = 0; i < islandCount; ++i)
	{
		sortedIslands[i].index = i;
		sortedIslands[i].constraintCount = 0;
	}

	if (contactCount > 0 || jointCount > 0)
	{
		const int32_t* contactIslandEnds = builder->contactIslandEnds;
		const int32_t* jointIslandEnds = builder->jointIslandEnds;

		if (contactCount > 0 && jointCount > 0)
		{
			sortedIslands[0].constraintCount = contactIslandEnds[0] + jointIslandEnds[0];
			for (int32_t i = 1; i < islandCount; ++i)
			{
				int32_t count1 = jointIslandEnds[i] - jointIslandEnds[i - 1];
				int32_t count2 = contactIslandEnds[i] - contactIslandEnds[i - 1];
				sortedIslands[i].constraintCount = count1 + count2;
			}
		}
		else if (contactCount > 0)
		{
			sortedIslands[0].constraintCount = contactIslandEnds[0];
			for (int32_t i = 1; i < islandCount; ++i)
			{
				sortedIslands[i].constraintCount = contactIslandEnds[i] - contactIslandEnds[i - 1];
			}
		}
		else
		{
			sortedIslands[0].constraintCount = jointIslandEnds[0];
			for (int32_t i = 1; i < islandCount; ++i)
			{
				sortedIslands[i].constraintCount = jointIslandEnds[i] - jointIslandEnds[i - 1];
			}
		}

		// Bigger islands go first
		qsort(sortedIslands, islandCount, sizeof(b2IslandIndex), b2CompareIslands);
	}

	builder->sortedIslands = sortedIslands;
}

void b2FinishIslands(b2IslandBuilder* builder, const int32_t* awakeBodies, int32_t bodyCount, int32_t jointCount, int32_t contactCount, b2StackAllocator *allocator)
{
	builder->jointCount = jointCount;
	builder->contactCount = contactCount;

	b2BuildBodyIslands(builder, awakeBodies, bodyCount, allocator);

	// Joints
	b2BuildJointIslands(builder, builder->jointLinks, jointCount, &builder->jointIslands, &builder->jointIslandEnds, allocator);
	
	// Contacts
	b2BuildContactIslands(builder, allocator);

	// Sort islands so largest islands come first
	b2SortIslands(builder, allocator);
}

void b2ResetIslands(b2IslandBuilder* builder, b2StackAllocator *allocator)
{
	// Some of this stuff is NULL if the time step is zero.

	if (builder->sortedIslands != NULL)
	{
		b2FreeStackItem(allocator, builder->sortedIslands);
		builder->sortedIslands = NULL;
	}

	if (builder->contactIslands != NULL)
	{
		b2FreeStackItem(allocator, builder->contactIslandEnds);
		builder->contactIslandEnds = NULL;
		b2FreeStackItem(allocator, builder->contactIslands);
		builder->contactIslands = NULL;
	}
	
	if (builder->jointIslands != NULL)
	{
		b2FreeStackItem(allocator, builder->jointIslandEnds);
		builder->jointIslandEnds = NULL;
		b2FreeStackItem(allocator, builder->jointIslands);
		builder->jointIslands = NULL;
	}
	
	if (builder->bodyIslands != NULL)
	{
		b2FreeStackItem(allocator, builder->bodyIslandEnds);
		builder->bodyIslandEnds = NULL;
		b2FreeStackItem(allocator, builder->bodyIslands);
		builder->bodyIslands = NULL;
	}

	b2FreeStackItem(allocator, builder->contactElements);
	builder->contactElements = NULL;

	b2FreeStackItem(allocator, builder->contactLinks);
	builder->contactLinks = NULL;

	b2FreeStackItem(allocator, builder->jointLinks);
	builder->jointLinks = NULL;

	builder->bodyCount = 0;
	builder->contactCount = 0;
	builder->contactCapacity = 0;
	builder->jointCount = 0;
	builder->islandCount = 0;
}
