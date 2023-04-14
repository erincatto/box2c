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

void b2InitializeIslands(b2IslandBuilder* builder, int32_t contactCapacity, int32_t jointCount, b2StackAllocator* allocator)
{
	builder->contactCapacity = contactCapacity;
	builder->contactLinks = b2AllocateStackItem(allocator, contactCapacity * sizeof(int32_t));

	builder->jointCount = jointCount;
	builder->jointLinks = b2AllocateStackItem(allocator, jointCount * sizeof(int32_t));
}

// body union
static void b2LinkBodies(b2IslandBuilder* builder, int32_t indexA, int32_t indexB)
{
	if (indexA >= builder->bodyCapacity || indexB >= builder->bodyCapacity)
	{
		return;
	}

	int32_t rootA = indexA;
	int32_t rootB = indexB;
	
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
		b2AtomicStoreMin(&links[indexA].parent, lowest);
		b2AtomicStoreMin(&links[indexB].parent, lowest);

		break;
	}
}

void b2LinkJoint(b2IslandBuilder* builder, int32_t jointIndex, int32_t indexA, int32_t indexB)
{
	b2LinkBodies(builder, indexA, indexB);
	builder->jointLinks[jointIndex] = B2_MIN(indexA, indexB);
}

void b2LinkContact(b2IslandBuilder* builder, int32_t contactIndex, int32_t indexA, int32_t indexB)
{
	assert(contactIndex < builder->contactCapacity);
	builder->contactLinks[contactIndex] = B2_MIN(indexA, indexB);
}

static void b2BuildBodyIslands(b2IslandBuilder* builder, const int32_t* bodies, int32_t bodyCount, b2StackAllocator* allocator)
{
	builder->bodyCount = bodyCount;
	int32_t* bodyIslands = b2AllocateStackItem(allocator, bodyCount * sizeof(int32_t));
	
	int32_t* baseIndices = b2AllocateStackItem(allocator, (bodyCount + 1) * sizeof(int32_t));
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
		bodyIslands[base] = bodies[i];

		baseIndices[link->islandIndex] += 1;

		atomic_store_long(&link->parent, i);
	}

	builder->bodyIslandEnds = baseIndices;

	builder->islandCount = islandCount;
	builder->bodyIslands = bodyIslands;
}

static void b2BuildConstraintIslands(b2IslandBuilder* builder, const int32_t* constraintToBody, int32_t constraintCount, int32_t** outConstraints, int32_t** outConstraintsEnd, b2StackAllocator* allocator)
{
	if (constraintCount == 0)
	{
		return;
	}

	const int32_t islandCount = builder->islandCount;
	const b2BodyLink* links = builder->links;

	int32_t* constraints = b2AllocateStackItem(allocator, constraintCount * sizeof(int32_t));
	int32_t* constraintEnds = b2AllocateStackItem(allocator, (islandCount + 1) * sizeof(int32_t));

	for (int32_t i = 0; i < islandCount; ++i)
	{
		constraintEnds[i] = 0;
	}

	for (int32_t i = 0; i < constraintCount; ++i)
	{
		int32_t bodyIndex = constraintToBody[i];
		int32_t nextIslandIndex = links[bodyIndex].islandIndex + 1;
		assert(nextIslandIndex <= islandCount);
		constraintEnds[nextIslandIndex]++;
	}

	for (int32_t i = 1; i < islandCount; ++i)
	{
		constraintEnds[i] += constraintEnds[i - 1];
	}

	for (int32_t i = 0; i < constraintCount; ++i)
	{
		int32_t bodyIndex = constraintToBody[i];
		int32_t islandIndex = links[bodyIndex].islandIndex;
		constraints[constraintEnds[islandIndex]++] = i;
	}

	*outConstraints = constraints;
	*outConstraintsEnd = constraintEnds;
}

static int b2ReverseCompareInt32(const void* count1Ptr, const void* count2Ptr, void* context)
{
	int32_t index1 = *(const int32_t*)count1Ptr;
	int32_t index2 = *(const int32_t*)count2Ptr;
	int32_t* counts = context;
	return counts[index2] - counts[index1];
}

void b2SortIslands(b2IslandBuilder* builder, b2StackAllocator* allocator)
{
	int32_t contactCount = builder->contactCount;
	int32_t jointCount = builder->jointCount;
	int32_t islandCount = builder->islandCount;

	if (contactCount > 0 || jointCount > 0)
	{
		const int32_t* contactIslandEnds = builder->contactIslandEnds;
		const int32_t* jointIslandEnds = builder->jointIslandEnds;

		int32_t* sortedIslands = b2AllocateStackItem(allocator, islandCount * sizeof(int32_t));

		// Initialize index
		for (int32_t i = 0; i < islandCount; ++i)
		{
			sortedIslands[i] = i;
		}

		int32_t* constraintCounts = b2AllocateStackItem(allocator, islandCount * sizeof(int32_t));
		if (contactCount > 0 && jointCount > 0)
		{
			constraintCounts[0] = contactIslandEnds[0] + jointIslandEnds[0];
			for (int32_t i = 1; i < islandCount; ++i)
			{
				int32_t jointCount = jointIslandEnds[i] - jointIslandEnds[i - 1];
				int32_t contactCount = contactIslandEnds[i] - contactIslandEnds[i - 1];
				constraintCounts[i] = jointCount + contactCount;
			}
		}
		else if (contactCount > 0)
		{
			constraintCounts[0] = contactIslandEnds[0];
			for (int32_t i = 1; i < islandCount; ++i)
			{
				constraintCounts[i] = contactIslandEnds[i] - contactIslandEnds[i - 1];
			}
		}
		else
		{
			constraintCounts[0] = jointIslandEnds[0];
			for (int32_t i = 1; i < islandCount; ++i)
			{
				constraintCounts[i] = jointIslandEnds[i] - jointIslandEnds[i - 1];
			}
		}

		// Bigger islands go first
		qsort_s(sortedIslands, islandCount, sizeof(int32_t), b2ReverseCompareInt32, constraintCounts);

		b2FreeStackItem(allocator, constraintCounts);
	}
}
void b2FinalizeIslands(b2IslandBuilder* builder, const int32_t* bodies, int32_t bodyCount, int32_t contactCount, b2StackAllocator *allocator)
{
	builder->contactCount = contactCount;

	b2BuildBodyIslands(builder, bodies, bodyCount, allocator);

	// Joints
	b2BuildConstraintIslands(builder, builder->jointLinks, builder->jointCount, &builder->jointIslands, &builder->jointIslandEnds, allocator);
	
	// Contacts
	b2BuildConstraintIslands(builder, builder->contactLinks, contactCount, &builder->contactIslands, &builder->contactIslandEnds, allocator);

	b2SortIslands(builder, allocator);
}

void b2DestroyIslands(b2IslandBuilder* builder, b2StackAllocator *allocator)
{
	b2FreeStackItem(allocator, builder->sortedIslands);
	builder->sortedIslands = NULL;

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
	
	b2FreeStackItem(allocator, builder->bodyIslandEnds);
	builder->bodyIslandEnds = NULL;
	b2FreeStackItem(allocator, builder->bodyIslands);
	builder->bodyIslands = NULL;

	b2FreeStackItem(allocator, builder->jointLinks);
	builder->jointLinks = NULL;

	b2FreeStackItem(allocator, builder->contactLinks);
	builder->contactLinks = NULL;

	builder->bodyCount = 0;
	builder->contactCount = 0;
	builder->contactCapacity = 0;
	builder->jointCount = 0;
	builder->islandCount = 0;
}
