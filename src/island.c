// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "island.h"

#include "aabb.h"
#include "stack_allocator.h"
#include "array.h"
#include "body.h"
#include "contact.h"
#include "core.h"
#include "joint.h"
#include "shape.h"
#include "solver.h"
#include "world.h"

#include "box2d/callbacks.h"
#include "box2d/color.h"
#include "box2d/timer.h"

#include <float.h>
#include <stdatomic.h>
#include <stdlib.h>

void b2CreateIsland(b2Island* island)
{
	island->world = NULL;
	island->headBody = B2_NULL_INDEX;
	island->tailBody = B2_NULL_INDEX;
	island->bodyCount = 0;
	island->headContact = B2_NULL_INDEX;
	island->tailContact = B2_NULL_INDEX;
	island->contactCount = 0;
	island->headJoint = B2_NULL_INDEX;
	island->tailJoint = B2_NULL_INDEX;
	island->jointCount = 0;
	island->parentIsland = B2_NULL_INDEX;
	island->constraintRemoveCount = 0;
}

void b2DestroyIsland(b2Island* island)
{
	b2FreeObject(&island->world->islandPool, &island->object);
}

static void b2AddContactToIsland(b2World* world, b2Island* island, b2Contact* contact)
{
	B2_ASSERT(contact->islandIndex == B2_NULL_INDEX);
	B2_ASSERT(contact->islandPrev == B2_NULL_INDEX);
	B2_ASSERT(contact->islandNext == B2_NULL_INDEX);

	if (island->headContact != B2_NULL_INDEX)
	{
		contact->islandNext = island->headContact;
		b2Contact* headContact = world->contacts + island->headContact;
		headContact->islandPrev = contact->object.index;
	}

	island->headContact = contact->object.index;
	if (island->tailContact == B2_NULL_INDEX)
	{
		island->tailContact = island->headContact;
	}

	island->contactCount += 1;
	contact->islandIndex = island->object.index;

	b2ValidateIsland(island, false);
}

void b2WakeIsland(b2Island* island)
{
	b2World* world = island->world;

	if (island->awakeIndex != B2_NULL_INDEX)
	{
		// already awake
		B2_ASSERT(world->awakeIslandArray[island->awakeIndex] == island->object.index);
		return;
	}

	int32_t islandIndex = island->object.index;
	island->awakeIndex = b2Array(world->awakeIslandArray).count;
	b2Array_Push(world->awakeIslandArray, islandIndex);

	// Reset sleep timers on bodies
	// TODO_ERIN make this parallel somehow?
	int32_t bodyIndex = island->headBody;
	while (bodyIndex != B2_NULL_INDEX)
	{
		b2Body* body = world->bodies + bodyIndex;
		B2_ASSERT(body->islandIndex == islandIndex);
		body->sleepTime = 0.0f;
		bodyIndex = body->islandNext;
	}

	// Add constraints to graph
	int32_t contactIndex = island->headContact;
	while (contactIndex != B2_NULL_INDEX)
	{
		b2Contact* contact = world->contacts + contactIndex;
		B2_ASSERT(contact->islandIndex == islandIndex);
		b2AddContactToGraph(world, contact);
		contactIndex = contact->islandNext;
	}

	int32_t jointIndex = island->headJoint;
	while (jointIndex != B2_NULL_INDEX)
	{
		b2Joint* joint = world->joints + jointIndex;
		B2_ASSERT(joint->islandIndex == islandIndex);
		b2AddJointToGraph(world, joint);
		jointIndex = joint->islandNext;
	}
}

// https://en.wikipedia.org/wiki/Disjoint-set_data_structure
void b2LinkContact(b2World* world, b2Contact* contact)
{
	B2_ASSERT(contact->manifold.pointCount > 0);

	b2Body* bodyA = world->bodies + contact->edges[0].bodyIndex;
	b2Body* bodyB = world->bodies + contact->edges[1].bodyIndex;

	int32_t islandIndexA = bodyA->islandIndex;
	int32_t islandIndexB = bodyB->islandIndex;

	// Static bodies have null island indices
	B2_ASSERT(bodyA->type != b2_staticBody || islandIndexA == B2_NULL_INDEX);
	B2_ASSERT(bodyB->type != b2_staticBody || islandIndexB == B2_NULL_INDEX);
	B2_ASSERT(islandIndexA != B2_NULL_INDEX || islandIndexB != B2_NULL_INDEX);

	if (islandIndexA == islandIndexB)
	{
		// Contact in same island
		b2AddContactToIsland(world, world->islands + islandIndexA, contact);
		return;
	}

	// Union-find root of islandA
	b2Island* islandA = NULL;
	if (islandIndexA != B2_NULL_INDEX)
	{
		islandA = world->islands + islandIndexA;
b2WakeIsland(islandA);
		while (islandA->parentIsland != B2_NULL_INDEX)
		{
			b2Island* parent = world->islands + islandA->parentIsland;
			if (parent->parentIsland != B2_NULL_INDEX)
			{
				// path compression
				islandA->parentIsland = parent->parentIsland;
			}

			islandA = parent;
b2WakeIsland(islandA);
		}
	}

	// Union-find root of islandB
	b2Island* islandB = NULL;
	if (islandIndexB != B2_NULL_INDEX)
	{
		islandB = world->islands + islandIndexB;
b2WakeIsland(islandB);
		while (islandB->parentIsland != B2_NULL_INDEX)
		{
			b2Island* parent = world->islands + islandB->parentIsland;
			if (parent->parentIsland != B2_NULL_INDEX)
			{
				// path compression
				islandB->parentIsland = parent->parentIsland;
			}

			islandB = parent;
b2WakeIsland(islandB);
		}
	}

	B2_ASSERT(islandA != NULL || islandB != NULL);

	// Union-Find link island roots
	if (islandA != islandB && islandA != NULL && islandB != NULL)
	{
		B2_ASSERT(islandA != islandB);
		B2_ASSERT(islandB->parentIsland == B2_NULL_INDEX);
		islandB->parentIsland = islandA->object.index;
	}

	if (islandA != NULL)
	{
		b2AddContactToIsland(world, islandA, contact);
	}
	else
	{
		b2AddContactToIsland(world, islandB, contact);
	}
}

// This is called when a contact no longer has contact points
void b2UnlinkContact(b2World* world, b2Contact* contact)
{
	B2_ASSERT(contact->islandIndex != B2_NULL_INDEX);

	// remove from island
	b2Island* island = world->islands + contact->islandIndex;

	if (contact->islandPrev != B2_NULL_INDEX)
	{
		b2Contact* prevContact = world->contacts + contact->islandPrev;
		B2_ASSERT(prevContact->islandNext == contact->object.index);
		prevContact->islandNext = contact->islandNext;
	}

	if (contact->islandNext != B2_NULL_INDEX)
	{
		b2Contact* nextContact = world->contacts + contact->islandNext;
		B2_ASSERT(nextContact->islandPrev == contact->object.index);
		nextContact->islandPrev = contact->islandPrev;
	}

	if (island->headContact == contact->object.index)
	{
		island->headContact = contact->islandNext;
	}

	if (island->tailContact == contact->object.index)
	{
		island->tailContact = contact->islandPrev;
	}

	B2_ASSERT(island->contactCount > 0);
	island->contactCount -= 1;
	island->constraintRemoveCount += 1;

	contact->islandIndex = B2_NULL_INDEX;
	contact->islandPrev = B2_NULL_INDEX;
	contact->islandNext = B2_NULL_INDEX;

	b2ValidateIsland(island, false);
}

static void b2AddJointToIsland(b2World* world, b2Island* island, b2Joint* joint)
{
	B2_ASSERT(joint->islandIndex == B2_NULL_INDEX);
	B2_ASSERT(joint->islandPrev == B2_NULL_INDEX);
	B2_ASSERT(joint->islandNext == B2_NULL_INDEX);

	if (island->headJoint != B2_NULL_INDEX)
	{
		joint->islandNext = island->headJoint;
		b2Joint* headJoint = world->joints + island->headJoint;
		headJoint->islandPrev = joint->object.index;
	}

	island->headJoint = joint->object.index;
	if (island->tailJoint == B2_NULL_INDEX)
	{
		island->tailJoint = island->headJoint;
	}

	island->jointCount += 1;
	joint->islandIndex = island->object.index;

	b2ValidateIsland(island, false);
}

void b2LinkJoint(b2World* world, b2Joint* joint)
{
	b2Body* bodyA = world->bodies + joint->edges[0].bodyIndex;
	b2Body* bodyB = world->bodies + joint->edges[1].bodyIndex;

	int32_t islandIndexA = bodyA->islandIndex;
	int32_t islandIndexB = bodyB->islandIndex;

	// Static bodies have null island indices
	B2_ASSERT(bodyA->type != b2_staticBody || islandIndexA == B2_NULL_INDEX);
	B2_ASSERT(bodyB->type != b2_staticBody || islandIndexB == B2_NULL_INDEX);
	B2_ASSERT(islandIndexA != B2_NULL_INDEX || islandIndexB != B2_NULL_INDEX);

	if (islandIndexA == islandIndexB)
	{
		// Joint in same island
		b2AddJointToIsland(world, world->islands + islandIndexA, joint);
		return;
	}

	// Union-find root of islandA
	b2Island* islandA = NULL;
	if (islandIndexA != B2_NULL_INDEX)
	{
		islandA = world->islands + islandIndexA;
		b2WakeIsland(islandA);
		while (islandA->parentIsland != B2_NULL_INDEX)
		{
			b2Island* parent = world->islands + islandA->parentIsland;
			if (parent->parentIsland != B2_NULL_INDEX)
			{
				// path compression
				islandA->parentIsland = parent->parentIsland;
			}

			islandA = parent;
			b2WakeIsland(islandA);
		}
	}

	// Union-find root of islandB
	b2Island* islandB = NULL;
	if (islandIndexB != B2_NULL_INDEX)
	{
		islandB = world->islands + islandIndexB;
		b2WakeIsland(islandB);
		while (islandB->parentIsland != B2_NULL_INDEX)
		{
			b2Island* parent = world->islands + islandB->parentIsland;
			if (parent->parentIsland != B2_NULL_INDEX)
			{
				// path compression
				islandB->parentIsland = parent->parentIsland;
			}

			islandB = parent;
			b2WakeIsland(islandB);
		}
	}

	B2_ASSERT(islandA != NULL || islandB != NULL);

	// Union-Find link island roots
	if (islandA != islandB && islandA != NULL && islandB != NULL)
	{
		B2_ASSERT(islandA != islandB);
		B2_ASSERT(islandB->parentIsland == B2_NULL_INDEX);
		islandB->parentIsland = islandA->object.index;
	}

	if (islandA != NULL)
	{
		b2AddJointToIsland(world, islandA, joint);
	}
	else
	{
		b2AddJointToIsland(world, islandB, joint);
	}
}

void b2UnlinkJoint(b2World* world, b2Joint* joint)
{
	B2_ASSERT(joint->islandIndex != B2_NULL_INDEX);

	// remove from island
	b2Island* island = world->islands + joint->islandIndex;

	if (joint->islandPrev != B2_NULL_INDEX)
	{
		b2Joint* prevJoint = world->joints + joint->islandPrev;
		B2_ASSERT(prevJoint->islandNext == joint->object.index);
		prevJoint->islandNext = joint->islandNext;
	}

	if (joint->islandNext != B2_NULL_INDEX)
	{
		b2Joint* nextJoint = world->joints + joint->islandNext;
		B2_ASSERT(nextJoint->islandPrev == joint->object.index);
		nextJoint->islandPrev = joint->islandPrev;
	}

	if (island->headJoint == joint->object.index)
	{
		island->headJoint = joint->islandNext;
	}

	if (island->tailJoint == joint->object.index)
	{
		island->tailJoint = joint->islandPrev;
	}

	B2_ASSERT(island->jointCount > 0);
	island->jointCount -= 1;
	island->constraintRemoveCount += 1;

	joint->islandIndex = B2_NULL_INDEX;
	joint->islandPrev = B2_NULL_INDEX;
	joint->islandNext = B2_NULL_INDEX;

	b2ValidateIsland(island, false);
}

// Merge an island into its root island.
// Returns the body count of the merged island.
static int32_t b2MergeIsland(b2Island* island)
{
	B2_ASSERT(island->parentIsland != B2_NULL_INDEX);

	b2World* world = island->world;
	b2Body* bodies = world->bodies;
	b2Contact* contacts = world->contacts;
	b2Joint* joints = world->joints;

	int32_t rootIndex = island->parentIsland;
	b2Island* rootIsland = world->islands + rootIndex;
	B2_ASSERT(rootIsland->parentIsland == B2_NULL_INDEX);

	// remap island indices
	int32_t bodyIndex = island->headBody;
	while (bodyIndex != B2_NULL_INDEX)
	{
		b2Body* body = bodies + bodyIndex;
		body->islandIndex = rootIndex;
		bodyIndex = body->islandNext;
	}

	int32_t contactIndex = island->headContact;
	while (contactIndex != B2_NULL_INDEX)
	{
		b2Contact* contact = contacts + contactIndex;
		contact->islandIndex = rootIndex;
		contactIndex = contact->islandNext;
	}

	int32_t jointIndex = island->headJoint;
	while (jointIndex != B2_NULL_INDEX)
	{
		b2Joint* joint = joints + jointIndex;
		joint->islandIndex = rootIndex;
		jointIndex = joint->islandNext;
	}

	// connect body lists
	B2_ASSERT(rootIsland->tailBody != B2_NULL_INDEX);
	b2Body* tailBody = bodies + rootIsland->tailBody;
	B2_ASSERT(tailBody->islandNext == B2_NULL_INDEX);
	tailBody->islandNext = island->headBody;

	B2_ASSERT(island->headBody != B2_NULL_INDEX);
	b2Body* headBody = bodies + island->headBody;
	B2_ASSERT(headBody->islandPrev == B2_NULL_INDEX);
	headBody->islandPrev = rootIsland->tailBody;

	rootIsland->tailBody = island->tailBody;
	rootIsland->bodyCount += island->bodyCount;

	// connect contact lists
	if (rootIsland->headContact == B2_NULL_INDEX)
	{
		// Root island has no contacts
		B2_ASSERT(rootIsland->tailContact == B2_NULL_INDEX && rootIsland->contactCount == 0);
		rootIsland->headContact = island->headContact;
		rootIsland->tailContact = island->tailContact;
		rootIsland->contactCount = island->contactCount;
	}
	else if (island->headContact != B2_NULL_INDEX)
	{
		// Both islands have contacts
		B2_ASSERT(island->tailContact != B2_NULL_INDEX && island->contactCount > 0);
		B2_ASSERT(rootIsland->tailContact != B2_NULL_INDEX && rootIsland->contactCount > 0);

		b2Contact* tailContact = contacts + rootIsland->tailContact;
		B2_ASSERT(tailContact->islandNext == B2_NULL_INDEX);
		tailContact->islandNext = island->headContact;

		b2Contact* headContact = contacts + island->headContact;
		B2_ASSERT(headContact->islandPrev == B2_NULL_INDEX);
		headContact->islandPrev = rootIsland->tailContact;

		rootIsland->tailContact = island->tailContact;
		rootIsland->contactCount += island->contactCount;
	}

	if (rootIsland->headJoint == B2_NULL_INDEX)
	{
		// Root island has no joints
		B2_ASSERT(rootIsland->tailJoint == B2_NULL_INDEX && rootIsland->jointCount == 0);
		rootIsland->headJoint = island->headJoint;
		rootIsland->tailJoint = island->tailJoint;
		rootIsland->jointCount = island->jointCount;
	}
	else if (island->headJoint != B2_NULL_INDEX)
	{
		// Both islands have joints
		B2_ASSERT(island->tailJoint != B2_NULL_INDEX && island->jointCount > 0);
		B2_ASSERT(rootIsland->tailJoint != B2_NULL_INDEX && rootIsland->jointCount > 0);

		b2Joint* tailJoint = joints + rootIsland->tailJoint;
		B2_ASSERT(tailJoint->islandNext == B2_NULL_INDEX);
		tailJoint->islandNext = island->headJoint;

		b2Joint* headJoint = joints + island->headJoint;
		B2_ASSERT(headJoint->islandPrev == B2_NULL_INDEX);
		headJoint->islandPrev = rootIsland->tailJoint;

		rootIsland->tailJoint = island->tailJoint;
		rootIsland->jointCount += island->jointCount;
	}

	// Merging a dirty islands means that splitting may still be needed
	rootIsland->constraintRemoveCount += island->constraintRemoveCount;
	b2ValidateIsland(rootIsland, true);

	return rootIsland->bodyCount;
}

// Iterate over all awake islands and merge any that need merging
// Islands that get merged into a root island will be removed from the awake island array
// and returned to the pool.
void b2MergeAwakeIslands(b2World* world)
{
	int32_t awakeIslandCount = b2Array(world->awakeIslandArray).count;
	b2Island* islands = world->islands;

	// Step 1: Ensure every child island points to its root island. This avoids merging a child island with
	// a parent island that has already been merged with a grand-parent island.
	for (int32_t i = 0; i < awakeIslandCount; ++i)
	{
		int32_t islandIndex = world->awakeIslandArray[i];
		b2Island* island = islands + islandIndex;

		b2Island* rootIsland = island;
		while (rootIsland->parentIsland != B2_NULL_INDEX)
		{
			b2Island* parent = islands + rootIsland->parentIsland;
			if (parent->parentIsland != B2_NULL_INDEX)
			{
				// path compression
				rootIsland->parentIsland = parent->parentIsland;
			}

			rootIsland = parent;
		}

		if (rootIsland != island)
		{
			island->parentIsland = rootIsland->object.index;
		}
	}

	// Step 2: merge every awake island into its parent (which must be a root island)
	// Reverse to support removal from awake array.
	int32_t maxBodyCount = 0;
	for (int32_t i = awakeIslandCount - 1; i >= 0; --i)
	{
		int32_t islandIndex = world->awakeIslandArray[i];
		b2Island* island = islands + islandIndex;

		if (island->parentIsland == B2_NULL_INDEX)
		{
			maxBodyCount = B2_MAX(maxBodyCount, island->bodyCount);
			continue;
		}

		int32_t mergedBodyCount = b2MergeIsland(island);
		maxBodyCount = B2_MAX(maxBodyCount, mergedBodyCount);

		b2DestroyIsland(island);
	}

	// Step 3: ensure island pool has sufficient space to split the largest island
	b2GrowPool(&world->islandPool, world->islandPool.count + maxBodyCount);
	world->islands = (b2Island*)world->islandPool.memory;
}

#define B2_CONTACT_REMOVE_THRESHOLD 1

// Split an island because some contacts and/or joints have been removed.
// This is called during the constraint solve while islands are not being touched. This uses DFS and touches a lot of memory,
// so it can be quite slow.
// Note: contacts/joints connected to static bodies must belong to an island but don't affect island connectivity
// Note: static bodies are never in an island
// Note: this task interacts with some allocators without locks under the assumption that no other tasks
// are interacting with these data structures.
void b2SplitIslandTask(int32_t startIndex, int32_t endIndex, uint32_t threadIndex, void* context)
{
	b2TracyCZoneNC(split, "Split Island", b2_colorHoneydew2, true);

	B2_MAYBE_UNUSED(startIndex);
	B2_MAYBE_UNUSED(endIndex);
	B2_MAYBE_UNUSED(threadIndex);

	b2World* world = context;

	B2_ASSERT(world->splitIslandIndex != B2_NULL_INDEX);

	b2Island* baseIsland = world->islands + world->splitIslandIndex;

	b2ValidateIsland(baseIsland, true);

	int32_t bodyCount = baseIsland->bodyCount;

	b2Body* bodies = world->bodies;
	b2Contact* contacts = world->contacts;
	b2Joint* joints = world->joints;

	b2StackAllocator* alloc = world->stackAllocator;

	// No lock is needed because I ensure these are not used while this task is active.
	int32_t* stack = b2AllocateStackItem(alloc, bodyCount * sizeof(int32_t), "island stack");
	int32_t* bodyIndices = b2AllocateStackItem(alloc, bodyCount * sizeof(int32_t), "body indices");

	// Build array containing all body indices from base island. These
	// serve as seed bodies for the depth first search (DFS).
	int32_t index = 0;
	int32_t nextBody = baseIsland->headBody;
	while (nextBody != B2_NULL_INDEX)
	{
		bodyIndices[index++] = nextBody;
		b2Body* body = bodies + nextBody;

		// Clear visitation mark
		body->isMarked = false;

		nextBody = body->islandNext;
	}
	B2_ASSERT(index == bodyCount);

	// Clear contact island flags. Only need to consider contacts
	// already in the base island.
	int32_t nextContact = baseIsland->headContact;
	while (nextContact != B2_NULL_INDEX)
	{
		b2Contact* contact = contacts + nextContact;
		contact->isMarked = false;
		nextContact = contact->islandNext;
	}

	// Clear joint island flags.
	int32_t nextJoint = baseIsland->headJoint;
	while (nextJoint != B2_NULL_INDEX)
	{
		b2Joint* joint = joints + nextJoint;
		joint->isMarked = false;
		nextJoint = joint->islandNext;
	}

	// Done with the base split island.
	b2DestroyIsland(baseIsland);
	baseIsland = NULL;

	// Each island is found as a depth first search starting from a seed body
	for (int32_t i = 0; i < bodyCount; ++i)
	{
		int32_t seedIndex = bodyIndices[i];
		b2Body* seed = bodies + seedIndex;
		B2_ASSERT(seed->object.next == seedIndex);
		B2_ASSERT(seed->isEnabled);
		B2_ASSERT(seed->type != b2_staticBody);

		if (seed->isMarked == true)
		{
			// The body has already been visited
			continue;
		}

		int32_t stackCount = 0;
		stack[stackCount++] = seedIndex;
		seed->isMarked = true;

		// Create new island
		// No lock needed because only a single island can split per time step
		// However, it is not safe to cause the island pool to grow because other islands are being processed.
		// So this allocation must come from the free list.
		B2_ASSERT(world->islandPool.capacity > world->islandPool.count);
		b2Island* island = (b2Island*)b2AllocObject(&world->islandPool);
		b2CreateIsland(island);
		island->world = world;

		int32_t islandIndex = island->object.index;

		// Perform a depth first search (DFS) on the constraint graph.
		while (stackCount > 0)
		{
			// Grab the next body off the stack and add it to the island.
			int32_t bodyIndex = stack[--stackCount];
			b2Body* body = bodies + bodyIndex;
			B2_ASSERT(body->type != b2_staticBody);
			B2_ASSERT(body->isMarked == true);

			// Add body to island
			body->islandIndex = islandIndex;
			if (island->tailBody != B2_NULL_INDEX)
			{
				bodies[island->tailBody].islandNext = bodyIndex;
			}
			body->islandPrev = island->tailBody;
			body->islandNext = B2_NULL_INDEX;
			island->tailBody = bodyIndex;

			if (island->headBody == B2_NULL_INDEX)
			{
				island->headBody = bodyIndex;
			}

			island->bodyCount += 1;

			// Search all contacts connected to this body.
			int32_t contactKey = body->contactList;
			while (contactKey != B2_NULL_INDEX)
			{
				int32_t contactIndex = contactKey >> 1;
				int32_t edgeIndex = contactKey & 1;

				b2Contact* contact = contacts + contactIndex;
				B2_ASSERT(contact->object.index == contactIndex);

				// Next key
				contactKey = contact->edges[edgeIndex].nextKey;

				// Has this contact already been added to this island?
				if (contact->isMarked)
				{
					continue;
				}

				// Skip sensors
				if (contact->flags & b2_contactSensorFlag)
				{
					continue;
				}

				// Is this contact enabled and touching?
				if ((contact->flags & b2_contactTouchingFlag) == 0)
				{
					continue;
				}

				contact->isMarked = true;

				int32_t otherEdgeIndex = edgeIndex ^ 1;
				int32_t otherBodyIndex = contact->edges[otherEdgeIndex].bodyIndex;
				b2Body* otherBody = world->bodies + otherBodyIndex;

				// Maybe add other body to stack
				if (otherBody->isMarked == false && otherBody->type != b2_staticBody)
				{
					B2_ASSERT(stackCount < bodyCount);
					stack[stackCount++] = otherBodyIndex;
					otherBody->isMarked = true;
				}

				// Add contact to island
				contact->islandIndex = islandIndex;
				if (island->tailContact != B2_NULL_INDEX)
				{
					contacts[island->tailContact].islandNext = contactIndex;
				}
				contact->islandPrev = island->tailContact;
				contact->islandNext = B2_NULL_INDEX;
				island->tailContact = contactIndex;

				if (island->headContact == B2_NULL_INDEX)
				{
					island->headContact = contactIndex;
				}

				island->contactCount += 1;
			}

			// Search all joints connect to this body.
			int32_t jointKey = body->jointList;
			while (jointKey != B2_NULL_INDEX)
			{
				int32_t jointIndex = jointKey >> 1;
				int32_t edgeIndex = jointKey & 1;

				b2Joint* joint = world->joints + jointIndex;
				B2_ASSERT(joint->object.index == jointIndex);

				// Next key
				jointKey = joint->edges[edgeIndex].nextKey;

				// Has this joint already been added to this island?
				if (joint->isMarked)
				{
					continue;
				}

				joint->isMarked = true;

				int32_t otherEdgeIndex = edgeIndex ^ 1;
				int32_t otherBodyIndex = joint->edges[otherEdgeIndex].bodyIndex;
				b2Body* otherBody = world->bodies + otherBodyIndex;

				// Don't simulate joints connected to disabled bodies.
				if (otherBody->isEnabled == false)
				{
					continue;
				}

				// Maybe add other body to stack
				if (otherBody->isMarked == false && otherBody->type != b2_staticBody)
				{
					B2_ASSERT(stackCount < bodyCount);
					stack[stackCount++] = otherBodyIndex;
					otherBody->isMarked = true;
				}

				// Add joint to island
				joint->islandIndex = islandIndex;
				if (island->tailJoint != B2_NULL_INDEX)
				{
					joints[island->tailJoint].islandNext = jointIndex;
				}
				joint->islandPrev = island->tailJoint;
				joint->islandNext = B2_NULL_INDEX;
				island->tailJoint = jointIndex;

				if (island->headJoint == B2_NULL_INDEX)
				{
					island->headJoint = jointIndex;
				}

				island->jointCount += 1;
			}
		}

		// For consistency, this island must be added to the awake island array. This should
		// be safe because no other task is accessing this and the solver has already gathered
		// all awake bodies.
		island->awakeIndex = b2Array(world->awakeIslandArray).count;
		b2Array_Push(world->awakeIslandArray, islandIndex);

		b2ValidateIsland(island, true);
	}

	b2FreeStackItem(alloc, bodyIndices);
	b2FreeStackItem(alloc, stack);

	b2TracyCZoneEnd(split);
}

#if B2_VALIDATE

void b2ValidateIsland(b2Island* island, bool checkSleep)
{
	b2World* world = island->world;

	int32_t islandIndex = island->object.index;
	B2_ASSERT(island->object.index == island->object.next);

	bool isAwake = false;
	if (island->awakeIndex != B2_NULL_INDEX)
	{
		b2Array_Check(world->awakeIslandArray, island->awakeIndex);
		B2_ASSERT(world->awakeIslandArray[island->awakeIndex] == islandIndex);
		isAwake = true;
	}

	B2_ASSERT(island->headBody != B2_NULL_INDEX);

	{
		b2Body* bodies = world->bodies;
		B2_ASSERT(island->tailBody != B2_NULL_INDEX);
		B2_ASSERT(island->bodyCount > 0);
		if (island->bodyCount > 1)
		{
			B2_ASSERT(island->tailBody != island->headBody);
		}
		B2_ASSERT(island->bodyCount <= world->bodyPool.count);

		int32_t count = 0;
		int32_t bodyIndex = island->headBody;
		while (bodyIndex != B2_NULL_INDEX)
		{
			b2Body* body = bodies + bodyIndex;
			B2_ASSERT(body->islandIndex == islandIndex);
			count += 1;

			if (count == island->bodyCount)
			{
				B2_ASSERT(bodyIndex == island->tailBody);
			}

			bodyIndex = body->islandNext;
		}
		B2_ASSERT(count == island->bodyCount);
	}

	if (island->headContact != B2_NULL_INDEX)
	{
		b2Contact* contacts = world->contacts;
		B2_ASSERT(island->tailContact != B2_NULL_INDEX);
		B2_ASSERT(island->contactCount > 0);
		if (island->contactCount > 1)
		{
			B2_ASSERT(island->tailContact != island->headContact);
		}
		B2_ASSERT(island->contactCount <= world->contactPool.count);

		int32_t count = 0;
		int32_t contactIndex = island->headContact;
		while (contactIndex != B2_NULL_INDEX)
		{
			b2Contact* contact = contacts + contactIndex;
			B2_ASSERT(contact->islandIndex == islandIndex);
			count += 1;

			if (checkSleep)
			{
				if (isAwake)
				{
					B2_ASSERT(contact->colorIndex != B2_NULL_INDEX);
					B2_ASSERT(contact->colorSubIndex != B2_NULL_INDEX);

					// int32_t awakeIndex = world->contactAwakeIndexArray[contactIndex];
					// B2_ASSERT(0 <= awakeIndex && awakeIndex < b2Array(world->awakeContactArray).count);
					// B2_ASSERT(world->awakeContactArray[awakeIndex] == contactIndex);
				}
				else
				{
					B2_ASSERT(contact->colorIndex == B2_NULL_INDEX);
					B2_ASSERT(contact->colorSubIndex == B2_NULL_INDEX);
					// B2_ASSERT(world->contactAwakeIndexArray[contactIndex] == B2_NULL_INDEX);
				}
			}

			if (count == island->contactCount)
			{
				B2_ASSERT(contactIndex == island->tailContact);
			}

			contactIndex = contact->islandNext;
		}
		B2_ASSERT(count == island->contactCount);
	}
	else
	{
		B2_ASSERT(island->tailContact == B2_NULL_INDEX);
		B2_ASSERT(island->contactCount == 0);
	}

	if (island->headJoint != B2_NULL_INDEX)
	{
		b2Joint* joints = world->joints;
		B2_ASSERT(island->tailJoint != B2_NULL_INDEX);
		B2_ASSERT(island->jointCount > 0);
		if (island->jointCount > 1)
		{
			B2_ASSERT(island->tailJoint != island->headJoint);
		}
		B2_ASSERT(island->jointCount <= world->jointPool.count);

		int32_t count = 0;
		int32_t jointIndex = island->headJoint;
		while (jointIndex != B2_NULL_INDEX)
		{
			b2Joint* joint = joints + jointIndex;
			B2_ASSERT(joint->islandIndex == islandIndex);
			count += 1;

			if (count == island->jointCount)
			{
				B2_ASSERT(jointIndex == island->tailJoint);
			}

			jointIndex = joint->islandNext;
		}
		B2_ASSERT(count == island->jointCount);
	}
	else
	{
		B2_ASSERT(island->tailJoint == B2_NULL_INDEX);
		B2_ASSERT(island->jointCount == 0);
	}
}

#else

void b2ValidateIsland(b2Island* island, bool checkSleep)
{
	B2_MAYBE_UNUSED(island);
	B2_MAYBE_UNUSED(checkSleep);
}

#endif
