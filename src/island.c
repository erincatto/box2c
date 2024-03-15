// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "island.h"

#include "stack_allocator.h"
#include "array.h"
#include "body.h"
#include "contact.h"
#include "core.h"
#include "joint.h"
#include "solver_set.h"
#include "util.h"
#include "world.h"

#include "box2d/color.h"
#include "box2d/timer.h"

#include <float.h>
#include <stdatomic.h>
#include <stdlib.h>

b2Island* b2CreateIsland(b2World* world, int setIndex)
{
	B2_ASSERT(setIndex == b2_awakeSet || setIndex >= b2_firstSleepingSet);
	b2CheckIndex(world->solverSetArray, setIndex);

	b2SolverSet* set = world->solverSetArray + setIndex;
	b2Island* island = b2AddIsland(&world->blockAllocator, &set->islands);
	int islandId = b2AllocId(&world->islandIdPool);

	if (islandId == b2Array(world->islandLookupArray).count)
	{
		b2IslandLookup lookup = {setIndex, set->islands.count - 1};
		b2Array_Push(world->islandLookupArray, lookup);
	}
	else
	{
		b2CheckIndex(world->islandLookupArray, islandId);
		b2IslandLookup* lookup = world->islandLookupArray + islandId;
		B2_ASSERT(lookup->setIndex == B2_NULL_INDEX && lookup->islandIndex == B2_NULL_INDEX);
		lookup->setIndex = setIndex;
		lookup->islandIndex = set->islands.count - 1;
	}

	island->islandId = islandId;
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

	return island;
}

void b2DestroyIsland(b2World* world, int islandId)
{
	// assume island is empty
	b2CheckIndex(world->islandLookupArray, islandId);
	b2IslandLookup* lookup = world->islandLookupArray + islandId;
	b2CheckIndex(world->solverSetArray, lookup->setIndex);
	b2SolverSet* set = world->solverSetArray + lookup->setIndex;
	int movedIndex = b2RemoveIsland(&world->blockAllocator, &set->islands, lookup->islandIndex);
	if (movedIndex != B2_NULL_INDEX)
	{
		// Fix lookup on moved element
		b2Island* movedElement = set->islands.data + lookup->islandIndex;
		int movedId = movedElement->islandId;
		b2IslandLookup* movedLookup = world->islandLookupArray + movedId;
		B2_ASSERT(movedLookup->islandIndex == movedIndex);
		movedLookup->islandIndex = lookup->islandIndex;
	}

	// Free lookup and id (preserve lookup revision)
	lookup->setIndex = B2_NULL_INDEX;
	lookup->islandIndex = B2_NULL_INDEX;
	b2FreeId(&world->islandIdPool, islandId);
}

b2Island* b2GetIsland(b2World* world, int islandId)
{
	b2CheckIndex(world->islandLookupArray, islandId);
	b2IslandLookup* lookup = world->islandLookupArray + islandId;
	b2CheckIndex(world->solverSetArray, lookup->setIndex);
	b2SolverSet* set = world->solverSetArray + lookup->setIndex;
	B2_ASSERT(0 <= lookup->islandIndex && lookup->islandIndex < set->islands.count);
	return set->islands.data + lookup->islandIndex;
}

static void b2AddContactToIsland(b2World* world, b2Island* island, b2Contact* contact)
{
	B2_ASSERT(contact->islandId == B2_NULL_INDEX);
	B2_ASSERT(contact->islandPrev == B2_NULL_INDEX);
	B2_ASSERT(contact->islandNext == B2_NULL_INDEX);

	if (island->headContact != B2_NULL_INDEX)
	{
		contact->islandNext = island->headContact;
		b2Contact* headContact = b2GetContactFromRawId(world, island->headContact);
		headContact->islandPrev = contact->contactId;
	}

	island->headContact = contact->contactId;
	if (island->tailContact == B2_NULL_INDEX)
	{
		island->tailContact = island->headContact;
	}

	island->contactCount += 1;
	contact->islandId = island->islandId;

	b2ValidateIsland(world, island, false);
}

void b2WakeIsland(b2World* world, b2Island* island)
{
	b2CheckIndex(world->islandLookupArray, island->islandId);
	b2IslandLookup* lookup = world->islandLookupArray + island->islandId;
	b2WakeSolverSet(world, lookup->setIndex);

	#if 0
	// Reset sleep timers on bodies
	// TODO_ERIN make this parallel somehow?
	int bodyIndex = island->headBody;
	while (bodyIndex != B2_NULL_INDEX)
	{
		b2Body* body = world->bodies + bodyIndex;
		B2_ASSERT(body->islandIndex == islandIndex);
		body->sleepTime = 0.0f;
		bodyIndex = body->islandNext;
	}

	// Add constraints to graph
	int contactIndex = island->headContact;
	while (contactIndex != B2_NULL_INDEX)
	{
		b2Contact* contact = world->contacts + contactIndex;
		B2_ASSERT(contact->islandId == islandIndex);
		b2AddContactToGraph(world, contact);
		contactIndex = contact->islandNext;
	}

	int jointIndex = island->headJoint;
	while (jointIndex != B2_NULL_INDEX)
	{
		b2Joint* joint = world->joints + jointIndex;
		B2_ASSERT(joint->islandIndex == islandIndex);
		b2AddJointToGraph(world, joint);
		jointIndex = joint->islandNext;
	}
	#endif
}

void b2SleepIsland(b2World* world, b2Island* island)
{
	B2_ASSERT(island->constraintRemoveCount == 0);
	// todo
}

// https://en.wikipedia.org/wiki/Disjoint-set_data_structure
void b2LinkContact(b2World* world, b2Contact* contact)
{
	B2_ASSERT(contact->manifold.pointCount > 0);

	// todo can assume body is either awake or static

	b2Body* bodyA = b2GetBodyFromRawId(world, contact->edges[0].bodyId);
	b2Body* bodyB = b2GetBodyFromRawId(world, contact->edges[1].bodyId);

	int islandIdA = bodyA->islandId;
	int islandIdB = bodyB->islandId;

	// Static bodies have null island indices
	B2_ASSERT(bodyA->type != b2_staticBody || islandIdA == B2_NULL_INDEX);
	B2_ASSERT(bodyB->type != b2_staticBody || islandIdB == B2_NULL_INDEX);
	B2_ASSERT(islandIdA != B2_NULL_INDEX || islandIdB != B2_NULL_INDEX);

	if (islandIdA == islandIdB)
	{
		// Contact in same island
		b2Island* island = b2GetIsland(world, islandIdA);
		b2AddContactToIsland(world, island, contact);
		return;
	}

	// Union-find root of islandA
	b2Island* islandA = NULL;
	if (islandIdA != B2_NULL_INDEX)
	{
		islandA = b2GetIsland(world, islandIdA);
		int parentId = islandA->parentIsland;
		b2WakeIsland(world, islandA);
		while (parentId != B2_NULL_INDEX)
		{
			b2Island* parent = b2GetIsland(world, parentId);
			if (parent->parentIsland != B2_NULL_INDEX)
			{
				// path compression
				islandA->parentIsland = parent->parentIsland;
			}

			islandA = parent;
			parentId = islandA->parentIsland;
			b2WakeIsland(world, islandA);
		}
	}

	// Union-find root of islandB
	b2Island* islandB = NULL;
	if (islandIdB != B2_NULL_INDEX)
	{
		islandB = b2GetIsland(world, islandIdB);
		int parentId = islandA->parentIsland;
		b2WakeIsland(world, islandB);
		while (islandB->parentIsland != B2_NULL_INDEX)
		{
			b2Island* parent = b2GetIsland(world, parentId);
			if (parent->parentIsland != B2_NULL_INDEX)
			{
				// path compression
				islandB->parentIsland = parent->parentIsland;
			}

			islandB = parent;
			parentId = islandB->parentIsland;
			b2WakeIsland(world, islandB);
		}
	}

	B2_ASSERT(islandA != NULL || islandB != NULL);

	// Union-Find link island roots
	if (islandA != islandB && islandA != NULL && islandB != NULL)
	{
		B2_ASSERT(islandA != islandB);
		B2_ASSERT(islandB->parentIsland == B2_NULL_INDEX);
		islandB->parentIsland = islandA->islandId;
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
	B2_ASSERT(contact->islandId != B2_NULL_INDEX);

	// remove from island
	b2Island* island = b2GetIsland(world, contact->islandId);

	if (contact->islandPrev != B2_NULL_INDEX)
	{
		b2Contact* prevContact = b2GetContactFromRawId(world, contact->islandPrev);
		B2_ASSERT(prevContact->islandNext == contact->contactId);
		prevContact->islandNext = contact->islandNext;
	}

	if (contact->islandNext != B2_NULL_INDEX)
	{
		b2Contact* nextContact = b2GetContactFromRawId(world, contact->islandNext);
		B2_ASSERT(nextContact->islandPrev == contact->contactId);
		nextContact->islandPrev = contact->islandPrev;
	}

	if (island->headContact == contact->contactId)
	{
		island->headContact = contact->islandNext;
	}

	if (island->tailContact == contact->contactId)
	{
		island->tailContact = contact->islandPrev;
	}

	B2_ASSERT(island->contactCount > 0);
	island->contactCount -= 1;
	island->constraintRemoveCount += 1;

	contact->islandId = B2_NULL_INDEX;
	contact->islandPrev = B2_NULL_INDEX;
	contact->islandNext = B2_NULL_INDEX;

	b2ValidateIsland(world, island, false);
}

static void b2AddJointToIsland(b2World* world, b2Island* island, b2Joint* joint)
{
	B2_ASSERT(joint->islandId == B2_NULL_INDEX);
	B2_ASSERT(joint->islandPrev == B2_NULL_INDEX);
	B2_ASSERT(joint->islandNext == B2_NULL_INDEX);

	if (island->headJoint != B2_NULL_INDEX)
	{
		joint->islandNext = island->headJoint;
		b2Joint* headJoint = b2GetJointFromKey(world, island->headJoint);
		headJoint->islandPrev = joint->jointId;
	}

	island->headJoint = joint->jointId;
	if (island->tailJoint == B2_NULL_INDEX)
	{
		island->tailJoint = island->headJoint;
	}

	island->jointCount += 1;
	joint->islandId = island->islandId;

	b2ValidateIsland(world, island, false);
}

void b2LinkJoint(b2World* world, b2Joint* joint)
{
	b2Body* bodyA = b2GetBodyFromRawId(world, joint->edges[0].bodyId);
	b2Body* bodyB = b2GetBodyFromRawId(world, joint->edges[1].bodyId);

	int islandIdA = bodyA->islandId;
	int islandIdB = bodyB->islandId;

	// Static bodies have null island indices
	B2_ASSERT(bodyA->type != b2_staticBody || islandIdA == B2_NULL_INDEX);
	B2_ASSERT(bodyB->type != b2_staticBody || islandIdB == B2_NULL_INDEX);
	B2_ASSERT(islandIdA != B2_NULL_INDEX || islandIdB != B2_NULL_INDEX);

	if (islandIdA == islandIdB)
	{
		// Joint in same island
		b2Island* island = b2GetIsland(world, islandIdA);
		b2AddJointToIsland(world, island, joint);
		return;
	}

	// Union-find root of islandA
	b2Island* islandA = NULL;
	if (islandIdA != B2_NULL_INDEX)
	{
		islandA = b2GetIsland(world, islandIdA);
		b2WakeIsland(world, islandA);
		while (islandA->parentIsland != B2_NULL_INDEX)
		{
			b2Island* parent = b2GetIsland(world, islandA->parentIsland);
			if (parent->parentIsland != B2_NULL_INDEX)
			{
				// path compression
				islandA->parentIsland = parent->parentIsland;
			}

			islandA = parent;
			b2WakeIsland(world, islandA);
		}
	}

	// Union-find root of islandB
	b2Island* islandB = NULL;
	if (islandIdB != B2_NULL_INDEX)
	{
		islandB = b2GetIsland(world, islandIdB);
		b2WakeIsland(world, islandB);
		while (islandB->parentIsland != B2_NULL_INDEX)
		{
			b2Island* parent = b2GetIsland(world, islandB->parentIsland);
			if (parent->parentIsland != B2_NULL_INDEX)
			{
				// path compression
				islandB->parentIsland = parent->parentIsland;
			}

			islandB = parent;
			b2WakeIsland(world, islandB);
		}
	}

	B2_ASSERT(islandA != NULL || islandB != NULL);

	// Union-Find link island roots
	if (islandA != islandB && islandA != NULL && islandB != NULL)
	{
		B2_ASSERT(islandA != islandB);
		B2_ASSERT(islandB->parentIsland == B2_NULL_INDEX);
		islandB->parentIsland = islandA->islandId;
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
	B2_ASSERT(joint->islandId != B2_NULL_INDEX);

	// remove from island
	b2Island* island = b2GetIsland(world, joint->islandId);

	if (joint->islandPrev != B2_NULL_INDEX)
	{
		b2Joint* prevJoint = b2GetJointFromKey(world, joint->islandPrev);
		B2_ASSERT(prevJoint->islandNext == joint->jointId);
		prevJoint->islandNext = joint->islandNext;
	}

	if (joint->islandNext != B2_NULL_INDEX)
	{
		b2Joint* nextJoint = b2GetJointFromKey(world, joint->islandNext);
		B2_ASSERT(nextJoint->islandPrev == joint->jointId);
		nextJoint->islandPrev = joint->islandPrev;
	}

	if (island->headJoint == joint->jointId)
	{
		island->headJoint = joint->islandNext;
	}

	if (island->tailJoint == joint->jointId)
	{
		island->tailJoint = joint->islandPrev;
	}

	B2_ASSERT(island->jointCount > 0);
	island->jointCount -= 1;
	island->constraintRemoveCount += 1;

	joint->islandId = B2_NULL_INDEX;
	joint->islandPrev = B2_NULL_INDEX;
	joint->islandNext = B2_NULL_INDEX;

	b2ValidateIsland(world, island, false);
}

// Merge an island into its root island.
// Returns the body count of the merged island.
// todo we can assume all islands are awake here
static int b2MergeIsland(b2World* world, b2Island* island)
{
	B2_ASSERT(island->parentIsland != B2_NULL_INDEX);

	int rootId = island->parentIsland;
	b2Island* rootIsland = b2GetIsland(world, rootId);
	B2_ASSERT(rootIsland->parentIsland == B2_NULL_INDEX);

	// remap island indices
	int bodyId = island->headBody;
	while (bodyId != B2_NULL_INDEX)
	{
		b2Body* body = b2GetBodyFromRawId(world, bodyId);
		body->islandId = rootId;
		bodyId = body->islandNext;
	}

	int contactId = island->headContact;
	while (contactId != B2_NULL_INDEX)
	{
		b2Contact* contact = b2GetContactFromRawId(world, contactId);
		contact->islandId = rootId;
		contactId = contact->islandNext;
	}

	int jointId = island->headJoint;
	while (jointId != B2_NULL_INDEX)
	{
		b2Joint* joint = b2GetJointFromKey(world, jointId);
		joint->islandId = rootId;
		jointId = joint->islandNext;
	}

	// connect body lists
	B2_ASSERT(rootIsland->tailBody != B2_NULL_INDEX);
	b2Body* tailBody = b2GetBodyFromRawId(world, rootIsland->tailBody);
	B2_ASSERT(tailBody->islandNext == B2_NULL_INDEX);
	tailBody->islandNext = island->headBody;

	B2_ASSERT(island->headBody != B2_NULL_INDEX);
	b2Body* headBody = b2GetBodyFromRawId(world, island->headBody);
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

		b2Contact* tailContact = b2GetContactFromRawId(world, rootIsland->tailContact);
		B2_ASSERT(tailContact->islandNext == B2_NULL_INDEX);
		tailContact->islandNext = island->headContact;

		b2Contact* headContact = b2GetContactFromRawId(world, island->headContact);
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

		b2Joint* tailJoint = b2GetJointFromKey(world, rootIsland->tailJoint);
		B2_ASSERT(tailJoint->islandNext == B2_NULL_INDEX);
		tailJoint->islandNext = island->headJoint;

		b2Joint* headJoint = b2GetJointFromKey(world, island->headJoint);
		B2_ASSERT(headJoint->islandPrev == B2_NULL_INDEX);
		headJoint->islandPrev = rootIsland->tailJoint;

		rootIsland->tailJoint = island->tailJoint;
		rootIsland->jointCount += island->jointCount;
	}

	// Merging a dirty islands means that splitting may still be needed
	rootIsland->constraintRemoveCount += island->constraintRemoveCount;
	b2ValidateIsland(world, rootIsland, true);

	return rootIsland->bodyCount;
}

// Iterate over all awake islands and merge any that need merging
// Islands that get merged into a root island will be removed from the awake island array
// and returned to the pool.
void b2MergeAwakeIslands(b2World* world)
{
	b2SolverSet* awakeSet = world->solverSetArray + b2_awakeSet;
	b2Island* islands = awakeSet->islands.data;
	int awakeIslandCount = awakeSet->islands.count;
	b2IslandLookup* lookup = world->islandLookupArray;

	// Step 1: Ensure every child island points to its root island. This avoids merging a child island with
	// a parent island that has already been merged with a grand-parent island.
	for (int i = 0; i < awakeIslandCount; ++i)
	{
		b2Island* island = islands + i;

		b2Island* rootIsland = island;
		while (rootIsland->parentIsland != B2_NULL_INDEX)
		{
			b2Island* parent = islands + lookup[rootIsland->parentIsland].islandIndex;
			if (parent->parentIsland != B2_NULL_INDEX)
			{
				// path compression
				rootIsland->parentIsland = parent->parentIsland;
			}

			rootIsland = parent;
		}

		if (rootIsland != island)
		{
			island->parentIsland = rootIsland->islandId;
		}
	}

	// Step 2: merge every awake island into its parent (which must be a root island)
	// Reverse to support removal from awake array.
	int maxBodyCount = 0;
	for (int i = awakeIslandCount - 1; i >= 0; --i)
	{
		b2Island* island = islands + i;

		if (island->parentIsland == B2_NULL_INDEX)
		{
			maxBodyCount = B2_MAX(maxBodyCount, island->bodyCount);
			continue;
		}

		int mergedBodyCount = b2MergeIsland(world, island);
		maxBodyCount = B2_MAX(maxBodyCount, mergedBodyCount);

		// todo destroy directly in awake set
		b2DestroyIsland(world, island->islandId);
	}

	// Step 3: ensure island pool has sufficient space to split the largest island
	// todo anything to do here?
	//b2GrowPool(&world->islandPool, world->islandPool.count + maxBodyCount);
	//world->islands = (b2Island*)world->islandPool.memory;
}

#define B2_CONTACT_REMOVE_THRESHOLD 1

// Split an island because some contacts and/or joints have been removed.
// This is called during the constraint solve while islands are not being touched. This uses DFS and touches a lot of memory,
// so it can be quite slow.
// Note: contacts/joints connected to static bodies must belong to an island but don't affect island connectivity
// Note: static bodies are never in an island
// Note: this task interacts with some allocators without locks under the assumption that no other tasks
// are interacting with these data structures.
#if 0
void b2SplitIslandTask(int startIndex, int endIndex, uint32_t threadIndex, void* context)
{
	b2TracyCZoneNC(split, "Split Island", b2_colorHoneydew2, true);

	B2_MAYBE_UNUSED(startIndex);
	B2_MAYBE_UNUSED(endIndex);
	B2_MAYBE_UNUSED(threadIndex);

	b2World* world = context;

	B2_ASSERT(world->splitIslandIndex != B2_NULL_INDEX);

	b2Island* baseIsland = world->islands + world->splitIslandIndex;

	b2ValidateIsland(world, baseIsland, true);

	int bodyCount = baseIsland->bodyCount;

	b2Body* bodies = world->bodies;
	b2Contact* contacts = world->contacts;
	b2Joint* joints = world->joints;

	b2StackAllocator* alloc = world->stackAllocator;

	// No lock is needed because I ensure these are not used while this task is active.
	int* stack = b2AllocateStackItem(alloc, bodyCount * sizeof(int), "island stack");
	int* bodyIndices = b2AllocateStackItem(alloc, bodyCount * sizeof(int), "body indices");

	// Build array containing all body indices from base island. These
	// serve as seed bodies for the depth first search (DFS).
	int index = 0;
	int nextBody = baseIsland->headBody;
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
	int nextContact = baseIsland->headContact;
	while (nextContact != B2_NULL_INDEX)
	{
		b2Contact* contact = contacts + nextContact;
		contact->isMarked = false;
		nextContact = contact->islandNext;
	}

	// Clear joint island flags.
	int nextJoint = baseIsland->headJoint;
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
	for (int i = 0; i < bodyCount; ++i)
	{
		int seedIndex = bodyIndices[i];
		b2Body* seed = bodies + seedIndex;
		B2_ASSERT(seed->object.next == seedIndex);
		B2_ASSERT(seed->isEnabled);
		B2_ASSERT(seed->type != b2_staticBody);

		if (seed->isMarked == true)
		{
			// The body has already been visited
			continue;
		}

		int stackCount = 0;
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

		int islandIndex = island->object.index;

		// Perform a depth first search (DFS) on the constraint graph.
		while (stackCount > 0)
		{
			// Grab the next body off the stack and add it to the island.
			int bodyIndex = stack[--stackCount];
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
			int contactKey = body->contactList;
			while (contactKey != B2_NULL_INDEX)
			{
				int contactIndex = contactKey >> 1;
				int edgeIndex = contactKey & 1;

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

				int otherEdgeIndex = edgeIndex ^ 1;
				int otherBodyIndex = contact->edges[otherEdgeIndex].bodyIndex;
				b2Body* otherBody = world->bodies + otherBodyIndex;

				// Maybe add other body to stack
				if (otherBody->isMarked == false && otherBody->type != b2_staticBody)
				{
					B2_ASSERT(stackCount < bodyCount);
					stack[stackCount++] = otherBodyIndex;
					otherBody->isMarked = true;
				}

				// Add contact to island
				contact->islandId = islandIndex;
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
			int jointKey = body->jointList;
			while (jointKey != B2_NULL_INDEX)
			{
				int jointIndex = jointKey >> 1;
				int edgeIndex = jointKey & 1;

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

				int otherEdgeIndex = edgeIndex ^ 1;
				int otherBodyIndex = joint->edges[otherEdgeIndex].bodyIndex;
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
				joint->islandId = islandIndex;
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

		b2ValidateIsland(world, island, true);
	}

	b2FreeStackItem(alloc, bodyIndices);
	b2FreeStackItem(alloc, stack);

	b2TracyCZoneEnd(split);
}
#endif

#if B2_VALIDATE && 0
void b2ValidateIsland(b2World* world, b2Island* island, bool checkSleep)
{
	b2World* world = island->world;

	int islandIndex = island->object.index;
	B2_ASSERT(island->object.index == island->object.next);

	bool isAwake = false;
	if (island->awakeIndex != B2_NULL_INDEX)
	{
		b2CheckIndex(world->awakeIslandArray, island->awakeIndex);
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

		int count = 0;
		int bodyIndex = island->headBody;
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

		int count = 0;
		int contactIndex = island->headContact;
		while (contactIndex != B2_NULL_INDEX)
		{
			b2Contact* contact = contacts + contactIndex;
			B2_ASSERT(contact->islandId == islandIndex);
			count += 1;

			if (checkSleep)
			{
				if (isAwake)
				{
					B2_ASSERT(contact->colorIndex != B2_NULL_INDEX);
					B2_ASSERT(contact->localIndex != B2_NULL_INDEX);

					// int awakeIndex = world->contactAwakeIndexArray[contactIndex];
					// B2_ASSERT(0 <= awakeIndex && awakeIndex < b2Array(world->awakeContactArray).count);
					// B2_ASSERT(world->awakeContactArray[awakeIndex] == contactIndex);
				}
				else
				{
					B2_ASSERT(contact->colorIndex == B2_NULL_INDEX);
					B2_ASSERT(contact->localIndex == B2_NULL_INDEX);
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

		int count = 0;
		int jointIndex = island->headJoint;
		while (jointIndex != B2_NULL_INDEX)
		{
			b2Joint* joint = joints + jointIndex;
			B2_ASSERT(joint->islandId == islandIndex);
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

void b2ValidateIsland(b2World* world, b2Island* island, bool checkSleep)
{
	B2_MAYBE_UNUSED(world);
	B2_MAYBE_UNUSED(island);
	B2_MAYBE_UNUSED(checkSleep);
}

#endif
