// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "island.h"

#include "array.h"
#include "body.h"
#include "contact.h"
#include "contact_solver.h"
#include "core.h"
#include "joint.h"
#include "shape.h"
#include "solver_data.h"
#include "stack_allocator.h"
#include "world.h"

#include "box2d/aabb.h"
#include "box2d/callbacks.h"
#include "box2d/timer.h"

#include <float.h>
#include <stdatomic.h>
#include <stdlib.h>

/*
Position Correction Notes
=========================
I tried the several algorithms for position correction of the 2D revolute joint.
I looked at these systems:
- simple pendulum (1m diameter sphere on massless 5m stick) with initial angular velocity of 100 rad/s.
- suspension bridge with 30 1m long planks of length 1m.
- multi-link chain with 30 1m long links.

Here are the algorithms:

Baumgarte - A fraction of the position error is added to the velocity error. There is no
separate position solver.

Pseudo Velocities - After the velocity solver and position integration,
the position error, Jacobian, and effective mass are recomputed. Then
the velocity constraints are solved with pseudo velocities and a fraction
of the position error is added to the pseudo velocity error. The pseudo
velocities are initialized to zero and there is no warm-starting. After
the position solver, the pseudo velocities are added to the positions.
This is also called the First Order World method or the Position LCP method.

Modified Nonlinear Gauss-Seidel (NGS) - Like Pseudo Velocities except the
position error is re-computed for each constraint and the positions are updated
after the constraint is solved. The radius vectors (aka Jacobians) are
re-computed too (otherwise the algorithm has horrible instability). The pseudo
velocity states are not needed because they are effectively zero at the beginning
of each iteration. Since we have the current position error, we allow the
iterations to terminate early if the error becomes smaller than b2_linearSlop.

Full NGS or just NGS - Like Modified NGS except the effective mass are re-computed
each time a constraint is solved.

Here are the results:
Baumgarte - this is the cheapest algorithm but it has some stability problems,
especially with the bridge. The chain links separate easily close to the root
and they jitter as they struggle to pull together. This is one of the most common
methods in the field. The big drawback is that the position correction artificially
affects the momentum, thus leading to instabilities and false bounce. I used a
bias factor of 0.2. A larger bias factor makes the bridge less stable, a smaller
factor makes joints and contacts more spongy.

Pseudo Velocities - the is more stable than the Baumgarte method. The bridge is
stable. However, joints still separate with large angular velocities. Drag the
simple pendulum in a circle quickly and the joint will separate. The chain separates
easily and does not recover. I used a bias factor of 0.2. A larger value lead to
the bridge collapsing when a heavy cube drops on it.

Modified NGS - this algorithm is better in some ways than Baumgarte and Pseudo
Velocities, but in other ways it is worse. The bridge and chain are much more
stable, but the simple pendulum goes unstable at high angular velocities.

Full NGS - stable in all tests. The joints display good stiffness. The bridge
still sags, but this is better than infinite forces.

Recommendations
Pseudo Velocities are not really worthwhile because the bridge and chain cannot
recover from joint separation. In other cases the benefit over Baumgarte is small.

Modified NGS is not a robust method for the revolute joint due to the violent
instability seen in the simple pendulum. Perhaps it is viable with other constraint
types, especially scalar constraints where the effective mass is a scalar.

This leaves Baumgarte and Full NGS. Baumgarte has small, but manageable instabilities
and is very fast. I don't think we can escape Baumgarte, especially in highly
demanding cases where high constraint fidelity is not needed.

Full NGS is robust and easy on the eyes. I recommend this as an option for
higher fidelity simulation and certainly for suspension bridges and long chains.
Full NGS might be a good choice for ragdolls, especially motorized ragdolls where
joint separation can be problematic. The number of NGS iterations can be reduced
for better performance without harming robustness much.

Each joint in a can be handled differently in the position solver. So I recommend
a system where the user can select the algorithm on a per joint basis. I would
probably default to the slower Full NGS and let the user select the faster
Baumgarte method in performance critical scenarios.
*/

/*
2D Rotation

R = [cos(theta) -sin(theta)]
	[sin(theta) cos(theta) ]

thetaDot = omega

Let q1 = cos(theta), q2 = sin(theta).
R = [q1 -q2]
	[q2  q1]

q1Dot = -thetaDot * q2
q2Dot = thetaDot * q1

q1_new = q1_old - dt * w * q2
q2_new = q2_old + dt * w * q1
then normalize.

This might be faster than computing sin+cos.
However, we can compute sin+cos of the same angle fast.
*/

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
	island->awakeIndex = B2_NULL_INDEX;
	island->constraintRemoveCount = 0;
	island->maySplit = false;
	island->stepContext = NULL;
	island->contactSolver = NULL;
}

void b2DestroyIsland(b2Island* island)
{
	B2_MAYBE_UNUSED(island);
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

	b2ValidateIsland(island);
}

void b2WakeIsland(b2Island* island)
{
	b2World* world = island->world;

	if (island->awakeIndex != B2_NULL_INDEX)
	{
		B2_ASSERT(world->awakeIslandArray[island->awakeIndex] == island->object.index);
		return;
	}

	island->awakeIndex = b2Array(world->awakeIslandArray).count;
	b2Array_Push(world->awakeIslandArray, island->object.index);
}

#if B2_GRAPH_COLOR == 1

void b2LinkContact(b2World* world, b2Contact* contact)
{
	B2_MAYBE_UNUSED(world);
	B2_MAYBE_UNUSED(contact);
}

void b2UnlinkContact(b2World* world, b2Contact* contact)
{
	B2_MAYBE_UNUSED(world);
	B2_MAYBE_UNUSED(contact);

}

#else

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
}

#endif

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

	b2ValidateIsland(island);
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
	b2ValidateIsland(rootIsland);

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

		int32_t count = b2Array(world->awakeIslandArray).count;
		int32_t awakeIndex = island->awakeIndex;
		b2Array_RemoveSwap(world->awakeIslandArray, awakeIndex);
		if (awakeIndex < count - 1)
		{
			// Fix awake index on swapped island
			int32_t swappedIslandIndex = world->awakeIslandArray[awakeIndex];
			world->islands[swappedIslandIndex].awakeIndex = awakeIndex;
		}

		b2DestroyIsland(island);
		b2FreeObject(&world->islandPool, &island->object);
	}

	// Step 3: ensure island pool has sufficient space to split the largest island
	b2GrowPool(&world->islandPool, world->islandPool.count + maxBodyCount);
	world->islands = (b2Island*)world->islandPool.memory;
}

static int b2CompareIslands(const void* A, const void* B)
{
	const b2Island* islandA = *(const b2Island**)A;
	const b2Island* islandB = *(const b2Island**)B;
	return islandB->bodyCount - islandA->bodyCount;
}

#define B2_CONTACT_REMOVE_THRESHOLD 1

// Sort islands so that the largest islands are solved first to avoid
// long tails in the island parallel-for loop.
void b2SortIslands(b2World* world, b2Island** islands, int32_t count)
{
	// Sort descending order (largest island first)
	qsort(islands, count, sizeof(b2Island*), b2CompareIslands);

	// Look for an island to split. Large islands have priority.
	world->splitIslandIndex = B2_NULL_INDEX;
	for (int32_t i = 0; i < count; ++i)
	{
		if (islands[i]->constraintRemoveCount >= B2_CONTACT_REMOVE_THRESHOLD)
		{
			// This and only this island may split this time step
			islands[i]->maySplit = true;
			world->splitIslandIndex = islands[i]->object.index;
			break;
		}
	}
}

void b2PrepareIsland(b2Island* island, b2StepContext* stepContext)
{
	island->stepContext = stepContext;

	b2ContactSolverDef contactSolverDef;
	contactSolverDef.context = island->stepContext;
	contactSolverDef.world = island->world;
	contactSolverDef.contactList = island->headContact;
	contactSolverDef.contactCount = island->contactCount;
	island->contactSolver = b2CreateContactSolver(&contactSolverDef);
}

#if 0
if (island->bodyCount > 16)
{
	int32_t k = 4;
	b2Vec2 clusterCenters[4] = { 0 };
	int32_t clusterCounts[4] = { 0 };
	int32_t m = island->bodyCount / k;

	// seed cluster positions
	for (int32_t i = 0; i < k; ++i)
	{
		int32_t j = (i * m) % island->bodyCount;
		clusterCenters[i] = island->bodies[j]->position;
	}

	for (int32_t i = 0; i < island->bodyCount; ++i)
	{
		b2Body* b = island->bodies[i];
		b2Vec2 p = b->position;
		float bestDist = b2DistanceSquared(clusterCenters[0], p);
		b->cluster = 0;

		for (int32_t j = 1; j < k; ++j)
		{
			float dist = b2DistanceSquared(clusterCenters[j], p);
			if (dist < bestDist)
			{
				bestDist = dist;
				b->cluster = j;
			}
		}
	}

	int32_t maxIter = 4;
	for (int32_t iter = 0; iter < maxIter; ++iter)
	{
		// reset clusters
		for (int32_t i = 0; i < k; ++i)
		{
			clusterCenters[i] = b2Vec2_zero;
			clusterCounts[i] = 0;
		}

		// computer new clusters
		for (int32_t i = 0; i < island->bodyCount; ++i)
		{
			b2Body* b = island->bodies[i];
			int32_t j = b->cluster;
			clusterCenters[j] = b2Add(clusterCenters[j], b->position);
			clusterCounts[j] += 1;
		}
	}
}
#endif

// Split an island because some contacts and/or joints have been removed
// Note: contacts/joints connecting to static bodies must belong to an island but don't affect island connectivity
// Note: static bodies are never in an island
// TODO_ERIN I think this can be done during collision
static void b2SplitIsland(b2Island* baseIsland)
{
	b2TracyCZoneNC(split, "Split Island", b2_colorHoneydew2, true);

	b2ValidateIsland(baseIsland);

	b2World* world = baseIsland->world;
	int32_t bodyCount = baseIsland->bodyCount;

	b2Body* bodies = world->bodies;
	b2Contact* contacts = world->contacts;
	b2Joint* joints = world->joints;

	b2StackAllocator* alloc = world->stackAllocator;

	// No lock is needed because only one island can split per time step.
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
		contact->flags &= ~b2_contactIslandFlag;
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
				if (contact->flags & b2_contactIslandFlag)
				{
					continue;
				}

				// Skip sensors
				if (contact->flags & b2_contactSensorFlag)
				{
					continue;
				}

				// Is this contact enabled and touching?
				if ((contact->flags & b2_contactEnabledFlag) == 0 || (contact->flags & b2_contactTouchingFlag) == 0)
				{
					continue;
				}

				contact->flags |= b2_contactIslandFlag;

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

		b2ValidateIsland(island);
		b2Array_Push(world->splitIslandArray, island->object.index);
	}

	b2FreeStackItem(alloc, bodyIndices);
	b2FreeStackItem(alloc, stack);

	b2TracyCZoneEnd(split);
}

// This must be thread safe
void b2SolveIsland(b2Island* island, uint32_t threadIndex)
{
	b2World* world = island->world;
	b2Body* bodies = world->bodies;
	b2StepContext* context = island->stepContext;
	b2Joint* joints = world->joints;

	b2Vec2 gravity = world->gravity;

	float h = context->dt;

	// Integrate velocities and apply damping. Initialize the body state.
	int32_t bodyIndex = island->headBody;
	while (bodyIndex != B2_NULL_INDEX)
	{
		b2Body* b = bodies + bodyIndex;

		float invMass = b->invMass;
		float invI = b->invI;

		if (b->type == b2_dynamicBody)
		{
			b2Vec2 v = b->linearVelocity;
			float w = b->angularVelocity;

			// Integrate velocities
			v = b2Add(v, b2MulSV(h * invMass, b2MulAdd(b->force, b->gravityScale * b->mass, gravity)));
			w = w + h * invI * b->torque;

			// Apply damping.
			// ODE: dv/dt + c * v = 0
			// Solution: v(t) = v0 * exp(-c * t)
			// Time step: v(t + dt) = v0 * exp(-c * (t + dt)) = v0 * exp(-c * t) * exp(-c * dt) = v * exp(-c * dt)
			// v2 = exp(-c * dt) * v1
			// Pade approximation:
			// v2 = v1 * 1 / (1 + c * dt)
			v = b2MulSV(1.0f / (1.0f + h * b->linearDamping), v);
			w *= 1.0f / (1.0f + h * b->angularDamping);

			b->linearVelocity = v;
			b->angularVelocity = w;
		}

		bodyIndex = b->islandNext;
	}

	// Solver data
	b2ContactSolver_Initialize(island->contactSolver);

	int32_t jointIndex = island->headJoint;
	while (jointIndex != B2_NULL_INDEX)
	{
		b2Joint* joint = joints + jointIndex;
		b2PrepareJoint(joint, context);
		jointIndex = joint->islandNext;
	}

	b2TracyCZoneNC(velc, "Velocity Constraints", b2_colorCadetBlue, true);
	// Solve velocity constraints
	for (int32_t i = 0; i < context->velocityIterations; ++i)
	{
		jointIndex = island->headJoint;
		while (jointIndex != B2_NULL_INDEX)
		{
			b2Joint* joint = joints + jointIndex;
			b2SolveJointVelocity(joint, context);
			jointIndex = joint->islandNext;
		}

		b2ContactSolver_SolveVelocityConstraints(island->contactSolver);
	}
	b2TracyCZoneEnd(velc);

	// Special handling for restitution
	b2ContactSolver_ApplyRestitution(island->contactSolver);

	// Store impulses for warm starting
	b2ContactSolver_StoreImpulses(island->contactSolver);

	// Integrate positions
	bool enableContinuous = world->enableContinuous;

	bodyIndex = island->headBody;
	while (bodyIndex != B2_NULL_INDEX)
	{
		b2Body* b = bodies + bodyIndex;

		b2Vec2 c = b->position;
		float a = b->angle;
		b2Vec2 v = b->linearVelocity;
		float w = b->angularVelocity;

		// Clamp large velocities
		b2Vec2 translation = b2MulSV(h, v);
		if (b2Dot(translation, translation) > b2_maxTranslationSquared)
		{
			float ratio = b2_maxTranslation / b2Length(translation);
			v = b2MulSV(ratio, v);
		}

		float rotation = h * w;
		if (rotation * rotation > b2_maxRotationSquared)
		{
			float ratio = b2_maxRotation / B2_ABS(rotation);
			w *= ratio;
		}

		// Integrate
		c = b2MulAdd(c, h, v);
		a += h * w;

		b->position = c;
		b->angle = a;
		b->linearVelocity = v;
		b->angularVelocity = w;

		const float saftetyFactor = 0.5f;
		if (enableContinuous && (b2Length(v) + B2_ABS(w) * b->maxExtent) * h > saftetyFactor * b->minExtent)
		{
			// Store in fast array for the continuous collision stage
			int fastIndex = atomic_fetch_add(&world->fastBodyCount, 1);
			world->fastBodies[fastIndex] = bodyIndex;
			b->isFast = true;
		}
		else
		{
			// Body is safe to advance
			b->isFast = false;
			b->position0 = b->position;
			b->angle0 = b->angle;
		}

		bodyIndex = b->islandNext;
	}

	b2TracyCZoneNC(posc, "Position Constraints", b2_colorBurlywood, true);

	// Solve position constraints
	bool positionSolved = false;
	for (int32_t i = 0; i < context->positionIterations; ++i)
	{
		bool contactsOkay = b2ContactSolver_SolvePositionConstraintsBlock(island->contactSolver);

		bool jointsOkay = true;
		jointIndex = island->headJoint;
		while (jointIndex != B2_NULL_INDEX)
		{
			b2Joint* joint = joints + jointIndex;

			bool jointOkay = b2SolveJointPosition(joint, context);
			jointsOkay = jointsOkay && jointOkay;

			jointIndex = joint->islandNext;
		}

		if (contactsOkay && jointsOkay)
		{
			// Exit early if the position errors are small.
			positionSolved = true;
			break;
		}
	}

	b2TracyCZoneEnd(posc);

	b2TracyCZoneNC(sleep, "Sleep", b2_colorSalmon2, true);

	// Update transform
	bodyIndex = island->headBody;
	while (bodyIndex != B2_NULL_INDEX)
	{
		b2Body* body = bodies + bodyIndex;
		body->transform.q = b2MakeRot(body->angle);
		body->transform.p = b2Sub(body->position, b2RotateVector(body->transform.q, body->localCenter));
		bodyIndex = body->islandNext;
	}

	// Update sleep
	bool isIslandAwake = true;

	// Don't allow an island that will be split to fall asleep just yet
	if (world->enableSleep && island->maySplit == false)
	{
		float minSleepTime = FLT_MAX;

		const float linTolSqr = b2_linearSleepTolerance * b2_linearSleepTolerance;
		const float angTolSqr = b2_angularSleepTolerance * b2_angularSleepTolerance;

		bodyIndex = island->headBody;
		while (bodyIndex != B2_NULL_INDEX)
		{
			b2Body* b = bodies + bodyIndex;

			if (b->enableSleep == false || b->angularVelocity * b->angularVelocity > angTolSqr ||
				b2Dot(b->linearVelocity, b->linearVelocity) > linTolSqr)
			{
				b->sleepTime = 0.0f;
				minSleepTime = 0.0f;
			}
			else
			{
				b->sleepTime += h;
				minSleepTime = B2_MIN(minSleepTime, b->sleepTime);
			}

			bodyIndex = b->islandNext;
		}

		if (minSleepTime >= b2_timeToSleep && positionSolved)
		{
			isIslandAwake = false;

			bodyIndex = island->headBody;
			while (bodyIndex != B2_NULL_INDEX)
			{
				b2Body* b = bodies + bodyIndex;
				B2_ASSERT(b->isFast == false);

				b->sleepTime = 0.0f;
				b->linearVelocity = b2Vec2_zero;
				b->angularVelocity = 0.0f;
				b->force = b2Vec2_zero;
				b->torque = 0.0f;

				bodyIndex = b->islandNext;
			}
		}
	}

	if (isIslandAwake == false)
	{
		// This signals that this island should not be added to awake island array
		island->awakeIndex = B2_NULL_INDEX;
	}
	else
	{
		b2Contact* contacts = world->contacts;
		const b2Vec2 aabbMargin = {b2_aabbMargin, b2_aabbMargin};
		b2BitSet* awakeContactBitSet = &world->taskContextArray[threadIndex].awakeContactBitSet;
		b2BitSet* shapeBitSet = &world->taskContextArray[threadIndex].shapeBitSet;

		bodyIndex = island->headBody;
		while (bodyIndex != B2_NULL_INDEX)
		{
			b2Body* body = bodies + bodyIndex;

			body->force = b2Vec2_zero;
			body->torque = 0.0f;

			bool isFast = body->isFast;

			// Update shapes AABBs
			int32_t shapeIndex = body->shapeList;
			while (shapeIndex != B2_NULL_INDEX)
			{
				b2Shape* shape = world->shapes + shapeIndex;

				B2_ASSERT(shape->isFast == false);

				if (isFast)
				{
					// The AABB is updated after continuous collision.
					// Add to moved shapes regardless of AABB changes.
					shape->isFast = true;

					// Bit-set to keep the move array sorted
					b2SetBit(shapeBitSet, shapeIndex);
				}
				else
				{
					shape->aabb = b2Shape_ComputeAABB(shape, body->transform);

					if (b2AABB_Contains(shape->fatAABB, shape->aabb) == false)
					{
						shape->fatAABB.lowerBound = b2Sub(shape->aabb.lowerBound, aabbMargin);
						shape->fatAABB.upperBound = b2Add(shape->aabb.upperBound, aabbMargin);

						// Bit-set to keep the move array sorted
						b2SetBit(shapeBitSet, shapeIndex);
					}
				}

				shapeIndex = shape->nextShapeIndex;
			}

			// Prepare awake contacts. May include contacts that are not touching
			// so they may not be island contacts.
			int32_t contactKey = body->contactList;
			while (contactKey != B2_NULL_INDEX)
			{
				int32_t contactIndex = contactKey >> 1;
				int32_t edgeIndex = contactKey & 1;
				b2Contact* contact = contacts + contactIndex;

				// Bit set to prevent duplicates
				b2SetBit(awakeContactBitSet, contactIndex);
				contactKey = contact->edges[edgeIndex].nextKey;
			}

			bodyIndex = body->islandNext;
		}
	}

	if (island->maySplit)
	{
		b2SplitIsland(island);
	}

	b2TracyCZoneEnd(sleep);
}

// Single threaded work
void b2CompleteIsland(b2Island* island)
{
	b2World* world = island->world;

#if 0
	// Report impulses
	b2PostSolveFcn* postSolveFcn = world->postSolveFcn;
	if (postSolveFcn != NULL)
	{
		b2Contact* contacts = world->contacts;
		int16_t worldIndex = world->index;
		const b2Shape* shapes = world->shapes;

		int32_t contactIndex = island->headContact;
		while (contactIndex != B2_NULL_INDEX)
		{
			const b2Contact* contact = contacts + contactIndex;

			const b2Shape* shapeA = shapes + contact->shapeIndexA;
			const b2Shape* shapeB = shapes + contact->shapeIndexB;

			b2ShapeId idA = {shapeA->object.index, worldIndex, shapeA->object.revision};
			b2ShapeId idB = {shapeB->object.index, worldIndex, shapeB->object.revision};
			postSolveFcn(idA, idB, &contact->manifold, world->postSolveContext);
		}
	}
#endif

	// Destroy in reverse order
	b2DestroyContactSolver(island->contactSolver, world->stackAllocator);
	island->contactSolver = NULL;

	// Wake island
	if (island->awakeIndex != B2_NULL_INDEX)
	{
		island->awakeIndex = B2_NULL_INDEX;
		b2WakeIsland(island);
	}
}

// This island was just split. Handle any remaining single threaded cleanup.
void b2CompleteBaseSplitIsland(b2Island* island)
{
	b2DestroyContactSolver(island->contactSolver, island->world->stackAllocator);
	island->contactSolver = NULL;
}

// This island was just created through splitting. Handle single thread work.
void b2CompleteSplitIsland(b2Island* island)
{
// Report impulses
#if 0
	b2World* world = island->world;
	b2PostSolveFcn* postSolveFcn = island->world->postSolveFcn;
	if (postSolveFcn != NULL)
	{
		b2Contact* contacts = world->contacts;
		int16_t worldIndex = world->index;
		const b2Shape* shapes = world->shapes;

		int32_t contactIndex = island->headContact;
		while (contactIndex != B2_NULL_INDEX)
		{
			const b2Contact* contact = contacts + contactIndex;

			const b2Shape* shapeA = shapes + contact->shapeIndexA;
			const b2Shape* shapeB = shapes + contact->shapeIndexB;

			b2ShapeId idA = {shapeA->object.index, worldIndex, shapeA->object.revision};
			b2ShapeId idB = {shapeB->object.index, worldIndex, shapeB->object.revision};
			postSolveFcn(idA, idB, &contact->manifold, world->postSolveContext);
		}
	}
#endif

	// Split islands are kept awake as part of the splitting process. They can
	// fall asleep the next time step.
	island->awakeIndex = B2_NULL_INDEX;
	b2WakeIsland(island);
}

#if B2_VALIDATE

void b2ValidateIsland(b2Island* island)
{
	b2World* world = island->world;

	int32_t islandIndex = island->object.index;
	B2_ASSERT(island->object.index == island->object.next);

	if (island->awakeIndex != B2_NULL_INDEX)
	{
		b2Array_Check(world->awakeIslandArray, island->awakeIndex);
		B2_ASSERT(world->awakeIslandArray[island->awakeIndex] == islandIndex);
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

void b2ValidateIsland(b2Island* island)
{
	B2_MAYBE_UNUSED(island);
}

#endif
