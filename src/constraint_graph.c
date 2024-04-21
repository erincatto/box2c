// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "constraint_graph.h"

#include "array.h"
#include "bitset.h"
#include "body.h"
#include "contact.h"
#include "joint.h"
#include "world.h"

#include <string.h>

// Solver using graph coloring. Islands are only used for sleep.
// High-Performance Physical Simulations on Next-Generation Architecture with Many Cores
// http://web.eecs.umich.edu/~msmelyan/papers/physsim_onmanycore_itj.pdf

// Kinematic bodies have to be treated like dynamic bodies in graph coloring. Unlike static bodies, we cannot use a dummy solver
// body for kinematic bodies. We cannot access a kinematic body from multiple threads efficiently because the SIMD solver body
// scatter would write to the same kinematic body from multiple threads. Even if these writes don't modify the body, they will
// cause horrible cache stalls. To make this feasible I would need a way to block these writes.

// This is used for debugging by making all constraints be assigned to overflow.
#define B2_FORCE_OVERFLOW 0

void b2CreateGraph(b2ConstraintGraph* graph, b2BlockAllocator* allocator, int bodyCapacity)
{
	_Static_assert(b2_graphColorCount >= 2, "must have at least two constraint graph colors");

	*graph = (b2ConstraintGraph){0};

	bodyCapacity = B2_MAX(bodyCapacity, 8);
	for (int i = 0; i < b2_graphColorCount; ++i)
	{
		b2GraphColor* color = graph->colors + i;

		if (i != b2_overflowIndex)
		{
			color->bodySet = b2CreateBitSet(bodyCapacity);
			b2SetBitCountAndClear(&color->bodySet, bodyCapacity);
		}
	}
}

void b2DestroyGraph(b2ConstraintGraph* graph)
{
	for (int i = 0; i < b2_graphColorCount; ++i)
	{
		b2GraphColor* color = graph->colors + i;
		b2DestroyBitSet(&color->bodySet);

		// the rest is from b2BlockAllocator
	}
}

// Contacts are always created as non-touching. They get cloned into the constraint
// graph once they are found to be touching.
// todo maybe kinematic bodies should not go into graph
void b2AddContactToGraph(b2World* world, b2Contact* contact, b2ContactLookup* contactLookup)
{
	B2_ASSERT(contact->manifold.pointCount > 0);
	B2_ASSERT(contact->simFlags & b2_simTouchingFlag);
	B2_ASSERT(contactLookup->flags & b2_contactTouchingFlag);

	b2ConstraintGraph* graph = &world->constraintGraph;
	int colorIndex = b2_overflowIndex;

#if B2_FORCE_OVERFLOW == 0
	int bodyIdA = contact->bodyIdA;
	int bodyIdB = contact->bodyIdB;
	b2CheckIndex(world->bodyArray, bodyIdA);
	b2CheckIndex(world->bodyArray, bodyIdB);

	b2Body* bodyA = world->bodyArray + bodyIdA;
	b2Body* bodyB = world->bodyArray + bodyIdB;
	bool staticA = bodyA->setIndex == b2_staticSet;
	bool staticB = bodyB->setIndex == b2_staticSet;
	B2_ASSERT(staticA == false || staticB == false);

	if (staticA == false && staticB == false)
	{
		for (int i = 0; i < b2_overflowIndex; ++i)
		{
			b2GraphColor* color = graph->colors + i;
			if (b2GetBit(&color->bodySet, bodyIdA) || b2GetBit(&color->bodySet, bodyIdB))
			{
				continue;
			}

			b2SetBitGrow(&color->bodySet, bodyIdA);
			b2SetBitGrow(&color->bodySet, bodyIdB);
			colorIndex = i;
			break;
		}
	}
	else if (staticA == false)
	{
		// No static contacts in color 0
		for (int i = 1; i < b2_overflowIndex; ++i)
		{
			b2GraphColor* color = graph->colors + i;
			if (b2GetBit(&color->bodySet, bodyIdA))
			{
				continue;
			}

			b2SetBitGrow(&color->bodySet, bodyIdA);
			colorIndex = i;
			break;
		}
	}
	else if (staticB == false)
	{
		// No static contacts in color 0
		for (int i = 1; i < b2_overflowIndex; ++i)
		{
			b2GraphColor* color = graph->colors + i;
			if (b2GetBit(&color->bodySet, bodyIdB))
			{
				continue;
			}

			b2SetBitGrow(&color->bodySet, bodyIdB);
			colorIndex = i;
			break;
		}
	}
#endif

	b2GraphColor* color = graph->colors + colorIndex;
	contactLookup->colorIndex = colorIndex;
	contactLookup->localIndex = color->contacts.count;

	b2Contact* newContact = b2AddContact(&world->blockAllocator, &color->contacts);
	memcpy(newContact, contact, sizeof(b2Contact));
}

void b2RemoveContactFromGraph(b2World* world, int bodyIdA, int bodyIdB, int colorIndex, int localIndex)
{
	b2ConstraintGraph* graph = &world->constraintGraph;

	B2_ASSERT(0 <= colorIndex && colorIndex < b2_graphColorCount);
	b2GraphColor* color = graph->colors + colorIndex;

	if (colorIndex != b2_overflowIndex)
	{
		// might clear a bit for a static body, but this has no effect
		b2ClearBit(&color->bodySet, bodyIdA);
		b2ClearBit(&color->bodySet, bodyIdB);
	}

	int movedIndex = b2RemoveContact(&color->contacts, localIndex);
	if (movedIndex != B2_NULL_INDEX)
	{
		// Fix index on swapped contact
		b2Contact* movedContact = color->contacts.data + localIndex;

		// Fix contact lookup for moved contact
		int movedId = movedContact->contactId;
		b2CheckIndex(world->contactLookupArray, movedId);
		b2ContactLookup* movedLookup = world->contactLookupArray + movedId;
		B2_ASSERT(movedLookup->setIndex == b2_awakeSet);
		B2_ASSERT(movedLookup->colorIndex == colorIndex);
		B2_ASSERT(movedLookup->localIndex == movedIndex);
		movedLookup->localIndex = localIndex;
	}
}

static int b2AssignJointColor(b2ConstraintGraph* graph, int bodyIdA, int bodyIdB, bool staticA, bool staticB)
{
	B2_ASSERT(staticA == false || staticB == false);

#if B2_FORCE_OVERFLOW == 0
	if (staticA == false && staticB == false)
	{
		for (int i = 0; i < b2_graphColorCount; ++i)
		{
			b2GraphColor* color = graph->colors + i;
			if (b2GetBit(&color->bodySet, bodyIdA) || b2GetBit(&color->bodySet, bodyIdB))
			{
				continue;
			}

			b2SetBitGrow(&color->bodySet, bodyIdA);
			b2SetBitGrow(&color->bodySet, bodyIdB);
			return i;
		}
	}
	else if (staticA == false)
	{
		for (int i = 0; i < b2_graphColorCount; ++i)
		{
			b2GraphColor* color = graph->colors + i;
			if (b2GetBit(&color->bodySet, bodyIdA))
			{
				continue;
			}

			b2SetBitGrow(&color->bodySet, bodyIdA);
			return i;
		}
	}
	else if (staticB == false)
	{
		for (int i = 0; i < b2_graphColorCount; ++i)
		{
			b2GraphColor* color = graph->colors + i;
			if (b2GetBit(&color->bodySet, bodyIdB))
			{
				continue;
			}

			b2SetBitGrow(&color->bodySet, bodyIdB);
			return i;
		}
	}
#endif

	return b2_overflowIndex;
}

b2Joint* b2CreateJointInGraph(b2World* world, b2JointLookup* joint)
{
	b2ConstraintGraph* graph = &world->constraintGraph;

	int bodyIdA = joint->edges[0].bodyId;
	int bodyIdB = joint->edges[1].bodyId;
	b2CheckIndex(world->bodyArray, bodyIdA);
	b2CheckIndex(world->bodyArray, bodyIdB);

	b2Body* bodyA = world->bodyArray + bodyIdA;
	b2Body* bodyB = world->bodyArray + bodyIdB;
	bool staticA = bodyA->setIndex == b2_staticSet;
	bool staticB = bodyB->setIndex == b2_staticSet;

	int colorIndex = b2AssignJointColor(graph, bodyIdA, bodyIdB, staticA, staticB);

	b2Joint* jointSim = b2AddJoint(&world->blockAllocator, &graph->colors[colorIndex].joints);
	joint->colorIndex = colorIndex;
	joint->localIndex = graph->colors[colorIndex].joints.count - 1;
	return jointSim;
}

void b2AddJointToGraph(b2World* world, b2Joint* joint, b2JointLookup* jointLookup)
{
	b2Joint* jointDst = b2CreateJointInGraph(world, jointLookup);
	memcpy(jointDst, joint, sizeof(b2Joint));
}

void b2RemoveJointFromGraph(b2World* world, int bodyIdA, int bodyIdB, int colorIndex, int localIndex)
{
	b2ConstraintGraph* graph = &world->constraintGraph;

	B2_ASSERT(0 <= colorIndex && colorIndex < b2_graphColorCount);
	b2GraphColor* color = graph->colors + colorIndex;

	if (colorIndex != b2_overflowIndex)
	{
		// may clear static bodies, no effect
		b2ClearBit(&color->bodySet, bodyIdA);
		b2ClearBit(&color->bodySet, bodyIdB);
	}

	int movedIndex = b2RemoveJoint(&color->joints, localIndex);
	if (movedIndex != B2_NULL_INDEX)
	{
		// fix lookup on swapped element
		b2Joint* movedJoint = color->joints.data + localIndex;
		int movedId = movedJoint->jointId;
		b2CheckIndex(world->jointLookupArray, movedId);
		b2JointLookup* movedLookup = world->jointLookupArray + movedId;
		B2_ASSERT(movedLookup->setIndex == b2_awakeSet);
		B2_ASSERT(movedLookup->colorIndex == colorIndex);
		B2_ASSERT(movedLookup->localIndex == movedIndex);
		movedLookup->localIndex = localIndex;
	}
}
