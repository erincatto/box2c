// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "constraint_graph.h"

#include "array.h"
#include "bitset.h"
#include "body.h"
#include "contact.h"
#include "joint.h"
#include "world.h"

#include <stdatomic.h>

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
	int contactCapacity = 16;
	int jointCapacity = 16;

	for (int32_t i = 0; i < b2_graphColorCount; ++i)
	{
		b2GraphColor* color = graph->colors + i;

		if (i != b2_overflowIndex)
		{
			color->bodySet = b2CreateBitSet(bodyCapacity);
			b2SetBitCountAndClear(&color->bodySet, bodyCapacity);
		}

		color->contacts = b2CreateContactArray(allocator, contactCapacity);
		color->joints = b2CreateJointArray(allocator, jointCapacity);
	}
}

void b2DestroyGraph(b2ConstraintGraph* graph)
{
	for (int32_t i = 0; i < b2_graphColorCount; ++i)
	{
		b2GraphColor* color = graph->colors + i;
		b2DestroyBitSet(&color->bodySet);

		// the rest is from b2BlockAllocator
	}
}

// Contacts are always created as non-touching. They get cloned into the constraint
// graph once they are found to be touching.
b2Contact* b2AddContactToGraph(b2World* world, b2Contact* contact)
{
	b2ConstraintGraph* graph = &world->constraintGraph;
	int colorIndex = b2_overflowIndex;

#if B2_FORCE_OVERFLOW == 0
	int bodyIdA = contact->edges[0].bodyId;
	int bodyIdB = contact->edges[1].bodyId;
	b2CheckIndex(world->bodyLookupArray, bodyIdA);
	b2CheckIndex(world->bodyLookupArray, bodyIdB);
	b2BodyLookup lookupA = world->bodyLookupArray[bodyIdA];
	b2BodyLookup lookupB = world->bodyLookupArray[bodyIdB];
	bool staticA = lookupA.setIndex == b2_staticSet;
	bool staticB = lookupB.setIndex == b2_staticSet;
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

	b2Contact* newContact = b2AddContact(&world->blockAllocator, &graph->colors[colorIndex].contacts);
	memcpy(newContact, contact, sizeof(b2Contact));
	newContact->colorIndex = colorIndex;
	newContact->localIndex = graph->colors[colorIndex].contacts.count - 1;
	return newContact;
}

void b2RemoveContactFromGraph(b2World* world, b2Contact* contact)
{
	int bodyKeyA = contact->edges[0].bodyId;
	int bodyKeyB = contact->edges[1].bodyId;

	b2ConstraintGraph* graph = &world->constraintGraph;

	int colorIndex = contact->colorIndex;
	B2_ASSERT(0 <= colorIndex && colorIndex < b2_graphColorCount);
	b2GraphColor* color = graph->colors + colorIndex;

	if (colorIndex != b2_overflowIndex)
	{
		b2ClearBit(&color->bodySet, bodyKeyA);
		b2ClearBit(&color->bodySet, bodyKeyB);
	}

	int colorSubIndex = contact->localIndex;
	int movedIndex = b2RemoveContact(&world->blockAllocator, &color->contacts, colorSubIndex);
	if (movedIndex != B2_NULL_INDEX)
	{
		// Fix index on swapped contact
		b2Contact* movedContact = color->contacts.data + colorSubIndex;
		movedContact->localIndex = colorSubIndex;

		// Fix contact lookup for moved contact
		int movedKey = movedContact->contactId;
		B2_ASSERT(0 <= movedKey && movedKey < b2Array(world->contactLookupArray).count);
		b2ContactLookup* lookup = world->contactLookupArray + movedKey;
		B2_ASSERT(lookup->setIndex == b2_awakeSet);
		B2_ASSERT(lookup->colorIndex == colorIndex);
		B2_ASSERT(lookup->contactIndex == movedIndex);
		lookup->contactIndex = colorSubIndex;
	}

	contact->colorIndex = B2_NULL_INDEX;
	contact->localIndex = B2_NULL_INDEX;
}

b2Joint* b2EmplaceJointInGraph(b2World* world, b2Body* bodyA, b2Body* bodyB)
{
	b2ConstraintGraph* graph = &world->constraintGraph;
	int colorIndex = b2_overflowIndex;

#if B2_FORCE_OVERFLOW == 0
	int32_t bodyKeyA = bodyA->bodyId;
	int32_t bodyKeyB = bodyB->bodyId;
	b2BodyType typeA = bodyA->type;
	b2BodyType typeB = bodyB->type;
	B2_ASSERT(typeA != b2_staticBody || typeB != b2_staticBody);

	if (typeA == b2_dynamicBody && typeB == b2_dynamicBody)
	{
		for (int32_t i = 0; i < b2_graphColorCount; ++i)
		{
			b2GraphColor* color = graph->colors + i;
			if (b2GetBit(&color->bodySet, bodyKeyA) || b2GetBit(&color->bodySet, bodyKeyB))
			{
				continue;
			}

			b2SetBitGrow(&color->bodySet, bodyKeyA);
			b2SetBitGrow(&color->bodySet, bodyKeyB);
			colorIndex = i;
			break;
		}
	}
	else if (typeA == b2_dynamicBody)
	{
		for (int32_t i = 0; i < b2_graphColorCount; ++i)
		{
			b2GraphColor* color = graph->colors + i;
			if (b2GetBit(&color->bodySet, bodyKeyA))
			{
				continue;
			}

			b2SetBitGrow(&color->bodySet, bodyKeyA);
			colorIndex = i;
			break;
		}
	}
	else if (typeB == b2_dynamicBody)
	{
		for (int32_t i = 0; i < b2_graphColorCount; ++i)
		{
			b2GraphColor* color = graph->colors + i;
			if (b2GetBit(&color->bodySet, bodyKeyB))
			{
				continue;
			}

			b2SetBitGrow(&color->bodySet, bodyKeyB);
			colorIndex = i;
			break;
		}
	}
#endif

	b2Joint* joint = b2AddJoint(&world->blockAllocator, &graph->colors[colorIndex].joints);
	memset(joint, 0, sizeof(b2Joint));
	joint->colorIndex = colorIndex;
	joint->colorSubIndex = graph->colors[colorIndex].joints.count - 1;
	return joint;
}

void b2RemoveJointFromGraph(b2World* world, b2Joint* joint)
{
	int bodyKeyA = joint->edges[0].bodyId;
	int bodyKeyB = joint->edges[1].bodyId;

	b2ConstraintGraph* graph = &world->constraintGraph;

	int colorIndex = joint->colorIndex;
	B2_ASSERT(0 <= colorIndex && colorIndex < b2_graphColorCount);
	b2GraphColor* color = graph->colors + colorIndex;

	if (colorIndex != b2_overflowIndex)
	{
		b2ClearBit(&color->bodySet, bodyKeyA);
		b2ClearBit(&color->bodySet, bodyKeyB);
	}

	int colorSubIndex = joint->colorSubIndex;
	int movedIndex = b2RemoveJoint(&world->blockAllocator, &color->joints, colorSubIndex);
	if (movedIndex != B2_NULL_INDEX)
	{
		// Fix index on swapped contact
		b2Joint* movedJoint = color->joints.data + colorSubIndex;
		movedJoint->colorSubIndex = colorSubIndex;

		// Fix contact lookup for moved contact
		int key = movedJoint->jointKey;
		B2_ASSERT(0 <= key && key < b2Array(world->jointLookupArray).count);
		b2JointLookup* lookup = world->jointLookupArray + key;
		B2_ASSERT(lookup->setIndex == b2_awakeSet);
		B2_ASSERT(lookup->colorIndex == colorIndex);
		B2_ASSERT(lookup->jointIndex == movedIndex);
		lookup->jointIndex = colorSubIndex;
	}

	joint->colorIndex = B2_NULL_INDEX;
	joint->colorSubIndex = B2_NULL_INDEX;
}
