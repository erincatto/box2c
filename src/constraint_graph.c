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

		// the rest is block allocation
	}
}

void b2AddContactToGraph(b2World* world, b2Contact* contact)
{
	B2_ASSERT(contact->colorIndex == B2_NULL_INDEX);
	B2_ASSERT(contact->colorSubIndex == B2_NULL_INDEX);

	b2ConstraintGraph* graph = &world->constraintGraph;
	int colorIndex = b2_overflowIndex;

#if B2_FORCE_OVERFLOW == 0
	int bodyKeyA = contact->edges[0].bodyKey;
	int bodyKeyB = contact->edges[1].bodyKey;
	b2Body* bodyA = b2GetBodyFromKey(world, bodyKeyA);
	b2Body* bodyB = b2GetBodyFromKey(world, bodyKeyB);
	b2BodyType typeA = bodyA->type;
	b2BodyType typeB = bodyB->type;
	B2_ASSERT(typeA != b2_staticBody || typeB != b2_staticBody);

	if (typeA != b2_staticBody && typeB != b2_staticBody)
	{
		for (int32_t i = 0; i < b2_overflowIndex; ++i)
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
	else if (typeA != b2_staticBody)
	{
		// Static contacts never in color 0
		for (int32_t i = 1; i < b2_overflowIndex; ++i)
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
	else if (typeB != b2_staticBody)
	{
		// Static contacts never in color 0
		for (int32_t i = 1; i < b2_overflowIndex; ++i)
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

	b2Contact* destinationContact = b2EmplaceContact(&world->blockAllocator, &graph->colors[colorIndex].contacts);
	memcpy(destinationContact, contact, sizeof(b2Contact));
	contact->colorIndex = colorIndex;
	contact->colorSubIndex = graph->colors[colorIndex].contacts.count - 1;

	// the caller should handle the contact lookup because b2RemoveContactFromGraph cannot adjust the lookup
	// because it doesn't know the destination of the contact
}

void b2RemoveContactFromGraph(b2World* world, b2Contact* contact)
{
	int bodyKeyA = contact->edges[0].bodyKey;
	int bodyKeyB = contact->edges[1].bodyKey;

	b2ConstraintGraph* graph = &world->constraintGraph;

	int colorIndex = contact->colorIndex;
	B2_ASSERT(0 <= colorIndex && colorIndex < b2_graphColorCount);
	b2GraphColor* color = graph->colors + colorIndex;

	if (colorIndex != b2_overflowIndex)
	{
		b2ClearBit(&color->bodySet, bodyKeyA);
		b2ClearBit(&color->bodySet, bodyKeyB);
	}

	int colorSubIndex = contact->colorSubIndex;
	int movedIndex = b2RemoveContact(&world->blockAllocator, &color->contacts, colorSubIndex);
	if (movedIndex != B2_NULL_INDEX)
	{
		// Fix index on swapped contact
		b2Contact* movedContact = color->contacts.data + colorSubIndex;
		movedContact->colorSubIndex = colorSubIndex;

		// Fix contact lookup for moved contact
		int key = movedContact->contactKey;
		B2_ASSERT(0 <= key && key < b2Array(world->contactLookupArray).count);
		b2ContactLookup* lookup = world->contactLookupArray + key;
		B2_ASSERT(lookup->setIndex == b2_awakeBodySet);
		B2_ASSERT(lookup->graphColorIndex == colorIndex);
		B2_ASSERT(lookup->contactIndex == movedIndex);
		lookup->contactIndex = colorSubIndex;
	}

	contact->colorIndex = B2_NULL_INDEX;
	contact->colorSubIndex = B2_NULL_INDEX;
}

void b2AddJointToGraph(b2World* world, b2Joint* joint)
{
	B2_ASSERT(joint->colorIndex == B2_NULL_INDEX);
	B2_ASSERT(joint->colorSubIndex == B2_NULL_INDEX);

	b2ConstraintGraph* graph = &world->constraintGraph;
	int colorIndex = b2_overflowIndex;

#if B2_FORCE_OVERFLOW == 0
	int32_t bodyKeyA = joint->edges[0].bodyKey;
	int32_t bodyKeyB = joint->edges[1].bodyKey;
	b2Body* bodyA = b2GetBodyFromKey(world, bodyKeyA);
	b2Body* bodyB = b2GetBodyFromKey(world, bodyKeyB);
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

	b2Joint* destinationJoint = b2EmplaceJoint(&world->blockAllocator, &graph->colors[colorIndex].joints);
	memcpy(destinationJoint, joint, sizeof(b2Joint));
	joint->colorIndex = colorIndex;
	joint->colorSubIndex = graph->colors[colorIndex].joints.count - 1;

	// the caller should fix the contact lookup because b2RemoveContactFromGraph cannot fix the lookup because it
	// doesn't know the destination of the contact
}

void b2RemoveJointFromGraph(b2World* world, b2Joint* joint)
{
	int bodyKeyA = joint->edges[0].bodyKey;
	int bodyKeyB = joint->edges[1].bodyKey;

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
		B2_ASSERT(lookup->setIndex == b2_awakeBodySet);
		B2_ASSERT(lookup->graphColorIndex == colorIndex);
		B2_ASSERT(lookup->jointIndex == movedIndex);
		lookup->jointIndex = colorSubIndex;
	}

	joint->colorIndex = B2_NULL_INDEX;
	joint->colorSubIndex = B2_NULL_INDEX;
}
