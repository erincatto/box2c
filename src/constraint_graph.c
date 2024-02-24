// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "constraint_graph.h"

#include "array.h"
#include "bitset.inl"
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

void b2CreateGraph(b2ConstraintGraph* graph, int32_t bodyCapacity, int32_t contactCapacity, int32_t jointCapacity)
{
	*graph = (b2ConstraintGraph){0};

	bodyCapacity = B2_MAX(bodyCapacity, 8);
	contactCapacity = B2_MAX(contactCapacity, 8);
	jointCapacity = B2_MAX(jointCapacity, 8);

	for (int32_t i = 0; i < b2_graphColorCount; ++i)
	{
		b2GraphColor* color = graph->colors + i;
		color->bodySet = b2CreateBitSet(bodyCapacity);
		b2SetBitCountAndClear(&color->bodySet, bodyCapacity);

		color->contactArray = b2CreateArray(sizeof(int32_t), contactCapacity);
		color->jointArray = b2CreateArray(sizeof(int32_t), jointCapacity);
		color->contactConstraints = NULL;
	}

	graph->overflow.contactArray = b2CreateArray(sizeof(int32_t), contactCapacity);
	graph->overflow.jointArray = b2CreateArray(sizeof(int32_t), jointCapacity);
	graph->overflow.contactConstraints = NULL;
}

void b2DestroyGraph(b2ConstraintGraph* graph)
{
	for (int32_t i = 0; i < b2_graphColorCount; ++i)
	{
		b2GraphColor* color = graph->colors + i;
		b2DestroyBitSet(&color->bodySet);
		b2DestroyArray(color->contactArray, sizeof(int32_t));
		b2DestroyArray(color->jointArray, sizeof(int32_t));
	}

	b2DestroyArray(graph->overflow.contactArray, sizeof(int32_t));
	b2DestroyArray(graph->overflow.jointArray, sizeof(int32_t));
}

void b2AddContactToGraph(b2World* world, b2Contact* contact)
{
	B2_ASSERT(contact->colorIndex == B2_NULL_INDEX);
	B2_ASSERT(contact->colorSubIndex == B2_NULL_INDEX);

	b2ConstraintGraph* graph = &world->graph;

#if B2_FORCE_OVERFLOW == 0
	int32_t bodyIndexA = contact->edges[0].bodyIndex;
	int32_t bodyIndexB = contact->edges[1].bodyIndex;

	b2BodyType typeA = world->bodies[bodyIndexA].type;
	b2BodyType typeB = world->bodies[bodyIndexB].type;
	B2_ASSERT(typeA != b2_staticBody || typeB != b2_staticBody);

	if (typeA != b2_staticBody && typeB != b2_staticBody)
	{
		for (int32_t i = 0; i < b2_graphColorCount; ++i)
		{
			b2GraphColor* color = graph->colors + i;
			if (b2GetBit(&color->bodySet, bodyIndexA) || b2GetBit(&color->bodySet, bodyIndexB))
			{
				continue;
			}

			b2SetBitGrow(&color->bodySet, bodyIndexA);
			b2SetBitGrow(&color->bodySet, bodyIndexB);

			contact->colorSubIndex = b2Array(color->contactArray).count;
			b2Array_Push(color->contactArray, contact->object.index);
			contact->colorIndex = i;
			break;
		}
	}
	else if (typeA != b2_staticBody)
	{
		// Static contacts never in color 0
		for (int32_t i = 1; i < b2_graphColorCount; ++i)
		{
			b2GraphColor* color = graph->colors + i;
			if (b2GetBit(&color->bodySet, bodyIndexA))
			{
				continue;
			}

			b2SetBitGrow(&color->bodySet, bodyIndexA);

			contact->colorSubIndex = b2Array(color->contactArray).count;
			b2Array_Push(color->contactArray, contact->object.index);
			contact->colorIndex = i;
			break;
		}
	}
	else if (typeB != b2_staticBody)
	{
		// Static contacts never in color 0
		for (int32_t i = 1; i < b2_graphColorCount; ++i)
		{
			b2GraphColor* color = graph->colors + i;
			if (b2GetBit(&color->bodySet, bodyIndexB))
			{
				continue;
			}

			b2SetBitGrow(&color->bodySet, bodyIndexB);

			contact->colorSubIndex = b2Array(color->contactArray).count;
			b2Array_Push(color->contactArray, contact->object.index);
			contact->colorIndex = i;
			break;
		}
	}
#endif

	// Overflow
	if (contact->colorIndex == B2_NULL_INDEX)
	{
		contact->colorSubIndex = b2Array(graph->overflow.contactArray).count;
		b2Array_Push(graph->overflow.contactArray, contact->object.index);
		contact->colorIndex = b2_overflowIndex;
	}
}

void b2RemoveContactFromGraph(b2World* world, b2Contact* contact)
{
	B2_ASSERT(contact->colorIndex != B2_NULL_INDEX);
	B2_ASSERT(contact->colorSubIndex != B2_NULL_INDEX);

	b2ConstraintGraph* graph = &world->graph;

	// Overflow
	if (contact->colorIndex == b2_overflowIndex)
	{
		int32_t colorSubIndex = contact->colorSubIndex;
		b2Array_RemoveSwap(graph->overflow.contactArray, colorSubIndex);
		if (colorSubIndex < b2Array(graph->overflow.contactArray).count)
		{
			// Fix index on swapped contact
			int32_t swappedIndex = graph->overflow.contactArray[colorSubIndex];
			B2_ASSERT(world->contacts[swappedIndex].colorIndex == b2_overflowIndex);
			world->contacts[swappedIndex].colorSubIndex = colorSubIndex;
		}

		contact->colorIndex = B2_NULL_INDEX;
		contact->colorSubIndex = B2_NULL_INDEX;

		return;
	}

	B2_ASSERT(0 <= contact->colorIndex && contact->colorIndex < b2_graphColorCount);
	int32_t bodyIndexA = contact->edges[0].bodyIndex;
	int32_t bodyIndexB = contact->edges[1].bodyIndex;

	b2BodyType typeA = world->bodies[bodyIndexA].type;
	b2BodyType typeB = world->bodies[bodyIndexB].type;
	B2_ASSERT(typeA != b2_staticBody || typeB != b2_staticBody);

	if (typeA != b2_staticBody && typeB != b2_staticBody)
	{
		b2GraphColor* color = graph->colors + contact->colorIndex;
		B2_ASSERT(b2GetBit(&color->bodySet, bodyIndexA) && b2GetBit(&color->bodySet, bodyIndexB));

		int32_t colorSubIndex = contact->colorSubIndex;
		b2Array_RemoveSwap(color->contactArray, colorSubIndex);
		if (colorSubIndex < b2Array(color->contactArray).count)
		{
			// Fix index on swapped contact
			int32_t swappedIndex = color->contactArray[colorSubIndex];
			world->contacts[swappedIndex].colorSubIndex = colorSubIndex;
		}

		b2ClearBit(&color->bodySet, bodyIndexA);
		b2ClearBit(&color->bodySet, bodyIndexB);
	}
	else if (typeA != b2_staticBody)
	{
		b2GraphColor* color = graph->colors + contact->colorIndex;
		B2_ASSERT(b2GetBit(&color->bodySet, bodyIndexA));

		int32_t colorSubIndex = contact->colorSubIndex;
		b2Array_RemoveSwap(color->contactArray, colorSubIndex);
		if (colorSubIndex < b2Array(color->contactArray).count)
		{
			// Fix index on swapped contact
			int32_t swappedIndex = color->contactArray[colorSubIndex];
			world->contacts[swappedIndex].colorSubIndex = colorSubIndex;
		}

		b2ClearBit(&color->bodySet, bodyIndexA);
	}
	else if (typeB != b2_staticBody)
	{
		b2GraphColor* color = graph->colors + contact->colorIndex;
		B2_ASSERT(b2GetBit(&color->bodySet, bodyIndexB));

		int32_t colorSubIndex = contact->colorSubIndex;
		b2Array_RemoveSwap(color->contactArray, colorSubIndex);
		if (colorSubIndex < b2Array(color->contactArray).count)
		{
			// Fix index on swapped contact
			int32_t swappedIndex = color->contactArray[colorSubIndex];
			world->contacts[swappedIndex].colorSubIndex = colorSubIndex;
		}

		b2ClearBit(&color->bodySet, bodyIndexB);
	}

	contact->colorIndex = B2_NULL_INDEX;
	contact->colorSubIndex = B2_NULL_INDEX;
}

void b2AddJointToGraph(b2World* world, b2Joint* joint)
{
	B2_ASSERT(joint->colorIndex == B2_NULL_INDEX);
	B2_ASSERT(joint->colorSubIndex == B2_NULL_INDEX);

	b2ConstraintGraph* graph = &world->graph;

#if B2_FORCE_OVERFLOW == 0
	int32_t bodyIndexA = joint->edges[0].bodyIndex;
	int32_t bodyIndexB = joint->edges[1].bodyIndex;

	b2BodyType typeA = world->bodies[bodyIndexA].type;
	b2BodyType typeB = world->bodies[bodyIndexB].type;

	if (typeA == b2_dynamicBody && typeB == b2_dynamicBody)
	{
		for (int32_t i = 0; i < b2_graphColorCount; ++i)
		{
			b2GraphColor* color = graph->colors + i;
			if (b2GetBit(&color->bodySet, bodyIndexA) || b2GetBit(&color->bodySet, bodyIndexB))
			{
				continue;
			}

			b2SetBitGrow(&color->bodySet, bodyIndexA);
			b2SetBitGrow(&color->bodySet, bodyIndexB);

			joint->colorSubIndex = b2Array(color->jointArray).count;
			b2Array_Push(color->jointArray, joint->object.index);
			joint->colorIndex = i;
			break;
		}
	}
	else if (typeA == b2_dynamicBody)
	{
		for (int32_t i = 0; i < b2_graphColorCount; ++i)
		{
			b2GraphColor* color = graph->colors + i;
			if (b2GetBit(&color->bodySet, bodyIndexA))
			{
				continue;
			}

			b2SetBitGrow(&color->bodySet, bodyIndexA);

			joint->colorSubIndex = b2Array(color->jointArray).count;
			b2Array_Push(color->jointArray, joint->object.index);
			joint->colorIndex = i;
			break;
		}
	}
	else if (typeB == b2_dynamicBody)
	{
		for (int32_t i = 0; i < b2_graphColorCount; ++i)
		{
			b2GraphColor* color = graph->colors + i;
			if (b2GetBit(&color->bodySet, bodyIndexB))
			{
				continue;
			}

			b2SetBitGrow(&color->bodySet, bodyIndexB);

			joint->colorSubIndex = b2Array(color->jointArray).count;
			b2Array_Push(color->jointArray, joint->object.index);
			joint->colorIndex = i;
			break;
		}
	}
#endif

	// Overflow
	if (joint->colorIndex == B2_NULL_INDEX)
	{
		joint->colorSubIndex = b2Array(graph->overflow.jointArray).count;
		b2Array_Push(graph->overflow.jointArray, joint->object.index);
		joint->colorIndex = b2_overflowIndex;
	}
}

void b2RemoveJointFromGraph(b2World* world, b2Joint* joint)
{
	B2_ASSERT(joint->colorIndex != B2_NULL_INDEX);
	B2_ASSERT(joint->colorSubIndex != B2_NULL_INDEX);

	b2ConstraintGraph* graph = &world->graph;

	// Overflow
	if (joint->colorIndex == b2_overflowIndex)
	{
		int32_t colorSubIndex = joint->colorSubIndex;
		b2Array_RemoveSwap(graph->overflow.jointArray, colorSubIndex);
		if (colorSubIndex < b2Array(graph->overflow.jointArray).count)
		{
			// Fix index on swapped joint
			int32_t swappedIndex = graph->overflow.jointArray[colorSubIndex];
			B2_ASSERT(world->joints[swappedIndex].colorIndex == b2_overflowIndex);
			world->joints[swappedIndex].colorSubIndex = colorSubIndex;
		}

		joint->colorIndex = B2_NULL_INDEX;
		joint->colorSubIndex = B2_NULL_INDEX;

		return;
	}

	B2_ASSERT(0 <= joint->colorIndex && joint->colorIndex < b2_graphColorCount);
	int32_t bodyIndexA = joint->edges[0].bodyIndex;
	int32_t bodyIndexB = joint->edges[1].bodyIndex;

	b2BodyType typeA = world->bodies[bodyIndexA].type;
	b2BodyType typeB = world->bodies[bodyIndexB].type;

	if (typeA == b2_dynamicBody && typeB == b2_dynamicBody)
	{
		b2GraphColor* color = graph->colors + joint->colorIndex;
		B2_ASSERT(b2GetBit(&color->bodySet, bodyIndexA) && b2GetBit(&color->bodySet, bodyIndexB));

		int32_t colorSubIndex = joint->colorSubIndex;
		b2Array_RemoveSwap(color->jointArray, colorSubIndex);
		if (colorSubIndex < b2Array(color->jointArray).count)
		{
			// Fix index on swapped joint
			int32_t swappedIndex = color->jointArray[colorSubIndex];
			world->joints[swappedIndex].colorSubIndex = colorSubIndex;
		}

		b2ClearBit(&color->bodySet, bodyIndexA);
		b2ClearBit(&color->bodySet, bodyIndexB);
	}
	else if (typeA == b2_dynamicBody)
	{
		b2GraphColor* color = graph->colors + joint->colorIndex;
		B2_ASSERT(b2GetBit(&color->bodySet, bodyIndexA));

		int32_t colorSubIndex = joint->colorSubIndex;
		b2Array_RemoveSwap(color->jointArray, colorSubIndex);
		if (colorSubIndex < b2Array(color->jointArray).count)
		{
			// Fix index on swapped joint
			int32_t swappedIndex = color->jointArray[colorSubIndex];
			world->joints[swappedIndex].colorSubIndex = colorSubIndex;
		}

		b2ClearBit(&color->bodySet, bodyIndexA);
	}
	else if (typeB == b2_dynamicBody)
	{
		b2GraphColor* color = graph->colors + joint->colorIndex;
		B2_ASSERT(b2GetBit(&color->bodySet, bodyIndexB));

		int32_t colorSubIndex = joint->colorSubIndex;
		b2Array_RemoveSwap(color->jointArray, colorSubIndex);
		if (colorSubIndex < b2Array(color->jointArray).count)
		{
			// Fix index on swapped joint
			int32_t swappedIndex = color->jointArray[colorSubIndex];
			world->joints[swappedIndex].colorSubIndex = colorSubIndex;
		}

		b2ClearBit(&color->bodySet, bodyIndexB);
	}

	joint->colorIndex = B2_NULL_INDEX;
	joint->colorSubIndex = B2_NULL_INDEX;
}
