// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "graph.h"

#include "allocate.h"
#include "array.h"
#include "body.h"
#include "contact.h"
#include "core.h"
#include "shape.h"
#include "stack_allocator.h"
#include "world.h"

#include <stdbool.h>

void b2CreateGraph(b2Graph* graph, int32_t bodyCapacity, int32_t contactCapacity)
{
	bodyCapacity = B2_MAX(bodyCapacity, 8);
	contactCapacity = B2_MAX(contactCapacity, 8);

	for (int32_t i = 0; i < b2_graphColorCount; ++i)
	{
		b2GraphColor* color = graph->colors + i;
		color->bodySet = b2CreateBitSet(bodyCapacity);
		b2SetBitCountAndClear(&color->bodySet, bodyCapacity);

		color->contactArray = b2CreateArray(sizeof(int32_t), contactCapacity);
	}
}

void b2DestroyGraph(b2Graph* graph)
{
	for (int32_t i = 0; i < b2_graphColorCount; ++i)
	{
		b2GraphColor* color = graph->colors + i;
		b2DestroyBitSet(&color->bodySet);
		b2DestroyArray(color->contactArray, sizeof(int32_t));
	}
}

void b2AddContactToGraph(b2World* world, b2Contact* contact)
{
	B2_ASSERT(contact->colorContactIndex == B2_NULL_INDEX);
	B2_ASSERT(contact->colorIndex == B2_NULL_INDEX);

	b2Graph* graph = &world->graph;

	int32_t bodyIndexA = contact->edges[0].bodyIndex;
	int32_t bodyIndexB = contact->edges[1].bodyIndex;

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

			b2SetBit(&color->bodySet, bodyIndexA);
			b2SetBit(&color->bodySet, bodyIndexB);

			contact->colorContactIndex = b2Array(color->contactArray).count;
			b2Array_Push(color->contactArray, contact->object.index);
			contact->colorIndex = i;
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

			b2SetBit(&color->bodySet, bodyIndexA);

			contact->colorContactIndex = b2Array(color->contactArray).count;
			b2Array_Push(color->contactArray, contact->object.index);
			contact->colorIndex = i;
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

			b2SetBit(&color->bodySet, bodyIndexB);

			contact->colorContactIndex = b2Array(color->contactArray).count;
			b2Array_Push(color->contactArray, contact->object.index);
			contact->colorIndex = i;
			break;
		}
	}
}

void b2RemoveContactFromGraph(b2World* world, b2Contact* contact)
{
	B2_ASSERT(contact->colorIndex != B2_NULL_INDEX);
	B2_ASSERT(contact->colorContactIndex != B2_NULL_INDEX);

	b2Graph* graph = &world->graph;

	B2_ASSERT(0 <= contact->colorIndex && contact->colorIndex < b2_graphColorCount);
	int32_t bodyIndexA = contact->edges[0].bodyIndex;
	int32_t bodyIndexB = contact->edges[1].bodyIndex;

	b2BodyType typeA = world->bodies[bodyIndexA].type;
	b2BodyType typeB = world->bodies[bodyIndexB].type;

	b2GraphColor* color = graph->colors + contact->colorIndex;

	int32_t colorContactIndex = contact->colorContactIndex;
	b2Array_RemoveSwap(color->contactArray, colorContactIndex);
	if (colorContactIndex < b2Array(color->contactArray).count)
	{
		// Fix index on swapped contact
		int32_t swappedContactIndex = color->contactArray[colorContactIndex];
		world->contacts[swappedContactIndex].colorContactIndex = colorContactIndex;
	}

	if (typeA == b2_dynamicBody && typeB == b2_dynamicBody)
	{
		B2_ASSERT(b2GetBit(&color->bodySet, bodyIndexA) && b2GetBit(&color->bodySet, bodyIndexB));

		b2ClearBit(&color->bodySet, bodyIndexA);
		b2ClearBit(&color->bodySet, bodyIndexB);
	}
	else if (typeA == b2_dynamicBody)
	{
		B2_ASSERT(b2GetBit(&color->bodySet, bodyIndexA));

		b2ClearBit(&color->bodySet, bodyIndexA);
	}
	else if (typeB == b2_dynamicBody)
	{
		B2_ASSERT(b2GetBit(&color->bodySet, bodyIndexB));

		b2ClearBit(&color->bodySet, bodyIndexB);
	}

	contact->colorIndex = B2_NULL_INDEX;
	contact->colorContactIndex = B2_NULL_INDEX;
}

typedef struct b2ConstraintPoint
{
	b2Vec2 rA;
	b2Vec2 rB;
	float normalImpulse;
	float tangentImpulse;
	float normalMass;
	float tangentMass;
	float velocityBias;
} b2ConstraintPoint;

typedef struct b2Constraint
{
	b2Contact* contact;
	b2ConstraintPoint points[2];
	b2Vec2 normal;
	float friction;
	int32_t pointCount;
} b2Constraint;

static void b2InitializeConstraints(b2World* world, b2GraphColor* color, const b2StepContext* stepContext)
{
	int32_t constraintCount = b2Array(color->contactArray).count;
	int32_t* contactIndices = color->contactArray;
	b2Contact* contacts = world->contacts;
	b2Body* bodies = world->bodies;
	float inv_dt = stepContext->inv_dt;

	for (int32_t i = 0; i < constraintCount; ++i)
	{
		b2Contact* contact = contacts + contactIndices[i];

		const b2Manifold* manifold = &contact->manifold;
		int32_t pointCount = manifold->pointCount;

		B2_ASSERT(0 < pointCount && pointCount <= 2);

		int32_t indexA = contact->edges[0].bodyIndex;
		int32_t indexB = contact->edges[1].bodyIndex;
		b2Body* bodyA = bodies + indexA;
		b2Body* bodyB = bodies + indexB;

		b2Constraint* constraint = color->contraints + i;
		constraint->contact = contact;
		constraint->normal = manifold->normal;
		constraint->friction = contact->friction;
		constraint->pointCount = pointCount;

		float mA = bodyA->invMass;
		float iA = bodyA->invI;
		float mB = bodyB->invMass;
		float iB = bodyB->invI;

		b2Rot qA = bodyA->transform.q;
		b2Vec2 cA = bodyA->position;
		b2Rot qB = bodyB->transform.q;
		b2Vec2 cB = bodyB->position;

		b2Vec2 vA = bodyA->linearVelocity;
		float wA = bodyA->angularVelocity;
		b2Vec2 vB = bodyB->linearVelocity;
		float wB = bodyB->angularVelocity;

		b2Vec2 tangent = b2RightPerp(constraint->normal);

		for (int32_t j = 0; j < pointCount; ++j)
		{
			const b2ManifoldPoint* cp = manifold->points + j;
			b2ConstraintPoint* constraintPoint = constraint->points + j;

			constraintPoint->normalImpulse = cp->normalImpulse;
			constraintPoint->tangentImpulse =  cp->tangentImpulse;

			constraintPoint->rA = b2Sub(cp->point, cA);
			constraintPoint->rB = b2Sub(cp->point, cB);

			float rnA = b2Cross(constraintPoint->rA, constraint->normal);
			float rnB = b2Cross(constraintPoint->rB, constraint->normal);

			float kNormal = mA + mB + iA * rnA * rnA + iB * rnB * rnB;

			constraintPoint->normalMass = kNormal > 0.0f ? 1.0f / kNormal : 0.0f;

			float rtA = b2Cross(constraintPoint->rA, tangent);
			float rtB = b2Cross(constraintPoint->rB, tangent);

			float kTangent = mA + mB + iA * rtA * rtA + iB * rtB * rtB;

			constraintPoint->tangentMass = kTangent > 0.0f ? 1.0f / kTangent : 0.0f;

			// Velocity bias for speculative collision
			constraintPoint->velocityBias = -B2_MAX(0.0f, cp->separation * inv_dt);
		}

		constraintCount += 1;
	}
}

void b2SolveGraph(b2World* world, const b2StepContext* stepContext)
{
	b2Graph* graph = &world->graph;
	b2GraphColor* colors = graph->colors;

	int32_t constraintCount = 0;
	for (int32_t i = 0; i < b2_graphColorCount; ++i)
	{
		constraintCount += b2Array(colors[i].contactArray).count;
	}

	b2Constraint* constraints = b2AllocateStackItem(&world->stackAllocator, constraintCount * sizeof(b2Constraint), "constraint");
	int32_t base = 0;

	for (int32_t i = 0; i < b2_graphColorCount; ++i)
	{
		colors[i].contraints = constraints + base;
		base += b2Array(colors[i].contactArray).count;
	}

	B2_ASSERT(base == constraintCount);

	for (int32_t i = 0; i < b2_graphColorCount; ++i)
	{
		colors[i].contraints = constraints + base;
		base += b2Array(colors[i].contactArray).count;
	}

	B2_MAYBE_UNUSED(world);
}
