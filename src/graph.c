// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "graph.h"

#include "allocate.h"
#include "array.h"
#include "body.h"
#include "contact.h"
#include "core.h"
#include "shape.h"
#include "solver_data.h"
#include "stack_allocator.h"
#include "world.h"

#include "box2d/aabb.h"

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

	B2_ASSERT(contact->colorIndex != B2_NULL_INDEX && contact->colorContactIndex != B2_NULL_INDEX);
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

static void b2IntegrateVelocities(b2World* world, const b2StepContext* context)
{
	b2Body* bodies = world->bodies;
	int32_t bodyCapacity = world->bodyPool.capacity;
	float timeStep = context->dt;
	b2Vec2 gravity = world->gravity;

	// Integrate velocities and apply damping. Initialize the body state.
	for (int32_t i = 0; i < bodyCapacity; ++i)
	{
		b2Body* body = bodies + i;
		if (b2ObjectValid(&body->object) == false)
		{
			continue;
		}

		if (body->type != b2_dynamicBody)
		{
			continue;
		}

		float invMass = body->invMass;
		float invI = body->invI;

		b2Vec2 v = body->linearVelocity;
		float w = body->angularVelocity;

		// Integrate velocities
		v = b2Add(v, b2MulSV(timeStep * invMass, b2MulAdd(body->force, body->gravityScale * body->mass, gravity)));
		w = w + timeStep * invI * body->torque;

		// Apply damping.
		// ODE: dv/dt + c * v = 0
		// Solution: v(t) = v0 * exp(-c * t)
		// Time step: v(t + dt) = v0 * exp(-c * (t + dt)) = v0 * exp(-c * t) * exp(-c * dt) = v * exp(-c * dt)
		// v2 = exp(-c * dt) * v1
		// Pade approximation:
		// v2 = v1 * 1 / (1 + c * dt)
		v = b2MulSV(1.0f / (1.0f + timeStep * body->linearDamping), v);
		w *= 1.0f / (1.0f + timeStep * body->angularDamping);

		body->linearVelocity = v;
		body->angularVelocity = w;
	}
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
	const int32_t constraintCount = b2Array(color->contactArray).count;
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

		b2Constraint* constraint = color->constraints + i;
		constraint->contact = contact;
		constraint->normal = manifold->normal;
		constraint->friction = contact->friction;
		constraint->pointCount = pointCount;

		float mA = bodyA->invMass;
		float iA = bodyA->invI;
		float mB = bodyB->invMass;
		float iB = bodyB->invI;

		b2Vec2 cA = bodyA->position;
		b2Vec2 cB = bodyB->position;

		b2Vec2 vA = bodyA->linearVelocity;
		float wA = bodyA->angularVelocity;
		b2Vec2 vB = bodyB->linearVelocity;
		float wB = bodyB->angularVelocity;

		b2Vec2 normal = constraint->normal;
		b2Vec2 tangent = b2RightPerp(constraint->normal);

		for (int32_t j = 0; j < pointCount; ++j)
		{
			const b2ManifoldPoint* mp = manifold->points + j;
			b2ConstraintPoint* cp = constraint->points + j;

			cp->normalImpulse = mp->normalImpulse;
			cp->tangentImpulse = mp->tangentImpulse;

			cp->rA = b2Sub(mp->point, cA);
			cp->rB = b2Sub(mp->point, cB);

			float rnA = b2Cross(cp->rA, normal);
			float rnB = b2Cross(cp->rB, normal);

			float kNormal = mA + mB + iA * rnA * rnA + iB * rnB * rnB;

			cp->normalMass = kNormal > 0.0f ? 1.0f / kNormal : 0.0f;

			float rtA = b2Cross(cp->rA, tangent);
			float rtB = b2Cross(cp->rB, tangent);

			float kTangent = mA + mB + iA * rtA * rtA + iB * rtB * rtB;

			cp->tangentMass = kTangent > 0.0f ? 1.0f / kTangent : 0.0f;

			// Velocity bias for speculative collision
			cp->velocityBias = -B2_MAX(0.0f, mp->separation * inv_dt);

			// Warm start
			b2Vec2 P = b2Add(b2MulSV(cp->normalImpulse, normal), b2MulSV(cp->tangentImpulse, tangent));
			wA -= iA * b2Cross(cp->rA, P);
			vA = b2MulAdd(vA, -mA, P);
			wB += iB * b2Cross(cp->rB, P);
			vB = b2MulAdd(vB, mB, P);
		}
	}
}

static void b2SolveConstraints(b2World* world, b2GraphColor* color)
{
	const int32_t constraintCount = b2Array(color->contactArray).count;
	b2Body* bodies = world->bodies;

	for (int32_t i = 0; i < constraintCount; ++i)
	{
		b2Constraint* constraint = color->constraints + i;

		const b2Contact* contact = constraint->contact;

		int32_t indexA = contact->edges[0].bodyIndex;
		int32_t indexB = contact->edges[1].bodyIndex;
		b2Body* bodyA = bodies + indexA;
		b2Body* bodyB = bodies + indexB;

		float mA = bodyA->invMass;
		float iA = bodyA->invI;
		float mB = bodyB->invMass;
		float iB = bodyB->invI;
		int32_t pointCount = constraint->pointCount;

		b2Vec2 vA = bodyA->linearVelocity;
		float wA = bodyA->angularVelocity;
		b2Vec2 vB = bodyB->linearVelocity;
		float wB = bodyB->angularVelocity;

		b2Vec2 normal = constraint->normal;
		b2Vec2 tangent = b2RightPerp(normal);
		float friction = constraint->friction;

		// Solve tangent constraints first because non-penetration is more important
		// than friction.
		for (int32_t j = 0; j < pointCount; ++j)
		{
			b2ConstraintPoint* cp = constraint->points + j;

			// Relative velocity at contact
			b2Vec2 vrB = b2Add(vB, b2CrossSV(wB, cp->rB));
			b2Vec2 vrA = b2Add(vA, b2CrossSV(wA, cp->rA));
			b2Vec2 dv = b2Sub(vrB, vrA);

			// Compute tangent force
			float vt = b2Dot(dv, tangent);
			float lambda = cp->tangentMass * (-vt);

			// Clamp the accumulated force
			float maxFriction = friction * cp->normalImpulse;
			float newImpulse = B2_CLAMP(cp->tangentImpulse + lambda, -maxFriction, maxFriction);
			lambda = newImpulse - cp->tangentImpulse;
			cp->tangentImpulse = newImpulse;

			// Apply contact impulse
			b2Vec2 P = b2MulSV(lambda, tangent);

			vA = b2MulSub(vA, mA, P);
			wA -= iA * b2Cross(cp->rA, P);

			vB = b2MulAdd(vB, mB, P);
			wB += iB * b2Cross(cp->rB, P);
		}

		for (int32_t j = 0; j < pointCount; ++j)
		{
			b2ConstraintPoint* cp = constraint->points + j;

			// Relative velocity at contact
			b2Vec2 vrB = b2Add(vB, b2CrossSV(wB, cp->rB));
			b2Vec2 vrA = b2Add(vA, b2CrossSV(wA, cp->rA));
			b2Vec2 dv = b2Sub(vrB, vrA);

			// Compute normal impulse
			float vn = b2Dot(dv, normal);
			float lambda = -cp->normalMass * (vn - cp->velocityBias);

			// Clamp the accumulated impulse
			float newImpulse = B2_MAX(cp->normalImpulse + lambda, 0.0f);
			lambda = newImpulse - cp->normalImpulse;
			cp->normalImpulse = newImpulse;

			// Apply contact impulse
			b2Vec2 P = b2MulSV(lambda, normal);
			vA = b2MulSub(vA, mA, P);
			wA -= iA * b2Cross(cp->rA, P);

			vB = b2MulAdd(vB, mB, P);
			wB += iB * b2Cross(cp->rB, P);
		}

		bodyA->linearVelocity = vA;
		bodyA->angularVelocity = wA;
		bodyB->linearVelocity = vB;
		bodyB->angularVelocity = wB;
	}
}

static void b2StoreImpulses(b2GraphColor* color)
{
	int32_t constraintCount = b2Array(color->contactArray).count;

	for (int32_t i = 0; i < constraintCount; ++i)
	{
		b2Constraint* constraint = color->constraints + i;
		b2Contact* contact = constraint->contact;

		b2Manifold* manifold = &contact->manifold;

		for (int32_t j = 0; j < constraint->pointCount; ++j)
		{
			manifold->points[j].normalImpulse = constraint->points[j].normalImpulse;
			manifold->points[j].tangentImpulse = constraint->points[j].tangentImpulse;
		}
	}
}

static void b2IntegratePositions(b2World* world, const b2StepContext* context)
{
	b2Body* bodies = world->bodies;
	int32_t bodyCapacity = world->bodyPool.capacity;
	float timeStep = context->dt;
	b2Contact* contacts = world->contacts;

	b2BitSet* awakeContactBitSet = &world->taskContextArray[0].awakeContactBitSet;
	b2BitSet* shapeBitSet = &world->taskContextArray[0].shapeBitSet;
	const b2Vec2 aabbMargin = {b2_aabbMargin, b2_aabbMargin};

	// Integrate velocities and apply damping. Initialize the body state.
	for (int32_t i = 0; i < bodyCapacity; ++i)
	{
		b2Body* body = bodies + i;
		if (b2ObjectValid(&body->object) == false)
		{
			continue;
		}

		if (body->type == b2_staticBody)
		{
			continue;
		}

		b2Vec2 c = body->position;
		float a = body->angle;
		b2Vec2 v = body->linearVelocity;
		float w = body->angularVelocity;

		// Clamp large velocities
		b2Vec2 translation = b2MulSV(timeStep, v);
		if (b2Dot(translation, translation) > b2_maxTranslationSquared)
		{
			float ratio = b2_maxTranslation / b2Length(translation);
			v = b2MulSV(ratio, v);
		}

		float rotation = timeStep * w;
		if (rotation * rotation > b2_maxRotationSquared)
		{
			float ratio = b2_maxRotation / B2_ABS(rotation);
			w *= ratio;
		}

		// Integrate
		c = b2MulAdd(c, timeStep, v);
		a += timeStep * w;

		body->position = c;
		body->angle = a;
		body->linearVelocity = v;
		body->angularVelocity = w;

		body->transform.q = b2MakeRot(body->angle);
		body->transform.p = b2Sub(body->position, b2RotateVector(body->transform.q, body->localCenter));

		body->force = b2Vec2_zero;
		body->torque = 0.0f;

		// Update shapes AABBs
		int32_t shapeIndex = body->shapeList;
		while (shapeIndex != B2_NULL_INDEX)
		{
			b2Shape* shape = world->shapes + shapeIndex;

			B2_ASSERT(shape->isFast == false);

			shape->aabb = b2Shape_ComputeAABB(shape, body->transform);

			if (b2AABB_Contains(shape->fatAABB, shape->aabb) == false)
			{
				shape->fatAABB.lowerBound = b2Sub(shape->aabb.lowerBound, aabbMargin);
				shape->fatAABB.upperBound = b2Add(shape->aabb.upperBound, aabbMargin);

				// Bit-set to keep the move array sorted
				b2SetBit(shapeBitSet, shapeIndex);
			}

			shapeIndex = shape->nextShapeIndex;
		}

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

	b2Constraint* constraints = b2AllocateStackItem(world->stackAllocator, constraintCount * sizeof(b2Constraint), "constraint");
	int32_t base = 0;

	for (int32_t i = 0; i < b2_graphColorCount; ++i)
	{
		colors[i].constraints = constraints + base;
		base += b2Array(colors[i].contactArray).count;
	}

	B2_ASSERT(base == constraintCount);

	b2IntegrateVelocities(world, stepContext);

	for (int32_t i = 0; i < b2_graphColorCount; ++i)
	{
		b2InitializeConstraints(world, colors + i, stepContext);
	}

	int32_t iterationCount = stepContext->velocityIterations;
	for (int32_t iter = 0; iter < iterationCount; ++iter)
	{
		for (int32_t i = 0; i < b2_graphColorCount; ++i)
		{
			b2SolveConstraints(world, colors + i);
		}
	}

	for (int32_t i = 0; i < b2_graphColorCount; ++i)
	{
		b2StoreImpulses(colors + i);
	}

	b2FreeStackItem(world->stackAllocator, constraints);

	b2IntegratePositions(world, stepContext);
}
