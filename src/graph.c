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

static void b2IntegrateVelocities(b2World* world, float h)
{
	b2Body* bodies = world->bodies;
	int32_t bodyCapacity = world->bodyPool.capacity;
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
		v = b2Add(v, b2MulSV(h * invMass, b2MulAdd(body->force, body->gravityScale * body->mass, gravity)));
		w = w + h * invI * body->torque;

		// Apply damping.
		// ODE: dv/dt + c * v = 0
		// Solution: v(t) = v0 * exp(-c * t)
		// Time step: v(t + dt) = v0 * exp(-c * (t + dt)) = v0 * exp(-c * t) * exp(-c * dt) = v * exp(-c * dt)
		// v2 = exp(-c * dt) * v1
		// Pade approximation:
		// v2 = v1 * 1 / (1 + c * dt)
		v = b2MulSV(1.0f / (1.0f + h * body->linearDamping), v);
		w *= 1.0f / (1.0f + h * body->angularDamping);

		body->linearVelocity = v;
		body->angularVelocity = w;
	}
}

typedef struct b2ConstraintPoint
{
	b2Vec2 rA, rB;
	b2Vec2 localAnchorA, localAnchorB;
	float separation;
	float baseSeparation;
	float normalImpulse;
	float tangentImpulse;
	float normalMass;
	float tangentMass;
	float bias;
	float gamma;
} b2ConstraintPoint;

typedef struct b2Constraint
{
	b2Contact* contact;
	int32_t indexA;
	int32_t indexB;
	b2ConstraintPoint points[2];
	b2Vec2 normal;
	float friction;
	int32_t pointCount;
} b2Constraint;

static void b2InitializeConstraintsAndWarmStart(b2World* world, b2GraphColor* color, float h)
{
	const int32_t constraintCount = b2Array(color->contactArray).count;
	int32_t* contactIndices = color->contactArray;
	b2Contact* contacts = world->contacts;
	b2Body* bodies = world->bodies;

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
		constraint->indexA = indexA;
		constraint->indexB = indexB;
		constraint->normal = manifold->normal;
		constraint->friction = contact->friction;
		constraint->pointCount = pointCount;

		float mA = bodyA->invMass;
		float iA = bodyA->invI;
		float mB = bodyB->invMass;
		float iB = bodyB->invI;

		b2Vec2 cA = bodyA->position;
		b2Vec2 cB = bodyB->position;
		b2Rot qA = b2MakeRot(bodyA->angle);
		b2Rot qB = b2MakeRot(bodyB->angle);

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
			cp->localAnchorA = b2InvRotateVector(qA, cp->rA);
			cp->localAnchorB = b2InvRotateVector(qB, cp->rB);

			float rnA = b2Cross(cp->rA, normal);
			float rnB = b2Cross(cp->rB, normal);
			float kNormal = mA + mB + iA * rnA * rnA + iB * rnB * rnB;

			float rtA = b2Cross(cp->rA, tangent);
			float rtB = b2Cross(cp->rB, tangent);
			float kTangent = mA + mB + iA * rtA * rtA + iB * rtB * rtB;

			cp->tangentMass = kTangent > 0.0f ? 1.0f / kTangent : 0.0f;

			// Soft contact with speculation
			const float hertz = 10.0f;
			const float zeta = 1.0f;
			float omega = 2.0f * b2_pi * hertz;
			//float d = 2.0f * zeta * omega / kNormal;
			//float k = omega * omega / kNormal;

			//cp->gamma = 1.0f / (h * (d + h * k));
			//cp->gamma = 1.0f / (h * (2.0f * zeta * omega / kNormal + h * omega * omega / kNormal));
			cp->gamma = kNormal / (h * omega * (2.0f * zeta + h * omega));

			//cp->bias = h * k * cp->gamma * mp->separation;
			//cp->bias = k / (d + h * k) * mp->separation;
			//cp->bias =
			//	(omega * omega / kNormal) / (2.0f * dampingRatio * omega / kNormal + h * omega * omega / kNormal) * mp->separation;
			cp->bias = (omega / (2.0f * zeta + h * omega)) * mp->separation;
			//cp->gamma = 0.0f;
			//cp->bias = (0.2f / h) * mp->separation;

			// TODO_ERIN this can be expanded
			cp->normalMass = 1.0f / (kNormal + cp->gamma);

			// Warm start
			b2Vec2 P = b2Add(b2MulSV(cp->normalImpulse, normal), b2MulSV(cp->tangentImpulse, tangent));
			wA -= iA * b2Cross(cp->rA, P);
			vA = b2MulAdd(vA, -mA, P);
			wB += iB * b2Cross(cp->rB, P);
			vB = b2MulAdd(vB, mB, P);

			cp->baseSeparation = mp->separation;
			cp->separation = mp->separation;
		}

		bodyA->linearVelocity = vA;
		bodyA->angularVelocity = wA;
		bodyB->linearVelocity = vB;
		bodyB->angularVelocity = wB;
	}
}

static void b2InitializeConstraints(b2World* world, b2GraphColor* color, float h)
{
	const int32_t constraintCount = b2Array(color->contactArray).count;
	int32_t* contactIndices = color->contactArray;
	b2Contact* contacts = world->contacts;
	b2Body* bodies = world->bodies;

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
		constraint->indexA = indexA;
		constraint->indexB = indexB;
		constraint->normal = manifold->normal;
		constraint->friction = contact->friction;
		constraint->pointCount = pointCount;

		float mA = bodyA->invMass;
		float iA = bodyA->invI;
		float mB = bodyB->invMass;
		float iB = bodyB->invI;

		b2Vec2 cA = bodyA->position;
		b2Vec2 cB = bodyB->position;
		b2Rot qA = b2MakeRot(bodyA->angle);
		b2Rot qB = b2MakeRot(bodyB->angle);

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
			cp->localAnchorA = b2InvRotateVector(qA, cp->rA);
			cp->localAnchorB = b2InvRotateVector(qB, cp->rB);

			float rnA = b2Cross(cp->rA, normal);
			float rnB = b2Cross(cp->rB, normal);
			float kNormal = mA + mB + iA * rnA * rnA + iB * rnB * rnB;

			float rtA = b2Cross(cp->rA, tangent);
			float rtB = b2Cross(cp->rB, tangent);
			float kTangent = mA + mB + iA * rtA * rtA + iB * rtB * rtB;

			cp->tangentMass = kTangent > 0.0f ? 1.0f / kTangent : 0.0f;

			// Soft contact with speculation
			const float hertz = 10.0f;
			const float zeta = 1.0f;
			float omega = 2.0f * b2_pi * hertz;
			// float d = 2.0f * zeta * omega / kNormal;
			// float k = omega * omega / kNormal;

			// cp->gamma = 1.0f / (h * (d + h * k));
			// cp->gamma = 1.0f / (h * (2.0f * zeta * omega / kNormal + h * omega * omega / kNormal));
			cp->gamma = kNormal / (h * omega * (2.0f * zeta + h * omega));

			// cp->bias = h * k * cp->gamma * mp->separation;
			// cp->bias = k / (d + h * k) * mp->separation;
			// cp->bias =
			//	(omega * omega / kNormal) / (2.0f * dampingRatio * omega / kNormal + h * omega * omega / kNormal) * mp->separation;
			cp->bias = (omega / (2.0f * zeta + h * omega)) * mp->separation;
			// cp->gamma = 0.0f;
			// cp->bias = (0.2f / h) * mp->separation;

			// TODO_ERIN this can be expanded
			cp->normalMass = 1.0f / (kNormal + cp->gamma);

			cp->baseSeparation = mp->separation;
			cp->separation = mp->separation;
		}
	}
}

static void b2WarmStart(b2World* world, b2GraphColor* color)
{
	const int32_t constraintCount = b2Array(color->contactArray).count;
	b2Body* bodies = world->bodies;

	for (int32_t i = 0; i < constraintCount; ++i)
	{
		b2Constraint* constraint = color->constraints + i;

		int32_t pointCount = constraint->pointCount;
		B2_ASSERT(0 < pointCount && pointCount <= 2);

		b2Body* bodyA = bodies + constraint->indexA;
		b2Body* bodyB = bodies + constraint->indexB;

		float mA = bodyA->invMass;
		float iA = bodyA->invI;
		float mB = bodyB->invMass;
		float iB = bodyB->invI;

		b2Vec2 vA = bodyA->linearVelocity;
		float wA = bodyA->angularVelocity;
		b2Vec2 vB = bodyB->linearVelocity;
		float wB = bodyB->angularVelocity;

		b2Vec2 normal = constraint->normal;
		b2Vec2 tangent = b2RightPerp(constraint->normal);

		for (int32_t j = 0; j < pointCount; ++j)
		{
			b2ConstraintPoint* cp = constraint->points + j;

			b2Vec2 P = b2Add(b2MulSV(cp->normalImpulse, normal), b2MulSV(cp->tangentImpulse, tangent));
			wA -= iA * b2Cross(cp->rA, P);
			vA = b2MulAdd(vA, -mA, P);
			wB += iB * b2Cross(cp->rB, P);
			vB = b2MulAdd(vB, mB, P);
		}

		bodyA->linearVelocity = vA;
		bodyA->angularVelocity = wA;
		bodyB->linearVelocity = vB;
		bodyB->angularVelocity = wB;
	}
}

// separation = dot(normal, pB - pA) + separation0
static void b2UpdateSeparation(b2World* world, b2GraphColor* color, float h)
{
	const int32_t constraintCount = b2Array(color->contactArray).count;
	b2Body* bodies = world->bodies;

	for (int32_t i = 0; i < constraintCount; ++i)
	{
		b2Constraint* constraint = color->constraints + i;

		int32_t pointCount = constraint->pointCount;
		B2_ASSERT(0 < pointCount && pointCount <= 2);

		b2Body* bodyA = bodies + constraint->indexA;
		b2Body* bodyB = bodies + constraint->indexB;

		b2Vec2 cA = bodyA->position;
		b2Vec2 cB = bodyB->position;
		b2Rot qA = b2MakeRot(bodyA->angle);
		b2Rot qB = b2MakeRot(bodyB->angle);

		b2Vec2 normal = constraint->normal;

		for (int32_t j = 0; j < pointCount; ++j)
		{
			b2ConstraintPoint* cp = constraint->points + j;

			b2Vec2 rA = b2RotateVector(qA, cp->localAnchorA);
			b2Vec2 rB = b2RotateVector(qB, cp->localAnchorB);

			// Current separation
			b2Vec2 d = b2Add(b2Sub(cB, cA), b2Sub(rB, rA));

			// TODO_ERIN really only need to update bias below
			cp->separation = b2Dot(d, normal) + cp->baseSeparation;

			// Soft contact with speculation
			const float hertz = 10.0f;
			const float zeta = 1.0f;
			float omega = 2.0f * b2_pi * hertz;
			// float d = 2.0f * zeta * omega / kNormal;
			// float k = omega * omega / kNormal;
			cp->bias = (omega / (2.0f * zeta + h * omega)) * cp->separation;
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

		b2Body* bodyA = bodies + constraint->indexA;
		b2Body* bodyB = bodies + constraint->indexB;

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
			float impulse = -cp->normalMass * (vn + cp->bias + cp->gamma * cp->normalImpulse);

			// Clamp the accumulated impulse
			float newImpulse = B2_MAX(cp->normalImpulse + impulse, 0.0f);
			impulse = newImpulse - cp->normalImpulse;
			cp->normalImpulse = newImpulse;

			// Apply contact impulse
			b2Vec2 P = b2MulSV(impulse, normal);
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

static void b2IntegratePositions(b2World* world, float h)
{
	b2Body* bodies = world->bodies;
	int32_t bodyCapacity = world->bodyPool.capacity;

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

		body->position = c;
		body->angle = a;
		body->linearVelocity = v;
		body->angularVelocity = w;
	}
}

static void b2FinalizePositions(b2World* world, float h)
{
	b2Body* bodies = world->bodies;
	int32_t bodyCapacity = world->bodyPool.capacity;
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

// Update body transform, mark broadphase AABB, build awake contact bits
static void b2FinalizeSolve(b2World* world)
{
	b2Body* bodies = world->bodies;
	int32_t bodyCapacity = world->bodyPool.capacity;
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

		// TODO_ERIN legacy
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

void b2SolveGraphPGS(b2World* world, const b2StepContext* stepContext)
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

	int32_t iterationCount = stepContext->velocityIterations;
	float h = stepContext->dt;

	b2IntegrateVelocities(world, h);

	for (int32_t i = 0; i < b2_graphColorCount; ++i)
	{
		b2InitializeConstraintsAndWarmStart(world, colors + i, h);
	}

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

	b2FinalizePositions(world, h);
}

void b2SolveGraphTGS(b2World* world, const b2StepContext* stepContext)
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

	int32_t substepCount = stepContext->velocityIterations;
	float h = stepContext->dt / substepCount;

	for (int32_t i = 0; i < b2_graphColorCount; ++i)
	{
		b2InitializeConstraints(world, colors + i, h);
	}

	for (int32_t substep = 0; substep < substepCount; ++substep)
	{
		b2IntegrateVelocities(world, h);

		for (int32_t i = 0; i < b2_graphColorCount; ++i)
		{
			b2WarmStart(world, colors + i);
			if (substep > 0)
			{
				b2UpdateSeparation(world, colors + i, h);
			}
			b2SolveConstraints(world, colors + i);
		}

		b2IntegratePositions(world, h);
	}

	for (int32_t i = 0; i < b2_graphColorCount; ++i)
	{
		b2StoreImpulses(colors + i);
	}

	b2FinalizeSolve(world);

	b2FreeStackItem(world->stackAllocator, constraints);
}
