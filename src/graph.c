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
#include <stdlib.h>

#define maxBaumgarteVelocity 3.0f

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

			b2SetBitGrow(&color->bodySet, bodyIndexA);
			b2SetBitGrow(&color->bodySet, bodyIndexB);

			contact->colorContactIndex = b2Array(color->contactArray).count;
			b2Array_Push(color->contactArray, contact->object.index);
			contact->colorIndex = i;
			contact->flags &= ~b2_contactStatic;
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

			b2SetBitGrow(&color->bodySet, bodyIndexB);

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

	if (typeA == b2_dynamicBody && typeB == b2_dynamicBody)
	{
		b2GraphColor* color = graph->colors + contact->colorIndex;
		B2_ASSERT(b2GetBit(&color->bodySet, bodyIndexA) && b2GetBit(&color->bodySet, bodyIndexB));
		
		int32_t colorContactIndex = contact->colorContactIndex;
		b2Array_RemoveSwap(color->contactArray, colorContactIndex);
		if (colorContactIndex < b2Array(color->contactArray).count)
		{
			// Fix index on swapped contact
			int32_t swappedContactIndex = color->contactArray[colorContactIndex];
			world->contacts[swappedContactIndex].colorContactIndex = colorContactIndex;
		}

		b2ClearBit(&color->bodySet, bodyIndexA);
		b2ClearBit(&color->bodySet, bodyIndexB);
	}
	else if (typeA == b2_dynamicBody)
	{
		b2GraphColor* color = graph->colors + contact->colorIndex;
		B2_ASSERT(b2GetBit(&color->bodySet, bodyIndexA));

		int32_t colorContactIndex = contact->colorContactIndex;
		b2Array_RemoveSwap(color->contactArray, colorContactIndex);
		if (colorContactIndex < b2Array(color->contactArray).count)
		{
			// Fix index on swapped contact
			int32_t swappedContactIndex = color->contactArray[colorContactIndex];
			world->contacts[swappedContactIndex].colorContactIndex = colorContactIndex;
		}

		b2ClearBit(&color->bodySet, bodyIndexA);
	}
	else if (typeB == b2_dynamicBody)
	{
		b2GraphColor* color = graph->colors + contact->colorIndex;
		B2_ASSERT(b2GetBit(&color->bodySet, bodyIndexB));

		int32_t colorContactIndex = contact->colorContactIndex;
		b2Array_RemoveSwap(color->contactArray, colorContactIndex);
		if (colorContactIndex < b2Array(color->contactArray).count)
		{
			// Fix index on swapped contact
			int32_t swappedContactIndex = color->contactArray[colorContactIndex];
			world->contacts[swappedContactIndex].colorContactIndex = colorContactIndex;
		}

		b2ClearBit(&color->bodySet, bodyIndexB);
	}

	contact->colorIndex = B2_NULL_INDEX;
	contact->colorContactIndex = B2_NULL_INDEX;
	contact->flags &= ~b2_contactStatic;
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

		body->deltaAngle = 0.0f;
		body->deltaPosition = b2Vec2_zero;
	}
}

#if 0 // no need?
static void b2IntegrateVelocitiesSoft(b2World* world, float h)
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

		body->deltaAngle = 0.0f;
		body->deltaPosition = b2Vec2_zero;
	}
}
#endif

static void b2IntegrateDeltaTransform(b2World* world, float h)
{
	b2Body* bodies = world->bodies;
	int32_t bodyCapacity = world->bodyPool.capacity;

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

		body->deltaAngle += h * body->angularVelocity;
		body->deltaPosition = b2MulAdd(body->deltaPosition, h, body->linearVelocity);
		i += 0;
	}
}

static void b2UpdatePositions(b2World* world)
{
	b2Body* bodies = world->bodies;
	int32_t bodyCapacity = world->bodyPool.capacity;

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

		body->position = b2Add(body->position, body->deltaPosition);
		body->angle += body->deltaAngle;
	}
}

typedef struct b2ConstraintPoint
{
	b2Vec2 rA, rB;
	b2Vec2 rAf, rBf;
	b2Vec2 localAnchorA, localAnchorB;
	float tangentSeparation;
	float separation;
	float normalImpulse;
	float tangentImpulse;
	float normalMass;
	float tangentMass;
	float gamma;
	float massCoefficient;
	float biasCoefficient;
	float impulseCoefficient;
	float baumgarte;
	bool frictionValid;
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

static void b2InitializeSoftConstraints(b2World* world, b2GraphColor* color, float h, bool warmStart)
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
			const float hertz = 30.0f;
			const float zeta = 1.0f;
			float omega = 2.0f * b2_pi * hertz;
			// float d = 2.0f * zeta * omega / kNormal;
			// float k = omega * omega / kNormal;

			// cp->gamma = 1.0f / (h * (d + h * k));
			// cp->gamma = 1.0f / (h * (2.0f * zeta * omega / kNormal + h * omega * omega / kNormal));
			cp->gamma = kNormal / (h * omega * (2.0f * zeta + h * omega));

			cp->separation = mp->separation;

			// cp->bias = h * k * cp->gamma * mp->separation;
			// cp->bias = k / (d + h * k) * mp->separation;
			// cp->bias =
			//	(omega * omega / kNormal) / (2 * zeta * omega / kNormal + h * omega * omega / kNormal) * mp->separation;
			cp->biasCoefficient = omega / (2.0f * zeta + h * omega);
			// cp->gamma = 0.0f;
			// cp->bias = (0.2f / h) * mp->separation;

			// TODO_ERIN this can be expanded
			cp->normalMass = kNormal > 0.0f ? 1.0f / kNormal : 0.0f;
			//cp->normalMass = 1.0f / (kNormal + cp->gamma);

			float c = h * omega * (2.0f * zeta + h * omega);
			cp->impulseCoefficient = 1.0f / (1.0f + c);
			cp->massCoefficient = c * cp->impulseCoefficient;

			// meff = 1.0f / kNormal * 1.0f / (1.0f + 1.0f / (h * omega * (2 * zeta + h * omega)))
			// float impulse = -cp->normalMass * (vn + bias + cp->gamma * cp->normalImpulse);
			// = -meff * mscale * (vn + bias) - imp_scale * impulse

			// Warm start
			if (warmStart)
			{
				b2Vec2 P = b2Add(b2MulSV(cp->normalImpulse, normal), b2MulSV(cp->tangentImpulse, tangent));
				wA -= iA * b2Cross(cp->rA, P);
				vA = b2MulAdd(vA, -mA, P);
				wB += iB * b2Cross(cp->rB, P);
				vB = b2MulAdd(vB, mB, P);
			}
		}

		bodyA->linearVelocity = vA;
		bodyA->angularVelocity = wA;
		bodyB->linearVelocity = vB;
		bodyB->angularVelocity = wB;
	}
}

static void b2InitializePGSConstraints(b2World* world, b2GraphColor* color)
{
	const int32_t constraintCount = b2Array(color->contactArray).count;
	int32_t* contactIndices = color->contactArray;
	b2Contact* contacts = world->contacts;
	b2Body* bodies = world->bodies;

	for (int32_t i = 0; i < constraintCount; ++i)
	{
		b2Contact* contact = contacts + contactIndices[i];
		b2Manifold* manifold = &contact->manifold;

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
		b2Vec2 tangent = b2RightPerp(normal);

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
			cp->separation = mp->separation;

			cp->baumgarte = 0.0f;
			cp->biasCoefficient = mp->separation > 0.0f ? 1.0f : 0.0f;

			float rtA = b2Cross(cp->rA, tangent);
			float rtB = b2Cross(cp->rB, tangent);
			float kTangent = mA + mB + iA * rtA * rtA + iB * rtB * rtB;
			cp->tangentMass = kTangent > 0.0f ? 1.0f / kTangent : 0.0f;

			float rnA = b2Cross(cp->rA, normal);
			float rnB = b2Cross(cp->rB, normal);
			float kNormal = mA + mB + iA * rnA * rnA + iB * rnB * rnB;
			cp->normalMass = kNormal > 0.0f ? 1.0f / kNormal : 0.0f;
		}
	}
}

static void b2InitializeStickyConstraints(b2World* world, b2GraphColor* color)
{
	const int32_t constraintCount = b2Array(color->contactArray).count;
	int32_t* contactIndices = color->contactArray;
	b2Contact* contacts = world->contacts;
	b2Body* bodies = world->bodies;

	for (int32_t i = 0; i < constraintCount; ++i)
	{
		b2Contact* contact = contacts + contactIndices[i];
		b2Manifold* manifold = &contact->manifold;

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
		b2Vec2 tangent = b2RightPerp(normal);

		for (int32_t j = 0; j < pointCount; ++j)
		{
			const b2ManifoldPoint* mp = manifold->points + j;
			b2ConstraintPoint* cp = constraint->points + j;

			cp->normalImpulse = 0.0f;
			cp->tangentImpulse = 0.0f;

			cp->rA = b2Sub(mp->point, cA);
			cp->rB = b2Sub(mp->point, cB);
			cp->localAnchorA = b2InvRotateVector(qA, cp->rA);
			cp->localAnchorB = b2InvRotateVector(qB, cp->rB);
			cp->separation = mp->separation;

			cp->baumgarte = 0.8f;

			float rtA = b2Cross(cp->rA, tangent);
			float rtB = b2Cross(cp->rB, tangent);
			float kTangent = mA + mB + iA * rtA * rtA + iB * rtB * rtB;
			cp->tangentMass = kTangent > 0.0f ? 1.0f / kTangent : 0.0f;

			float rnA = b2Cross(cp->rA, normal);
			float rnB = b2Cross(cp->rB, normal);
			float kNormal = mA + mB + iA * rnA * rnA + iB * rnB * rnB;
			cp->normalMass = kNormal > 0.0f ? 1.0f / kNormal : 0.0f;
		}

		bool frictionConfirmed = false;
		if (manifold->frictionPersisted)
		{
			int32_t confirmCount = 0;
			for (int32_t j = 0; j < pointCount; ++j)
			{
				const b2ManifoldPoint* mp = manifold->points + j;
				b2ConstraintPoint* cp = constraint->points + j;

				b2Vec2 normalA = b2RotateVector(qA, mp->localNormalA);
				b2Vec2 normalB = b2RotateVector(qB, mp->localNormalB);

				float nn = b2Dot(normalA, normalB);
				if (nn < 0.98f)
				{
					// Relative rotation has invalidated cached friction anchors
					break;
				}

				b2Vec2 anchorA = b2RotateVector(qA, mp->localAnchorA);
				b2Vec2 anchorB = b2RotateVector(qB, mp->localAnchorB);
				b2Vec2 offset = b2Add(b2Sub(cB, cA), b2Sub(anchorB, anchorA));
				float normalSeparation = b2Dot(offset, normalA);
				if (B2_ABS(normalSeparation) > 2.0f * b2_linearSlop)
				{
					// Normal separation has invalidated cached friction anchors
					break;
				}

				cp->rAf = anchorA;
				cp->rBf = anchorB;
				cp->tangentSeparation = b2Dot(offset, tangent);

				float rtA = b2Cross(anchorA, tangent);
				float rtB = b2Cross(anchorB, tangent);
				float kTangent = mA + mB + iA * rtA * rtA + iB * rtB * rtB;
				cp->tangentMass = kTangent > 0.0f ? 1.0f / kTangent : 0.0f;

				confirmCount += 1;
			}

			if (confirmCount == pointCount)
			{
				frictionConfirmed = true;
			}
		}

		if (frictionConfirmed == false)
		{
			for (int32_t j = 0; j < pointCount; ++j)
			{
				b2ManifoldPoint* mp = manifold->points + j;
				b2ConstraintPoint* cp = constraint->points + j;

				mp->localNormalA = b2InvRotateVector(qA, normal);
				mp->localNormalB = b2InvRotateVector(qB, normal);
				mp->localAnchorA = b2InvRotateVector(qA, cp->rA);
				mp->localAnchorB = b2InvRotateVector(qB, cp->rB);

				cp->rAf = cp->rA;
				cp->rBf = cp->rB;
				cp->tangentSeparation = 0.0f;

				float rtA = b2Cross(cp->rAf, tangent);
				float rtB = b2Cross(cp->rBf, tangent);
				float kTangent = mA + mB + iA * rtA * rtA + iB * rtB * rtB;
				cp->tangentMass = kTangent > 0.0f ? 1.0f / kTangent : 0.0f;
			}
		}

		manifold->frictionPersisted = true;
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
		b2Vec2 tangent = b2RightPerp(normal);

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

static void b2WarmStartAll(b2World* world, b2Constraint* constraints, int32_t constraintCount)
{
	b2Body* bodies = world->bodies;

	for (int32_t i = 0; i < constraintCount; ++i)
	{
		b2Constraint* constraint = constraints + i;

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
		b2Vec2 tangent = b2RightPerp(normal);

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

static void b2SolveVelocityConstraints(b2World* world, b2GraphColor* color, float inv_dt)
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
		b2Vec2 tangent = b2CrossVS(normal, 1.0f);
		float friction = constraint->friction;

		for (int32_t j = 0; j < pointCount; ++j)
		{
			b2ConstraintPoint* cp = constraint->points + j;

			// Relative velocity at contact
			b2Vec2 vrB = b2Add(vB, b2CrossSV(wB, cp->rB));
			b2Vec2 vrA = b2Add(vA, b2CrossSV(wA, cp->rA));
			b2Vec2 dv = b2Sub(vrB, vrA);

			// Compute normal impulse
			float vn = b2Dot(dv, normal);
			float impulse = -cp->normalMass * (vn + cp->biasCoefficient * cp->separation * inv_dt);

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

		bodyA->linearVelocity = vA;
		bodyA->angularVelocity = wA;
		bodyB->linearVelocity = vB;
		bodyB->angularVelocity = wB;
	}
}

static void b2SolveVelocityConstraintsSorted(b2World* world, b2Constraint* constraints, int32_t constraintCount, float inv_dt)
{
	b2Body* bodies = world->bodies;

	for (int32_t i = 0; i < constraintCount; ++i)
	{
		b2Constraint* constraint = constraints + i;

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
		b2Vec2 tangent = b2CrossVS(normal, 1.0f);
		float friction = constraint->friction;

		for (int32_t j = 0; j < pointCount; ++j)
		{
			b2ConstraintPoint* cp = constraint->points + j;

			// Relative velocity at contact
			b2Vec2 vrB = b2Add(vB, b2CrossSV(wB, cp->rB));
			b2Vec2 vrA = b2Add(vA, b2CrossSV(wA, cp->rA));
			b2Vec2 dv = b2Sub(vrB, vrA);

			// Compute normal impulse
			float vn = b2Dot(dv, normal);
			float impulse = -cp->normalMass * (vn + cp->biasCoefficient * cp->separation * inv_dt);

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

		bodyA->linearVelocity = vA;
		bodyA->angularVelocity = wA;
		bodyB->linearVelocity = vB;
		bodyB->angularVelocity = wB;
	}
}

static void b2SolveVelocityConstraintsSoft(b2World* world, b2GraphColor* color, float inv_dt, bool removeOverlap)
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

		const b2Vec2 dpA = bodyA->deltaPosition;
		const float daA = bodyA->deltaAngle;
		const b2Vec2 dpB = bodyB->deltaPosition;
		const float daB = bodyB->deltaAngle;

		b2Vec2 normal = constraint->normal;
		b2Vec2 tangent = b2RightPerp(normal);
		float friction = constraint->friction;

		for (int32_t j = 0; j < pointCount; ++j)
		{
			b2ConstraintPoint* cp = constraint->points + j;

			// Relative velocity at contact
			b2Vec2 vrB = b2Add(vB, b2CrossSV(wB, cp->rB));
			b2Vec2 vrA = b2Add(vA, b2CrossSV(wA, cp->rA));
			b2Vec2 dv = b2Sub(vrB, vrA);

			// Compute change in separation
			b2Vec2 prB = b2Add(dpB, b2CrossSV(daB, cp->rB));
			b2Vec2 prA = b2Add(dpA, b2CrossSV(daA, cp->rA));
			float ds = b2Dot(b2Sub(prB, prA), normal);
			float s = cp->separation + ds;
			float bias = 0.0f;
			float massScale = 1.0f;
			float impulseScale = 0.0f;
			if (s > 0.0f)
			{
				// Speculative
				bias = s * inv_dt;
			}
			else if (removeOverlap)
			{
				bias = B2_MAX(cp->biasCoefficient * s, -maxBaumgarteVelocity);
				//bias = cp->biasCoefficient * s;
				massScale = cp->massCoefficient;
				impulseScale = cp->impulseCoefficient;
			}
			
			// Compute normal impulse
			float vn = b2Dot(dv, normal);
			float impulse = -cp->normalMass * massScale * (vn + bias) - impulseScale * cp->normalImpulse;
			//float impulse = -cp->normalMass * (vn + bias + cp->gamma * cp->normalImpulse);

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

		bodyA->linearVelocity = vA;
		bodyA->angularVelocity = wA;
		bodyB->linearVelocity = vB;
		bodyB->angularVelocity = wB;
	}
}

static void b2SolveVelocityConstraintsSticky(b2World* world, b2GraphColor* color, float minSeparation, float invh)
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

		const b2Vec2 dpA = bodyA->deltaPosition;
		const float daA = bodyA->deltaAngle;
		const b2Vec2 dpB = bodyB->deltaPosition;
		const float daB = bodyB->deltaAngle;

		b2Vec2 normal = constraint->normal;
		b2Vec2 tangent = b2RightPerp(normal);
		float friction = 0.3f; //constraint->friction;

		float totalNormalImpulse = 0.0f;

		// Non-penetration constraints
		for (int32_t j = 0; j < pointCount; ++j)
		{
			b2ConstraintPoint* cp = constraint->points + j;

			// Relative velocity at contact
			b2Vec2 vrB = b2Add(vB, b2CrossSV(wB, cp->rB));
			b2Vec2 vrA = b2Add(vA, b2CrossSV(wA, cp->rA));
			b2Vec2 dv = b2Sub(vrB, vrA);

			// Compute change in separation
			b2Vec2 prB = b2Add(dpB, b2CrossSV(daB, cp->rB));
			b2Vec2 prA = b2Add(dpA, b2CrossSV(daA, cp->rA));
			float ds = b2Dot(b2Sub(prB, prA), normal);
			float s = cp->separation + ds;

			float bias = 0.0f;
			if (s > 0.0f)
			{
				// Speculative
				bias = s * invh;
			
			}
			else if (minSeparation < 0.0f)
			{
				bias = B2_MAX(-maxBaumgarteVelocity, cp->baumgarte * s * invh);
			}

			// Compute normal impulse
			float vn = b2Dot(dv, normal);
			float impulse = -cp->normalMass * (vn + bias);

			// Clamp the accumulated impulse
			float newImpulse = B2_MAX(cp->normalImpulse + impulse, 0.0f);
			impulse = newImpulse - cp->normalImpulse;
			cp->normalImpulse = newImpulse;

			totalNormalImpulse += cp->normalImpulse;

			// Apply contact impulse
			b2Vec2 P = b2MulSV(impulse, normal);
			vA = b2MulSub(vA, mA, P);
			wA -= iA * b2Cross(cp->rA, P);

			vB = b2MulAdd(vB, mB, P);
			wB += iB * b2Cross(cp->rB, P);
		}

		// Sticky friction constraints
		for (int32_t j = 0; j < pointCount; ++j)
		{
			b2ConstraintPoint* cp = constraint->points + j;

			// Relative velocity at contact
			b2Vec2 vrB = b2Add(vB, b2CrossSV(wB, cp->rBf));
			b2Vec2 vrA = b2Add(vA, b2CrossSV(wA, cp->rAf));
			b2Vec2 dv = b2Sub(vrB, vrA);

			// Compute change in separation
			b2Vec2 prB = b2Add(dpB, b2CrossSV(daB, cp->rBf));
			b2Vec2 prA = b2Add(dpA, b2CrossSV(daA, cp->rAf));
			float ds = b2Dot(b2Sub(prB, prA), tangent);
			float s = cp->tangentSeparation + ds;
			float bias = 0.5f * s * invh;

			// Compute tangent impulse
			float vt = b2Dot(dv, tangent);
			float impulse = -cp->tangentMass * (vt + bias);

			// max friction uses an average of the total normal impulse because persistent friction anchors don't line up with normal anchors
			float maxFriction = 0.5f * friction * totalNormalImpulse;

			// Clamp the accumulated impulse
			float newImpulse = cp->tangentImpulse + impulse;
			if (newImpulse < -maxFriction)
			{
				newImpulse = -maxFriction;
				constraint->contact->manifold.frictionPersisted = false;
			}
			else if (newImpulse > maxFriction)
			{
				newImpulse = maxFriction;
				constraint->contact->manifold.frictionPersisted = false;
			}

			impulse = newImpulse - cp->tangentImpulse;
			cp->tangentImpulse = newImpulse;

			// Apply contact impulse
			b2Vec2 P = b2MulSV(impulse, tangent);

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

static void b2StoreImpulses(b2Constraint* constraints, int32_t constraintCount)
{
	for (int32_t i = 0; i < constraintCount; ++i)
	{
		b2Constraint* constraint = constraints + i;
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

static void b2SolvePositionConstraints(b2World* world, b2GraphColor* color)
{
	const int32_t constraintCount = b2Array(color->contactArray).count;
	b2Body* bodies = world->bodies;
	float slop = b2_linearSlop;

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

		b2Vec2 cA = bodyA->position;
		float aA = bodyA->angle;
		b2Vec2 cB = bodyB->position;
		float aB = bodyB->angle;

		b2Vec2 normal = constraint->normal;

		for (int32_t j = 0; j < pointCount; ++j)
		{
			b2ConstraintPoint* cp = constraint->points + j;

			b2Rot qA = b2MakeRot(aA);
			b2Rot qB = b2MakeRot(aB);

			b2Vec2 rA = b2RotateVector(qA, cp->localAnchorA);
			b2Vec2 rB = b2RotateVector(qB, cp->localAnchorB);

			// Current separation
			b2Vec2 d = b2Sub(b2Add(cB, rB), b2Add(cA, rA));
			float separation = b2Dot(d, normal) + cp->separation;

			// Prevent large corrections. Need to maintain a small overlap to avoid overshoot.
			// This improves stacking stability significantly.
			float C = B2_CLAMP(b2_baumgarte * (separation + slop), -b2_maxLinearCorrection, 0.0f);

			// Compute the effective mass.
			float rnA = b2Cross(rA, normal);
			float rnB = b2Cross(rB, normal);
			float K = mA + mB + iA * rnA * rnA + iB * rnB * rnB;

			// Compute normal impulse
			float impulse = K > 0.0f ? -C / K : 0.0f;

			b2Vec2 P = b2MulSV(impulse, normal);

			cA = b2MulSub(cA, mA, P);
			aA -= iA * b2Cross(cp->rA, P);

			cB = b2MulAdd(cB, mB, P);
			aB += iB * b2Cross(cp->rB, P);
		}

		bodyA->position = cA;
		bodyA->angle = aA;
		bodyB->position = cB;
		bodyB->angle = aB;
	}
}

static void b2SolvePositionConstraintsSorted(b2World* world, b2Constraint* constraints, int32_t constraintCount)
{
	b2Body* bodies = world->bodies;

	for (int32_t i = 0; i < constraintCount; ++i)
	{
		b2Constraint* constraint = constraints + i;

		b2Body* bodyA = bodies + constraint->indexA;
		b2Body* bodyB = bodies + constraint->indexB;

		float mA = bodyA->invMass;
		float iA = bodyA->invI;
		float mB = bodyB->invMass;
		float iB = bodyB->invI;
		int32_t pointCount = constraint->pointCount;

		b2Vec2 cA = bodyA->position;
		float aA = bodyA->angle;
		b2Vec2 cB = bodyB->position;
		float aB = bodyB->angle;

		b2Vec2 normal = constraint->normal;
		float slop = b2_linearSlop;

		for (int32_t j = 0; j < pointCount; ++j)
		{
			b2ConstraintPoint* cp = constraint->points + j;

			b2Rot qA = b2MakeRot(aA);
			b2Rot qB = b2MakeRot(aB);

			b2Vec2 rA = b2RotateVector(qA, cp->localAnchorA);
			b2Vec2 rB = b2RotateVector(qB, cp->localAnchorB);

			// Current separation
			b2Vec2 d = b2Sub(b2Add(cB, rB), b2Add(cA, rA));
			float separation = b2Dot(d, normal) + cp->separation;

			// Prevent large corrections. Need to maintain a small overlap to avoid overshoot.
			// This improves stacking stability significantly.
			float C = B2_CLAMP(b2_baumgarte * (separation + slop), -b2_maxLinearCorrection, 0.0f);

			// Compute the effective mass.
			float rnA = b2Cross(rA, normal);
			float rnB = b2Cross(rB, normal);
			float K = mA + mB + iA * rnA * rnA + iB * rnB * rnB;

			// Compute normal impulse
			float impulse = K > 0.0f ? -C / K : 0.0f;

			b2Vec2 P = b2MulSV(impulse, normal);

			cA = b2MulSub(cA, mA, P);
			aA -= iA * b2Cross(cp->rA, P);

			cB = b2MulAdd(cB, mB, P);
			aB += iB * b2Cross(cp->rB, P);
		}

		bodyA->position = cA;
		bodyA->angle = aA;
		bodyB->position = cB;
		bodyB->angle = aB;
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

int b2CompareConstraints(const void* ptr1, const void* ptr2)
{
	const b2Constraint* c1 = ptr1;
	const b2Constraint* c2 = ptr2;

	b2Vec2 point1 = c1->contact->manifold.points[0].point;
	b2Vec2 point2 = c2->contact->manifold.points[0].point;

	if (B2_ABS(point1.y - point2.y) > 5.0f * b2_linearSlop)
	{
		if (point1.y < point2.y)
		{
			return 1;
		}

		return -1;
	}
	else if (point1.x < point2.x)
	{
		return -1;
	}

	return 1;
}

int b2RandomizeConstraints(const void* ptr1, const void* ptr2)
{
	B2_MAYBE_UNUSED(ptr1);
	B2_MAYBE_UNUSED(ptr2);

	if (rand() & 1)
	{
		return -1;
	}

	return 1;
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

	int32_t velocityIterations = stepContext->velocityIterations;
	int32_t positionIterations = stepContext->positionIterations;
	float h = stepContext->dt;
	float inv_h = stepContext->inv_dt;

	b2IntegrateVelocities(world, h);

	for (int32_t i = 0; i < b2_graphColorCount; ++i)
	{
		b2InitializePGSConstraints(world, colors + i);
	}

	b2WarmStartAll(world, constraints, constraintCount);

	for (int32_t i = 0; i < constraintCount; ++i)
	{
		constraints[i].contact->manifold.constraintIndex = i;
	}

	for (int32_t iter = 0; iter < velocityIterations; ++iter)
	{
		for (int32_t i = 0; i < b2_graphColorCount; ++i)
		{
			b2SolveVelocityConstraints(world, colors + i, inv_h);
		}
	}

	b2StoreImpulses(constraints, constraintCount);

	b2IntegratePositions(world, h);

	for (int32_t iter = 0; iter < positionIterations; ++iter)
	{
		for (int32_t i = 0; i < b2_graphColorCount; ++i)
		{
			b2SolvePositionConstraints(world, colors + i);
		}
	}

	b2FinalizeSolve(world);
	
	b2FreeStackItem(world->stackAllocator, constraints);
}

void b2SolveGraphSoftPGS(b2World* world, const b2StepContext* stepContext)
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

	int32_t velocityIterations = stepContext->velocityIterations;
	int32_t positionIterations = stepContext->positionIterations;
	float h = stepContext->dt;

	b2IntegrateVelocities(world, h);

	for (int32_t i = 0; i < b2_graphColorCount; ++i)
	{
		b2InitializeSoftConstraints(world, colors + i, h, true);
	}

	for (int32_t iter = 0; iter < velocityIterations; ++iter)
	{
		for (int32_t i = 0; i < b2_graphColorCount; ++i)
		{
			b2SolveVelocityConstraintsSoft(world, colors + i, stepContext->inv_dt, true);
		}
	}
	
	b2IntegratePositions(world, h);

	for (int32_t iter = 0; iter < positionIterations; ++iter)
	{
		for (int32_t i = 0; i < b2_graphColorCount; ++i)
		{
			b2SolveVelocityConstraintsSoft(world, colors + i, stepContext->inv_dt, false);
		}
	}

	b2StoreImpulses(constraints, constraintCount);

	b2FinalizeSolve(world);

	b2FreeStackItem(world->stackAllocator, constraints);
}

void b2SolveGraphSoftTGS(b2World* world, const b2StepContext* stepContext)
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

	// Full step apply gravity
	b2IntegrateVelocities(world, stepContext->dt);

	for (int32_t i = 0; i < b2_graphColorCount; ++i)
	{
		bool warmStart = true;
		b2InitializeSoftConstraints(world, colors + i, stepContext->dt, warmStart);
	}
	
	int32_t substepCount = stepContext->velocityIterations;
	float h = stepContext->dt / substepCount;
	float inv_h = 1.0f / h;

	for (int32_t substep = 0; substep < substepCount; ++substep)
	{
		// One constraint iteration
		for (int32_t i = 0; i < b2_graphColorCount; ++i)
		{
			bool removeOverlap = true;
			b2SolveVelocityConstraintsSoft(world, colors + i, inv_h, removeOverlap);
		}

		b2IntegrateDeltaTransform(world, h);
	}

	b2UpdatePositions(world);

	int32_t positionIterations = stepContext->positionIterations;
	for (int32_t iter = 0; iter < positionIterations; ++iter)
	{
		for (int32_t i = 0; i < b2_graphColorCount; ++i)
		{
			bool removeOverlap = false;
			b2SolveVelocityConstraintsSoft(world, colors + i, 0.0f, removeOverlap);
		}
	}

	b2StoreImpulses(constraints, constraintCount);

	b2FinalizeSolve(world);

	b2FreeStackItem(world->stackAllocator, constraints);
}

// Sticky
void b2SolveGraphStickyTGS(b2World* world, const b2StepContext* stepContext)
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

	b2IntegrateVelocities(world, stepContext->dt);

	for (int32_t i = 0; i < b2_graphColorCount; ++i)
	{
		b2InitializeStickyConstraints(world, colors + i);
	}

	int32_t substepCount = stepContext->velocityIterations;
	float h = stepContext->dt / substepCount;
	float invh = substepCount / stepContext->dt;

	for (int32_t substep = 0; substep < substepCount; ++substep)
	{
		// One constraint iteration
		for (int32_t i = 0; i < b2_graphColorCount; ++i)
		{
			b2SolveVelocityConstraintsSticky(world, colors + i, -b2_huge, invh);
		}

		b2IntegrateDeltaTransform(world, h);
	}

	b2UpdatePositions(world);

	int32_t positionIterations = stepContext->positionIterations;
	for (int32_t iter = 0; iter < positionIterations; ++iter)
	{
		// Solve with no baumgarte and no affect on position
		for (int32_t i = 0; i < b2_graphColorCount; ++i)
		{
			b2SolveVelocityConstraintsSticky(world, colors + i, 0.0f, 0.0f);
		}
	}

	b2FinalizeSolve(world);

	b2FreeStackItem(world->stackAllocator, constraints);
}
