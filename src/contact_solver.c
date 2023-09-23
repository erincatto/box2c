// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "contact_solver.h"

#include "array.h"
#include "body.h"
#include "contact.h"
#include "core.h"
#include "graph.h"
#include "world.h"

#define maxBaumgarteVelocity 3.0f

// TODO_ERIN prepare contact constraints directly in collision phase?
void b2PrepareContactsTask(int32_t startIndex, int32_t endIndex, b2SolverTaskContext* context, int32_t colorIndex)
{
	b2TracyCZoneNC(prepare_contact, "Prepare Contact", b2_colorYellow, true);

	b2World* world = context->world;
	b2Graph* graph = context->graph;
	b2GraphColor* color = graph->colors + colorIndex;
	int32_t* contactIndices = color->contactArray;
	b2Contact* contacts = world->contacts;
	const int32_t* bodyMap = context->bodyMap;
	b2SolverBody* solverBodies = context->solverBodies;

	// 30 is a bit soft, 60 oscillates too much
	// const float contactHertz = 45.0f;
	// const float contactHertz = B2_MAX(15.0f, stepContext->inv_dt * stepContext->velocityIterations / 8.0f);
	const float contactHertz = 30.0f;

	float h = context->timeStep;
	bool enableWarmStarting = world->enableWarmStarting;

	B2_ASSERT(startIndex <= b2Array(color->contactArray).count);
	B2_ASSERT(endIndex <= b2Array(color->contactArray).count);

	for (int32_t i = startIndex; i < endIndex; ++i)
	{
		b2Contact* contact = contacts + contactIndices[i];

		const b2Manifold* manifold = &contact->manifold;
		int32_t pointCount = manifold->pointCount;

		B2_ASSERT(0 < pointCount && pointCount <= 2);

		int32_t indexA = bodyMap[contact->edges[0].bodyIndex];
		int32_t indexB = bodyMap[contact->edges[1].bodyIndex];

		b2ContactConstraint* constraint = color->contactConstraints + i;
		constraint->contact = contact;
		constraint->indexA = indexA;
		constraint->indexB = indexB;
		constraint->normal = manifold->normal;
		constraint->friction = contact->friction;

		b2SolverBody* solverBodyA;
		b2Vec2 vA;
		float wA;
		float mA;
		float iA;

		float hertz;

		if (indexA == B2_NULL_INDEX)
		{
			solverBodyA = NULL;
			vA.x = vA.y = 0.0;
			wA = 0.0f;
			mA = 0.0f;
			iA = 0.0f;
			hertz = 2.0f * contactHertz;
			constraint->type = pointCount == 1 ? b2_onePointStaticType : b2_twoPointStaticType;
		}
		else
		{
			solverBodyA = solverBodies + indexA;
			vA = solverBodyA->linearVelocity;
			wA = solverBodyA->angularVelocity;
			mA = solverBodyA->invMass;
			iA = solverBodyA->invI;
			hertz = contactHertz;
			constraint->type = pointCount == 1 ? b2_onePointType : b2_twoPointType;
		}

		B2_ASSERT(indexB != B2_NULL_INDEX);
		b2SolverBody* solverBodyB = solverBodies + indexB;
		b2Vec2 vB = solverBodyB->linearVelocity;
		float wB = solverBodyB->angularVelocity;
		float mB = solverBodyB->invMass;
		float iB = solverBodyB->invI;

		// Stiffer for static contacts to avoid bodies getting pushed through the ground
		const float zeta = 1.0f;
		float omega = 2.0f * b2_pi * hertz;
		float c = h * omega * (2.0f * zeta + h * omega);
		constraint->impulseCoefficient = 1.0f / (1.0f + c);
		constraint->massCoefficient = c * constraint->impulseCoefficient;
		constraint->biasCoefficient = omega / (2.0f * zeta + h * omega);

		b2Vec2 normal = constraint->normal;
		b2Vec2 tangent = b2RightPerp(constraint->normal);

		for (int32_t j = 0; j < pointCount; ++j)
		{
			const b2ManifoldPoint* mp = manifold->points + j;
			b2ContactConstraintPoint* cp = constraint->points + j;

			cp->normalImpulse = mp->normalImpulse;
			cp->tangentImpulse = mp->tangentImpulse;

			cp->rA = mp->anchorA;
			cp->rB = mp->anchorB;

			float rnA = b2Cross(cp->rA, normal);
			float rnB = b2Cross(cp->rB, normal);
			float kNormal = mA + mB + iA * rnA * rnA + iB * rnB * rnB;

			float rtA = b2Cross(cp->rA, tangent);
			float rtB = b2Cross(cp->rB, tangent);
			float kTangent = mA + mB + iA * rtA * rtA + iB * rtB * rtB;

			cp->tangentMass = kTangent > 0.0f ? 1.0f / kTangent : 0.0f;
			cp->separation = mp->separation;
			cp->normalMass = kNormal > 0.0f ? 1.0f / kNormal : 0.0f;

			// Warm start
			if (enableWarmStarting)
			{
				b2Vec2 P = b2Add(b2MulSV(cp->normalImpulse, normal), b2MulSV(cp->tangentImpulse, tangent));
				wA -= iA * b2Cross(cp->rA, P);
				vA = b2MulAdd(vA, -mA, P);
				wB += iB * b2Cross(cp->rB, P);
				vB = b2MulAdd(vB, mB, P);
			}
		}

		if (solverBodyA != NULL)
		{
			solverBodyA->linearVelocity = vA;
			solverBodyA->angularVelocity = wA;
		}
		solverBodyB->linearVelocity = vB;
		solverBodyB->angularVelocity = wB;
	}

	b2TracyCZoneEnd(prepare_contact);
}

static void b2SolveContactOnePoint(b2ContactConstraint* constraint, b2SolverBody* bodies, float inv_dt, bool useBias)
{
	b2SolverBody* bodyA = bodies + constraint->indexA;
	b2Vec2 vA = bodyA->linearVelocity;
	float wA = bodyA->angularVelocity;
	b2Vec2 dpA = bodyA->deltaPosition;
	float daA = bodyA->deltaAngle;
	float mA = bodyA->invMass;
	float iA = bodyA->invI;

	b2SolverBody* bodyB = bodies + constraint->indexB;
	b2Vec2 vB = bodyB->linearVelocity;
	float wB = bodyB->angularVelocity;
	b2Vec2 dpB = bodyB->deltaPosition;
	float daB = bodyB->deltaAngle;
	float mB = bodyB->invMass;
	float iB = bodyB->invI;

	b2Vec2 normal = constraint->normal;
	b2Vec2 tangent = b2RightPerp(normal);
	float friction = constraint->friction;
	float biasCoefficient = constraint->biasCoefficient;
	float massCoefficient = constraint->massCoefficient;
	float impulseCoefficient = constraint->impulseCoefficient;

	{
		b2ContactConstraintPoint* cp = constraint->points + 0;

		// Relative velocity at contact
		b2Vec2 vrB = b2Add(vB, b2CrossSV(wB, cp->rB));
		b2Vec2 vrA = b2Add(vA, b2CrossSV(wA, cp->rA));
		b2Vec2 dv = b2Sub(vrB, vrA);

		// Compute change in separation (small angle approximation of sin(angle) == angle)
		b2Vec2 prB = b2Add(dpB, b2CrossSV(daB, cp->rB));
		b2Vec2 prA = b2Add(dpA, b2CrossSV(daA, cp->rA));
		float ds = b2Dot(b2Sub(prB, prA), normal);
		float s = cp->separation + ds;
		float bias = 0.0f;
		float massScale = 1.0f;
		float impulseScale = 0.0f;
		if (s > 0.0f)
		{
			// TODO_ERIN what time to use?
			// Speculative (inverse of full time step)
			bias = s * inv_dt;
		}
		else if (useBias)
		{
			bias = B2_MAX(biasCoefficient * s, -maxBaumgarteVelocity);
			// bias = cp->biasCoefficient * s;
			massScale = massCoefficient;
			impulseScale = impulseCoefficient;
		}

		// Compute normal impulse
		float vn = b2Dot(dv, normal);
		float impulse = -cp->normalMass * massScale * (vn + bias) - impulseScale * cp->normalImpulse;
		// float impulse = -cp->normalMass * (vn + bias + cp->gamma * cp->normalImpulse);

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

	{
		b2ContactConstraintPoint* cp = constraint->points + 0;

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

static void b2SolveContactTwoPoints(b2ContactConstraint* constraint, b2SolverBody* bodies, float inv_dt, bool useBias)
{
	b2SolverBody* bodyA = bodies + constraint->indexA;
	b2Vec2 vA = bodyA->linearVelocity;
	float wA = bodyA->angularVelocity;
	b2Vec2 dpA = bodyA->deltaPosition;
	float daA = bodyA->deltaAngle;
	float mA = bodyA->invMass;
	float iA = bodyA->invI;

	b2SolverBody* bodyB = bodies + constraint->indexB;
	b2Vec2 vB = bodyB->linearVelocity;
	float wB = bodyB->angularVelocity;
	b2Vec2 dpB = bodyB->deltaPosition;
	float daB = bodyB->deltaAngle;
	float mB = bodyB->invMass;
	float iB = bodyB->invI;

	b2Vec2 normal = constraint->normal;
	b2Vec2 tangent = b2RightPerp(normal);
	float friction = constraint->friction;
	float biasCoefficient = constraint->biasCoefficient;
	float massCoefficient = constraint->massCoefficient;
	float impulseCoefficient = constraint->impulseCoefficient;

	{
		b2ContactConstraintPoint* cp = constraint->points + 0;

		// Relative velocity at contact
		b2Vec2 vrB = b2Add(vB, b2CrossSV(wB, cp->rB));
		b2Vec2 vrA = b2Add(vA, b2CrossSV(wA, cp->rA));
		b2Vec2 dv = b2Sub(vrB, vrA);

		// Compute change in separation (small angle approximation of sin(angle) == angle)
		b2Vec2 prB = b2Add(dpB, b2CrossSV(daB, cp->rB));
		b2Vec2 prA = b2Add(dpA, b2CrossSV(daA, cp->rA));
		float ds = b2Dot(b2Sub(prB, prA), normal);
		float s = cp->separation + ds;
		float bias = 0.0f;
		float massScale = 1.0f;
		float impulseScale = 0.0f;
		if (s > 0.0f)
		{
			// TODO_ERIN what time to use?
			// Speculative (inverse of full time step)
			bias = s * inv_dt;
		}
		else if (useBias)
		{
			bias = B2_MAX(biasCoefficient * s, -maxBaumgarteVelocity);
			// bias = cp->biasCoefficient * s;
			massScale = massCoefficient;
			impulseScale = impulseCoefficient;
		}

		// Compute normal impulse
		float vn = b2Dot(dv, normal);
		float impulse = -cp->normalMass * massScale * (vn + bias) - impulseScale * cp->normalImpulse;
		// float impulse = -cp->normalMass * (vn + bias + cp->gamma * cp->normalImpulse);

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

	{
		b2ContactConstraintPoint* cp = constraint->points + 1;

		// Relative velocity at contact
		b2Vec2 vrB = b2Add(vB, b2CrossSV(wB, cp->rB));
		b2Vec2 vrA = b2Add(vA, b2CrossSV(wA, cp->rA));
		b2Vec2 dv = b2Sub(vrB, vrA);

		// Compute change in separation (small angle approximation of sin(angle) == angle)
		b2Vec2 prB = b2Add(dpB, b2CrossSV(daB, cp->rB));
		b2Vec2 prA = b2Add(dpA, b2CrossSV(daA, cp->rA));
		float ds = b2Dot(b2Sub(prB, prA), normal);
		float s = cp->separation + ds;
		float bias = 0.0f;
		float massScale = 1.0f;
		float impulseScale = 0.0f;
		if (s > 0.0f)
		{
			// TODO_ERIN what time to use?
			// Speculative (inverse of full time step)
			bias = s * inv_dt;
		}
		else if (useBias)
		{
			bias = B2_MAX(biasCoefficient * s, -maxBaumgarteVelocity);
			// bias = cp->biasCoefficient * s;
			massScale = massCoefficient;
			impulseScale = impulseCoefficient;
		}

		// Compute normal impulse
		float vn = b2Dot(dv, normal);
		float impulse = -cp->normalMass * massScale * (vn + bias) - impulseScale * cp->normalImpulse;
		// float impulse = -cp->normalMass * (vn + bias + cp->gamma * cp->normalImpulse);

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

	{
		b2ContactConstraintPoint* cp = constraint->points + 0;

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

	{
		b2ContactConstraintPoint* cp = constraint->points + 1;

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

static void b2SolveContactOnePointStatic(b2ContactConstraint* constraint, b2SolverBody* bodies, float inv_dt, bool useBias)
{
	b2SolverBody* bodyB = bodies + constraint->indexB;
	b2Vec2 vB = bodyB->linearVelocity;
	float wB = bodyB->angularVelocity;
	b2Vec2 dpB = bodyB->deltaPosition;
	float daB = bodyB->deltaAngle;
	float mB = bodyB->invMass;
	float iB = bodyB->invI;

	b2Vec2 normal = constraint->normal;
	b2Vec2 tangent = b2RightPerp(normal);
	float friction = constraint->friction;
	float biasCoefficient = constraint->biasCoefficient;
	float massCoefficient = constraint->massCoefficient;
	float impulseCoefficient = constraint->impulseCoefficient;

	{
		b2ContactConstraintPoint* cp = constraint->points + 0;

		// Relative velocity at contact
		b2Vec2 dv = b2Add(vB, b2CrossSV(wB, cp->rB));

		// Compute change in separation (small angle approximation of sin(angle) == angle)
		b2Vec2 prB = b2Add(dpB, b2CrossSV(daB, cp->rB));
		float ds = b2Dot(prB, normal);
		float s = cp->separation + ds;
		float bias = 0.0f;
		float massScale = 1.0f;
		float impulseScale = 0.0f;
		if (s > 0.0f)
		{
			// TODO_ERIN what time to use?
			// Speculative (inverse of full time step)
			bias = s * inv_dt;
		}
		else if (useBias)
		{
			bias = B2_MAX(biasCoefficient * s, -maxBaumgarteVelocity);
			// bias = cp->biasCoefficient * s;
			massScale = massCoefficient;
			impulseScale = impulseCoefficient;
		}

		// Compute normal impulse
		float vn = b2Dot(dv, normal);
		float impulse = -cp->normalMass * massScale * (vn + bias) - impulseScale * cp->normalImpulse;
		// float impulse = -cp->normalMass * (vn + bias + cp->gamma * cp->normalImpulse);

		// Clamp the accumulated impulse
		float newImpulse = B2_MAX(cp->normalImpulse + impulse, 0.0f);
		impulse = newImpulse - cp->normalImpulse;
		cp->normalImpulse = newImpulse;

		// Apply contact impulse
		b2Vec2 P = b2MulSV(impulse, normal);
		vB = b2MulAdd(vB, mB, P);
		wB += iB * b2Cross(cp->rB, P);
	}

	{
		b2ContactConstraintPoint* cp = constraint->points + 0;

		// Relative velocity at contact
		b2Vec2 dv = b2Add(vB, b2CrossSV(wB, cp->rB));

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

		vB = b2MulAdd(vB, mB, P);
		wB += iB * b2Cross(cp->rB, P);
	}

	bodyB->linearVelocity = vB;
	bodyB->angularVelocity = wB;
}

static void b2SolveContactTwoPointsStatic(b2ContactConstraint* constraint, b2SolverBody* bodies, float inv_dt, bool useBias)
{
	b2SolverBody* bodyB = bodies + constraint->indexB;
	b2Vec2 vB = bodyB->linearVelocity;
	float wB = bodyB->angularVelocity;
	b2Vec2 dpB = bodyB->deltaPosition;
	float daB = bodyB->deltaAngle;
	float mB = bodyB->invMass;
	float iB = bodyB->invI;

	b2Vec2 normal = constraint->normal;
	b2Vec2 tangent = b2RightPerp(normal);
	float friction = constraint->friction;
	float biasCoefficient = constraint->biasCoefficient;
	float massCoefficient = constraint->massCoefficient;
	float impulseCoefficient = constraint->impulseCoefficient;

	{
		b2ContactConstraintPoint* cp = constraint->points + 0;

		// Relative velocity at contact
		b2Vec2 dv = b2Add(vB, b2CrossSV(wB, cp->rB));

		// Compute change in separation (small angle approximation of sin(angle) == angle)
		b2Vec2 prB = b2Add(dpB, b2CrossSV(daB, cp->rB));
		float ds = b2Dot(prB, normal);
		float s = cp->separation + ds;
		float bias = 0.0f;
		float massScale = 1.0f;
		float impulseScale = 0.0f;
		if (s > 0.0f)
		{
			// TODO_ERIN what time to use?
			// Speculative (inverse of full time step)
			bias = s * inv_dt;
		}
		else if (useBias)
		{
			bias = B2_MAX(biasCoefficient * s, -maxBaumgarteVelocity);
			// bias = cp->biasCoefficient * s;
			massScale = massCoefficient;
			impulseScale = impulseCoefficient;
		}

		// Compute normal impulse
		float vn = b2Dot(dv, normal);
		float impulse = -cp->normalMass * massScale * (vn + bias) - impulseScale * cp->normalImpulse;
		// float impulse = -cp->normalMass * (vn + bias + cp->gamma * cp->normalImpulse);

		// Clamp the accumulated impulse
		float newImpulse = B2_MAX(cp->normalImpulse + impulse, 0.0f);
		impulse = newImpulse - cp->normalImpulse;
		cp->normalImpulse = newImpulse;

		// Apply contact impulse
		b2Vec2 P = b2MulSV(impulse, normal);
		vB = b2MulAdd(vB, mB, P);
		wB += iB * b2Cross(cp->rB, P);
	}

	{
		b2ContactConstraintPoint* cp = constraint->points + 1;

		// Relative velocity at contact
		b2Vec2 dv = b2Add(vB, b2CrossSV(wB, cp->rB));

		// Compute change in separation (small angle approximation of sin(angle) == angle)
		b2Vec2 prB = b2Add(dpB, b2CrossSV(daB, cp->rB));
		float ds = b2Dot(prB, normal);
		float s = cp->separation + ds;
		float bias = 0.0f;
		float massScale = 1.0f;
		float impulseScale = 0.0f;
		if (s > 0.0f)
		{
			// TODO_ERIN what time to use?
			// Speculative (inverse of full time step)
			bias = s * inv_dt;
		}
		else if (useBias)
		{
			bias = B2_MAX(biasCoefficient * s, -maxBaumgarteVelocity);
			// bias = cp->biasCoefficient * s;
			massScale = massCoefficient;
			impulseScale = impulseCoefficient;
		}

		// Compute normal impulse
		float vn = b2Dot(dv, normal);
		float impulse = -cp->normalMass * massScale * (vn + bias) - impulseScale * cp->normalImpulse;
		// float impulse = -cp->normalMass * (vn + bias + cp->gamma * cp->normalImpulse);

		// Clamp the accumulated impulse
		float newImpulse = B2_MAX(cp->normalImpulse + impulse, 0.0f);
		impulse = newImpulse - cp->normalImpulse;
		cp->normalImpulse = newImpulse;

		// Apply contact impulse
		b2Vec2 P = b2MulSV(impulse, normal);
		vB = b2MulAdd(vB, mB, P);
		wB += iB * b2Cross(cp->rB, P);
	}

	{
		b2ContactConstraintPoint* cp = constraint->points + 0;

		// Relative velocity at contact
		b2Vec2 dv = b2Add(vB, b2CrossSV(wB, cp->rB));

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
		vB = b2MulAdd(vB, mB, P);
		wB += iB * b2Cross(cp->rB, P);
	}

	{
		b2ContactConstraintPoint* cp = constraint->points + 1;

		// Relative velocity at contact
		b2Vec2 dv = b2Add(vB, b2CrossSV(wB, cp->rB));

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
		vB = b2MulAdd(vB, mB, P);
		wB += iB * b2Cross(cp->rB, P);
	}

	bodyB->linearVelocity = vB;
	bodyB->angularVelocity = wB;
}

void b2SolveContactsTask(int32_t startIndex, int32_t endIndex, b2SolverTaskContext* context, int32_t colorIndex, bool useBias)
{
	b2TracyCZoneNC(solve_contact, "Solve Contact", b2_colorAliceBlue, true);

	b2SolverBody* bodies = context->solverBodies;
	b2ContactConstraint* constraints = context->graph->colors[colorIndex].contactConstraints;
	float inv_dt = context->invTimeStep;

	for (int32_t i = startIndex; i < endIndex; ++i)
	{
		b2ContactConstraint* constraint = constraints + i;

		switch (constraint->type)
		{
			case b2_onePointType:
				b2SolveContactOnePoint(constraint, bodies, inv_dt, useBias);
				break;

			case b2_twoPointType:
				b2SolveContactTwoPoints(constraint, bodies, inv_dt, useBias);
				break;

			case b2_onePointStaticType:
				b2SolveContactOnePointStatic(constraint, bodies, inv_dt, useBias);
				break;

			case b2_twoPointStaticType:
				b2SolveContactTwoPointsStatic(constraint, bodies, inv_dt, useBias);
				break;

			default:
				B2_ASSERT(false);
		}
	}

	b2TracyCZoneEnd(solve_contact);
}

void b2StoreImpulsesTask(int32_t startIndex, int32_t endIndex, b2SolverTaskContext* context)
{
	b2TracyCZoneNC(store_impulses, "Store", b2_colorFirebrick, true);

	b2ContactConstraint* constraints = context->constraints;

	for (int32_t i = startIndex; i < endIndex; ++i)
	{
		b2ContactConstraint* constraint = constraints + i;
		b2Contact* contact = constraint->contact;
		b2Manifold* manifold = &contact->manifold;
		int32_t pointCount = manifold->pointCount;

		for (int32_t j = 0; j < pointCount; ++j)
		{
			manifold->points[j].normalImpulse = constraint->points[j].normalImpulse;
			manifold->points[j].tangentImpulse = constraint->points[j].tangentImpulse;
		}
	}

	b2TracyCZoneEnd(store_impulses);
}
