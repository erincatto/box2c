// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#define _CRT_SECURE_NO_WARNINGS

#include "body.h"
#include "core.h"
#include "joint.h"
#include "solver_data.h"
#include "world.h"

#include "box2d/debug_draw.h"

#include <stdio.h>

// 1-D constrained system
// m (v2 - v1) = lambda
// v2 + (beta/h) * x1 + gamma * lambda = 0, gamma has units of inverse mass.
// x2 = x1 + h * v2

// 1-D mass-damper-spring system
// m (v2 - v1) + h * d * v2 + h * k *

// C = norm(p2 - p1) - L
// u = (p2 - p1) / norm(p2 - p1)
// Cdot = dot(u, v2 + cross(w2, r2) - v1 - cross(w1, r1))
// J = [-u -cross(r1, u) u cross(r2, u)]
// K = J * invM * JT
//   = invMass1 + invI1 * cross(r1, u)^2 + invMass2 + invI2 * cross(r2, u)^2

void b2PrepareDistanceJoint(b2Joint* base, b2StepContext* context)
{
	B2_ASSERT(base->type == b2_distanceJoint);

	int32_t indexA = base->edges[0].bodyIndex;
	int32_t indexB = base->edges[1].bodyIndex;
	b2Body* bodyA = context->bodies + indexA;
	b2Body* bodyB = context->bodies + indexB;

	B2_ASSERT(b2ObjectValid(&bodyA->object));
	B2_ASSERT(b2ObjectValid(&bodyB->object));

	b2DistanceJoint* joint = &base->distanceJoint;

	joint->indexA = context->bodyToSolverMap[indexA];
	joint->indexB = context->bodyToSolverMap[indexB];

	float mA = bodyA->invMass;
	float iA = bodyA->invI;
	float mB = bodyB->invMass;
	float iB = bodyB->invI;

	// Compute the effective masses.
	joint->rA = b2RotateVector(bodyA->transform.q, b2Sub(base->localAnchorA, bodyA->localCenter));
	joint->rB = b2RotateVector(bodyB->transform.q, b2Sub(base->localAnchorB, bodyB->localCenter));
	joint->separation = b2Add(b2Sub(joint->rB, joint->rA), b2Sub(bodyB->position, bodyA->position));

	b2Vec2 rA = joint->rA;
	b2Vec2 rB = joint->rB;

	b2Vec2 axis = b2Normalize(joint->separation);

	float crA = b2Cross(rA, axis);
	float crB = b2Cross(rB, axis);
	float k = mA + mB + iA * crA * crA + iB * crB * crB;
	joint->axialMass = k > 0.0f ? 1.0f / k : 0.0f;

	float dt = context->dt;

	// Spring parameters
	if (joint->hertz > 0.0f)
	{
		float omega = 2.0f * b2_pi * joint->hertz;
		float a1 = 2.0f * joint->dampingRatio + dt * omega;
		float a2 = dt * omega * a1;
		float a3 = 1.0f / (1.0f + a2);
		joint->springBiasCoefficient = omega / a1;
		joint->springImpulseCoefficient = a3;
		joint->springMassCoefficient = a2 * a3;
	}
	else
	{
		joint->springBiasCoefficient = 0.0f;
		joint->springImpulseCoefficient = 0.0f;
		joint->springMassCoefficient = 0.0f;
	}

	// Limit parameters
	{
		// as rigid as possible: hertz = 1/4 * substep Hz
		float hertz = 0.25f * context->velocityIterations * context->inv_dt;
		float zeta = 1.0f;

		float omega = 2.0f * b2_pi * hertz;
		float a1 = 2.0f * zeta + dt * omega;
		float a2 = dt * omega * a1;
		float a3 = 1.0f / (1.0f + a2);
		joint->limitBiasCoefficient = omega / a1;
		joint->limitImpulseCoefficient = a3;
		joint->limitMassCoefficient = a2 * a3;
	}

	if (context->enableWarmStarting)
	{
		float dtRatio = context->dtRatio;

		// Soft step works best when bilateral constraints have no warm starting.
		joint->impulse = 0.0f;
		joint->lowerImpulse *= dtRatio;
		joint->upperImpulse *= dtRatio;
	}
	else
	{
		joint->impulse = 0.0f;
		joint->lowerImpulse = 0.0f;
		joint->upperImpulse = 0.0f;
	}
}

void b2WarmStartDistanceJoint(b2Joint* base, b2StepContext* context)
{
	B2_ASSERT(base->type == b2_distanceJoint);

	b2DistanceJoint* joint = &base->distanceJoint;

	// This is a dummy body to represent a static body since static bodies don't have a solver body.
	b2SolverBody dummyBody = {0};

	// Note: must warm start solver bodies
	b2SolverBody* bodyA = joint->indexA == B2_NULL_INDEX ? &dummyBody : context->solverBodies + joint->indexA;
	float mA = bodyA->invMass;
	float iA = bodyA->invI;

	b2SolverBody* bodyB = joint->indexB == B2_NULL_INDEX ? &dummyBody : context->solverBodies + joint->indexB;
	float mB = bodyB->invMass;
	float iB = bodyB->invI;

	b2Vec2 rA = joint->rA;
	b2Vec2 rB = joint->rB;

	b2Vec2 axis = b2Normalize(joint->separation);

	float axialImpulse = joint->impulse + joint->lowerImpulse - joint->upperImpulse;
	b2Vec2 P = b2MulSV(axialImpulse, axis);

	bodyA->linearVelocity = b2MulSub(bodyA->linearVelocity, mA, P);
	bodyA->angularVelocity -= iA * b2Cross(rA, P);
	bodyB->linearVelocity = b2MulAdd(bodyB->linearVelocity, mB, P);
	bodyB->angularVelocity += iB * b2Cross(rB, P);
}

void b2SolveDistanceJoint(b2Joint* base, b2StepContext* context, bool useBias)
{
	B2_ASSERT(base->type == b2_distanceJoint);

	b2DistanceJoint* joint = &base->distanceJoint;

	// This is a dummy body to represent a static body since static bodies don't have a solver body.
	b2SolverBody dummyBody = {0};

	b2SolverBody* bodyA = joint->indexA == B2_NULL_INDEX ? &dummyBody : context->solverBodies + joint->indexA;
	b2Vec2 vA = bodyA->linearVelocity;
	float wA = bodyA->angularVelocity;
	float mA = bodyA->invMass;
	float iA = bodyA->invI;

	b2SolverBody* bodyB = joint->indexB == B2_NULL_INDEX ? &dummyBody : context->solverBodies + joint->indexB;
	b2Vec2 vB = bodyB->linearVelocity;
	float wB = bodyB->angularVelocity;
	float mB = bodyB->invMass;
	float iB = bodyB->invI;

	// Approximate change in anchors
	// small angle approximation of sin(delta_angle) == delta_angle, cos(delta_angle) == 1
	b2Vec2 drA = b2CrossSV(bodyA->deltaAngle, joint->rA);
	b2Vec2 drB = b2CrossSV(bodyB->deltaAngle, joint->rB);

	b2Vec2 rA = b2Add(joint->rA, drA);
	b2Vec2 rB = b2Add(joint->rB, drB);
	b2Vec2 ds = b2Add(b2Sub(bodyB->deltaPosition, bodyA->deltaPosition), b2Sub(drB, drA));
	b2Vec2 separation = b2Add(joint->separation, ds);

	float L = b2Length(separation);
	b2Vec2 axis = b2Normalize(separation);

	if (joint->minLength < joint->maxLength)
	{
		if (joint->hertz > 0.0f)
		{
			// Cdot = dot(u, v + cross(w, r))
			b2Vec2 vr = b2Add(b2Sub(vB, vA), b2Sub(b2CrossSV(wB, rB), b2CrossSV(wA, rA)));
			float Cdot = b2Dot(axis, vr);
			float C = L - joint->length;
			float bias = joint->springBiasCoefficient * C;

			float m = joint->springMassCoefficient * joint->axialMass;
			float impulse = -m * (Cdot + bias) - joint->springImpulseCoefficient * joint->impulse;
			joint->impulse += impulse;

			b2Vec2 P = b2MulSV(impulse, axis);
			vA = b2MulSub(vA, mA,  P);
			wA -= iA * b2Cross(rA, P);
			vB = b2MulAdd(vB, mB, P);
			wB += iB * b2Cross(rB, P);
		}

		// lower limit
		{
			b2Vec2 vr = b2Add(b2Sub(vB, vA), b2Sub(b2CrossSV(wB, rB), b2CrossSV(wA, rA)));
			float Cdot = b2Dot(axis, vr);

			float C = L - joint->minLength;

			float bias = 0.0f;
			float massScale = 1.0f;
			float impulseScale = 0.0f;
			if (C > 0.0f)
			{
				// speculative
				bias = C * context->inv_dt;
			}
			else if (useBias)
			{
				bias = joint->limitBiasCoefficient * C;
				massScale = joint->limitMassCoefficient;
				impulseScale = joint->limitImpulseCoefficient;
			}

			float impulse = -massScale * joint->axialMass * (Cdot + bias) - impulseScale * joint->lowerImpulse;
			float newImpulse = B2_MAX(0.0f, joint->lowerImpulse + impulse);
			impulse = newImpulse - joint->lowerImpulse;
			joint->lowerImpulse = newImpulse;

			b2Vec2 P = b2MulSV(impulse, axis);
			vA = b2MulSub(vA, mA, P);
			wA -= iA * b2Cross(rA, P);
			vB = b2MulAdd(vB, mB, P);
			wB += iB * b2Cross(rB, P);
		}

		// upper
		{
			b2Vec2 vr = b2Add(b2Sub(vA, vB), b2Sub(b2CrossSV(wA, rA), b2CrossSV(wB, rB)));
			float Cdot = b2Dot(axis, vr);

			float C = joint->maxLength - L;

			float bias = 0.0f;
			float massScale = 1.0f;
			float impulseScale = 0.0f;
			if (C > 0.0f)
			{
				// speculative
				bias = C * context->inv_dt;
			}
			else if (useBias)
			{
				bias = joint->limitBiasCoefficient * C;
				massScale = joint->limitMassCoefficient;
				impulseScale = joint->limitImpulseCoefficient;
			}

			float impulse = -massScale * joint->axialMass * (Cdot + bias) - impulseScale * joint->upperImpulse;
			float newImpulse = B2_MAX(0.0f, joint->upperImpulse + impulse);
			impulse = newImpulse - joint->upperImpulse;
			joint->upperImpulse = newImpulse;

			b2Vec2 P = b2MulSV(-impulse, axis);
			vA = b2MulSub(vA, mA, P);
			wA -= iA * b2Cross(rA, P);
			vB = b2MulAdd(vB, mB, P);
			wB += iB * b2Cross(rB, P);
		}
	}
	else
	{
		// Equal limits
		b2Vec2 vr = b2Add(b2Sub(vB, vA), b2Sub(b2CrossSV(wB, rB), b2CrossSV(wA, rA)));
		float Cdot = b2Dot(axis, vr);

		float C = L - joint->minLength;

		float bias = 0.0f;
		float massScale = 1.0f;
		float impulseScale = 0.0f;
		if (useBias)
		{
			bias = joint->limitBiasCoefficient * C;
			massScale = joint->limitMassCoefficient;
			impulseScale = joint->limitImpulseCoefficient;
		}

		float impulse = -massScale * joint->axialMass * (Cdot + bias) - impulseScale * joint->impulse;
		joint->impulse += impulse;

		b2Vec2 P = b2MulSV(impulse, axis);
		vA = b2MulSub(vA, mA, P);
		wA -= iA * b2Cross(rA, P);
		vB = b2MulAdd(vB, mB, P);
		wB += iB * b2Cross(rB, P);
	}

	bodyA->linearVelocity = vA;
	bodyA->angularVelocity = wA;
	bodyB->linearVelocity = vB;
	bodyB->angularVelocity = wB;
}

float b2DistanceJoint_GetConstraintForce(b2JointId jointId, float timeStep)
{
	b2Joint* base = b2GetJointCheckType(jointId, b2_distanceJoint);
	b2DistanceJoint* joint = &base->distanceJoint;

	if (timeStep > 0.0f)
	{
		float F = (joint->impulse + joint->lowerImpulse - joint->upperImpulse) / timeStep;
		return F;
	}

	return 0.0f;
}

void b2DistanceJoint_SetLength(b2JointId jointId, float length, float minLength, float maxLength)
{
	b2Joint* base = b2GetJointCheckType(jointId, b2_distanceJoint);
	b2DistanceJoint* joint = &base->distanceJoint;

	joint->length = B2_CLAMP(length, b2_linearSlop, b2_huge);

	minLength = B2_CLAMP(minLength, b2_linearSlop, b2_huge);
	maxLength = B2_CLAMP(maxLength, b2_linearSlop, b2_huge);
	joint->minLength = B2_MIN(minLength, maxLength);
	joint->maxLength = B2_MAX(minLength, maxLength);

	joint->impulse = 0.0f;
	joint->lowerImpulse = 0.0f;
	joint->upperImpulse = 0.0f;
}

float b2DistanceJoint_GetCurrentLength(b2JointId jointId)
{
	b2Joint* base = b2GetJointCheckType(jointId, b2_distanceJoint);

	b2World* world = b2GetWorldFromIndex(jointId.world);
	B2_ASSERT(world->locked == false);
	if (world->locked)
	{
		return 0.0f;
	}

	int32_t indexA = base->edges[0].bodyIndex;
	int32_t indexB = base->edges[1].bodyIndex;
	b2Body* bodyA = world->bodies + indexA;
	b2Body* bodyB = world->bodies + indexB;

	B2_ASSERT(b2ObjectValid(&bodyA->object));
	B2_ASSERT(b2ObjectValid(&bodyB->object));

	b2Vec2 pA = b2TransformPoint(bodyA->transform, base->localAnchorA);
	b2Vec2 pB = b2TransformPoint(bodyB->transform, base->localAnchorB);
	b2Vec2 d = b2Sub(pB, pA);
	float length = b2Length(d);
	return length;
}

void b2DistanceJoint_SetTuning(b2JointId jointId, float hertz, float dampingRatio)
{
	b2Joint* base = b2GetJointCheckType(jointId, b2_distanceJoint);
	b2DistanceJoint* joint = &base->distanceJoint;
	joint->hertz = hertz;
	joint->dampingRatio = dampingRatio;
}

#if 0
void b2DistanceJoint::Dump()
{
	int32 indexA = m_bodyA->m_islandIndex;
	int32 indexB = m_bodyB->m_islandIndex;

	b2Dump("  b2DistanceJointDef jd;\n");
	b2Dump("  jd.bodyA = bodies[%d];\n", indexA);
	b2Dump("  jd.bodyB = bodies[%d];\n", indexB);
	b2Dump("  jd.collideConnected = bool(%d);\n", m_collideConnected);
	b2Dump("  jd.localAnchorA.Set(%.9g, %.9g);\n", m_localAnchorA.x, m_localAnchorA.y);
	b2Dump("  jd.localAnchorB.Set(%.9g, %.9g);\n", m_localAnchorB.x, m_localAnchorB.y);
	b2Dump("  jd.length = %.9g;\n", m_length);
	b2Dump("  jd.minLength = %.9g;\n", m_minLength);
	b2Dump("  jd.maxLength = %.9g;\n", m_maxLength);
	b2Dump("  jd.stiffness = %.9g;\n", m_stiffness);
	b2Dump("  jd.damping = %.9g;\n", m_damping);
	b2Dump("  joints[%d] = m_world->CreateJoint(&jd);\n", m_index);
}
#endif

void b2DrawDistance(b2DebugDraw* draw, b2Joint* base, b2Body* bodyA, b2Body* bodyB)
{
	B2_ASSERT(base->type == b2_distanceJoint);

	b2DistanceJoint* joint = &base->distanceJoint;

	b2Transform xfA = bodyA->transform;
	b2Transform xfB = bodyB->transform;
	b2Vec2 pA = b2TransformPoint(xfA, base->localAnchorA);
	b2Vec2 pB = b2TransformPoint(xfB, base->localAnchorB);

	b2Vec2 axis = b2Normalize(b2Sub(pB, pA));

	b2Color c1 = {0.7f, 0.7f, 0.7f, 1.0f};
	b2Color c2 = {0.3f, 0.9f, 0.3f, 1.0f};
	b2Color c3 = {0.9f, 0.3f, 0.3f, 1.0f};
	b2Color c4 = {0.4f, 0.4f, 0.4f, 1.0f};

	draw->DrawSegment(pA, pB, c4, draw->context);

	b2Vec2 pRest = b2MulAdd(pA, joint->length, axis);
	draw->DrawPoint(pRest, 8.0f, c1, draw->context);

	if (joint->minLength < joint->maxLength)
	{
		b2Vec2 pMin = b2MulAdd(pA, joint->minLength, axis);
		b2Vec2 pMax = b2MulAdd(pA, joint->maxLength, axis);
		b2Vec2 offset = b2MulSV(0.05f * b2_lengthUnitsPerMeter, b2RightPerp(axis));

		if (joint->minLength > b2_linearSlop)
		{
			//draw->DrawPoint(pMin, 4.0f, c2, draw->context);
			draw->DrawSegment(b2Sub(pMin, offset), b2Add(pMin, offset), c2, draw->context);
		}

		if (joint->maxLength < b2_huge)
		{
			//draw->DrawPoint(pMax, 4.0f, c3, draw->context);
			draw->DrawSegment(b2Sub(pMax, offset), b2Add(pMax, offset), c3, draw->context);
		}

		if (joint->minLength > b2_linearSlop && joint->maxLength < b2_huge)
		{
			draw->DrawSegment(pMin, pMax, c4, draw->context);
		}
	}
}
