// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#define _CRT_SECURE_NO_WARNINGS

#include "body.h"
#include "core.h"
#include "joint.h"
#include "solver.h"
#include "solver_set.h"
#include "world.h"

// needed for dll export
#include "box2d/box2d.h"
#include "box2d/debug_draw.h"
#include "box2d/joint_types.h"

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

void b2PrepareDistanceJoint(b2JointSim* base, b2StepContext* context)
{
	B2_ASSERT(base->type == b2_distanceJoint);

	// chase body id to the solver set where the body lives
	int idA = base->bodyIdA;
	int idB = base->bodyIdB;

	b2World* world = context->world;
	b2Body* bodies = world->bodyArray;

	b2CheckIndex(bodies, idA);
	b2CheckIndex(bodies, idB);

	b2Body* bodyA = bodies + idA;
	b2Body* bodyB = bodies + idB;

	B2_ASSERT(bodyA->setIndex == b2_awakeSet || bodyB->setIndex == b2_awakeSet);
	b2CheckIndex(world->solverSetArray, bodyA->setIndex);
	b2CheckIndex(world->solverSetArray, bodyB->setIndex);

	b2SolverSet* setA = world->solverSetArray + bodyA->setIndex;
	b2SolverSet* setB = world->solverSetArray + bodyB->setIndex;

	B2_ASSERT(0 <= bodyA->localIndex && bodyA->localIndex <= setA->sims.count);
	B2_ASSERT(0 <= bodyB->localIndex && bodyB->localIndex <= setB->sims.count);

	b2BodySim* bodySimA = setA->sims.data + bodyA->localIndex;
	b2BodySim* bodySimB = setB->sims.data + bodyB->localIndex;

	float mA = bodySimA->invMass;
	float iA = bodySimA->invI;
	float mB = bodySimB->invMass;
	float iB = bodySimB->invI;

	base->invMassA = mA;
	base->invMassB = mB;
	base->invIA = iA;
	base->invIB = iB;

	b2DistanceJoint* joint = &base->distanceJoint;

	joint->indexA = bodyA->setIndex == b2_awakeSet ? bodyA->localIndex : B2_NULL_INDEX;
	joint->indexB = bodyB->setIndex == b2_awakeSet ? bodyB->localIndex : B2_NULL_INDEX;

	// initial anchors in world space
	joint->anchorA = b2RotateVector(bodySimA->transform.q, b2Sub(base->localOriginAnchorA, bodySimA->localCenter));
	joint->anchorB = b2RotateVector(bodySimB->transform.q, b2Sub(base->localOriginAnchorB, bodySimB->localCenter));
	joint->deltaCenter = b2Sub(bodySimB->center, bodySimA->center);

	b2Vec2 rA = joint->anchorA;
	b2Vec2 rB = joint->anchorB;
	b2Vec2 separation = b2Add(b2Sub(rB, rA), joint->deltaCenter);
	b2Vec2 axis = b2Normalize(separation);

	// compute effective mass
	float crA = b2Cross(rA, axis);
	float crB = b2Cross(rB, axis);
	float k = mA + mB + iA * crA * crA + iB * crB * crB;
	joint->axialMass = k > 0.0f ? 1.0f / k : 0.0f;

	joint->distanceSoftness = b2MakeSoft(joint->hertz, joint->dampingRatio, context->h);

	if (context->enableWarmStarting == false)
	{
		joint->impulse = 0.0f;
		joint->lowerImpulse = 0.0f;
		joint->upperImpulse = 0.0f;
	}
}

void b2WarmStartDistanceJoint(b2JointSim* base, b2StepContext* context)
{
	B2_ASSERT(base->type == b2_distanceJoint);

	float mA = base->invMassA;
	float mB = base->invMassB;
	float iA = base->invIA;
	float iB = base->invIB;

	// dummy state for static sims
	b2BodyState dummyState = b2_identityBodyState;

	b2DistanceJoint* joint = &base->distanceJoint;
	b2BodyState* stateA = joint->indexA == B2_NULL_INDEX ? &dummyState : context->states + joint->indexA;
	b2BodyState* stateB = joint->indexB == B2_NULL_INDEX ? &dummyState : context->states + joint->indexB;

	b2Vec2 rA = b2RotateVector(stateA->deltaRotation, joint->anchorA);
	b2Vec2 rB = b2RotateVector(stateB->deltaRotation, joint->anchorB);

	b2Vec2 ds = b2Add(b2Sub(stateB->deltaPosition, stateA->deltaPosition), b2Sub(rB, rA));
	b2Vec2 separation = b2Add(joint->deltaCenter, ds);
	b2Vec2 axis = b2Normalize(separation);

	float axialImpulse = joint->impulse + joint->lowerImpulse - joint->upperImpulse;
	b2Vec2 P = b2MulSV(axialImpulse, axis);

	stateA->linearVelocity = b2MulSub(stateA->linearVelocity, mA, P);
	stateA->angularVelocity -= iA * b2Cross(rA, P);
	stateB->linearVelocity = b2MulAdd(stateB->linearVelocity, mB, P);
	stateB->angularVelocity += iB * b2Cross(rB, P);
}

void b2SolveDistanceJoint(b2JointSim* base, b2StepContext* context, bool useBias)
{
	B2_ASSERT(base->type == b2_distanceJoint);

	float mA = base->invMassA;
	float mB = base->invMassB;
	float iA = base->invIA;
	float iB = base->invIB;

	// dummy state for static sims
	b2BodyState dummyState = b2_identityBodyState;

	b2DistanceJoint* joint = &base->distanceJoint;
	b2BodyState* stateA = joint->indexA == B2_NULL_INDEX ? &dummyState : context->states + joint->indexA;
	b2BodyState* stateB = joint->indexB == B2_NULL_INDEX ? &dummyState : context->states + joint->indexB;
	
	b2Vec2 vA = stateA->linearVelocity;
	float wA = stateA->angularVelocity;
	b2Vec2 vB = stateB->linearVelocity;
	float wB = stateB->angularVelocity;

	// current anchors
	b2Vec2 rA = b2RotateVector(stateA->deltaRotation, joint->anchorA);
	b2Vec2 rB = b2RotateVector(stateB->deltaRotation, joint->anchorB);

	// current separation
	b2Vec2 ds = b2Add(b2Sub(stateB->deltaPosition, stateA->deltaPosition), b2Sub(rB, rA));
	b2Vec2 separation = b2Add(joint->deltaCenter, ds);

	float length = b2Length(separation);
	b2Vec2 axis = b2Normalize(separation);

	if (joint->minLength < joint->maxLength)
	{
		if (joint->hertz > 0.0f)
		{
			// Cdot = dot(u, v + cross(w, r))
			b2Vec2 vr = b2Add(b2Sub(vB, vA), b2Sub(b2CrossSV(wB, rB), b2CrossSV(wA, rA)));
			float Cdot = b2Dot(axis, vr);
			float C = length - joint->length;
			float bias = joint->distanceSoftness.biasRate * C;

			float m = joint->distanceSoftness.massScale * joint->axialMass;
			float impulse = -m * (Cdot + bias) - joint->distanceSoftness.impulseScale * joint->impulse;
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

			float C = length - joint->minLength;

			float bias = 0.0f;
			float massCoeff = 1.0f;
			float impulseCoeff = 0.0f;
			if (C > 0.0f)
			{
				// speculative
				bias = C * context->inv_h;
			}
			else if (useBias)
			{
				bias = context->jointSoftness.biasRate * C;
				massCoeff = context->jointSoftness.massScale;
				impulseCoeff = context->jointSoftness.impulseScale;
			}

			float impulse = -massCoeff * joint->axialMass * (Cdot + bias) - impulseCoeff * joint->lowerImpulse;
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

			float C = joint->maxLength - length;

			float bias = 0.0f;
			float massScale = 1.0f;
			float impulseScale = 0.0f;
			if (C > 0.0f)
			{
				// speculative
				bias = C * context->inv_h;
			}
			else if (useBias)
			{
				bias = context->jointSoftness.biasRate * C;
				massScale = context->jointSoftness.massScale;
				impulseScale = context->jointSoftness.impulseScale;
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
		// equal limits
		b2Vec2 vr = b2Add(b2Sub(vB, vA), b2Sub(b2CrossSV(wB, rB), b2CrossSV(wA, rA)));
		float Cdot = b2Dot(axis, vr);

		float C = length - joint->minLength;

		float bias = 0.0f;
		float massScale = 1.0f;
		float impulseScale = 0.0f;
		if (useBias)
		{
			bias = context->jointSoftness.biasRate * C;
			massScale = context->jointSoftness.massScale;
			impulseScale = context->jointSoftness.impulseScale;
		}

		float impulse = -massScale * joint->axialMass * (Cdot + bias) - impulseScale * joint->impulse;
		joint->impulse += impulse;

		b2Vec2 P = b2MulSV(impulse, axis);
		vA = b2MulSub(vA, mA, P);
		wA -= iA * b2Cross(rA, P);
		vB = b2MulAdd(vB, mB, P);
		wB += iB * b2Cross(rB, P);
	}

	stateA->linearVelocity = vA;
	stateA->angularVelocity = wA;
	stateB->linearVelocity = vB;
	stateB->angularVelocity = wB;
}

float b2DistanceJoint_GetConstraintForce(b2JointId jointId, float inverseTimeStep)
{
	b2JointSim* base = b2GetJointSimCheckType(jointId, b2_distanceJoint);
	b2DistanceJoint* joint = &base->distanceJoint;

	return (joint->impulse + joint->lowerImpulse - joint->upperImpulse) * inverseTimeStep;
}

void b2DistanceJoint_SetLength(b2JointId jointId, float length)
{
	b2JointSim* base = b2GetJointSimCheckType(jointId, b2_distanceJoint);
	b2DistanceJoint* joint = &base->distanceJoint;

	joint->length = b2ClampFloat(length, b2_linearSlop, b2_huge);
	joint->impulse = 0.0f;
	joint->lowerImpulse = 0.0f;
	joint->upperImpulse = 0.0f;
}

float b2DistanceJoint_GetLength(b2JointId jointId)
{
	b2JointSim* base = b2GetJointSimCheckType(jointId, b2_distanceJoint);
	b2DistanceJoint* joint = &base->distanceJoint;
	return joint->length;
}

void b2DistanceJoint_SetLengthRange(b2JointId jointId, float minLength, float maxLength)
{
	b2JointSim* base = b2GetJointSimCheckType(jointId, b2_distanceJoint);
	b2DistanceJoint* joint = &base->distanceJoint;

	minLength = b2ClampFloat(minLength, b2_linearSlop, b2_huge);
	maxLength = b2ClampFloat(maxLength, b2_linearSlop, b2_huge);
	joint->minLength = b2MinFloat(minLength, maxLength);
	joint->maxLength = B2_MAX(minLength, maxLength);
	joint->impulse = 0.0f;
	joint->lowerImpulse = 0.0f;
	joint->upperImpulse = 0.0f;
}

float b2DistanceJoint_GetMinLength(b2JointId jointId)
{
	b2JointSim* base = b2GetJointSimCheckType(jointId, b2_distanceJoint);
	b2DistanceJoint* joint = &base->distanceJoint;
	return joint->minLength;
}

float b2DistanceJoint_GetMaxLength(b2JointId jointId)
{
	b2JointSim* base = b2GetJointSimCheckType(jointId, b2_distanceJoint);
	b2DistanceJoint* joint = &base->distanceJoint;
	return joint->maxLength;
}

float b2DistanceJoint_GetCurrentLength(b2JointId jointId)
{
	b2JointSim* base = b2GetJointSimCheckType(jointId, b2_distanceJoint);

	b2World* world = b2GetWorld(jointId.world0);
	B2_ASSERT(world->locked == false);
	if (world->locked)
	{
		return 0.0f;
	}

	b2Transform transformA = b2GetBodyTransform(world, base->bodyIdA);
	b2Transform transformB = b2GetBodyTransform(world, base->bodyIdB);

	b2Vec2 pA = b2TransformPoint(transformA, base->localOriginAnchorA);
	b2Vec2 pB = b2TransformPoint(transformB, base->localOriginAnchorB);
	b2Vec2 d = b2Sub(pB, pA);
	float length = b2Length(d);
	return length;
}

void b2DistanceJoint_SetTuning(b2JointId jointId, float hertz, float dampingRatio)
{
	b2JointSim* base = b2GetJointSimCheckType(jointId, b2_distanceJoint);
	b2DistanceJoint* joint = &base->distanceJoint;
	joint->hertz = hertz;
	joint->dampingRatio = dampingRatio;
}

float b2DistanceJoint_GetHertz(b2JointId jointId)
{
	b2JointSim* base = b2GetJointSimCheckType(jointId, b2_distanceJoint);
	b2DistanceJoint* joint = &base->distanceJoint;
	return joint->hertz;
}

float b2DistanceJoint_GetDampingRatio(b2JointId jointId)
{
	b2JointSim* base = b2GetJointSimCheckType(jointId, b2_distanceJoint);
	b2DistanceJoint* joint = &base->distanceJoint;
	return joint->dampingRatio;
}

#if 0
void b2DistanceJoint::Dump()
{
	int32 indexA = m_bodyA->m_islandIndex;
	int32 indexB = m_bodyB->m_islandIndex;

	b2Dump("  b2DistanceJointDef jd;\n");
	b2Dump("  jd.bodyA = sims[%d];\n", indexA);
	b2Dump("  jd.bodyB = sims[%d];\n", indexB);
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

void b2DrawDistanceJoint(b2DebugDraw* draw, b2JointSim* base, b2Transform transformA, b2Transform transformB)
{
	B2_ASSERT(base->type == b2_distanceJoint);

	b2DistanceJoint* joint = &base->distanceJoint;

	b2Vec2 pA = b2TransformPoint(transformA, base->localOriginAnchorA);
	b2Vec2 pB = b2TransformPoint(transformB, base->localOriginAnchorB);

	b2Vec2 axis = b2Normalize(b2Sub(pB, pA));

	b2HexColor c1 = b2_colorGray74;
	b2HexColor c2 = b2_colorLightGreen;
	b2HexColor c3 = b2_colorRed3;
	b2HexColor c4 = b2_colorGray4;

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
