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

#include <stdio.h>

void b2RevoluteJoint_EnableLimit(b2JointId jointId, bool enableLimit)
{
	b2Joint* joint = b2GetJointCheckType(jointId, b2_revoluteJoint);
	if (enableLimit != joint->revoluteJoint.enableLimit)
	{
		joint->revoluteJoint.enableLimit = enableLimit;
		joint->revoluteJoint.lowerImpulse = 0.0f;
		joint->revoluteJoint.upperImpulse = 0.0f;
	}
}

bool b2RevoluteJoint_IsLimitEnabled(b2JointId jointId)
{
	b2Joint* joint = b2GetJointCheckType(jointId, b2_revoluteJoint);
	return joint->revoluteJoint.enableLimit;
}

float b2RevoluteJoint_GetLowerLimit(b2JointId jointId)
{
	b2Joint* joint = b2GetJointCheckType(jointId, b2_revoluteJoint);
	return joint->revoluteJoint.lowerAngle;
}

float b2RevoluteJoint_GetUpperLimit(b2JointId jointId)
{
	b2Joint* joint = b2GetJointCheckType(jointId, b2_revoluteJoint);
	return joint->revoluteJoint.upperAngle;
}

void b2RevoluteJoint_SetLimits(b2JointId jointId, float lower, float upper)
{
	b2Joint* joint = b2GetJointCheckType(jointId, b2_revoluteJoint);
	if (lower != joint->revoluteJoint.lowerAngle || upper != joint->revoluteJoint.upperAngle)
	{
		joint->revoluteJoint.lowerAngle = B2_MIN(lower, upper);
		joint->revoluteJoint.upperAngle = B2_MAX(lower, upper);
		joint->revoluteJoint.lowerImpulse = 0.0f;
		joint->revoluteJoint.upperImpulse = 0.0f;
	}
}

void b2RevoluteJoint_EnableMotor(b2JointId jointId, bool enableMotor)
{
	b2Joint* joint = b2GetJointCheckType(jointId, b2_revoluteJoint);
	if (enableMotor != joint->revoluteJoint.enableMotor)
	{
		joint->revoluteJoint.enableMotor = enableMotor;
		joint->revoluteJoint.motorImpulse = 0.0f;
	}
}

bool b2RevoluteJoint_IsMotorEnabled(b2JointId jointId)
{
	b2Joint* joint = b2GetJointCheckType(jointId, b2_revoluteJoint);
	return joint->revoluteJoint.enableMotor;
}

void b2RevoluteJoint_SetMotorSpeed(b2JointId jointId, float motorSpeed)
{
	b2Joint* joint = b2GetJointCheckType(jointId, b2_revoluteJoint);
	joint->revoluteJoint.motorSpeed = motorSpeed;
}

float b2RevoluteJoint_GetMotorSpeed(b2JointId jointId)
{
	b2Joint* joint = b2GetJointCheckType(jointId, b2_revoluteJoint);
	return joint->revoluteJoint.motorSpeed;
}

float b2RevoluteJoint_GetMotorTorque(b2JointId jointId)
{
	b2World* world = b2GetWorldFromIndex(jointId.world0);
	b2Joint* joint = b2GetJoint(world, jointId);
	B2_ASSERT(joint->type == b2_revoluteJoint);

	return world->inv_h * joint->revoluteJoint.motorImpulse;
}

void b2RevoluteJoint_SetMaxMotorTorque(b2JointId jointId, float torque)
{
	b2Joint* joint = b2GetJointCheckType(jointId, b2_revoluteJoint);
	joint->revoluteJoint.maxMotorTorque = torque;
}

float b2RevoluteJoint_GetMaxMotorTorque(b2JointId jointId)
{
	b2Joint* joint = b2GetJointCheckType(jointId, b2_revoluteJoint);
	return joint->revoluteJoint.maxMotorTorque;
}

b2Vec2 b2RevoluteJoint_GetConstraintForce(b2JointId jointId)
{
	b2World* world = b2GetWorldFromIndex(jointId.world0);
	b2Joint* joint = b2GetJoint(world, jointId);
	B2_ASSERT(joint->type == b2_revoluteJoint);

	b2Vec2 force = b2MulSV(world->inv_h, joint->revoluteJoint.linearImpulse);
	return force;
}

float b2RevoluteJoint_GetConstraintTorque(b2JointId jointId)
{
	b2World* world = b2GetWorldFromIndex(jointId.world0);
	b2Joint* joint = b2GetJoint(world, jointId);
	B2_ASSERT(joint->type == b2_revoluteJoint);

	const b2RevoluteJoint* revolute = &joint->revoluteJoint;
	float torque = world->inv_h * (revolute->motorImpulse + revolute->lowerImpulse - revolute->upperImpulse);
	return torque;
}

// Point-to-point constraint
// C = p2 - p1
// Cdot = v2 - v1
//      = v2 + cross(w2, r2) - v1 - cross(w1, r1)
// J = [-I -r1_skew I r2_skew ]
// Identity used:
// w k % (rx i + ry j) = w * (-ry i + rx j)

// Motor constraint
// Cdot = w2 - w1
// J = [0 0 -1 0 0 1]
// K = invI1 + invI2

// Body State
// The solver operates on the body state. The body state array does not hold static bodies. Static bodies are shared
// across worker threads. It would be okay to read their states, but writing to them would cause cache thrashing across
// workers, even if the values don't change.
// This causes some trouble when computing anchors. I rotate the anchors using the body rotation every sub-step. For static
// bodies the anchor doesn't rotate. Body A or B could be static and this can lead to lots of branching. This branching
// should be minimized.
// 
// Solution 1:
// Use delta rotations. This means anchors need to be prepared in world space. The delta rotation for static bodies will be identity.
// Base separation and angles need to be computed. Manifolds will be behind a frame, but that is probably best if bodies move fast.
//
// Solution 2:
// Use full rotation. The anchors for static bodies will be in world space while the anchors for dynamic bodies will be in local space.
// Potentially confusing and bug prone.

void b2PrepareRevoluteJoint(b2Joint* base, b2StepContext* context)
{
	B2_ASSERT(base->type == b2_revoluteJoint);

	// chase body id to the solver set where the body lives
	int idA = base->edges[0].bodyId;
	int idB = base->edges[1].bodyId;

	b2World* world = context->world;
	b2BodyLookup* lookup = world->bodyLookupArray;

	b2CheckIndex(lookup, idA);
	b2CheckIndex(lookup, idB);

	b2BodyLookup lookupA = lookup[idA];
	b2BodyLookup lookupB = lookup[idB];

	B2_ASSERT(lookupA.setIndex == b2_awakeSet || lookupB.setIndex == b2_awakeSet);
	b2CheckIndex(world->solverSetArray, lookupA.setIndex);
	b2CheckIndex(world->solverSetArray, lookupB.setIndex);

	b2SolverSet* setA = world->solverSetArray + lookupA.setIndex;
	b2SolverSet* setB = world->solverSetArray + lookupB.setIndex;

	B2_ASSERT(0 <= lookupA.bodyIndex && lookupA.bodyIndex <= setA->bodies.count);
	B2_ASSERT(0 <= lookupB.bodyIndex && lookupB.bodyIndex <= setB->bodies.count);

	b2Body* bodyA = setA->bodies.data + lookupA.bodyIndex;
	b2Body* bodyB = setB->bodies.data + lookupB.bodyIndex;

	float mA = bodyA->invMass;
	float iA = bodyA->invI;
	float mB = bodyB->invMass;
	float iB = bodyB->invI;

	base->invMassA = mA;
	base->invMassB = mB;
	base->invIA = iA;
	base->invIB = iB;

	b2RevoluteJoint* joint = &base->revoluteJoint;

	joint->indexA = lookupA.setIndex == b2_awakeSet ? lookupA.bodyIndex : B2_NULL_INDEX;
	joint->indexB = lookupB.setIndex == b2_awakeSet ? lookupB.bodyIndex : B2_NULL_INDEX;

	// initial anchors in world space
	joint->anchorA = b2RotateVector(bodyA->rotation, b2Sub(base->localOriginAnchorA, bodyA->localCenter));
	joint->anchorB = b2RotateVector(bodyB->rotation, b2Sub(base->localOriginAnchorB, bodyB->localCenter));
	joint->deltaCenter = b2Sub(bodyB->position, bodyA->position);
	joint->deltaAngle = b2RelativeAngle(bodyB->rotation, bodyA->rotation) - joint->referenceAngle;

	float k = iA + iB;
	joint->axialMass = k > 0.0f ? 1.0f / k : 0.0f;

	if (context->enableWarmStarting == false)
	{
		joint->linearImpulse = b2Vec2_zero;
		joint->motorImpulse = 0.0f;
		joint->lowerImpulse = 0.0f;
		joint->upperImpulse = 0.0f;
	}
}

void b2WarmStartRevoluteJoint(b2Joint* base, b2StepContext* context)
{
	B2_ASSERT(base->type == b2_revoluteJoint);

	float mA = base->invMassA;
	float mB = base->invMassB;
	float iA = base->invIA;
	float iB = base->invIB;

	// dummy state for static bodies
	b2BodyState dummyState = b2_identityBodyState;

	b2RevoluteJoint* joint = &base->revoluteJoint;
	b2BodyState* stateA = joint->indexA == B2_NULL_INDEX ? &dummyState : context->states + joint->indexA;
	b2BodyState* stateB = joint->indexB == B2_NULL_INDEX ? &dummyState : context->states + joint->indexB;

	b2Vec2 rA = b2RotateVector(stateA->deltaRotation, joint->anchorA);
	b2Vec2 rB = b2RotateVector(stateB->deltaRotation, joint->anchorB);

	float axialImpulse = joint->motorImpulse + joint->lowerImpulse - joint->upperImpulse;

	stateA->linearVelocity = b2MulSub(stateA->linearVelocity, mA, joint->linearImpulse);
	stateA->angularVelocity -= iA * (b2Cross(rA, joint->linearImpulse) + axialImpulse);

	stateB->linearVelocity = b2MulAdd(stateB->linearVelocity, mB, joint->linearImpulse);
	stateB->angularVelocity += iB * (b2Cross(rB, joint->linearImpulse) + axialImpulse);
}

void b2SolveRevoluteJoint(b2Joint* base, b2StepContext* context, bool useBias)
{
	B2_ASSERT(base->type == b2_revoluteJoint);

	float mA = base->invMassA;
	float mB = base->invMassB;
	float iA = base->invIA;
	float iB = base->invIB;

	// dummy state for static bodies
	b2BodyState dummyState = b2_identityBodyState;

	b2RevoluteJoint* joint = &base->revoluteJoint;

	b2BodyState* stateA = joint->indexA == B2_NULL_INDEX ? &dummyState : context->states + joint->indexA;
	b2BodyState* stateB = joint->indexB == B2_NULL_INDEX ? &dummyState : context->states + joint->indexB;

	b2Vec2 vA = stateA->linearVelocity;
	float wA = stateA->angularVelocity;
	b2Vec2 vB = stateB->linearVelocity;
	float wB = stateB->angularVelocity;

	bool fixedRotation = (iA + iB == 0.0f);
	//const float maxBias = context->maxBiasVelocity;

	// Solve motor constraint.
	if (joint->enableMotor && fixedRotation == false)
	{
		float Cdot = wB - wA - joint->motorSpeed;
		float impulse = -joint->axialMass * Cdot;
		float oldImpulse = joint->motorImpulse;
		float maxImpulse = context->h * joint->maxMotorTorque;
		joint->motorImpulse = B2_CLAMP(joint->motorImpulse + impulse, -maxImpulse, maxImpulse);
		impulse = joint->motorImpulse - oldImpulse;

		wA -= iA * impulse;
		wB += iB * impulse;
	}

	if (joint->enableLimit && fixedRotation == false)
	{
		float jointAngle = b2RelativeAngle(stateB->deltaRotation, stateA->deltaRotation) + joint->deltaAngle;

		// Lower limit
		{
			float C = jointAngle - joint->lowerAngle;
			float bias = 0.0f;
			float massScale = 1.0f;
			float impulseScale = 0.0f;
			if (C > 0.0f)
			{
				// speculation
				bias = C * context->inv_h;
			}
			else if (useBias)
			{
				bias = context->jointSoftness.biasRate * C;
				massScale = context->jointSoftness.massScale;
				impulseScale = context->jointSoftness.impulseScale;
			}

			float Cdot = wB - wA;
			float impulse = -massScale * joint->axialMass * (Cdot + bias) - impulseScale * joint->lowerImpulse;
			float oldImpulse = joint->lowerImpulse;
			joint->lowerImpulse = B2_MAX(joint->lowerImpulse + impulse, 0.0f);
			impulse = joint->lowerImpulse - oldImpulse;

			wA -= iA * impulse;
			wB += iB * impulse;
		}

		// Upper limit
		// Note: signs are flipped to keep C positive when the constraint is satisfied.
		// This also keeps the impulse positive when the limit is active.
		{
			float C = joint->upperAngle - jointAngle;
			float bias = 0.0f;
			float massScale = 1.0f;
			float impulseScale = 0.0f;
			if (C > 0.0f)
			{
				// speculation
				bias = C * context->inv_h;
			}
			else if (useBias)
			{
				bias = context->jointSoftness.biasRate * C;
				massScale = context->jointSoftness.massScale;
				impulseScale = context->jointSoftness.impulseScale;
			}

			// sign flipped on Cdot
			float Cdot = wA - wB;
			float impulse = -massScale * joint->axialMass * (Cdot + bias) - impulseScale * joint->lowerImpulse;
			float oldImpulse = joint->upperImpulse;
			joint->upperImpulse = B2_MAX(joint->upperImpulse + impulse, 0.0f);
			impulse = joint->upperImpulse - oldImpulse;

			// sign flipped on applied impulse
			wA += iA * impulse;
			wB -= iB * impulse;
		}
	}

	// Solve point-to-point constraint
	{
		// J = [-I -r1_skew I r2_skew]
		// r_skew = [-ry; rx]
		// K = [ mA+r1y^2*iA+mB+r2y^2*iB,  -r1y*iA*r1x-r2y*iB*r2x]
		//     [  -r1y*iA*r1x-r2y*iB*r2x, mA+r1x^2*iA+mB+r2x^2*iB]

		// current anchors
		b2Vec2 rA = b2RotateVector(stateA->deltaRotation, joint->anchorA);
		b2Vec2 rB = b2RotateVector(stateB->deltaRotation, joint->anchorB);

		b2Vec2 Cdot = b2Sub(b2Add(vB, b2CrossSV(wB, rB)), b2Add(vA, b2CrossSV(wA, rA)));

		b2Vec2 bias = b2Vec2_zero;
		float massScale = 1.0f;
		float impulseScale = 0.0f;
		if (useBias)
		{
			b2Vec2 dcA = stateA->deltaPosition;
			b2Vec2 dcB = stateB->deltaPosition;

			b2Vec2 separation = b2Add(b2Add(b2Sub(dcB, dcA), b2Sub(rB, rA)), joint->deltaCenter);
			bias = b2MulSV(context->jointSoftness.biasRate, separation);
			massScale = context->jointSoftness.massScale;
			impulseScale = context->jointSoftness.impulseScale;
		}

		b2Mat22 K;
		K.cx.x = mA + mB + rA.y * rA.y * iA + rB.y * rB.y * iB;
		K.cy.x = -rA.y * rA.x * iA - rB.y * rB.x * iB;
		K.cx.y = K.cy.x;
		K.cy.y = mA + mB + rA.x * rA.x * iA + rB.x * rB.x * iB;
		b2Vec2 b = b2Solve22(K, b2Add(Cdot, bias));
		
		b2Vec2 impulse;
		impulse.x = -massScale * b.x - impulseScale * joint->linearImpulse.x;
		impulse.y = -massScale * b.y - impulseScale * joint->linearImpulse.y;
		joint->linearImpulse.x += impulse.x;
		joint->linearImpulse.y += impulse.y;
		
		vA = b2MulSub(vA, mA, impulse);
		wA -= iA * b2Cross(rA, impulse);
		vB = b2MulAdd(vB, mB, impulse);
		wB += iB * b2Cross(rB, impulse);
	}

	stateA->linearVelocity = vA;
	stateA->angularVelocity = wA;
	stateB->linearVelocity = vB;
	stateB->angularVelocity = wB;
}

#if 0
void b2RevoluteJoint::Dump()
{
	int32 indexA = joint->bodyA->joint->islandIndex;
	int32 indexB = joint->bodyB->joint->islandIndex;

	b2Dump("  b2RevoluteJointDef jd;\n");
	b2Dump("  jd.bodyA = bodies[%d];\n", indexA);
	b2Dump("  jd.bodyB = bodies[%d];\n", indexB);
	b2Dump("  jd.collideConnected = bool(%d);\n", joint->collideConnected);
	b2Dump("  jd.localAnchorA.Set(%.9g, %.9g);\n", joint->localAnchorA.x, joint->localAnchorA.y);
	b2Dump("  jd.localAnchorB.Set(%.9g, %.9g);\n", joint->localAnchorB.x, joint->localAnchorB.y);
	b2Dump("  jd.referenceAngle = %.9g;\n", joint->referenceAngle);
	b2Dump("  jd.enableLimit = bool(%d);\n", joint->enableLimit);
	b2Dump("  jd.lowerAngle = %.9g;\n", joint->lowerAngle);
	b2Dump("  jd.upperAngle = %.9g;\n", joint->upperAngle);
	b2Dump("  jd.enableMotor = bool(%d);\n", joint->enableMotor);
	b2Dump("  jd.motorSpeed = %.9g;\n", joint->motorSpeed);
	b2Dump("  jd.maxMotorTorque = %.9g;\n", joint->maxMotorTorque);
	b2Dump("  joints[%d] = joint->world->CreateJoint(&jd);\n", joint->index);
}
#endif

void b2DrawRevolute(b2DebugDraw* draw, b2Joint* base, b2Body* bodyA, b2Body* bodyB)
{
	B2_ASSERT(base->type == b2_revoluteJoint);

	b2RevoluteJoint* joint = &base->revoluteJoint;

	b2Transform xfA = b2MakeTransform(bodyA);
	b2Transform xfB = b2MakeTransform(bodyB);
	b2Vec2 pA = b2TransformPoint(xfA, base->localOriginAnchorA);
	b2Vec2 pB = b2TransformPoint(xfB, base->localOriginAnchorB);

	b2Color c1 = {0.7f, 0.7f, 0.7f, 1.0f};
	b2Color c2 = {0.3f, 0.9f, 0.3f, 1.0f};
	b2Color c3 = {0.9f, 0.3f, 0.3f, 1.0f};
	b2Color c4 = {0.3f, 0.3f, 0.9f, 1.0f};
	b2Color c5 = {0.4f, 0.4f, 0.4f, 1.0f};

	draw->DrawPoint(pA, 5.0f, c4, draw->context);
	draw->DrawPoint(pB, 5.0f, c5, draw->context);

	float angle = b2RelativeAngle(bodyB->rotation, bodyA->rotation) - joint->referenceAngle;

	const float L = base->drawSize;
	b2Vec2 r = {L * cosf(angle), L * sinf(angle)};
	draw->DrawSegment(pB, b2Add(pB, r), c1, draw->context);
	draw->DrawCircle(pB, L, c1, draw->context);

	if (joint->enableLimit)
	{
		b2Vec2 rlo = {L * cosf(joint->lowerAngle), L * sinf(joint->lowerAngle)};
		b2Vec2 rhi = {L * cosf(joint->upperAngle), L * sinf(joint->upperAngle)};

		draw->DrawSegment(pB, b2Add(pB, rlo), c2, draw->context);
		draw->DrawSegment(pB, b2Add(pB, rhi), c3, draw->context);
	}

	b2Color color = {0.5f, 0.8f, 0.8f, 1.0f};
	draw->DrawSegment(xfA.p, pA, color, draw->context);
	draw->DrawSegment(pA, pB, color, draw->context);
	draw->DrawSegment(xfB.p, pB, color, draw->context);

	// char buffer[32];
	// sprintf(buffer, "%.1f", b2Length(joint->impulse));
	// draw->DrawString(pA, buffer, draw->context);
}
