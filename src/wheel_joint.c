// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "body.h"
#include "core.h"
#include "joint.h"
#include "solver.h"
#include "world.h"

// needed for dll export
#include "box2d/box2d.h"
#include "box2d/debug_draw.h"

#include <stdio.h>

void b2WheelJoint_SetSpringHertz(b2JointId jointId, float hertz)
{
	b2Joint* joint = b2GetJointCheckType(jointId, b2_wheelJoint);
	joint->wheelJoint.hertz = hertz;
}

float b2WheelJoint_GetSpringHertz(b2JointId jointId)
{
	b2Joint* joint = b2GetJointCheckType(jointId, b2_wheelJoint);
	return joint->wheelJoint.hertz;
}

void b2WheelJoint_SetSpringDampingRatio(b2JointId jointId, float dampingRatio)
{
	b2Joint* joint = b2GetJointCheckType(jointId, b2_wheelJoint);
	joint->wheelJoint.dampingRatio = dampingRatio;
}

float b2WheelJoint_GetSpringDampingRatio(b2JointId jointId)
{
	b2Joint* joint = b2GetJointCheckType(jointId, b2_wheelJoint);
	return joint->wheelJoint.dampingRatio;
}

void b2WheelJoint_EnableLimit(b2JointId jointId, bool enableLimit)
{
	b2Joint* joint = b2GetJointCheckType(jointId, b2_wheelJoint);
	if (joint->wheelJoint.enableLimit != enableLimit)
	{
		joint->wheelJoint.lowerImpulse = 0.0f;
		joint->wheelJoint.upperImpulse = 0.0f;
		joint->wheelJoint.enableLimit = enableLimit;
	}
}

bool b2WheelJoint_IsLimitEnabled(b2JointId jointId)
{
	b2Joint* joint = b2GetJointCheckType(jointId, b2_wheelJoint);
	return joint->wheelJoint.enableLimit;
}

float b2WheelJoint_GetLowerLimit(b2JointId jointId)
{
	b2Joint* joint = b2GetJointCheckType(jointId, b2_wheelJoint);
	return joint->wheelJoint.lowerTranslation;
}

float b2WheelJoint_GetUpperLimit(b2JointId jointId)
{
	b2Joint* joint = b2GetJointCheckType(jointId, b2_wheelJoint);
	return joint->wheelJoint.upperTranslation;
}

void b2WheelJoint_SetLimits(b2JointId jointId, float lower, float upper)
{
	b2Joint* joint = b2GetJointCheckType(jointId, b2_wheelJoint);
	if (lower != joint->wheelJoint.lowerTranslation || upper != joint->wheelJoint.upperTranslation)
	{
		joint->wheelJoint.lowerTranslation = B2_MIN(lower, upper);
		joint->wheelJoint.upperTranslation = B2_MAX(lower, upper);
		joint->wheelJoint.lowerImpulse = 0.0f;
		joint->wheelJoint.upperImpulse = 0.0f;
	}
}

void b2WheelJoint_EnableMotor(b2JointId jointId, bool enableMotor)
{
	b2Joint* joint = b2GetJointCheckType(jointId, b2_wheelJoint);
	if (joint->wheelJoint.enableMotor != enableMotor)
	{
		joint->wheelJoint.motorImpulse = 0.0f;
		joint->wheelJoint.enableMotor = enableMotor;
	}
}

bool b2WheelJoint_IsMotorEnabled(b2JointId jointId)
{
	b2Joint* joint = b2GetJointCheckType(jointId, b2_wheelJoint);
	return joint->wheelJoint.enableMotor;
}

void b2WheelJoint_SetMotorSpeed(b2JointId jointId, float motorSpeed)
{
	b2Joint* joint = b2GetJointCheckType(jointId, b2_wheelJoint);
	joint->wheelJoint.motorSpeed = motorSpeed;
}

float b2WheelJoint_GetMotorSpeed(b2JointId jointId)
{
	b2Joint* joint = b2GetJointCheckType(jointId, b2_wheelJoint);
	return joint->wheelJoint.motorSpeed;
}

float b2WheelJoint_GetMotorTorque(b2JointId jointId)
{
	b2World* world = b2GetWorldFromIndex(jointId.world);
	b2Joint* joint = b2GetJoint(world, jointId);
	B2_ASSERT(joint->type == b2_wheelJoint);

	return world->inv_h * joint->wheelJoint.motorImpulse;
}

void b2WheelJoint_SetMaxMotorTorque(b2JointId jointId, float torque)
{
	b2Joint* joint = b2GetJointCheckType(jointId, b2_wheelJoint);
	joint->wheelJoint.maxMotorTorque = torque;
}

float b2WheelJoint_GetMaxMotorTorque(b2JointId jointId)
{
	b2Joint* joint = b2GetJointCheckType(jointId, b2_wheelJoint);
	return joint->wheelJoint.maxMotorTorque;
}

b2Vec2 b2WheelJoint_GetConstraintForce(b2JointId jointId)
{
	b2World* world = b2GetWorldFromIndex(jointId.world);
	b2Joint* base = b2GetJoint(world, jointId);
	B2_ASSERT(base->type == b2_wheelJoint);

	b2WheelJoint* joint = &base->wheelJoint;

	// This is a frame behind
	b2Vec2 axisA = joint->axisA;
	b2Vec2 perpA = b2LeftPerp(axisA);

	float perpForce = world->inv_h * joint->perpImpulse;
	float axialForce = world->inv_h * (joint->springImpulse + joint->lowerImpulse - joint->upperImpulse);

	b2Vec2 force = b2Add(b2MulSV(perpForce, perpA), b2MulSV(axialForce, axisA));
	return force;
}

float b2WheelJoint_GetConstraintTorque(b2JointId jointId)
{
	b2World* world = b2GetWorldFromIndex(jointId.world);
	b2Joint* joint = b2GetJoint(world, jointId);
	B2_ASSERT(joint->type == b2_wheelJoint);

	return world->inv_h * joint->wheelJoint.motorImpulse;
}

// Linear constraint (point-to-line)
// d = pB - pA = xB + rB - xA - rA
// C = dot(ay, d)
// Cdot = dot(d, cross(wA, ay)) + dot(ay, vB + cross(wB, rB) - vA - cross(wA, rA))
//      = -dot(ay, vA) - dot(cross(d + rA, ay), wA) + dot(ay, vB) + dot(cross(rB, ay), vB)
// J = [-ay, -cross(d + rA, ay), ay, cross(rB, ay)]

// Spring linear constraint
// C = dot(ax, d)
// Cdot = = -dot(ax, vA) - dot(cross(d + rA, ax), wA) + dot(ax, vB) + dot(cross(rB, ax), vB)
// J = [-ax -cross(d+rA, ax) ax cross(rB, ax)]

// Motor rotational constraint
// Cdot = wB - wA
// J = [0 0 -1 0 0 1]

void b2PrepareWheelJoint(b2Joint* base, b2StepContext* context)
{
	B2_ASSERT(base->type == b2_wheelJoint);

	int32_t indexA = base->edges[0].bodyIndex;
	int32_t indexB = base->edges[1].bodyIndex;
	b2Body* bodyA = context->bodies + indexA;
	b2Body* bodyB = context->bodies + indexB;
	B2_ASSERT(b2IsValidObject(&bodyA->object));
	B2_ASSERT(b2IsValidObject(&bodyB->object));

	float mA = bodyA->invMass;
	float iA = bodyA->invI;
	float mB = bodyB->invMass;
	float iB = bodyB->invI;

	base->invMassA = mA;
	base->invMassB = mB;
	base->invIA = iA;
	base->invIB = iB;

	b2WheelJoint* joint = &base->wheelJoint;

	joint->indexA = bodyA->solverIndex;
	joint->indexB = bodyB->solverIndex;

	joint->anchorA = b2RotateVector(bodyA->rotation, b2Sub(base->localOriginAnchorA, bodyA->localCenter));
	joint->anchorB = b2RotateVector(bodyB->rotation, b2Sub(base->localOriginAnchorB, bodyB->localCenter));
	joint->axisA = b2RotateVector(bodyA->rotation, joint->localAxisA);
	joint->deltaCenter = b2Sub(bodyB->position, bodyA->position);

	b2Vec2 rA = joint->anchorA;
	b2Vec2 rB = joint->anchorB;

	b2Vec2 d = b2Add(joint->deltaCenter, b2Sub(rB, rA));
	b2Vec2 axisA = joint->axisA;
	b2Vec2 perpA = b2LeftPerp(axisA);

	// perpendicular constraint (keep wheel on line)
	float s1 = b2Cross(b2Add(d, rA), perpA);
	float s2 = b2Cross(rB, perpA);
	
	float kp = mA + mB + iA * s1 * s1 + iB * s2 * s2;
	joint->perpMass = kp > 0.0f ? 1.0f / kp : 0.0f;

	// spring constraint
	float a1 = b2Cross(b2Add(d, rA), axisA);
	float a2 = b2Cross(rB, axisA);

	float ka = mA + mB + iA * a1 * a1 + iB * a2 * a2;
	joint->axialMass = ka > 0.0f ? 1.0f / ka : 0.0f;

	joint->springSoftness = b2MakeSoft(joint->hertz, joint->dampingRatio, context->h);

	float km = iA + iB;
	joint->motorMass = km > 0.0f ? 1.0f / km : 0.0f;

	if (context->enableWarmStarting == false)
	{
		joint->perpImpulse = 0.0f;
		joint->springImpulse = 0.0f;
		joint->motorImpulse = 0.0f;
		joint->lowerImpulse = 0.0f;
		joint->upperImpulse = 0.0f;
	}
}

void b2WarmStartWheelJoint(b2Joint* base, b2StepContext* context)
{
	B2_ASSERT(base->type == b2_wheelJoint);

	float mA = base->invMassA;
	float mB = base->invMassB;
	float iA = base->invIA;
	float iB = base->invIB;

	// dummy state for static bodies
	b2BodyState dummyState = b2_identityBodyState;

	b2WheelJoint* joint = &base->wheelJoint;

	b2BodyState* stateA = joint->indexA == B2_NULL_INDEX ? &dummyState : context->bodyStates + joint->indexA;
	b2BodyState* stateB = joint->indexB == B2_NULL_INDEX ? &dummyState : context->bodyStates + joint->indexB;

	b2Vec2 rA = b2RotateVector(stateA->deltaRotation, joint->anchorA);
	b2Vec2 rB = b2RotateVector(stateB->deltaRotation, joint->anchorB);

	b2Vec2 d = b2Add(b2Add(b2Sub(stateB->deltaPosition, stateA->deltaPosition), joint->deltaCenter), b2Sub(rB, rA));
	b2Vec2 axisA = b2RotateVector(stateA->deltaRotation, joint->axisA);
	b2Vec2 perpA = b2LeftPerp(axisA);

	float a1 = b2Cross(b2Add(d, rA), axisA);
	float a2 = b2Cross(rB, axisA);
	float s1 = b2Cross(b2Add(d, rA), perpA);
	float s2 = b2Cross(rB, perpA);

	float axialImpulse = joint->springImpulse + joint->lowerImpulse - joint->upperImpulse;
	
	b2Vec2 P = b2Add(b2MulSV(axialImpulse, axisA), b2MulSV(joint->perpImpulse, perpA));
	float LA = axialImpulse * a1 + joint->perpImpulse * s1 + joint->motorImpulse;
	float LB = axialImpulse * a2 + joint->perpImpulse * s2 + joint->motorImpulse;

	stateA->linearVelocity = b2MulSub(stateA->linearVelocity, mA, P);
	stateA->angularVelocity -= iA * LA;
	stateB->linearVelocity = b2MulAdd(stateB->linearVelocity, mB, P);
	stateB->angularVelocity += iB * LB;
}

void b2SolveWheelJoint(b2Joint* base, b2StepContext* context, bool useBias)
{
	B2_ASSERT(base->type == b2_wheelJoint);

	float mA = base->invMassA;
	float mB = base->invMassB;
	float iA = base->invIA;
	float iB = base->invIB;

	// dummy state for static bodies
	b2BodyState dummyState = b2_identityBodyState;

	b2WheelJoint* joint = &base->wheelJoint;

	// This is a dummy body to represent a static body since static bodies don't have a solver body.
	b2BodyState dummyBody = {0};

	b2BodyState* stateA = joint->indexA == B2_NULL_INDEX ? &dummyState : context->bodyStates + joint->indexA;
	b2BodyState* stateB = joint->indexB == B2_NULL_INDEX ? &dummyState : context->bodyStates + joint->indexB;

	b2Vec2 vA = stateA->linearVelocity;
	float wA = stateA->angularVelocity;
	b2Vec2 vB = stateB->linearVelocity;
	float wB = stateB->angularVelocity;

	bool fixedRotation = (iA + iB == 0.0f);

	// current anchors
	b2Vec2 rA = b2RotateVector(stateA->deltaRotation, joint->anchorA);
	b2Vec2 rB = b2RotateVector(stateB->deltaRotation, joint->anchorB);

	b2Vec2 d = b2Add(b2Add(b2Sub(stateB->deltaPosition, stateA->deltaPosition), joint->deltaCenter), b2Sub(rB, rA));
	b2Vec2 axisA = b2RotateVector(stateA->deltaRotation, joint->axisA);
	float translation = b2Dot(axisA, d);

	float a1 = b2Cross(b2Add(d, rA), axisA);
	float a2 = b2Cross(rB, axisA);

	// motor constraint
	if (joint->enableMotor && fixedRotation == false)
	{
		float Cdot = wB - wA - joint->motorSpeed;
		float impulse = -joint->motorMass * Cdot;
		float oldImpulse = joint->motorImpulse;
		float maxImpulse = context->h * joint->maxMotorTorque;
		joint->motorImpulse = B2_CLAMP(joint->motorImpulse + impulse, -maxImpulse, maxImpulse);
		impulse = joint->motorImpulse - oldImpulse;

		wA -= iA * impulse;
		wB += iB * impulse;
	}

	// spring constraint
	{
		// This is a real spring and should be applied even during relax
		float C = translation;
		float bias = joint->springSoftness.biasRate * C;
		float massScale = joint->springSoftness.massScale;
		float impulseScale = joint->springSoftness.impulseScale;

		float Cdot = b2Dot(axisA, b2Sub(vB, vA)) + a2 * wB - a1 * wA;
		float impulse = -massScale * joint->axialMass * (Cdot + bias) - impulseScale * joint->springImpulse;
		joint->springImpulse += impulse;

		b2Vec2 P = b2MulSV(impulse, axisA);
		float LA = impulse * a1;
		float LB = impulse * a2;

		vA = b2MulSub(vA, mA, P);
		wA -= iA * LA;
		vB = b2MulAdd(vB, mB, P);
		wB += iB * LB;
	}

	if (joint->enableLimit)
	{
		float translation = b2Dot(axisA, d);

		// Lower limit
		{
			float C = translation - joint->lowerTranslation;
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

			float Cdot = b2Dot(axisA, b2Sub(vB, vA)) + a2 * wB - a1 * wA;
			float impulse = -massScale * joint->axialMass * (Cdot + bias) - impulseScale * joint->lowerImpulse;
			float oldImpulse = joint->lowerImpulse;
			joint->lowerImpulse = B2_MAX(oldImpulse + impulse, 0.0f);
			impulse = joint->lowerImpulse - oldImpulse;

			b2Vec2 P = b2MulSV(impulse, axisA);
			float LA = impulse * a1;
			float LB = impulse * a2;

			vA = b2MulSub(vA, mA, P);
			wA -= iA * LA;
			vB = b2MulAdd(vB, mB, P);
			wB += iB * LB;
		}

		// Upper limit
		// Note: signs are flipped to keep C positive when the constraint is satisfied.
		// This also keeps the impulse positive when the limit is active.
		{
			// sign flipped
			float C = joint->upperTranslation - translation;
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
			float Cdot = b2Dot(axisA, b2Sub(vA, vB)) + a1 * wA - a2 * wB;
			float impulse = -massScale * joint->axialMass * (Cdot + bias) - impulseScale * joint->upperImpulse;
			float oldImpulse = joint->upperImpulse;
			joint->upperImpulse = B2_MAX(oldImpulse + impulse, 0.0f);
			impulse = joint->upperImpulse - oldImpulse;

			b2Vec2 P = b2MulSV(impulse, axisA);
			float LA = impulse * a1;
			float LB = impulse * a2;

			// sign flipped on applied impulse
			vA = b2MulAdd(vA, mA, P);
			wA += iA * LA;
			vB = b2MulSub(vB, mB, P);
			wB -= iB * LB;
		}
	}

	// point to line constraint
	{
		b2Vec2 perpA = b2LeftPerp(axisA);

		float bias = 0.0f;
		float massScale = 1.0f;
		float impulseScale = 0.0f;
		if (useBias)
		{
			float C = b2Dot(perpA, d);
			bias = context->jointSoftness.biasRate * C;
			massScale = context->jointSoftness.massScale;
			impulseScale = context->jointSoftness.impulseScale;
		}

		float s1 = b2Cross(b2Add(d, rA), perpA);
		float s2 = b2Cross(rB, perpA);
		float Cdot = b2Dot(perpA, b2Sub(vB, vA)) + s2 * wB - s1 * wA;

		float impulse = -massScale * joint->perpMass * (Cdot + bias) - impulseScale * joint->perpImpulse;
		joint->perpImpulse += impulse;

		b2Vec2 P = b2MulSV(impulse, perpA);
		float LA = impulse * s1;
		float LB = impulse * s2;

		vA = b2MulSub(vA, mA, P);
		wA -= iA * LA;
		vB = b2MulAdd(vB, mB, P);
		wB += iB * LB;
	}

	stateA->linearVelocity = vA;
	stateA->angularVelocity = wA;
	stateB->linearVelocity = vB;
	stateB->angularVelocity = wB;
}

#if 0
void b2WheelJoint_Dump()
{
	int32 indexA = joint->bodyA->joint->islandIndex;
	int32 indexB = joint->bodyB->joint->islandIndex;

	b2Dump("  b2WheelJointDef jd;\n");
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

void b2DrawWheelJoint(b2DebugDraw* draw, b2Joint* base, b2Body* bodyA, b2Body* bodyB)
{
	B2_ASSERT(base->type == b2_wheelJoint);

	b2WheelJoint* joint = &base->wheelJoint;

	b2Transform xfA = b2MakeTransform(bodyA);
	b2Transform xfB = b2MakeTransform(bodyB);
	b2Vec2 pA = b2TransformPoint(xfA, base->localOriginAnchorA);
	b2Vec2 pB = b2TransformPoint(xfB, base->localOriginAnchorB);

	b2Vec2 axis = b2RotateVector(xfA.q, joint->localAxisA);

	b2Color c1 = {0.7f, 0.7f, 0.7f, 1.0f};
	b2Color c2 = {0.3f, 0.9f, 0.3f, 1.0f};
	b2Color c3 = {0.9f, 0.3f, 0.3f, 1.0f};
	b2Color c4 = {0.3f, 0.3f, 0.9f, 1.0f};
	b2Color c5 = {0.4f, 0.4f, 0.4f, 1.0f};

	draw->DrawSegment(pA, pB, c5, draw->context);

	if (joint->enableLimit)
	{
		b2Vec2 lower = b2MulAdd(pA, joint->lowerTranslation, axis);
		b2Vec2 upper = b2MulAdd(pA, joint->upperTranslation, axis);
		b2Vec2 perp = b2LeftPerp(axis);
		draw->DrawSegment(lower, upper, c1, draw->context);
		draw->DrawSegment(b2MulSub(lower, 0.5f, perp), b2MulAdd(lower, 0.5f, perp), c2, draw->context);
		draw->DrawSegment(b2MulSub(upper, 0.5f, perp), b2MulAdd(upper, 0.5f, perp), c3, draw->context);
	}
	else
	{
		draw->DrawSegment(b2MulSub(pA, 1.0f, axis), b2MulAdd(pA, 1.0f, axis), c1, draw->context);
	}

	draw->DrawPoint(pA, 5.0f, c1, draw->context);
	draw->DrawPoint(pB, 5.0f, c4, draw->context);
}
