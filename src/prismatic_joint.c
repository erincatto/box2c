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

void b2PrismaticJoint_EnableLimit(b2JointId jointId, bool enableLimit)
{
	b2Joint* joint = b2GetJointCheckType(jointId, b2_prismaticJoint);
	if (enableLimit != joint->prismaticJoint.enableLimit)
	{
		joint->prismaticJoint.enableLimit = enableLimit;
		joint->prismaticJoint.lowerImpulse = 0.0f;
		joint->prismaticJoint.upperImpulse = 0.0f;
	}
}

bool b2PrismaticJoint_IsLimitEnabled(b2JointId jointId)
{
	b2Joint* joint = b2GetJointCheckType(jointId, b2_prismaticJoint);
	return joint->prismaticJoint.enableLimit;
}

float b2PrismaticJoint_GetLowerLimit(b2JointId jointId)
{
	b2Joint* joint = b2GetJointCheckType(jointId, b2_prismaticJoint);
	return joint->prismaticJoint.lowerTranslation;
}

float b2PrismaticJoint_GetUpperLimit(b2JointId jointId)
{
	b2Joint* joint = b2GetJointCheckType(jointId, b2_prismaticJoint);
	return joint->prismaticJoint.upperTranslation;
}

void b2PrismaticJoint_SetLimits(b2JointId jointId, float lower, float upper)
{
	b2Joint* joint = b2GetJointCheckType(jointId, b2_prismaticJoint);
	if (lower != joint->prismaticJoint.lowerTranslation || upper != joint->prismaticJoint.upperTranslation)
	{
		joint->prismaticJoint.lowerTranslation = B2_MIN(lower, upper);
		joint->prismaticJoint.upperTranslation = B2_MAX(lower, upper);
		joint->prismaticJoint.lowerImpulse = 0.0f;
		joint->prismaticJoint.upperImpulse = 0.0f;
	}
}

void b2PrismaticJoint_EnableMotor(b2JointId jointId, bool enableMotor)
{
	b2Joint* joint = b2GetJointCheckType(jointId, b2_prismaticJoint);
	if (enableMotor != joint->prismaticJoint.enableMotor)
	{
		joint->prismaticJoint.enableMotor = enableMotor;
		joint->prismaticJoint.motorImpulse = 0.0f;
	}
}

bool b2PrismaticJoint_IsMotorEnabled(b2JointId jointId)
{
	b2Joint* joint = b2GetJointCheckType(jointId, b2_prismaticJoint);
	return joint->prismaticJoint.enableMotor;
}

void b2PrismaticJoint_SetMotorSpeed(b2JointId jointId, float motorSpeed)
{
	b2Joint* joint = b2GetJointCheckType(jointId, b2_prismaticJoint);
	joint->prismaticJoint.motorSpeed = motorSpeed;
}

float b2PrismaticJoint_GetMotorSpeed(b2JointId jointId)
{
	b2Joint* joint = b2GetJointCheckType(jointId, b2_prismaticJoint);
	return joint->prismaticJoint.motorSpeed;
}

float b2PrismaticJoint_GetMotorForce(b2JointId jointId)
{
	b2World* world = b2GetWorldFromIndex(jointId.world);
	b2Joint* base = b2GetJoint(world, jointId);
	B2_ASSERT(base->type == b2_prismaticJoint);
	return world->inv_h * base->prismaticJoint.motorImpulse;
}

void b2PrismaticJoint_SetMaxMotorForce(b2JointId jointId, float force)
{
	b2Joint* joint = b2GetJointCheckType(jointId, b2_prismaticJoint);
	joint->prismaticJoint.maxMotorForce = force;
}

float b2PrismaticJoint_GetMaxMotorForce(b2JointId jointId)
{
	b2Joint* joint = b2GetJointCheckType(jointId, b2_prismaticJoint);
	return joint->prismaticJoint.maxMotorForce;
}

b2Vec2 b2PrismaticJoint_GetConstraintForce(b2JointId jointId)
{
	b2World* world = b2GetWorldFromIndex(jointId.world);
	b2Joint* base = b2GetJoint(world, jointId);
	B2_ASSERT(base->type == b2_prismaticJoint);

	int32_t indexA = base->edges[0].bodyIndex;
	b2Body* bodyA = world->bodies + indexA;
	B2_ASSERT(b2IsValidObject(&bodyA->object));

	b2PrismaticJoint* joint = &base->prismaticJoint;

	b2Vec2 axisA = b2RotateVector(bodyA->rotation, joint->localAxisA);
	b2Vec2 perpA = b2LeftPerp(axisA);

	float inv_h = world->inv_h;
	float perpForce = inv_h * joint->impulse.x;
	float axialForce = inv_h * (joint->motorImpulse + joint->lowerImpulse - joint->upperImpulse);

	b2Vec2 force = b2Add(b2MulSV(perpForce, perpA), b2MulSV(axialForce, axisA));
	return force;
}

float b2PrismaticJoint_GetConstraintTorque(b2JointId jointId)
{
	b2World* world = b2GetWorldFromIndex(jointId.world);
	b2Joint* joint = b2GetJoint(world, jointId);
	B2_ASSERT(joint->type == b2_prismaticJoint);

	return world->inv_h * joint->prismaticJoint.impulse.y;
}

// Linear constraint (point-to-line)
// d = p2 - p1 = x2 + r2 - x1 - r1
// C = dot(perp, d)
// Cdot = dot(d, cross(w1, perp)) + dot(perp, v2 + cross(w2, r2) - v1 - cross(w1, r1))
//      = -dot(perp, v1) - dot(cross(d + r1, perp), w1) + dot(perp, v2) + dot(cross(r2, perp), v2)
// J = [-perp, -cross(d + r1, perp), perp, cross(r2,perp)]
//
// Angular constraint
// C = a2 - a1 + a_initial
// Cdot = w2 - w1
// J = [0 0 -1 0 0 1]
//
// K = J * invM * JT
//
// J = [-a -s1 a s2]
//     [0  -1  0  1]
// a = perp
// s1 = cross(d + r1, a) = cross(p2 - x1, a)
// s2 = cross(r2, a) = cross(p2 - x2, a)

// Motor/Limit linear constraint
// C = dot(ax1, d)
// Cdot = -dot(ax1, v1) - dot(cross(d + r1, ax1), w1) + dot(ax1, v2) + dot(cross(r2, ax1), v2)
// J = [-ax1 -cross(d+r1,ax1) ax1 cross(r2,ax1)]

// Predictive limit is applied even when the limit is not active.
// Prevents a constraint speed that can lead to a constraint error in one time step.
// Want C2 = C1 + h * Cdot >= 0
// Or:
// Cdot + C1/h >= 0
// I do not apply a negative constraint error because that is handled in position correction.
// So:
// Cdot + max(C1, 0)/h >= 0

// Block Solver
// We develop a block solver that includes the angular and linear constraints. This makes the limit stiffer.
//
// The Jacobian has 2 rows:
// J = [-uT -s1 uT s2] // linear
//     [0   -1   0  1] // angular
//
// u = perp
// s1 = cross(d + r1, u), s2 = cross(r2, u)
// a1 = cross(d + r1, v), a2 = cross(r2, v)

void b2PreparePrismaticJoint(b2Joint* base, b2StepContext* context)
{
	B2_ASSERT(base->type == b2_prismaticJoint);

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

	b2PrismaticJoint* joint = &base->prismaticJoint;

	joint->indexA = bodyA->solverIndex;
	joint->indexB = bodyB->solverIndex;

	joint->anchorA = b2RotateVector(bodyA->rotation, b2Sub(base->localOriginAnchorA, bodyA->localCenter));
	joint->anchorB = b2RotateVector(bodyB->rotation, b2Sub(base->localOriginAnchorB, bodyB->localCenter));
	joint->axisA = b2RotateVector(bodyA->rotation, joint->localAxisA);
	joint->deltaCenter = b2Sub(bodyB->position, bodyA->position);
	joint->deltaAngle = b2RelativeAngle(bodyB->rotation, bodyA->rotation) - joint->referenceAngle;

	b2Vec2 rA = joint->anchorA;
	b2Vec2 rB = joint->anchorB;

	b2Vec2 d = b2Add(joint->deltaCenter, b2Sub(rB, rA));
	float a1 = b2Cross(b2Add(d, rA), joint->axisA);
	float a2 = b2Cross(rB, joint->axisA);

	// effective masses
	float k = mA + mB + iA * a1 * a1 + iB * a2 * a2;
	joint->axialMass = k > 0.0f ? 1.0f / k : 0.0f;

	if (context->enableWarmStarting == false)
	{
		joint->impulse = b2Vec2_zero;
		joint->motorImpulse = 0.0f;
		joint->lowerImpulse = 0.0f;
		joint->upperImpulse = 0.0f;
	}
}

void b2WarmStartPrismaticJoint(b2Joint* base, b2StepContext* context)
{
	B2_ASSERT(base->type == b2_prismaticJoint);

	float mA = base->invMassA;
	float mB = base->invMassB;
	float iA = base->invIA;
	float iB = base->invIB;

	// dummy state for static bodies
	b2BodyState dummyState = b2_identityBodyState;

	b2PrismaticJoint* joint = &base->prismaticJoint;

	b2BodyState* stateA = joint->indexA == B2_NULL_INDEX ? &dummyState : context->bodyStates + joint->indexA;
	b2BodyState* stateB = joint->indexB == B2_NULL_INDEX ? &dummyState : context->bodyStates + joint->indexB;

	b2Vec2 rA = b2RotateVector(stateA->deltaRotation, joint->anchorA);
	b2Vec2 rB = b2RotateVector(stateB->deltaRotation, joint->anchorB);

	b2Vec2 d = b2Add(b2Add(b2Sub(stateB->deltaPosition, stateA->deltaPosition), joint->deltaCenter), b2Sub(rB, rA));
	b2Vec2 axisA = b2RotateVector(stateA->deltaRotation, joint->axisA);

	// impulse is applied at anchor point on body B
	float a1 = b2Cross(b2Add(d, rA), axisA);
	float a2 = b2Cross(rB, axisA);
	float axialImpulse = joint->motorImpulse + joint->lowerImpulse - joint->upperImpulse;

	// perpendicular constraint
	b2Vec2 perpA = b2LeftPerp(axisA);
	float s1 = b2Cross(b2Add(d, rA), perpA);
	float s2 = b2Cross(rB, perpA);
	float perpImpulse = joint->impulse.x;
	float angleImpulse = joint->impulse.y;

	b2Vec2 P = b2Add(b2MulSV(axialImpulse, axisA), b2MulSV(perpImpulse, perpA));
	float LA = axialImpulse * a1 + perpImpulse * s1 + angleImpulse;
	float LB = axialImpulse * a2 + perpImpulse * s2 + angleImpulse;

	stateA->linearVelocity = b2MulSub(stateA->linearVelocity, mA, P);
	stateA->angularVelocity -= iA * LA;
	stateB->linearVelocity = b2MulAdd(stateB->linearVelocity, mB, P);
	stateB->angularVelocity += iB * LB;
}

void b2SolvePrismaticJoint(b2Joint* base, b2StepContext* context, bool useBias)
{
	B2_ASSERT(base->type == b2_prismaticJoint);

	float mA = base->invMassA;
	float mB = base->invMassB;
	float iA = base->invIA;
	float iB = base->invIB;

	// dummy state for static bodies
	b2BodyState dummyState = b2_identityBodyState;

	b2PrismaticJoint* joint = &base->prismaticJoint;

	b2BodyState* stateA = joint->indexA == B2_NULL_INDEX ? &dummyState : context->bodyStates + joint->indexA;
	b2BodyState* stateB = joint->indexB == B2_NULL_INDEX ? &dummyState : context->bodyStates + joint->indexB;

	b2Vec2 vA = stateA->linearVelocity;
	float wA = stateA->angularVelocity;
	b2Vec2 vB = stateB->linearVelocity;
	float wB = stateB->angularVelocity;

	// current anchors
	b2Vec2 rA = b2RotateVector(stateA->deltaRotation, joint->anchorA);
	b2Vec2 rB = b2RotateVector(stateB->deltaRotation, joint->anchorB);

	b2Vec2 d = b2Add(b2Add(b2Sub(stateB->deltaPosition, stateA->deltaPosition), joint->deltaCenter), b2Sub(rB, rA));
	b2Vec2 axisA = b2RotateVector(stateA->deltaRotation, joint->axisA);

	// These scalars are for torques generated by axial forces
	float a1 = b2Cross(b2Add(d, rA), axisA);
	float a2 = b2Cross(rB, axisA);

	// Solve motor constraint
	if (joint->enableMotor)
	{
		float Cdot = b2Dot(axisA, b2Sub(vB, vA)) + a2 * wB - a1 * wA;
		float impulse = joint->axialMass * (joint->motorSpeed - Cdot);
		float oldImpulse = joint->motorImpulse;
		float maxImpulse = context->h * joint->maxMotorForce;
		joint->motorImpulse = B2_CLAMP(joint->motorImpulse + impulse, -maxImpulse, maxImpulse);
		impulse = joint->motorImpulse - oldImpulse;

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

			float oldImpulse = joint->lowerImpulse;
			float Cdot = b2Dot(axisA, b2Sub(vB, vA)) + a2 * wB - a1 * wA;
			float impulse = -joint->axialMass * massScale * (Cdot + bias) - impulseScale * oldImpulse;
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

			float oldImpulse = joint->upperImpulse;
			// sign flipped
			float Cdot = b2Dot(axisA, b2Sub(vA, vB)) + a1 * wA - a2 * wB;
			float impulse = -joint->axialMass * massScale * (Cdot + bias) - impulseScale * oldImpulse;
			joint->upperImpulse = B2_MAX(oldImpulse + impulse, 0.0f);
			impulse = joint->upperImpulse - oldImpulse;

			b2Vec2 P = b2MulSV(impulse, axisA);
			float LA = impulse * a1;
			float LB = impulse * a2;

			// sign flipped
			vA = b2MulAdd(vA, mA, P);
			wA += iA * LA;
			vB = b2MulSub(vB, mB, P);
			wB -= iB * LB;
		}
	}

	// Solve the prismatic constraint in block form
	{
		b2Vec2 perpA = b2LeftPerp(axisA);

		// These scalars are for torques generated by the perpendicular constraint force
		float s1 = b2Cross(b2Add(d, rA), perpA);
		float s2 = b2Cross(rB, perpA);

		b2Vec2 Cdot;
		Cdot.x = b2Dot(perpA, b2Sub(vB, vA)) + s2 * wB - s1 * wA;
		Cdot.y = wB - wA;

		b2Vec2 bias = b2Vec2_zero;
		float massScale = 1.0f;
		float impulseScale = 0.0f;
		if (useBias)
		{
			b2Vec2 C;
			C.x = b2Dot(perpA, d);
			C.y = b2RelativeAngle(stateB->deltaRotation, stateA->deltaRotation) + joint->deltaAngle;

			bias = b2MulSV(context->jointSoftness.biasRate, C);
			massScale = context->jointSoftness.massScale;
			impulseScale = context->jointSoftness.impulseScale;
		}

		float k11 = mA + mB + iA * s1 * s1 + iB * s2 * s2;
		float k12 = iA * s1 + iB * s2;
		float k22 = iA + iB;
		if (k22 == 0.0f)
		{
			// For bodies with fixed rotation.
			k22 = 1.0f;
		}

		b2Mat22 K = {{k11, k12}, {k12, k22}};

		b2Vec2 b = b2Solve22(K, b2Add(Cdot, bias));
		b2Vec2 impulse;
		impulse.x = -massScale * b.x - impulseScale * joint->impulse.x;
		impulse.y = -massScale * b.y - impulseScale * joint->impulse.y;

		joint->impulse.x += impulse.x;
		joint->impulse.y += impulse.y;

		b2Vec2 P = b2MulSV(impulse.x, perpA);
		float LA = impulse.x * s1 + impulse.y;
		float LB = impulse.x * s2 + impulse.y;

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
void b2PrismaticJoint::Dump()
{
	int32 indexA = joint->bodyA->joint->islandIndex;
	int32 indexB = joint->bodyB->joint->islandIndex;

	b2Dump("  b2PrismaticJointDef jd;\n");
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

void b2DrawPrismatic(b2DebugDraw* draw, b2Joint* base, b2Body* bodyA, b2Body* bodyB)
{
	B2_ASSERT(base->type == b2_prismaticJoint);

	b2PrismaticJoint* joint = &base->prismaticJoint;

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
