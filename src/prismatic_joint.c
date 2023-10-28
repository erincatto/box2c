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

void b2PreparePrismatic(b2Joint* base, b2StepContext* context)
{
	B2_ASSERT(base->type == b2_prismaticJoint);

	int32_t indexA = base->edges[0].bodyIndex;
	int32_t indexB = base->edges[1].bodyIndex;
	b2Body* bodyA = context->bodies + indexA;
	b2Body* bodyB = context->bodies + indexB;
	B2_ASSERT(b2ObjectValid(&bodyA->object));
	B2_ASSERT(b2ObjectValid(&bodyB->object));

	b2PrismaticJoint* joint = &base->prismaticJoint;

	joint->indexA = context->bodyToSolverMap[indexA];
	joint->indexB = context->bodyToSolverMap[indexB];
	joint->localCenterA = bodyA->localCenter;
	joint->localCenterB = bodyB->localCenter;
	joint->positionA = bodyA->position;
	joint->positionB = bodyB->position;
	joint->angleA = bodyA->angle;
	joint->angleB = bodyB->angle;

	// This is a dummy body to represent a static body since static bodies don't have a solver body.
	b2SolverBody dummyBody = {0};

	// Note: must warm start solver bodies
	b2SolverBody* solverBodyA = joint->indexA == B2_NULL_INDEX ? &dummyBody : context->solverBodies + joint->indexA;
	float mA = solverBodyA->invMass;
	float iA = solverBodyA->invI;

	b2SolverBody* solverBodyB = joint->indexB == B2_NULL_INDEX ? &dummyBody : context->solverBodies + joint->indexB;
	float mB = solverBodyB->invMass;
	float iB = solverBodyB->invI;

	b2Rot qA = bodyA->transform.q;
	b2Rot qB = bodyB->transform.q;

	// Compute the effective masses.
	b2Vec2 rA = b2RotateVector(qA, b2Sub(base->localAnchorA, joint->localCenterA));
	b2Vec2 rB = b2RotateVector(qB, b2Sub(base->localAnchorB, joint->localCenterB));
	b2Vec2 d = b2Add(b2Sub(bodyB->position, bodyA->position),  b2Sub(rB, rA));

	b2Vec2 axis = b2RotateVector(qA, joint->localAxisA);
	float a1 = b2Cross(b2Add(d, rA), axis);
	float a2 = b2Cross(rB, axis);

	float k = mA + mB + iA * a1 * a1 + iB * a2 * a2;
	joint->axialMass = k > 0.0f ? 1.0f / k : 0.0f;

	// hertz = 1/4 * substep Hz
	const float hertz = 0.25f * context->velocityIterations * context->inv_dt;
	const float zeta = 1.0f;
	float omega = 2.0f * b2_pi * hertz;
	float h = context->dt;

	joint->biasCoefficient = omega / (2.0f * zeta + h * omega);
	float c = h * omega * (2.0f * zeta + h * omega);
	joint->impulseCoefficient = 1.0f / (1.0f + c);
	joint->massCoefficient = c * joint->impulseCoefficient;

	if (joint->enableLimit == false)
	{
		joint->lowerImpulse = 0.0f;
		joint->upperImpulse = 0.0f;
	}

	if (joint->enableMotor == false)
	{
		joint->motorImpulse = 0.0f;
	}

	if (context->enableWarmStarting)
	{
		float dtRatio = context->dtRatio;

		// Soft step works best when bilateral constraints have no warm starting.
		joint->impulse = b2Vec2_zero;
		joint->motorImpulse *= dtRatio;
		joint->lowerImpulse *= dtRatio;
		joint->upperImpulse *= dtRatio;

		float axialImpulse = joint->motorImpulse + joint->lowerImpulse - joint->upperImpulse;
\
		b2Vec2 P = b2MulSV(axialImpulse, axis);
		float LA = axialImpulse * a1;
		float LB = axialImpulse * a2;

		solverBodyA->linearVelocity = b2MulSub(solverBodyA->linearVelocity, mA, P);
		solverBodyA->angularVelocity -= iA * LA;

		solverBodyB->linearVelocity = b2MulAdd(solverBodyB->linearVelocity, mB, P);
		solverBodyB->angularVelocity += iB * LB;
	}
	else
	{
		joint->impulse = b2Vec2_zero;
		joint->motorImpulse = 0.0f;
		joint->lowerImpulse = 0.0f;
		joint->upperImpulse = 0.0f;
	}
}

void b2SolvePrismaticVelocity(b2Joint* base, b2StepContext* context, bool useBias)
{
	B2_ASSERT(base->type == b2_prismaticJoint);

	b2PrismaticJoint* joint = &base->prismaticJoint;

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

	const b2Vec2 cA = b2Add(joint->positionA, bodyA->deltaPosition);
	const float aA = joint->angleA + bodyA->deltaAngle;
	const b2Vec2 cB = b2Add(joint->positionB, bodyB->deltaPosition);
	const float aB = joint->angleB + bodyB->deltaAngle;

	b2Rot qA = b2MakeRot(aA);
	b2Rot qB = b2MakeRot(aB);

	b2Vec2 rA = b2RotateVector(qA, b2Sub(base->localAnchorA, joint->localCenterA));
	b2Vec2 rB = b2RotateVector(qB, b2Sub(base->localAnchorB, joint->localCenterB));
	b2Vec2 d = b2Add(b2Sub(cB, cA), b2Sub(rB, rA));

	b2Vec2 axis = b2RotateVector(qA, joint->localAxisA);
	float a1 = b2Cross(b2Add(d, rA), axis);
	float a2 = b2Cross(rB, axis);

	// Solve motor constraint
	if (joint->enableMotor)
	{
		float Cdot = b2Dot(axis, b2Sub(vB, vA)) + a2 * wB - a1 * wA;
		float impulse = joint->axialMass * (joint->motorSpeed - Cdot);
		float oldImpulse = joint->motorImpulse;
		float maxImpulse = context->dt * joint->maxMotorForce;
		joint->motorImpulse = B2_CLAMP(joint->motorImpulse + impulse, -maxImpulse, maxImpulse);
		impulse = joint->motorImpulse - oldImpulse;

		b2Vec2 P = b2MulSV(impulse, axis);
		float LA = impulse * a1;
		float LB = impulse * a2;

		vA = b2MulSub(vA, mA, P);
		wA -= iA * LA;
		vB = b2MulAdd(vB, mB, P);
		wB += iB * LB;
	}

	if (joint->enableLimit)
	{
		float translation = b2Dot(axis, d);

		// Lower limit
		{
			float C = translation - joint->lowerTranslation;
			float bias = 0.0f;
			float massScale = 1.0f;
			float impulseScale = 0.0f;

			if (C > 0.0f)
			{
				// speculation
				bias = C * context->inv_dt;
			}
			else if (useBias)
			{
				bias = joint->biasCoefficient * C;
				massScale = joint->massCoefficient;
				impulseScale = joint->impulseCoefficient;
			}

			float oldImpulse = joint->lowerImpulse;
			float Cdot = b2Dot(axis, b2Sub(vB, vA)) + a2 * wB - a1 * wA;
			float impulse = -joint->axialMass * massScale * (Cdot + bias) - impulseScale * oldImpulse;
			joint->lowerImpulse = B2_MAX(oldImpulse + impulse, 0.0f);
			impulse = joint->lowerImpulse - oldImpulse;

			b2Vec2 P = b2MulSV(impulse, axis);
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
				bias = C * context->inv_dt;
			}
			else if (useBias)
			{
				bias = joint->biasCoefficient * C;
				massScale = joint->massCoefficient;
				impulseScale = joint->impulseCoefficient;
			}

			float oldImpulse = joint->upperImpulse;
			// sign flipped
			float Cdot = b2Dot(axis, b2Sub(vA, vB)) + a1 * wA - a2 * wB;
			float impulse = -joint->axialMass * massScale * (Cdot + bias) - impulseScale * oldImpulse;
			joint->upperImpulse = B2_MAX(oldImpulse + impulse, 0.0f);
			impulse = joint->upperImpulse - oldImpulse;

			b2Vec2 P = b2MulSV(impulse, axis);
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
		b2Vec2 perp = b2LeftPerp(axis);

		float s1 = b2Cross(b2Add(d, rA), perp);
		float s2 = b2Cross(rB, perp);

		float k11 = mA + mB + iA * s1 * s1 + iB * s2 * s2;
		float k12 = iA * s1 + iB * s2;
		float k22 = iA + iB;
		if (k22 == 0.0f)
		{
			// For bodies with fixed rotation.
			k22 = 1.0f;
		}

		b2Mat22 K = {{k11, k12}, {k12, k22}};

		b2Vec2 Cdot;
		Cdot.x = b2Dot(perp, b2Sub(vB, vA)) + s2 * wB - s1 * wA;
		Cdot.y = wB - wA;

		b2Vec2 bias = b2Vec2_zero;
		float massScale = 1.0f;
		float impulseScale = 0.0f;
		if (useBias)
		{
			b2Vec2 C;
			C.x = b2Dot(perp, d);
			C.y = aB - aA - joint->referenceAngle;

			bias = b2MulSV(joint->biasCoefficient, C);
			massScale = joint->massCoefficient;
			impulseScale = joint->impulseCoefficient;
		}

		b2Vec2 b = b2Solve22(K, b2Add(Cdot, bias));
		b2Vec2 impulse;
		impulse.x = -massScale * b.x - impulseScale * joint->impulse.x;
		impulse.y = -massScale * b.y - impulseScale * joint->impulse.y;

		b2Vec2 P = b2MulSV(impulse.x, perp);
		float LA = impulse.x * s1 + impulse.y;
		float LB = impulse.x * s2 + impulse.y;

		vA = b2MulSub(vA, mA, P);
		wA -= iA * LA;
		vB = b2MulAdd(vB, mB, P);
		wB += iB * LB;
	}

	bodyA->linearVelocity = vA;
	bodyA->angularVelocity = wA;
	bodyB->linearVelocity = vB;
	bodyB->angularVelocity = wB;
}

void b2PrismaticJoint_EnableLimit(b2JointId jointId, bool enableLimit)
{
	b2World* world = b2GetWorldFromIndex(jointId.world);
	B2_ASSERT(world->locked == false);
	if (world->locked)
	{
		return;
	}

	B2_ASSERT(0 <= jointId.index && jointId.index < world->jointPool.capacity);

	b2Joint* joint = world->joints + jointId.index;
	B2_ASSERT(joint->object.index == joint->object.next);
	B2_ASSERT(joint->object.revision == jointId.revision);
	B2_ASSERT(joint->type == b2_prismaticJoint);
	joint->prismaticJoint.enableLimit = enableLimit;
}

void b2PrismaticJoint_EnableMotor(b2JointId jointId, bool enableMotor)
{
	b2World* world = b2GetWorldFromIndex(jointId.world);
	B2_ASSERT(world->locked == false);
	if (world->locked)
	{
		return;
	}

	B2_ASSERT(0 <= jointId.index && jointId.index < world->jointPool.capacity);

	b2Joint* joint = world->joints + jointId.index;
	B2_ASSERT(joint->object.index == joint->object.next);
	B2_ASSERT(joint->object.revision == jointId.revision);
	B2_ASSERT(joint->type == b2_prismaticJoint);
	joint->prismaticJoint.enableMotor = enableMotor;
}

void b2PrismaticJoint_SetMotorSpeed(b2JointId jointId, float motorSpeed)
{
	b2World* world = b2GetWorldFromIndex(jointId.world);
	B2_ASSERT(world->locked == false);
	if (world->locked)
	{
		return;
	}

	B2_ASSERT(0 <= jointId.index && jointId.index < world->jointPool.capacity);

	b2Joint* joint = world->joints + jointId.index;
	B2_ASSERT(joint->object.index == joint->object.next);
	B2_ASSERT(joint->object.revision == jointId.revision);
	B2_ASSERT(joint->type == b2_prismaticJoint);
	joint->prismaticJoint.motorSpeed = motorSpeed;
}

float b2PrismaticJoint_GetMotorForce(b2JointId jointId, float inverseTimeStep)
{
	b2World* world = b2GetWorldFromIndex(jointId.world);
	B2_ASSERT(world->locked == false);
	if (world->locked)
	{
		return 0.0f;
	}

	B2_ASSERT(0 <= jointId.index && jointId.index < world->jointPool.capacity);

	b2Joint* joint = world->joints + jointId.index;
	B2_ASSERT(joint->object.index == joint->object.next);
	B2_ASSERT(joint->object.revision == jointId.revision);
	B2_ASSERT(joint->type == b2_prismaticJoint);
	return inverseTimeStep * joint->prismaticJoint.motorImpulse;
}

void b2PrismaticJoint_SetMaxMotorForce(b2JointId jointId, float force)
{
	b2World* world = b2GetWorldFromIndex(jointId.world);
	B2_ASSERT(world->locked == false);
	if (world->locked)
	{
		return;
	}

	B2_ASSERT(0 <= jointId.index && jointId.index < world->jointPool.capacity);

	b2Joint* joint = world->joints + jointId.index;
	B2_ASSERT(joint->object.index == joint->object.next);
	B2_ASSERT(joint->object.revision == jointId.revision);
	B2_ASSERT(joint->type == b2_prismaticJoint);
	joint->prismaticJoint.maxMotorForce = force;
}

b2Vec2 b2PrismaticJoint_GetConstraintForce(b2JointId jointId)
{
	b2World* world = b2GetWorldFromIndex(jointId.world);
	B2_ASSERT(world->locked == false);
	if (world->locked)
	{
		return b2Vec2_zero;
	}

	B2_ASSERT(0 <= jointId.index && jointId.index < world->jointPool.capacity);

	b2Joint* joint = world->joints + jointId.index;
	B2_ASSERT(joint->object.index == joint->object.next);
	B2_ASSERT(joint->object.revision == jointId.revision);
	B2_ASSERT(joint->type == b2_prismaticJoint);
	return joint->prismaticJoint.impulse;
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

	b2Transform xfA = bodyA->transform;
	b2Transform xfB = bodyB->transform;
	b2Vec2 pA = b2TransformPoint(xfA, base->localAnchorA);
	b2Vec2 pB = b2TransformPoint(xfB, base->localAnchorB);

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
