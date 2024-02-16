// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "body.h"
#include "core.h"
#include "joint.h"
#include "solver_data.h"
#include "world.h"

// needed for dll export
#include "box2d/box2d.h"
#include "box2d/debug_draw.h"

#include <stdio.h>

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
	B2_ASSERT(b2ObjectValid(&bodyA->object));
	B2_ASSERT(b2ObjectValid(&bodyB->object));

	float mA = bodyA->invMass;
	float iA = bodyA->invI;
	float mB = bodyB->invMass;
	float iB = bodyB->invI;

	base->invMassA = mA;
	base->invMassB = mB;
	base->invIA = iA;
	base->invIB = iB;

	b2WheelJoint* joint = &base->wheelJoint;

	joint->localAnchorA = b2Sub(base->localOriginAnchorA, bodyA->localCenter);
	joint->localAnchorB = b2Sub(base->localOriginAnchorB, bodyB->localCenter);
	joint->deltaCenter = b2Sub(bodyB->position, bodyA->position);

	b2Rot qA = bodyA->rotation;
	b2Rot qB = bodyB->rotation;

	// compute the effective masses.
	b2Vec2 rA = b2RotateVector(qA, joint->localAnchorA);
	b2Vec2 rB = b2RotateVector(qB, joint->localAnchorB);

	b2Vec2 d = b2Add(joint->deltaCenter, b2Sub(rB, rA));

	b2Vec2 axisA = b2RotateVector(qA, joint->localAxisA);
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

	joint->springSoftness = b2MakeSoft(joint->)
	if (joint->stiffness > 0.0f && ka > 0.0f)
	{
		float C = b2Dot(d, axisA);

		float dt = context->dt;
		joint->gamma = dt * (joint->damping + dt * joint->stiffness);
		joint->gamma = joint->gamma > 0.0f ? 1.0f / joint->gamma : 0.0f;

		joint->bias = dt * C * joint->stiffness * joint->gamma;

		float ks = ka + joint->gamma;
		joint->springMass = ks > 0.0f ? 1.0f / ks : 0.0f;
	}

	float km = iA + iB;
	joint->motorMass = km > 0.0f ? 1.0f / km : 0.0f;

	//// hertz = 1/4 * substep Hz
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
		joint->perpImpulse = 0.0f;
		joint->motorImpulse *= dtRatio;
		joint->springImpulse *= dtRatio;
		joint->lowerImpulse *= dtRatio;
		joint->upperImpulse *= dtRatio;
	}
	else
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

	b2WheelJoint* joint = &base->wheelJoint;

	// This is a dummy body to represent a static body since static bodies don't have a solver body.
	b2BodyState dummyBody = {0};

	// Note: must warm start solver bodies
	b2BodyState* bodyA = joint->indexA == B2_NULL_INDEX ? &dummyBody : context->solverBodies + joint->indexA;
	float mA = bodyA->invMass;
	float iA = bodyA->invI;

	b2BodyState* bodyB = joint->indexB == B2_NULL_INDEX ? &dummyBody : context->solverBodies + joint->indexB;
	float mB = bodyB->invMass;
	float iB = bodyB->invI;

	b2Vec2 rA = joint->rA;
	b2Vec2 rB = joint->rB;
	b2Vec2 d = joint->pivotSeparation;

	b2Vec2 axisA = joint->axisA;
	float a1 = b2Cross(b2Add(d, rA), axisA);
	float a2 = b2Cross(rB, axisA);

	float axialImpulse = joint->springImpulse + joint->lowerImpulse - joint->upperImpulse;

	b2Vec2 P = b2MulSV(axialImpulse, axisA);
	float LA = axialImpulse * a1 + joint->motorImpulse;
	float LB = axialImpulse * a2 + joint->motorImpulse;

	bodyA->linearVelocity = b2MulSub(bodyA->linearVelocity, mA, P);
	bodyA->angularVelocity -= iA * LA;
	bodyB->linearVelocity = b2MulAdd(bodyB->linearVelocity, mB, P);
	bodyB->angularVelocity += iB * LB;
}

void b2SolveWheelJoint(b2Joint* base, b2StepContext* context, bool useBias)
{
	B2_ASSERT(base->type == b2_wheelJoint);

	b2WheelJoint* joint = &base->wheelJoint;

	// This is a dummy body to represent a static body since static bodies don't have a solver body.
	b2BodyState dummyBody = {0};

	b2BodyState* bodyA = joint->indexA == B2_NULL_INDEX ? &dummyBody : context->solverBodies + joint->indexA;
	b2Vec2 vA = bodyA->linearVelocity;
	float wA = bodyA->angularVelocity;
	float mA = bodyA->invMass;
	float iA = bodyA->invI;

	b2BodyState* bodyB = joint->indexB == B2_NULL_INDEX ? &dummyBody : context->solverBodies + joint->indexB;
	b2Vec2 vB = bodyB->linearVelocity;
	float wB = bodyB->angularVelocity;
	float mB = bodyB->invMass;
	float iB = bodyB->invI;

	bool fixedRotation = (iA + iB == 0.0f);

	// Small angle approximation
	b2Vec2 drA = b2CrossSV(bodyA->deltaAngle, joint->rA);
	b2Vec2 drB = b2CrossSV(bodyB->deltaAngle, joint->rB);

	b2Vec2 rA = b2Add(joint->rA, drA);
	b2Vec2 rB = b2Add(joint->rB, drB);

	b2Vec2 d = b2Add(joint->pivotSeparation, b2Sub(drB, drA));

	float dAngleA = bodyA->deltaAngle;
	
	// Small angle approximation
	b2Vec2 axisA = {joint->axisA.x - dAngleA * joint->axisA.y, dAngleA * joint->axisA.x + joint->axisA.y};
	axisA = b2Normalize(axisA);

	// Solve motor constraint
	if (joint->enableMotor && fixedRotation == false)
	{
		float Cdot = wB - wA - joint->motorSpeed;
		float impulse = -joint->motorMass * Cdot;
		float oldImpulse = joint->motorImpulse;
		float maxImpulse = context->dt * joint->maxMotorTorque;
		joint->motorImpulse = B2_CLAMP(joint->motorImpulse + impulse, -maxImpulse, maxImpulse);
		impulse = joint->motorImpulse - oldImpulse;

		wA -= iA * impulse;
		wB += iB * impulse;
	}

	float a1 = b2Cross(b2Add(d, rA), axisA);
	float a2 = b2Cross(rB, axisA);

	// Solve spring constraint
	{
		float Cdot = b2Dot(axisA, b2Sub(vB, vA)) + a2 * wB - a1 * wA;
		float impulse = -joint->springMass * (Cdot + joint->bias + joint->gamma * joint->springImpulse);
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
				bias = C * context->inv_dt;
			}
			else if (useBias)
			{
				bias = joint->biasCoefficient * C;
				massScale = joint->massCoefficient;
				impulseScale = joint->impulseCoefficient;
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

		float s1 = b2Cross(b2Add(d, rA), perpA);
		float s2 = b2Cross(rB, perpA);

		float Cdot = b2Dot(perpA, b2Sub(vB, vA)) + s2 * wB - s1 * wA;

		float bias = 0.0f;
		float massScale = 1.0f;
		float impulseScale = 0.0f;
		if (useBias)
		{
			float C = b2Dot(perpA, d);
			bias = joint->biasCoefficient * C;
			massScale = joint->massCoefficient;
			impulseScale = joint->impulseCoefficient;
		}

		float oldImpulse = joint->perpImpulse;
		float impulse = -joint->perpMass * massScale * (Cdot + bias) - impulseScale * oldImpulse;
		joint->perpImpulse = impulse;

		b2Vec2 P = b2MulSV(impulse, perpA);
		float LA = impulse * s1;
		float LB = impulse * s2;

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

void b2WheelJoint_SetStiffness(b2JointId jointId, float stiffness)
{
	b2World* world = b2GetWorldFromIndex(jointId.world);
	B2_ASSERT(world->locked == false);
	if (world->locked)
	{
		return;
	}

	b2Joint* joint = b2GetJoint(world, jointId);
	B2_ASSERT(joint->type == b2_wheelJoint);

	joint->wheelJoint.stiffness = stiffness;
}

void b2WheelJoint_SetDamping(b2JointId jointId, float damping)
{
	b2World* world = b2GetWorldFromIndex(jointId.world);
	B2_ASSERT(world->locked == false);
	if (world->locked)
	{
		return;
	}

	b2Joint* joint = b2GetJoint(world, jointId);
	B2_ASSERT(joint->type == b2_wheelJoint);

	joint->wheelJoint.damping = damping;
}

void b2WheelJoint_EnableLimit(b2JointId jointId, bool enableLimit)
{
	b2World* world = b2GetWorldFromIndex(jointId.world);
	B2_ASSERT(world->locked == false);
	if (world->locked)
	{
		return;
	}

	b2Joint* joint = b2GetJoint(world, jointId);
	B2_ASSERT(joint->type == b2_wheelJoint);

	joint->wheelJoint.enableLimit = enableLimit;
}

void b2WheelJoint_EnableMotor(b2JointId jointId, bool enableMotor)
{
	b2World* world = b2GetWorldFromIndex(jointId.world);
	B2_ASSERT(world->locked == false);
	if (world->locked)
	{
		return;
	}

	b2Joint* joint = b2GetJoint(world, jointId);
	B2_ASSERT(joint->type == b2_wheelJoint);

	joint->wheelJoint.enableMotor = enableMotor;
}

void b2WheelJoint_SetMotorSpeed(b2JointId jointId, float motorSpeed)
{
	b2World* world = b2GetWorldFromIndex(jointId.world);
	B2_ASSERT(world->locked == false);
	if (world->locked)
	{
		return;
	}

	b2Joint* joint = b2GetJoint(world, jointId);
	B2_ASSERT(joint->type == b2_wheelJoint);

	joint->wheelJoint.motorSpeed = motorSpeed;
}

float b2WheelJoint_GetMotorTorque(b2JointId jointId, float inverseTimeStep)
{
	b2World* world = b2GetWorldFromIndex(jointId.world);
	b2Joint* joint = b2GetJoint(world, jointId);
	B2_ASSERT(joint->type == b2_wheelJoint);

	return inverseTimeStep * joint->wheelJoint.motorImpulse;
}

void b2WheelJoint_SetMaxMotorTorque(b2JointId jointId, float torque)
{
	b2World* world = b2GetWorldFromIndex(jointId.world);
	B2_ASSERT(world->locked == false);
	if (world->locked)
	{
		return;
	}

	b2Joint* joint = b2GetJoint(world, jointId);
	B2_ASSERT(joint->type == b2_wheelJoint);

	joint->wheelJoint.maxMotorTorque = torque;
}

b2Vec2 b2WheelJoint_GetConstraintForce(b2JointId jointId, float inverseTimeStep)
{
	b2World* world = b2GetWorldFromIndex(jointId.world);
	b2Joint* base = b2GetJoint(world, jointId);
	B2_ASSERT(base->type == b2_wheelJoint);

	b2WheelJoint* joint = &base->wheelJoint;

	// This is a frame behind
	b2Vec2 axisA = joint->axisA;
	b2Vec2 perpA = b2LeftPerp(axisA);

	float perpForce = inverseTimeStep * joint->perpImpulse;
	float axialForce = inverseTimeStep * (joint->springImpulse + joint->lowerImpulse - joint->upperImpulse);

	b2Vec2 force = b2Add(b2MulSV(perpForce, perpA), b2MulSV(axialForce, axisA));
	return force;
}

float b2WheelJoint_GetConstraintTorque(b2JointId jointId, float inverseTimeStep)
{
	b2World* world = b2GetWorldFromIndex(jointId.world);
	b2Joint* joint = b2GetJoint(world, jointId);
	B2_ASSERT(joint->type == b2_wheelJoint);

	return inverseTimeStep * joint->wheelJoint.motorImpulse;
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
