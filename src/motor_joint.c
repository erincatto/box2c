// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "body.h"
#include "core.h"
#include "joint.h"
#include "solver_data.h"
#include "world.h"

#include "box2d/debug_draw.h"

// Point-to-point constraint
// C = p2 - p1
// Cdot = v2 - v1
//      = v2 + cross(w2, r2) - v1 - cross(w1, r1)
// J = [-I -r1_skew I r2_skew ]
// Identity used:
// w k % (rx i + ry j) = w * (-ry i + rx j)

// Angle constraint
// C = angle2 - angle1 - referenceAngle
// Cdot = w2 - w1
// J = [0 0 -1 0 0 1]
// K = invI1 + invI2

void b2PrepareMotorJoint(b2Joint* base, b2StepContext* context)
{
	B2_ASSERT(base->type == b2_motorJoint);

	int32_t indexA = base->edges[0].bodyIndex;
	int32_t indexB = base->edges[1].bodyIndex;
	B2_ASSERT(0 <= indexA && indexA < context->bodyCapacity);
	B2_ASSERT(0 <= indexB && indexB < context->bodyCapacity);

	b2Body* bodyA = context->bodies + indexA;
	b2Body* bodyB = context->bodies + indexB;
	B2_ASSERT(bodyA->object.index == bodyA->object.next);
	B2_ASSERT(bodyB->object.index == bodyB->object.next);

	float mA = bodyA->invMass;
	float iA = bodyA->invI;
	float mB = bodyB->invMass;
	float iB = bodyB->invI;

	b2MotorJoint* joint = &base->motorJoint;
	joint->indexA = context->bodyToSolverMap[indexA];
	joint->indexB = context->bodyToSolverMap[indexB];

	joint->rA = b2RotateVector(bodyA->transform.q, b2Sub(base->localAnchorA, bodyA->localCenter));
	joint->rB = b2RotateVector(bodyB->transform.q, b2Sub(base->localAnchorB, bodyB->localCenter));
	joint->linearSeparation = b2Sub(b2Add(b2Sub(joint->rB, joint->rA), b2Sub(bodyB->position, bodyA->position)), joint->linearOffset);
	joint->angularSeparation = bodyB->angle - bodyA->angle - joint->angularOffset;

	b2Vec2 rA = joint->rA;
	b2Vec2 rB = joint->rB;

	b2Mat22 K;
	K.cx.x = mA + mB + rA.y * rA.y * iA + rB.y * rB.y * iB;
	K.cx.y = -rA.y * rA.x * iA - rB.y * rB.x * iB;
	K.cy.x = K.cx.y;
	K.cy.y = mA + mB + rA.x * rA.x * iA + rB.x * rB.x * iB;
	joint->linearMass = b2GetInverse22(K);

	float ka = iA + iB;
	joint->angularMass = ka > 0.0f ? 1.0f / ka : 0.0f;

	if (context->enableWarmStarting)
	{
		float dtRatio = context->dtRatio;
		joint->linearImpulse.x *= dtRatio;
		joint->linearImpulse.y *= dtRatio;
		joint->angularImpulse *= dtRatio;
	}
	else
	{
		joint->linearImpulse = b2Vec2_zero;
		joint->angularImpulse = 0.0f;
	}
}

void b2WarmStartMotorJoint(b2Joint* base, b2StepContext* context)
{
	b2MotorJoint* joint = &base->motorJoint;

	b2SolverBody* bodyA = context->solverBodies + joint->indexA;
	b2Vec2 vA = bodyA->linearVelocity;
	float wA = bodyA->angularVelocity;
	float mA = bodyA->invMass;
	float iA = bodyA->invI;

	b2SolverBody* bodyB = context->solverBodies + joint->indexB;
	b2Vec2 vB = bodyB->linearVelocity;
	float wB = bodyB->angularVelocity;
	float mB = bodyB->invMass;
	float iB = bodyB->invI;

	vA = b2MulSub(vA, mA, joint->linearImpulse);
	wA -= iA * (b2Cross(joint->rA, joint->linearImpulse) + joint->angularImpulse);

	vB = b2MulAdd(vB, mB, joint->linearImpulse);
	wB += iB * (b2Cross(joint->rB, joint->linearImpulse) + joint->angularImpulse);

	bodyA->linearVelocity = vA;
	bodyA->angularVelocity = wA;
	bodyB->linearVelocity = vB;
	bodyB->angularVelocity = wB;
}

void b2SolveMotorJoint(b2Joint* base, const b2StepContext* context, bool useBias)
{
	if (useBias == false)
	{
		return;
	}

	B2_ASSERT(base->type == b2_motorJoint);

	b2MotorJoint* joint = &base->motorJoint;

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
	b2Vec2 linearSeparation = b2Add(joint->linearSeparation, ds);
	b2Vec2 linearBias = b2MulSV(context->inv_dt * joint->correctionFactor, linearSeparation);

	float angularSeperation = joint->angularSeparation + bodyB->deltaAngle - bodyA->deltaAngle;
	float angularBias = context->inv_dt * joint->correctionFactor * angularSeperation;

	// Note: don't relax user softness

	// Axial constraint
	{
		float Cdot = wB - wA;
		float impulse = -joint->angularMass * (Cdot + angularBias);

		float oldImpulse = joint->angularImpulse;
		float maxImpulse = context->dt * joint->maxTorque;
		joint->angularImpulse = B2_CLAMP(joint->angularImpulse + impulse, -maxImpulse, maxImpulse);
		impulse = joint->angularImpulse - oldImpulse;

		wA -= iA * impulse;
		wB += iB * impulse;
	}

	// Linear constraint
	{
		b2Vec2 Cdot = b2Sub(b2Add(vB, b2CrossSV(wB, rB)), b2Add(vA, b2CrossSV(wA, rA)));
		b2Vec2 b = b2MulMV(joint->linearMass, b2Add(Cdot, linearBias));
		b2Vec2 impulse = {-b.x, -b.y};

		b2Vec2 oldImpulse = joint->linearImpulse;
		float maxImpulse = context->dt * joint->maxForce; 
		joint->linearImpulse = b2Add(joint->linearImpulse, impulse);

		if (b2LengthSquared(joint->linearImpulse) > maxImpulse * maxImpulse)
		{
			joint->linearImpulse = b2Normalize(joint->linearImpulse);
			joint->linearImpulse.x *= maxImpulse;
			joint->linearImpulse.y *= maxImpulse;
		}

		impulse = b2Sub(joint->linearImpulse, oldImpulse);

		vA = b2MulSub(vA, mA, impulse);
		wA -= iA * b2Cross(rA, impulse);

		vB = b2MulAdd(vB, mB, impulse);
		wB += iB * b2Cross(rB, impulse);
	}

	bodyA->linearVelocity = vA;
	bodyA->angularVelocity = wA;
	bodyB->linearVelocity = vB;
	bodyB->angularVelocity = wB;
}

void b2MotorJoint_SetLinearOffset(b2JointId jointId, b2Vec2 linearOffset)
{
	b2World* world = b2GetWorldFromIndex(jointId.world);
	B2_ASSERT(world->locked == false);
	if (world->locked)
	{
		return;
	}

	b2Joint* joint = b2GetJoint(world, jointId);
	B2_ASSERT(joint->type == b2_motorJoint);

	joint->motorJoint.linearOffset = linearOffset;
}

void b2MotorJoint_SetAngularOffset(b2JointId jointId, float angularOffset)
{
	b2World* world = b2GetWorldFromIndex(jointId.world);
	B2_ASSERT(world->locked == false);
	if (world->locked)
	{
		return;
	}

	b2Joint* joint = b2GetJoint(world, jointId);
	B2_ASSERT(joint->type == b2_motorJoint);

	joint->motorJoint.angularOffset = angularOffset;
}

void b2MotorJoint_SetMaxForce(b2JointId jointId, float maxForce)
{
	b2World* world = b2GetWorldFromIndex(jointId.world);
	B2_ASSERT(world->locked == false);
	if (world->locked)
	{
		return;
	}

	b2Joint* joint = b2GetJoint(world, jointId);
	B2_ASSERT(joint->type == b2_motorJoint);

	joint->motorJoint.maxForce = B2_MAX(0.0f, maxForce);
}

void b2MotorJoint_SetMaxTorque(b2JointId jointId, float maxTorque)
{
	b2World* world = b2GetWorldFromIndex(jointId.world);
	B2_ASSERT(world->locked == false);
	if (world->locked)
	{
		return;
	}

	b2Joint* joint = b2GetJoint(world, jointId);
	B2_ASSERT(joint->type == b2_motorJoint);

	joint->motorJoint.maxTorque = B2_MAX(0.0f, maxTorque);
}

void b2MotorJoint_SetCorrectionFactor(b2JointId jointId, float correctionFactor)
{
	b2World* world = b2GetWorldFromIndex(jointId.world);
	B2_ASSERT(world->locked == false);
	if (world->locked)
	{
		return;
	}

	b2Joint* joint = b2GetJoint(world, jointId);
	B2_ASSERT(joint->type == b2_motorJoint);

	joint->motorJoint.correctionFactor = B2_CLAMP(correctionFactor, 0.0f, 1.0f);
}

b2Vec2 b2MotorJoint_GetConstraintForce(b2JointId jointId, float inverseTimeStep)
{
	b2World* world = b2GetWorldFromIndex(jointId.world);
	b2Joint* base = b2GetJoint(world, jointId);
	B2_ASSERT(base->type == b2_motorJoint);

	b2MotorJoint* joint = &base->motorJoint;
	b2Vec2 force = b2MulSV(inverseTimeStep, joint->linearImpulse);
	return force;
}

float b2MotorJoint_GetConstraintTorque(b2JointId jointId, float inverseTimeStep)
{
	b2World* world = b2GetWorldFromIndex(jointId.world);
	b2Joint* joint = b2GetJoint(world, jointId);
	B2_ASSERT(joint->type == b2_motorJoint);

	return inverseTimeStep * joint->motorJoint.angularImpulse;
}

#if 0
void b2DumpMotorJoint()
{
	int32 indexA = m_bodyA->m_islandIndex;
	int32 indexB = m_bodyB->m_islandIndex;

	b2Dump("  b2MotorJointDef jd;\n");
	b2Dump("  jd.bodyA = bodies[%d];\n", indexA);
	b2Dump("  jd.bodyB = bodies[%d];\n", indexB);
	b2Dump("  jd.collideConnected = bool(%d);\n", m_collideConnected);
	b2Dump("  jd.localAnchorA.Set(%.9g, %.9g);\n", m_localAnchorA.x, m_localAnchorA.y);
	b2Dump("  jd.localAnchorB.Set(%.9g, %.9g);\n", m_localAnchorB.x, m_localAnchorB.y);
	b2Dump("  jd.referenceAngle = %.9g;\n", m_referenceAngle);
	b2Dump("  jd.stiffness = %.9g;\n", m_stiffness);
	b2Dump("  jd.damping = %.9g;\n", m_damping);
	b2Dump("  joints[%d] = m_world->CreateJoint(&jd);\n", m_index);
}
#endif
