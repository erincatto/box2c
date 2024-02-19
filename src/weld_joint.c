// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "body.h"
#include "core.h"
#include "joint.h"
#include "solver_data.h"
#include "world.h"

// needed for dll export
#include "box2d/box2d.h"

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

void b2PrepareWeldJoint(b2Joint* base, b2StepContext* context)
{
	B2_ASSERT(base->type == b2_weldJoint);

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

	b2WeldJoint* joint = &base->weldJoint;
	joint->indexA = context->bodyToSolverMap[indexA];
	joint->indexB = context->bodyToSolverMap[indexB];

	joint->anchorA = b2RotateVector(bodyA->rotation, b2Sub(base->localOriginAnchorA, bodyA->localCenter));
	joint->anchorB = b2RotateVector(bodyB->rotation, b2Sub(base->localOriginAnchorB, bodyB->localCenter));
	joint->deltaCenter = b2Sub(bodyB->position, bodyA->position);
	joint->deltaAngle = b2RelativeAngle(bodyB->rotation, bodyA->rotation) - joint->referenceAngle;

	float ka = iA + iB;
	joint->axialMass = ka > 0.0f ? 1.0f / ka : 0.0f;

	const float h = context->dt;

	if (joint->linearHertz == 0.0f)
	{
		joint->linearSoftness = context->jointSoftness;
	}
	else
	{
		joint->linearSoftness = b2MakeSoft(joint->linearHertz, joint->linearDampingRatio, context->h);
	}

	if (joint->angularHertz == 0.0f)
	{
		joint->angularSoftness = context->jointSoftness;
	}
	else
	{
		joint->angularSoftness = b2MakeSoft(joint->angularHertz, joint->angularDampingRatio, context->h);
	}

	if (context->enableWarmStarting == false)
	{
		joint->linearImpulse = b2Vec2_zero;
		joint->angularImpulse = 0.0f;
	}
}

void b2WarmStartWeldJoint(b2Joint* base, b2StepContext* context)
{
	float mA = base->invMassA;
	float mB = base->invMassB;
	float iA = base->invIA;
	float iB = base->invIB;

	// dummy state for static bodies
	b2BodyState dummyState = b2_identityBodyState;

	b2WeldJoint* joint = &base->weldJoint;

	b2BodyState* stateA = joint->indexA == B2_NULL_INDEX ? &dummyState : context->bodyStates + joint->indexA;
	b2BodyState* stateB = joint->indexB == B2_NULL_INDEX ? &dummyState : context->bodyStates + joint->indexB;

	b2Vec2 rA = b2RotateVector(stateA->deltaRotation, joint->anchorA);
	b2Vec2 rB = b2RotateVector(stateB->deltaRotation, joint->anchorB);

	stateA->linearVelocity = b2MulSub(stateA->linearVelocity, mA, joint->linearImpulse);
	stateA->angularVelocity -= iA * (b2Cross(rA, joint->linearImpulse) + joint->angularImpulse);

	stateB->linearVelocity = b2MulAdd(stateB->linearVelocity, mB, joint->linearImpulse);
	stateB->angularVelocity += iB * (b2Cross(rB, joint->linearImpulse) + joint->angularImpulse);
}

void b2SolveWeldJoint(b2Joint* base, const b2StepContext* context, bool useBias)
{
	B2_ASSERT(base->type == b2_weldJoint);

	float mA = base->invMassA;
	float mB = base->invMassB;
	float iA = base->invIA;
	float iB = base->invIB;

	// dummy state for static bodies
	b2BodyState dummyState = b2_identityBodyState;

	b2WeldJoint* joint = &base->weldJoint;

	b2BodyState* stateA = joint->indexA == B2_NULL_INDEX ? &dummyState : context->bodyStates + joint->indexA;
	b2BodyState* stateB = joint->indexB == B2_NULL_INDEX ? &dummyState : context->bodyStates + joint->indexB;

	b2Vec2 vA = stateA->linearVelocity;
	float wA = stateA->angularVelocity;
	b2Vec2 vB = stateB->linearVelocity;
	float wB = stateB->angularVelocity;

	// angular constraint
	{
		float bias = 0.0f;
		float massScale = 1.0f;
		float impulseScale = 0.0f;
		if (useBias || joint->angularHertz > 0.0f)
		{
			float C = b2RelativeAngle(stateB->deltaRotation, stateA->deltaRotation) + joint->deltaAngle;
			bias = joint->angularSoftness.biasRate * C;
			massScale = joint->angularSoftness.massScale;
			impulseScale = joint->angularSoftness.impulseScale;
		}

		float Cdot = wB - wA;
		float impulse = -massScale * joint->axialMass * (Cdot + bias) - impulseScale * joint->angularImpulse;
		joint->angularImpulse += impulse;
		wA -= iA * impulse;
		wB += iB * impulse;
	}

	// linear constraint
	{
		b2Vec2 rA = b2RotateVector(stateA->deltaRotation, joint->anchorA);
		b2Vec2 rB = b2RotateVector(stateB->deltaRotation, joint->anchorB);

		b2Vec2 bias = b2Vec2_zero;
		float massScale = 1.0f;
		float impulseScale = 0.0f;
		if (useBias || joint->linearHertz > 0.0f)
		{
			b2Vec2 dcA = stateA->deltaPosition;
			b2Vec2 dcB = stateB->deltaPosition;
			b2Vec2 C = b2Add(b2Add(b2Sub(dcB, dcA), b2Sub(rB, rA)), joint->deltaCenter);

			bias = b2MulSV(joint->linearSoftness.biasRate, C);
			massScale = joint->linearSoftness.massScale;
			impulseScale = joint->linearSoftness.impulseScale;
		}

		b2Vec2 Cdot = b2Sub(b2Add(vB, b2CrossSV(wB, rB)), b2Add(vA, b2CrossSV(wA, rA)));

		b2Mat22 K;
		K.cx.x = mA + mB + rA.y * rA.y * iA + rB.y * rB.y * iB;
		K.cy.x = -rA.y * rA.x * iA - rB.y * rB.x * iB;
		K.cx.y = K.cy.x;
		K.cy.y = mA + mB + rA.x * rA.x * iA + rB.x * rB.x * iB;
		b2Vec2 b = b2Solve22(K, b2Add(Cdot, bias));

		b2Vec2 impulse = {
			-massScale * b.x - impulseScale * joint->linearImpulse.x,
			-massScale * b.y - impulseScale * joint->linearImpulse.y,
		};

		joint->linearImpulse = b2Add(joint->linearImpulse, impulse);

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
void b2DumpWeldJoint()
{
	int32 indexA = m_bodyA->m_islandIndex;
	int32 indexB = m_bodyB->m_islandIndex;

	b2Dump("  b2WeldJointDef jd;\n");
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

void b2WeldJoint_SetLinearHertz(b2JointId jointId, float hertz)
{
	b2World* world = b2GetWorldFromIndex(jointId.world);
	b2Joint* joint = b2GetJoint(world, jointId);
	B2_ASSERT(joint->type == b2_weldJoint);
	B2_ASSERT(b2IsValid(hertz) && hertz >= 0.0f);

	joint->weldJoint.linearHertz = hertz;
}

void b2WeldJoint_SetLinearDampingRatio(b2JointId jointId, float dampingRatio)
{
	b2World* world = b2GetWorldFromIndex(jointId.world);
	b2Joint* joint = b2GetJoint(world, jointId);
	B2_ASSERT(joint->type == b2_weldJoint);
	B2_ASSERT(b2IsValid(dampingRatio) && dampingRatio >= 0.0f);

	joint->weldJoint.linearDampingRatio = dampingRatio;
}

void b2WeldJoint_SetAngularHertz(b2JointId jointId, float hertz)
{
	b2World* world = b2GetWorldFromIndex(jointId.world);
	b2Joint* joint = b2GetJoint(world, jointId);
	B2_ASSERT(joint->type == b2_weldJoint);
	B2_ASSERT(b2IsValid(hertz) && hertz >= 0.0f);

	joint->weldJoint.angularHertz = hertz;
}

void b2WeldJoint_SetAngularDampingRatio(b2JointId jointId, float dampingRatio)
{
	b2World* world = b2GetWorldFromIndex(jointId.world);
	b2Joint* joint = b2GetJoint(world, jointId);
	B2_ASSERT(joint->type == b2_weldJoint);
	B2_ASSERT(b2IsValid(dampingRatio) && dampingRatio >= 0.0f);

	joint->weldJoint.angularDampingRatio = dampingRatio;
}
