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

void b2PrepareWeld(b2Joint* base, b2StepContext* context)
{
	B2_ASSERT(base->type == b2_weldJoint);

	int32_t indexA = base->edges[0].bodyIndex;
	int32_t indexB = base->edges[1].bodyIndex;
	B2_ASSERT(0 <= indexA && indexA < context->bodyCapacity);
	B2_ASSERT(0 <= indexB && indexB < context->bodyCapacity);

	b2Body* bodyA = context->bodies + indexA;
	b2Body* bodyB = context->bodies + indexB;
	B2_ASSERT(bodyA->object.index == bodyA->object.next);
	B2_ASSERT(bodyB->object.index == bodyB->object.next);

	b2WeldJoint* joint = &base->weldJoint;
	joint->localCenterA = bodyA->localCenter;
	joint->invMassA = bodyA->invMass;
	joint->invIA = bodyA->invI;

	joint->localCenterB = bodyB->localCenter;
	joint->invMassB = bodyB->invMass;
	joint->invIB = bodyB->invI;

	const float h = context->dt;

	float linearHertz = joint->linearHertz;
	if (linearHertz == 0.0f)
	{
		linearHertz = 0.25f * context->velocityIterations * context->inv_dt;
	}

	{
		const float zeta = joint->linearDampingRatio;
		const float omega = 2.0f * b2_pi * linearHertz;
		joint->linearBiasCoefficient = omega / (2.0f * zeta + h * omega);
		float c = h * omega * (2.0f * zeta + h * omega);
		joint->linearImpulseCoefficient = 1.0f / (1.0f + c);
		joint->linearMassCoefficient = c * joint->linearImpulseCoefficient;
	}

	float angularHertz = joint->angularHertz;
	if (angularHertz == 0.0f)
	{
		angularHertz = 0.25f * context->velocityIterations * context->inv_dt;
	}

	{
		const float zeta = joint->angularDampingRatio;
		const float omega = 2.0f * b2_pi * angularHertz;
		joint->angularBiasCoefficient = omega / (2.0f * zeta + h * omega);
		float c = h * omega * (2.0f * zeta + h * omega);
		joint->angularImpulseCoefficient = 1.0f / (1.0f + c);
		joint->angularMassCoefficient = c * joint->angularImpulseCoefficient;
	}

	joint->impulse = b2Vec3_zero;
}

void b2SolveWeldVelocitySoft(b2Joint* base, const b2StepContext* context, bool removeOverlap)
{
	B2_ASSERT(base->type == b2_weldJoint);

	b2WeldJoint* joint = &base->weldJoint;

	b2Body* bodyA = context->bodies + base->edges[0].bodyIndex;
	b2Body* bodyB = context->bodies + base->edges[1].bodyIndex;

	b2Vec2 vA = bodyA->linearVelocity;
	float wA = bodyA->angularVelocity;
	b2Vec2 vB = bodyB->linearVelocity;
	float wB = bodyB->angularVelocity;

	const b2Vec2 cA = b2Add(bodyA->position, bodyA->deltaPosition);
	const float aA = bodyA->angle + bodyA->deltaAngle;
	const b2Vec2 cB = b2Add(bodyB->position, bodyB->deltaPosition);
	const float aB = bodyB->angle + bodyB->deltaAngle;

	float mA = joint->invMassA, mB = joint->invMassB;
	float iA = joint->invIA, iB = joint->invIB;

	b2Rot qA = b2MakeRot(aA);
	b2Rot qB = b2MakeRot(aB);

	b2Vec2 rA = b2RotateVector(qA, b2Sub(base->localAnchorA, joint->localCenterA));
	b2Vec2 rB = b2RotateVector(qB, b2Sub(base->localAnchorB, joint->localCenterB));

	b2Mat33 K;
	K.cx.x = mA + mB + rA.y * rA.y * iA + rB.y * rB.y * iB;
	K.cy.x = -rA.y * rA.x * iA - rB.y * rB.x * iB;
	K.cz.x = -rA.y * iA - rB.y * iB;
	K.cx.y = K.cy.x;
	K.cy.y = mA + mB + rA.x * rA.x * iA + rB.x * rB.x * iB;
	K.cz.y = rA.x * iA + rB.x * iB;
	K.cx.z = K.cz.x;
	K.cy.z = K.cz.y;
	K.cz.z = iA + iB;
	
	b2Vec2 Cdot1 = b2Add(b2Sub(vB, vA), b2Sub(b2CrossSV(wB, rB), b2CrossSV(wA, rA)));
	float Cdot2 = wB - wA;

	float linearBiasScale = 0.0f;
	float linearMassScale = 1.0f;
	float linearImpulseScale = 0.0f;
	float angularBiasScale = 0.0f;
	float angularMassScale = 1.0f;
	float angularImpulseScale = 0.0f;
	if (removeOverlap)
	{
		linearBiasScale = joint->linearBiasCoefficient;
		linearMassScale = joint->linearMassCoefficient;
		linearImpulseScale = joint->linearImpulseCoefficient;
		angularBiasScale = joint->angularBiasCoefficient;
		angularMassScale = joint->angularMassCoefficient;
		angularImpulseScale = joint->angularImpulseCoefficient;
	}

	b2Vec2 C1 = b2Add(b2Sub(cB, cA), b2Sub(rB, rA));
	float C2 = aB - aA - joint->referenceAngle;

	b2Vec3 c;
	c.x = Cdot1.x + linearBiasScale * C1.x;
	c.y = Cdot1.y + linearBiasScale * C1.y;
	c.z = Cdot2 + angularBiasScale * C2;

	b2Vec3 b = b2Solve33(K, c);
	b2Vec3 impulse;
	impulse.x = -linearMassScale * b.x - linearImpulseScale * joint->impulse.x;
	impulse.y = -linearMassScale * b.y - linearImpulseScale * joint->impulse.y;
	impulse.z = -angularMassScale * b.z - angularImpulseScale * joint->impulse.z;

	joint->impulse.x += impulse.x;
	joint->impulse.y += impulse.y;
	joint->impulse.z += impulse.z;

	b2Vec2 P = {impulse.x, impulse.y};

	vA = b2MulSub(vA, mA, P);
	wA -= iA * (b2Cross(rA, P) + impulse.z);

	vB = b2MulAdd(vB, mB, P);
	wB += iB * (b2Cross(rB, P) + impulse.z);

	bodyA->linearVelocity = vA;
	bodyA->angularVelocity = wA;
	bodyB->linearVelocity = vB;
	bodyB->angularVelocity = wB;
}

#if 0
void b2WeldJoint::Dump()
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
