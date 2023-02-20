// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "box2d/debug_draw.h"

#include "body.h"
#include "joint.h"
#include "solver_data.h"
#include "world.h"

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

void b2InitializeRevolute(b2World* world, b2Joint* base, b2SolverData* data)
{
	assert(base->type == b2_revoluteJoint);

	b2Body* bodyA = world->bodies + base->edgeA.bodyIndex;
	b2Body* bodyB = world->bodies + base->edgeB.bodyIndex;

	b2RevoluteJoint* joint = &base->revoluteJoint;

	joint->indexA = bodyA->islandIndex;
	joint->localCenterA = bodyA->localCenter;
	joint->invMassA = bodyA->invMass;
	joint->invIA = bodyA->invI;

	joint->indexB = bodyB->islandIndex;
	joint->localCenterB = bodyB->localCenter;
	joint->invMassB = bodyB->invMass;
	joint->invIB = bodyB->invI;

	float aA = data->positions[joint->indexA].a;
	b2Vec2 vA = data->velocities[joint->indexA].v;
	float wA = data->velocities[joint->indexA].w;

	float aB = data->positions[joint->indexB].a;
	b2Vec2 vB = data->velocities[joint->indexB].v;
	float wB = data->velocities[joint->indexB].w;

	b2Rot qA = b2MakeRot(aA);
	b2Rot qB = b2MakeRot(aB);

	joint->rA = b2RotateVector(qA, b2Sub(base->localAnchorA, joint->localCenterA));
	joint->rB = b2RotateVector(qB, b2Sub(base->localAnchorB, joint->localCenterB));

	// J = [-I -r1_skew I r2_skew]
	// r_skew = [-ry; rx]

	// Matlab
	// K = [ mA+r1y^2*iA+mB+r2y^2*iB,  -r1y*iA*r1x-r2y*iB*r2x]
	//     [  -r1y*iA*r1x-r2y*iB*r2x, mA+r1x^2*iA+mB+r2x^2*iB]

	float mA = joint->invMassA, mB = joint->invMassB;
	float iA = joint->invIA, iB = joint->invIB;

	joint->K.cx.x = mA + mB + joint->rA.y * joint->rA.y * iA + joint->rB.y * joint->rB.y * iB;
	joint->K.cy.x = -joint->rA.y * joint->rA.x * iA - joint->rB.y * joint->rB.x * iB;
	joint->K.cx.y = joint->K.cy.x;
	joint->K.cy.y = mA + mB + joint->rA.x * joint->rA.x * iA + joint->rB.x * joint->rB.x * iB;

	joint->axialMass = iA + iB;
	bool fixedRotation;
	if (joint->axialMass > 0.0f)
	{
		joint->axialMass = 1.0f / joint->axialMass;
		fixedRotation = false;
	}
	else
	{
		fixedRotation = true;
	}

	joint->angle = aB - aA - joint->referenceAngle;
	if (joint->enableLimit == false || fixedRotation)
	{
		joint->lowerImpulse = 0.0f;
		joint->upperImpulse = 0.0f;
	}

	if (joint->enableMotor == false || fixedRotation)
	{
		joint->motorImpulse = 0.0f;
	}

	if (data->step.warmStarting)
	{
		// Scale impulses to support a variable time step.
		joint->impulse = b2MulSV(data->step.dtRatio, joint->impulse);
		joint->motorImpulse *= data->step.dtRatio;
		joint->lowerImpulse *= data->step.dtRatio;
		joint->upperImpulse *= data->step.dtRatio;

		float axialImpulse = joint->motorImpulse + joint->lowerImpulse - joint->upperImpulse;
		b2Vec2 P = {joint->impulse.x, joint->impulse.y};

		vA = b2MulSub(vA, mA, P);
		wA -= iA * (b2Cross(joint->rA, P) + axialImpulse);

		vB = b2MulAdd(vB, mB, P);
		wB += iB * (b2Cross(joint->rB, P) + axialImpulse);
	}
	else
	{
		joint->impulse = b2Vec2_zero;
		joint->motorImpulse = 0.0f;
		joint->lowerImpulse = 0.0f;
		joint->upperImpulse = 0.0f;
	}

	data->velocities[joint->indexA].v = vA;
	data->velocities[joint->indexA].w = wA;
	data->velocities[joint->indexB].v = vB;
	data->velocities[joint->indexB].w = wB;
}

void b2SolveRevoluteVelocity(b2Joint* base, b2SolverData* data)
{
	assert(base->type == b2_revoluteJoint);

	b2RevoluteJoint* joint = &base->revoluteJoint;

	b2Vec2 vA = data->velocities[joint->indexA].v;
	float wA = data->velocities[joint->indexA].w;
	b2Vec2 vB = data->velocities[joint->indexB].v;
	float wB = data->velocities[joint->indexB].w;

	float mA = joint->invMassA, mB = joint->invMassB;
	float iA = joint->invIA, iB = joint->invIB;

	bool fixedRotation = (iA + iB == 0.0f);

	// Solve motor constraint.
	if (joint->enableMotor && fixedRotation == false)
	{
		float Cdot = wB - wA - joint->motorSpeed;
		float impulse = -joint->axialMass * Cdot;
		float oldImpulse = joint->motorImpulse;
		float maxImpulse = data->step.dt * joint->maxMotorTorque;
		joint->motorImpulse = B2_CLAMP(joint->motorImpulse + impulse, -maxImpulse, maxImpulse);
		impulse = joint->motorImpulse - oldImpulse;

		wA -= iA * impulse;
		wB += iB * impulse;
	}

	if (joint->enableLimit && fixedRotation == false)
	{
		// Lower limit
		{
			float C = joint->angle - joint->lowerAngle;
			float Cdot = wB - wA;
			float impulse = -joint->axialMass * (Cdot + B2_MAX(C, 0.0f) * data->step.inv_dt);
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
			float C = joint->upperAngle - joint->angle;
			float Cdot = wA - wB;
			float impulse = -joint->axialMass * (Cdot + B2_MAX(C, 0.0f) * data->step.inv_dt);
			float oldImpulse = joint->upperImpulse;
			joint->upperImpulse = B2_MAX(joint->upperImpulse + impulse, 0.0f);
			impulse = joint->upperImpulse - oldImpulse;

			wA += iA * impulse;
			wB -= iB * impulse;
		}
	}

	// Solve point-to-point constraint
	{
		b2Vec2 Cdot = b2Sub(b2Add(vB, b2CrossSV(wB, joint->rB)), b2Add(vA, b2CrossSV(wA, joint->rA)));
		b2Vec2 impulse = b2Solve22(joint->K, b2Neg(Cdot));

		joint->impulse.x += impulse.x;
		joint->impulse.y += impulse.y;

		vA = b2MulSub(vA, mA, impulse);
		wA -= iA * b2Cross(joint->rA, impulse);

		vB = b2MulAdd(vB, mB, impulse);
		wB += iB * b2Cross(joint->rB, impulse);
	}

	data->velocities[joint->indexA].v = vA;
	data->velocities[joint->indexA].w = wA;
	data->velocities[joint->indexB].v = vB;
	data->velocities[joint->indexB].w = wB;
}

bool b2SolveRevolutePosition(b2Joint* base, b2SolverData* data)
{
	assert(base->type == b2_revoluteJoint);

	b2RevoluteJoint* joint = &base->revoluteJoint;

	b2Vec2 cA = data->positions[joint->indexA].c;
	float aA = data->positions[joint->indexA].a;
	b2Vec2 cB = data->positions[joint->indexB].c;
	float aB = data->positions[joint->indexB].a;

	b2Rot qA = b2MakeRot(aA), qB = b2MakeRot(aB);

	float angularError = 0.0f;
	float positionError = 0.0f;

	bool fixedRotation = (joint->invIA + joint->invIB == 0.0f);

	// Solve angular limit constraint
	if (joint->enableLimit && fixedRotation == false)
	{
		float angle = aB - aA - joint->referenceAngle;
		float C = 0.0f;

		if (B2_ABS(joint->upperAngle - joint->lowerAngle) < 2.0f * b2_angularSlop)
		{
			// Prevent large angular corrections
			C = B2_CLAMP(angle - joint->lowerAngle, -b2_maxAngularCorrection, b2_maxAngularCorrection);
		}
		else if (angle <= joint->lowerAngle)
		{
			// Prevent large angular corrections and allow some slop.
			C = B2_CLAMP(angle - joint->lowerAngle + b2_angularSlop, -b2_maxAngularCorrection, 0.0f);
		}
		else if (angle >= joint->upperAngle)
		{
			// Prevent large angular corrections and allow some slop.
			C = B2_CLAMP(angle - joint->upperAngle - b2_angularSlop, 0.0f, b2_maxAngularCorrection);
		}

		float limitImpulse = -joint->axialMass * C;
		aA -= joint->invIA * limitImpulse;
		aB += joint->invIB * limitImpulse;
		angularError = B2_ABS(C);
	}

	// Solve point-to-point constraint.
	{
		qA = b2MakeRot(aA);
		qB = b2MakeRot(aB);
		b2Vec2 rA = b2RotateVector(qA, b2Sub(base->localAnchorA, joint->localCenterA));
		b2Vec2 rB = b2RotateVector(qB, b2Sub(base->localAnchorB, joint->localCenterB));

		b2Vec2 C = b2Sub(b2Add(cB, rB), b2Add(cA, rA));
		positionError = b2Length(C);

		float mA = joint->invMassA, mB = joint->invMassB;
		float iA = joint->invIA, iB = joint->invIB;

		b2Mat22 K;
		K.cx.x = mA + mB + iA * rA.y * rA.y + iB * rB.y * rB.y;
		K.cx.y = -iA * rA.x * rA.y - iB * rB.x * rB.y;
		K.cy.x = K.cx.y;
		K.cy.y = mA + mB + iA * rA.x * rA.x + iB * rB.x * rB.x;

		b2Vec2 impulse = b2Solve22(K, b2Neg(C));

		cA = b2MulSub(cA, mA, impulse);
		aA -= iA * b2Cross(rA, impulse);

		cB = b2MulAdd(cB, mB, impulse);
		aB += iB * b2Cross(rB, impulse);
	}

	data->positions[joint->indexA].c = cA;
	data->positions[joint->indexA].a = aA;
	data->positions[joint->indexB].c = cB;
	data->positions[joint->indexB].a = aB;

	return positionError <= b2_linearSlop && angularError <= b2_angularSlop;
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
	assert(base->type == b2_revoluteJoint);

	b2RevoluteJoint* joint = &base->revoluteJoint;

	b2Transform xfA = bodyA->transform;
	b2Transform xfB = bodyB->transform;
	b2Vec2 pA = b2TransformPoint(xfA, base->localAnchorA);
	b2Vec2 pB = b2TransformPoint(xfB, base->localAnchorB);

	b2Color c1 = {0.7f, 0.7f, 0.7f, 1.0f};
	b2Color c2 = {0.3f, 0.9f, 0.3f, 1.0f};
	b2Color c3 = {0.9f, 0.3f, 0.3f, 1.0f};
	b2Color c4 = {0.3f, 0.3f, 0.9f, 1.0f};
	b2Color c5 = {0.4f, 0.4f, 0.4f, 1.0f};

	draw->DrawPoint(pA, 5.0f, c4, draw->context);
	draw->DrawPoint(pB, 5.0f, c5, draw->context);

	float aA = bodyA->angle;
	float aB = bodyB->angle;
	float angle = aB - aA - joint->referenceAngle;

	const float L = 0.5f;

	b2Vec2 r = { L * cosf(angle), L * sinf(angle)};
	draw->DrawSegment(pB, b2Add(pB, r), c1, draw->context);
	draw->DrawCircle(pB, L, c1, draw->context);

	if (joint->enableLimit)
	{
		b2Vec2 rlo = { L * cosf(joint->lowerAngle), L * sinf(joint->lowerAngle)};
		b2Vec2 rhi = { L * cosf(joint->upperAngle), L * sinf(joint->upperAngle)};

		draw->DrawSegment(pB, b2Add(pB, rlo), c2, draw->context);
		draw->DrawSegment(pB, b2Add(pB, rhi), c3, draw->context);
	}

	b2Color color = {0.5f, 0.8f, 0.8f, 1.0f};
	draw->DrawSegment(xfA.p, pA, color, draw->context);
	draw->DrawSegment(pA, pB, color, draw->context);
	draw->DrawSegment(xfB.p, pB, color, draw->context);
}
