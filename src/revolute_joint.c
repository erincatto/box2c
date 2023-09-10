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

// Motor constraint
// Cdot = w2 - w1
// J = [0 0 -1 0 0 1]
// K = invI1 + invI2

void b2PrepareRevolute(b2Joint* base, b2StepContext* context)
{
	B2_ASSERT(base->type == b2_revoluteJoint);

	int32_t indexA = base->edges[0].bodyIndex;
	int32_t indexB = base->edges[1].bodyIndex;
	B2_ASSERT(0 <= indexA && indexA < context->bodyCapacity);
	B2_ASSERT(0 <= indexB && indexB < context->bodyCapacity);

	b2Body* bodyA = context->bodies + indexA;
	b2Body* bodyB = context->bodies + indexB;
	B2_ASSERT(bodyA->object.index == bodyA->object.next);
	B2_ASSERT(bodyB->object.index == bodyB->object.next);

	b2RevoluteJoint* joint = &base->revoluteJoint;
	joint->localCenterA = bodyA->localCenter;
	joint->invMassA = bodyA->invMass;
	joint->invIA = bodyA->invI;

	joint->localCenterB = bodyB->localCenter;
	joint->invMassB = bodyB->invMass;
	joint->invIB = bodyB->invI;

	float aA = bodyA->angle;
	b2Vec2 vA = bodyA->linearVelocity;
	float wA = bodyA->angularVelocity;

	float aB = bodyB->angle;
	b2Vec2 vB = bodyB->linearVelocity;
	float wB = bodyB->angularVelocity;

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

	// hertz = 1/4 * substep Hz
	const float hertz = (1.0f / 4.0f) * context->velocityIterations * context->inv_dt;
	const float zeta = 1.0f;
	float omega = 2.0f * b2_pi * hertz;
	float h = context->dt;

	joint->separation = b2Add(b2Sub(joint->rB, joint->rA), b2Sub(bodyB->position, bodyA->position));
	joint->biasCoefficient = omega / (2.0f * zeta + h * omega);
	float c = h * omega * (2.0f * zeta + h * omega);
	joint->impulseCoefficient = 1.0f / (1.0f + c);
	joint->massCoefficient = c * joint->impulseCoefficient;

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

	if (context->enableWarmStarting)
	{
		float dtRatio = context->dtRatio;

		// Soft step works best when bilateral constraints have no warm starting.
		joint->impulse = b2Vec2_zero;
		//joint->impulse.x = 0.0f;
		joint->motorImpulse *= dtRatio;
		joint->lowerImpulse *= dtRatio;
		joint->upperImpulse *= dtRatio;

		float axialImpulse = joint->motorImpulse + joint->lowerImpulse - joint->upperImpulse;
		b2Vec2 P = {joint->impulse.x, joint->impulse.y};

		vA = b2MulSub(vA, mA, P);
		wA -= iA * (b2Cross(joint->rA, P) + axialImpulse);

		vB = b2MulAdd(vB, mB, P);
		wB += iB * (b2Cross(joint->rB, P) + axialImpulse);

		//vA.x = 0.0f;
		//wA = 0.0f;
		//vB.x = 0.0f;
		//wB = 0.0f;
	}
	else
	{
		joint->impulse = b2Vec2_zero;
		joint->motorImpulse = 0.0f;
		joint->lowerImpulse = 0.0f;
		joint->upperImpulse = 0.0f;
	}

	bodyA->linearVelocity = vA;
	bodyA->angularVelocity = wA;
	bodyB->linearVelocity = vB;
	bodyB->angularVelocity = wB;
}

void b2SolveRevoluteVelocity(b2Joint* base, const b2StepContext* context)
{
	B2_ASSERT(base->type == b2_revoluteJoint);

	b2RevoluteJoint* joint = &base->revoluteJoint;

	b2Body* bodyA = context->bodies + base->edges[0].bodyIndex;
	b2Body* bodyB = context->bodies + base->edges[1].bodyIndex;

	b2Vec2 vA = bodyA->linearVelocity;
	float wA = bodyA->angularVelocity;
	b2Vec2 vB = bodyB->linearVelocity;
	float wB = bodyB->angularVelocity;

	float mA = joint->invMassA, mB = joint->invMassB;
	float iA = joint->invIA, iB = joint->invIB;

	bool fixedRotation = (iA + iB == 0.0f);

	// Solve motor constraint.
	if (joint->enableMotor && fixedRotation == false)
	{
		float Cdot = wB - wA - joint->motorSpeed;
		float impulse = -joint->axialMass * Cdot;
		float oldImpulse = joint->motorImpulse;
		float maxImpulse = context->dt * joint->maxMotorTorque;
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
			float impulse = -joint->axialMass * (Cdot + B2_MAX(C, 0.0f) * context->inv_dt);
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
			float impulse = -joint->axialMass * (Cdot + B2_MAX(C, 0.0f) * context->inv_dt);
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

	bodyA->linearVelocity = vA;
	bodyA->angularVelocity = wA;
	bodyB->linearVelocity = vB;
	bodyB->angularVelocity = wB;
}

void b2SolveRevoluteVelocitySoft(b2Joint* base, const b2StepContext* context, bool removeOverlap)
{
	B2_ASSERT(base->type == b2_revoluteJoint);

	b2RevoluteJoint* joint = &base->revoluteJoint;

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

	bool fixedRotation = (iA + iB == 0.0f);

	// Solve motor constraint.
	if (joint->enableMotor && fixedRotation == false)
	{
		float Cdot = wB - wA - joint->motorSpeed;
		float impulse = -joint->axialMass * Cdot;
		float oldImpulse = joint->motorImpulse;
		float maxImpulse = context->dt * joint->maxMotorTorque;
		joint->motorImpulse = B2_CLAMP(joint->motorImpulse + impulse, -maxImpulse, maxImpulse);
		impulse = joint->motorImpulse - oldImpulse;

		wA -= iA * impulse;
		wB += iB * impulse;
	}

	if (joint->enableLimit && fixedRotation == false)
	{
		float jointAngle = aB - aA - joint->referenceAngle;

		// Lower limit
		{
			float C = jointAngle - joint->lowerAngle;
			float bias = 0.0f;
			float massScale = 1.0f;
			float impulseScale = 0.0f;
			if (C > 0.0f)
			{
				bias = C * context->inv_dt;
			}
			else if (removeOverlap)
			{
				bias = joint->biasCoefficient * C;
				massScale = joint->massCoefficient;
				impulseScale = joint->impulseCoefficient;
			}

			float Cdot = wB - wA;
			float impulse = -joint->axialMass * massScale *  (Cdot + bias) - impulseScale * joint->lowerImpulse;
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
				bias = C * context->inv_dt;
			}
			else if (removeOverlap)
			{
				bias = joint->biasCoefficient * C;
				massScale = joint->massCoefficient;
				impulseScale = joint->impulseCoefficient;
			}

			float Cdot = wA - wB;
			float impulse = -joint->axialMass * massScale * (Cdot + bias) - impulseScale * joint->lowerImpulse;
			float oldImpulse = joint->upperImpulse;
			joint->upperImpulse = B2_MAX(joint->upperImpulse + impulse, 0.0f);
			impulse = joint->upperImpulse - oldImpulse;

			wA += iA * impulse;
			wB -= iB * impulse;
		}
	}

	// Solve point-to-point constraint
	{
		b2Rot qA = b2MakeRot(aA);
		b2Rot qB = b2MakeRot(aB);

		b2Vec2 rA = b2RotateVector(qA, b2Sub(base->localAnchorA, joint->localCenterA));
		b2Vec2 rB = b2RotateVector(qB, b2Sub(base->localAnchorB, joint->localCenterB));

		b2Mat22 K;
		K.cx.x = mA + mB + rA.y * rA.y * iA + rB.y * rB.y * iB;
		K.cy.x = -rA.y * rA.x * iA - rB.y * rB.x * iB;
		K.cx.y = K.cy.x;
		K.cy.y = mA + mB + rA.x * rA.x * iA + rB.x * rB.x * iB;

		b2Vec2 separation = b2Add(b2Sub(rB, rA), b2Sub(cB, cA));

		b2Vec2 Cdot = b2Sub(b2Add(vB, b2CrossSV(wB, rB)), b2Add(vA, b2CrossSV(wA, rA)));

		float biasScale = 0.0f;
		float massScale = 1.0f;
		float impulseScale = 0.0f;
		if (removeOverlap)
		{
			biasScale = joint->biasCoefficient;
			massScale = joint->massCoefficient;
			impulseScale = joint->impulseCoefficient;
		}

		b2Vec2 b = b2Solve22(K, b2MulAdd(Cdot, biasScale, separation));
		b2Vec2 impulse;
		impulse.x = -massScale * b.x - impulseScale * joint->impulse.x;
		impulse.y = -massScale * b.y - impulseScale * joint->impulse.y;

		joint->impulse.x += impulse.x;
		joint->impulse.y += impulse.y;

		vA = b2MulSub(vA, mA, impulse);
		wA -= iA * b2Cross(rA, impulse);

		vB = b2MulAdd(vB, mB, impulse);
		wB += iB * b2Cross(rB, impulse);
	}
		
	//vA.x = 0.0f;
	//wA = 0.0f;
	//vB.x = 0.0f;
	//wB = 0.0f;

	bodyA->linearVelocity = vA;
	bodyA->angularVelocity = wA;
	bodyB->linearVelocity = vB;
	bodyB->angularVelocity = wB;
}

bool b2SolveRevolutePosition(b2Joint* base, b2StepContext* context)
{
	B2_ASSERT(base->type == b2_revoluteJoint);

	b2RevoluteJoint* joint = &base->revoluteJoint;

	b2Body* bodyA = context->bodies + base->edges[0].bodyIndex;
	b2Body* bodyB = context->bodies + base->edges[1].bodyIndex;

	b2Vec2 cA = bodyA->position;
	float aA = bodyA->angle;
	b2Vec2 cB = bodyB->position;
	float aB = bodyB->angle;

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

	bodyA->position = cA;
	bodyA->angle = aA;
	bodyB->position = cB;
	bodyB->angle = aB;

	return positionError <= b2_linearSlop && angularError <= b2_angularSlop;
}

void b2RevoluteJoint_EnableLimit(b2JointId jointId, bool enableLimit)
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
	B2_ASSERT(joint->type == b2_revoluteJoint);
	joint->revoluteJoint.enableLimit = enableLimit;
}

void b2RevoluteJoint_EnableMotor(b2JointId jointId, bool enableMotor)
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
	B2_ASSERT(joint->type == b2_revoluteJoint);
	joint->revoluteJoint.enableMotor = enableMotor;
}

void b2RevoluteJoint_SetMotorSpeed(b2JointId jointId, float motorSpeed)
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
	B2_ASSERT(joint->type == b2_revoluteJoint);
	joint->revoluteJoint.motorSpeed = motorSpeed;
}

float b2RevoluteJoint_GetMotorTorque(b2JointId jointId, float inverseTimeStep)
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
	B2_ASSERT(joint->type == b2_revoluteJoint);
	return inverseTimeStep * joint->revoluteJoint.motorImpulse;
}

void b2RevoluteJoint_SetMaxMotorTorque(b2JointId jointId, float torque)
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
	B2_ASSERT(joint->type == b2_revoluteJoint);
	joint->revoluteJoint.maxMotorTorque = torque;
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
}
