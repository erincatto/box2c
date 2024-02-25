// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "body.h"
#include "core.h"
#include "joint.h"
#include "solver.h"
#include "world.h"

// needed for dll export
#include "box2d/box2d.h"

void b2MouseJoint_SetTarget(b2JointId jointId, b2Vec2 target)
{
	b2Joint* base = b2GetJointCheckType(jointId, b2_mouseJoint);
	base->mouseJoint.targetA = target;
}

b2Vec2 b2MouseJoint_GetTarget(b2JointId jointId)
{
	b2Joint* base = b2GetJointCheckType(jointId, b2_mouseJoint);
	return base->mouseJoint.targetA;
}

void b2MouseJoint_SetTuning(b2JointId jointId, float hertz, float dampingRatio)
{
	b2Joint* base = b2GetJointCheckType(jointId, b2_mouseJoint);
	base->mouseJoint.hertz = hertz;
	base->mouseJoint.dampingRatio = dampingRatio;
}

float b2MouseJoint_GetHertz(b2JointId jointId)
{
	b2Joint* base = b2GetJointCheckType(jointId, b2_mouseJoint);
	return base->mouseJoint.hertz;
}

float b2MouseJoint_GetDampingRatio(b2JointId jointId)
{
	b2Joint* base = b2GetJointCheckType(jointId, b2_mouseJoint);
	return base->mouseJoint.dampingRatio;
}

void b2PrepareMouseJoint(b2Joint* base, b2StepContext* context)
{
	B2_ASSERT(base->type == b2_mouseJoint);

	int32_t indexB = base->edges[1].bodyIndex;
	B2_ASSERT(0 <= indexB && indexB < context->bodyCapacity);

	b2Body* bodyB = context->bodies + indexB;
	B2_ASSERT(bodyB->object.index == bodyB->object.next);

	base->invMassB = bodyB->invMass;
	base->invIB = bodyB->invI;

	b2MouseJoint* joint = &base->mouseJoint;
	joint->indexB = context->bodyToSolverMap[indexB];
	joint->anchorB = b2RotateVector(bodyB->rotation, b2Sub(base->localOriginAnchorB, bodyB->localCenter));

	joint->linearSoftness = b2MakeSoft(joint->hertz, joint->dampingRatio, context->h);

	float angularHertz = 0.5f;
	float angularDampingRatio = 0.1f;
	joint->angularSoftness = b2MakeSoft(angularHertz, angularDampingRatio, context->h);

	b2Rot qB = bodyB->rotation;
	b2Vec2 rB = joint->anchorB;
	float mB = bodyB->invMass;
	float iB = bodyB->invI;

	// K = [(1/m1 + 1/m2) * eye(2) - skew(r1) * invI1 * skew(r1) - skew(r2) * invI2 * skew(r2)]
	//   = [1/m1+1/m2     0    ] + invI1 * [r1.y*r1.y -r1.x*r1.y] + invI2 * [r1.y*r1.y -r1.x*r1.y]
	//     [    0     1/m1+1/m2]           [-r1.x*r1.y r1.x*r1.x]           [-r1.x*r1.y r1.x*r1.x]
	b2Mat22 K;
	K.cx.x = mB + iB * rB.y * rB.y;
	K.cx.y = -iB * rB.x * rB.y;
	K.cy.x = K.cx.y;
	K.cy.y = mB + iB * rB.x * rB.x;

	joint->linearMass = b2GetInverse22(K);
	joint->deltaCenter = b2Sub(bodyB->position, joint->targetA);

	if (context->enableWarmStarting == false)
	{
		joint->linearImpulse = b2Vec2_zero;
		joint->angularImpulse = 0.0f;
	}
}

void b2WarmStartMouseJoint(b2Joint* base, b2StepContext* context)
{
	B2_ASSERT(base->type == b2_mouseJoint);

	float mB = base->invMassB;
	float iB = base->invIB;

	b2MouseJoint* joint = &base->mouseJoint;

	b2BodyState* stateB = context->bodyStates + joint->indexB;
	b2Vec2 vB = stateB->linearVelocity;
	float wB = stateB->angularVelocity;

	b2Rot dqB = stateB->deltaRotation;
	b2Vec2 rB = b2RotateVector(dqB, joint->anchorB);

	vB = b2MulAdd(vB, mB, joint->linearImpulse);
	wB += iB * (b2Cross(rB, joint->linearImpulse) + joint->angularImpulse);

	stateB->linearVelocity = vB;
	stateB->angularVelocity = wB;
}

void b2SolveMouseJoint(b2Joint* base, b2StepContext* context)
{
	float mB = base->invMassB;
	float iB = base->invIB;

	b2MouseJoint* joint = &base->mouseJoint;
	b2BodyState* stateB = context->bodyStates + joint->indexB;

	b2Vec2 vB = stateB->linearVelocity;
	float wB = stateB->angularVelocity;

	// Softness with no bias to reduce rotation speed
	{
		float massScale = joint->angularSoftness.massScale;
		float impulseScale = joint->angularSoftness.impulseScale;

		float impulse = iB > 0.0f ? -wB / iB : 0.0f;
		impulse = massScale * impulse - impulseScale * joint->angularImpulse;
		joint->angularImpulse += impulse;

		wB += iB * impulse;
	}

	{
		b2Rot dqB = stateB->deltaRotation;
		b2Vec2 rB = b2RotateVector(dqB, joint->anchorB);
		b2Vec2 Cdot = b2Add(vB, b2CrossSV(wB, rB));

		b2Vec2 separation = b2Add(b2Add(stateB->deltaPosition, rB), joint->deltaCenter);
		b2Vec2 bias = b2MulSV(joint->linearSoftness.biasRate, separation);

		float massScale = joint->linearSoftness.massScale;
		float impulseScale = joint->linearSoftness.impulseScale;

		b2Vec2 b = b2MulMV(joint->linearMass, b2Add(Cdot, bias));

		b2Vec2 impulse;
		impulse.x = -massScale * b.x - impulseScale * joint->linearImpulse.x;
		impulse.y = -massScale * b.y - impulseScale * joint->linearImpulse.y;
		joint->linearImpulse.x += impulse.x;
		joint->linearImpulse.y += impulse.y;

		vB = b2MulAdd(vB, mB, impulse);
		wB += iB * b2Cross(rB, impulse);
	}

	stateB->linearVelocity = vB;
	stateB->angularVelocity = wB;
}
