// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "body.h"
#include "joint.h"
#include "solver_data.h"
#include "world.h"

// p = attached point, m = mouse point
// C = p - m
// Cdot = v
//      = v + cross(w, r)
// J = [I r_skew]
// Identity used:
// w k % (rx i + ry mouse) = w * (-ry i + rx mouse)

void b2MouseJoint_SetTarget(b2JointId jointId, b2Vec2 target)
{
	b2World* world = b2GetWorldFromIndex(jointId.world);
	assert(world->locked == false);
	if (world->locked)
	{
		return;
	}

	assert(0 <= jointId.index && jointId.index < world->jointPool.capacity);

	b2Joint* joint = world->joints + jointId.index;
	assert(joint->object.index == joint->object.next);
	assert(joint->object.revision == jointId.revision);
	assert(joint->type == b2_mouseJoint);

	b2Body* bodyA = world->bodies + joint->edgeA.bodyIndex;

	joint->localAnchorA = b2InvTransformPoint(bodyA->transform, target);
}

void b2InitializeMouse(b2World* world, b2Joint* base, b2SolverData* data)
{
	assert(base->type == b2_mouseJoint);

	b2Body* bodyA = world->bodies + base->edgeA.bodyIndex;
	b2Body* bodyB = world->bodies + base->edgeB.bodyIndex;

	b2MouseJoint* joint = &base->mouseJoint;

	joint->indexB = bodyB->islandIndex;
	joint->localCenterB = bodyB->localCenter;
	joint->invMassB = bodyB->invMass;
	joint->invIB = bodyB->invI;

	b2Vec2 cB = data->positions[joint->indexB].c;
	float aB = data->positions[joint->indexB].a;
	b2Vec2 vB = data->velocities[joint->indexB].v;
	float wB = data->velocities[joint->indexB].w;

	b2Rot qB = b2MakeRot(aB);

	float d = joint->damping;
	float k = joint->stiffness;

	// magic formulas
	// gamma has units of inverse mass.
	// beta has units of inverse time.
	float h = data->step.dt;
	joint->gamma = h * (d + h * k);
	if (joint->gamma != 0.0f)
	{
		joint->gamma = 1.0f / joint->gamma;
	}
	joint->beta = h * k * joint->gamma;

	// Compute the effective mass matrix.
	joint->rB = b2RotateVector(qB, b2Sub(base->localAnchorB, joint->localCenterB));

	// K    = [(1/m1 + 1/m2) * eye(2) - skew(r1) * invI1 * skew(r1) - skew(r2) * invI2 * skew(r2)]
	//      = [1/m1+1/m2     0    ] + invI1 * [r1.y*r1.y -r1.x*r1.y] + invI2 * [r1.y*r1.y -r1.x*r1.y]
	//        [    0     1/m1+1/m2]           [-r1.x*r1.y r1.x*r1.x]           [-r1.x*r1.y r1.x*r1.x]
	b2Mat22 K;
	K.cx.x = joint->invMassB + joint->invIB * joint->rB.y * joint->rB.y + joint->gamma;
	K.cx.y = -joint->invIB * joint->rB.x * joint->rB.y;
	K.cy.x = K.cx.y;
	K.cy.y = joint->invMassB + joint->invIB * joint->rB.x * joint->rB.x + joint->gamma;

	joint->mass = b2GetInverse22(K);

	b2Vec2 targetA = b2TransformPoint(bodyA->transform, base->localAnchorA);
	joint->C = b2Add(cB, b2Sub(joint->rB, targetA));
	joint->C = b2MulSV(joint->beta, joint->C);

	// Cheat with some damping
	wB *= B2_MAX(0.0f, 1.0f - 0.02f * (60.0f * data->step.dt));

	if (data->step.warmStarting)
	{
		joint->impulse = b2MulSV(data->step.dtRatio, joint->impulse);
		vB = b2MulAdd(vB, joint->invMassB, joint->impulse);
		wB += joint->invIB * b2Cross(joint->rB, joint->impulse);
	}
	else
	{
		joint->impulse = b2Vec2_zero;
	}

	data->velocities[joint->indexB].v = vB;
	data->velocities[joint->indexB].w = wB;
}

void b2SolveMouseVelocity(b2Joint* base, b2SolverData* data)
{
	b2MouseJoint* joint = &base->mouseJoint;

	b2Vec2 vB = data->velocities[joint->indexB].v;
	float wB = data->velocities[joint->indexB].w;

	// Cdot = v + cross(w, r)
	b2Vec2 Cdot = b2Add(vB, b2CrossSV(wB, joint->rB));
	b2Vec2 SoftCdot = b2Add(Cdot, b2MulAdd(joint->C, joint->gamma, joint->impulse));
	b2Vec2 impulse = b2Neg(b2MulMV(joint->mass, SoftCdot));

	b2Vec2 oldImpulse = joint->impulse;
	joint->impulse = b2Add(joint->impulse, impulse);
	float maxImpulse = data->step.dt * joint->maxForce;
	if (b2LengthSquared(joint->impulse) > maxImpulse * maxImpulse)
	{
		joint->impulse = b2MulSV(maxImpulse / b2Length(joint->impulse), joint->impulse);
	}
	impulse = b2Sub(joint->impulse, oldImpulse);

	vB = b2MulAdd(vB, joint->invMassB, impulse);
	wB += joint->invIB * b2Cross(joint->rB, impulse);

	data->velocities[joint->indexB].v = vB;
	data->velocities[joint->indexB].w = wB;
}
