// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "box2d/debug_draw.h"
#include "box2d/joint_types.h"

#include "body.h"
#include "contact.h"
#include "joint.h"
#include "shape.h"
#include "world.h"

void b2LinearStiffness(float* stiffness, float* damping, float frequencyHertz, float dampingRatio, b2BodyId bodyIdA,
					   b2BodyId bodyIdB)
{
	assert(bodyIdA.world == bodyIdB.world);

	b2World* world = b2GetWorldFromIndex(bodyIdA.world);
	assert(0 <= bodyIdA.index && bodyIdA.index < world->bodyPool.capacity);
	assert(0 <= bodyIdB.index && bodyIdB.index < world->bodyPool.capacity);

	b2Body* bodyA = world->bodies + bodyIdA.index;
	b2Body* bodyB = world->bodies + bodyIdB.index;

	float massA = bodyA->mass;
	float massB = bodyB->mass;
	float mass;
	if (massA > 0.0f && massB > 0.0f)
	{
		mass = massA * massB / (massA + massB);
	}
	else if (massA > 0.0f)
	{
		mass = massA;
	}
	else
	{
		mass = massB;
	}

	float omega = 2.0f * b2_pi * frequencyHertz;
	*stiffness = mass * omega * omega;
	*damping = 2.0f * mass * dampingRatio * omega;
}

void b2AngularStiffness(float* stiffness, float* damping, float frequencyHertz, float dampingRatio, b2BodyId bodyIdA,
						b2BodyId bodyIdB)
{
	assert(bodyIdA.world == bodyIdB.world);

	b2World* world = b2GetWorldFromIndex(bodyIdA.world);
	assert(0 <= bodyIdA.index && bodyIdA.index < world->bodyPool.capacity);
	assert(0 <= bodyIdB.index && bodyIdB.index < world->bodyPool.capacity);

	b2Body* bodyA = world->bodies + bodyIdA.index;
	b2Body* bodyB = world->bodies + bodyIdB.index;

	float IA = bodyA->I;
	float IB = bodyB->I;
	float I;
	if (IA > 0.0f && IB > 0.0f)
	{
		I = IA * IB / (IA + IB);
	}
	else if (IA > 0.0f)
	{
		I = IA;
	}
	else
	{
		I = IB;
	}

	float omega = 2.0f * b2_pi * frequencyHertz;
	*stiffness = I * omega * omega;
	*damping = 2.0f * I * dampingRatio * omega;
}

b2JointId b2World_CreateMouseJoint(b2WorldId worldId, const b2MouseJointDef* def)
{
	b2World* world = b2GetWorldFromId(worldId);

	assert(world->locked == false);

	if (world->locked)
	{
		return b2_nullJointId;
	}

	assert(b2IsBodyIdValid(world, def->bodyId));

	b2Joint* joint = (b2Joint*)b2AllocObject(&world->jointPool);
	world->joints = (b2Joint*)world->jointPool.memory;

	joint->type = b2_mouseJoint;
	joint->edgeA.bodyIndex = world->groundBodyIndex;
	joint->edgeB.bodyIndex = def->bodyId.index;
	joint->collideConnected = false;
	joint->islandId = 0;

	b2Body* bodyA = world->bodies + joint->edgeA.bodyIndex;
	b2Body* bodyB = world->bodies + joint->edgeB.bodyIndex;

	joint->edgeA.nextJointIndex = bodyA->jointIndex;
	bodyA->jointIndex = joint->object.index;

	joint->edgeB.nextJointIndex = bodyB->jointIndex;
	bodyB->jointIndex = joint->object.index;

	joint->localAnchorA = b2InvTransformPoint(bodyA->transform, def->target);
	joint->localAnchorB = b2InvTransformPoint(bodyB->transform, def->target);

	b2MouseJoint empty = {0};
	joint->mouseJoint = empty;

	joint->mouseJoint.maxForce = def->maxForce;
	joint->mouseJoint.stiffness = def->stiffness;
	joint->mouseJoint.damping = def->damping;

	b2JointId jointId = {joint->object.index, world->index, joint->object.revision};

	return jointId;
}

b2JointId b2World_CreateRevoluteJoint(b2WorldId worldId, const b2RevoluteJointDef* def)
{
	b2World* world = b2GetWorldFromId(worldId);

	assert(world->locked == false);

	if (world->locked)
	{
		return b2_nullJointId;
	}

	assert(b2IsBodyIdValid(world, def->bodyIdA));
	assert(b2IsBodyIdValid(world, def->bodyIdB));

	b2Joint* joint = (b2Joint*)b2AllocObject(&world->jointPool);
	world->joints = (b2Joint*)world->jointPool.memory;

	joint->type = b2_revoluteJoint;
	joint->edgeA.bodyIndex = def->bodyIdA.index;
	joint->edgeB.bodyIndex = def->bodyIdB.index;
	joint->collideConnected = def->collideConnected;
	joint->islandId = 0;

	b2Body* bodyA = world->bodies + joint->edgeA.bodyIndex;
	b2Body* bodyB = world->bodies + joint->edgeB.bodyIndex;

	joint->edgeA.nextJointIndex = bodyA->jointIndex;
	bodyA->jointIndex = joint->object.index;

	joint->edgeB.nextJointIndex = bodyB->jointIndex;
	bodyB->jointIndex = joint->object.index;

	joint->localAnchorA = def->localAnchorA;
	joint->localAnchorB = def->localAnchorB;

	b2RevoluteJoint empty = {0};
	joint->revoluteJoint = empty;

	joint->revoluteJoint.referenceAngle = def->referenceAngle;
	joint->revoluteJoint.impulse = b2Vec2_zero;
	joint->revoluteJoint.axialMass = 0.0f;
	joint->revoluteJoint.motorImpulse = 0.0f;
	joint->revoluteJoint.lowerImpulse = 0.0f;
	joint->revoluteJoint.upperImpulse = 0.0f;
	joint->revoluteJoint.lowerAngle = def->lowerAngle;
	joint->revoluteJoint.upperAngle = def->upperAngle;
	joint->revoluteJoint.maxMotorTorque = def->maxMotorTorque;
	joint->revoluteJoint.motorSpeed = def->motorSpeed;
	joint->revoluteJoint.enableLimit = def->enableLimit;
	joint->revoluteJoint.enableMotor = def->enableMotor;
	joint->revoluteJoint.angle = 0.0f;

	// If the joint prevents collisions, then destroy all contacts between attached shapes
	if (def->collideConnected == false)
	{
		int32_t shapeIndex = bodyB->shapeIndex;
		while (shapeIndex != B2_NULL_INDEX)
		{
			b2Shape* shape = world->shapes + shapeIndex;
			b2ContactEdge* edge = shape->contacts;
			while (edge)
			{
				b2ContactEdge* next = edge->next;
				b2Shape* otherShape = world->shapes + edge->otherShapeIndex;
				if (otherShape->bodyIndex == bodyA->object.index)
				{
					b2DestroyContact(world, edge->contact);
				}

				edge = next;
			}
		
			shapeIndex = shape->nextShapeIndex;
		}
	}

	b2JointId jointId = {joint->object.index, world->index, joint->object.revision};

	return jointId;
}

void b2World_DestroyJoint(b2JointId jointId)
{
	b2World* world = b2GetWorldFromIndex(jointId.world);
	assert(world->locked == false);

	if (world->locked)
	{
		return;
	}

	assert(0 <= jointId.index && jointId.index < world->jointPool.capacity);

	b2Joint* joint = world->joints + jointId.index;

	assert(0 <= joint->edgeA.bodyIndex && joint->edgeA.bodyIndex < world->bodyPool.capacity);
	assert(0 <= joint->edgeB.bodyIndex && joint->edgeB.bodyIndex < world->bodyPool.capacity);

	int32_t bodyIndexA = joint->edgeA.bodyIndex;
	int32_t bodyIndexB = joint->edgeB.bodyIndex;

	b2Body* bodyA = world->bodies + joint->edgeA.bodyIndex;
	b2Body* bodyB = world->bodies + joint->edgeB.bodyIndex;

	// Remove the joint from the bodyA singly linked list.
	int32_t* jointIndex = &bodyA->jointIndex;
	bool found = false;
	while (*jointIndex != B2_NULL_INDEX)
	{
		if (*jointIndex == jointId.index)
		{
			*jointIndex = joint->edgeA.nextJointIndex;
			found = true;
			break;
		}

		// bodyA may be bodyB on other joints in the linked list
		b2Joint* otherJoint = world->joints + *jointIndex;
		if (joint->edgeA.bodyIndex == bodyIndexA)
		{
			jointIndex = &(otherJoint->edgeA.nextJointIndex);
		}

		if (joint->edgeB.bodyIndex == bodyIndexA)
		{
			jointIndex = &(otherJoint->edgeB.nextJointIndex);
		}

		assert(false);
		return;
	}

	assert(found);
	if (found == false)
	{
		return;
	}

	// Remove the joint from the bodyB singly linked list.
	jointIndex = &bodyB->jointIndex;
	found = false;
	while (*jointIndex != B2_NULL_INDEX)
	{
		if (*jointIndex == jointId.index)
		{
			*jointIndex = joint->edgeB.nextJointIndex;
			found = true;
			break;
		}

		// bodyB may be bodyA on other joints in the linked list
		b2Joint* otherJoint = world->joints + *jointIndex;
		if (joint->edgeA.bodyIndex == bodyIndexB)
		{
			jointIndex = &(otherJoint->edgeA.nextJointIndex);
		}

		if (joint->edgeB.bodyIndex == bodyIndexB)
		{
			jointIndex = &(otherJoint->edgeB.nextJointIndex);
		}

		assert(false);
		return;
	}

	assert(found);
	if (found == false)
	{
		return;
	}

	b2FreeObject(&world->jointPool, &joint->object);
}

extern void b2InitializeMouse(b2World* world, b2Joint* base, b2SolverData* data);
extern void b2InitializeRevolute(b2World* world, b2Joint* base, b2SolverData* data);

void b2InitVelocityConstraints(b2World* world, b2Joint* joint, b2SolverData* data)
{
	switch (joint->type)
	{
		case b2_mouseJoint:
			b2InitializeMouse(world, joint, data);
			break;

		case b2_revoluteJoint:
			b2InitializeRevolute(world, joint, data);
			break;

		default:
			assert(false);
	}
}

extern void b2SolveMouseVelocity(b2Joint* base, b2SolverData* data);
extern void b2SolveRevoluteVelocity(b2Joint* base, b2SolverData* data);

void b2SolveVelocityConstraints(b2Joint* joint, b2SolverData* data)
{
	switch (joint->type)
	{
		case b2_mouseJoint:
			b2SolveMouseVelocity(joint, data);
			break;

		case b2_revoluteJoint:
			b2SolveRevoluteVelocity(joint, data);
			break;

		default:
			assert(false);
	}
}

extern bool b2SolveRevolutePosition(b2Joint* base, b2SolverData* data);

// This returns true if the position errors are within tolerance.
bool b2SolvePositionConstraints(b2Joint* joint, b2SolverData* data)
{
	switch (joint->type)
	{
		case b2_revoluteJoint:
			return b2SolveRevolutePosition(joint, data);

		default:
			return true;
	}
}

extern void b2DrawRevolute(b2DebugDraw* draw, b2Joint* base, b2Body* bodyA, b2Body* bodyB);

void b2DrawJoint(b2DebugDraw* draw, b2World* world, b2Joint* joint)
{
	b2Body* bodyA = world->bodies + joint->edgeA.bodyIndex;
	b2Body* bodyB = world->bodies + joint->edgeB.bodyIndex;

	b2Transform xfA = bodyA->transform;
	b2Transform xfB = bodyB->transform;
	b2Vec2 pA = b2TransformPoint(bodyA->transform, joint->localAnchorA);
	b2Vec2 pB = b2TransformPoint(bodyB->transform, joint->localAnchorB);

	b2Color color = {0.5f, 0.8f, 0.8f, 1.0f};

	switch (joint->type)
	{
	case b2_distanceJoint:
		draw->DrawSegment(pA, pB, color, draw->context);
		break;

	//case b2_pulleyJoint:
	//{
	//	b2PulleyJoint* pulley = (b2PulleyJoint*)this;
	//	b2Vec2 sA = pulley->GetGroundAnchorA();
	//	b2Vec2 sB = pulley->GetGroundAnchorB();
	//	draw->DrawSegment(sA, pA, color);
	//	draw->DrawSegment(sB, pB, color);
	//	draw->DrawSegment(sA, sB, color);
	//}
	//break;

	case b2_mouseJoint:
	{
		b2Color c1 = {0.0f, 1.0f, 0.0f, 1.0f};
		draw->DrawPoint(pA, 4.0f, c1, draw->context);
		draw->DrawPoint(pB, 4.0f, c1, draw->context);

		b2Color c2 = {0.8f, 0.8f, 0.8f, 1.0f};
		draw->DrawSegment(pA, pB, c2, draw->context);

	}
	break;

	case b2_revoluteJoint:
		b2DrawRevolute(draw, joint, bodyA, bodyB);
		break;

	default:
		draw->DrawSegment(xfA.p, pA, color, draw->context);
		draw->DrawSegment(pA, pB, color, draw->context);
		draw->DrawSegment(xfB.p, pB, color, draw->context);
	}
}
