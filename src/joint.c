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

static b2Joint* b2CreateJoint(b2World* world, b2Body* bodyA, b2Body* bodyB)
{
	b2Joint* joint = (b2Joint*)b2AllocObject(&world->jointPool);
	world->joints = (b2Joint*)world->jointPool.memory;
	
	int32_t jointIndex = joint->object.index;

	// Doubly linked list on bodyA
	joint->edges[0].bodyIndex = bodyA->object.index;
	joint->edges[0].prevKey = B2_NULL_INDEX;
	joint->edges[0].nextKey = bodyA->jointList;

	int32_t keyA = (jointIndex << 1) | 0;
	if (bodyA->jointList != B2_NULL_INDEX)
	{
		b2Joint* jointA = world->joints + (bodyA->jointList >> 1);
		b2JointEdge* edgeA = jointA->edges + (bodyA->jointList & 1);
		edgeA->prevKey = keyA;
	}
	bodyA->jointList = keyA;
	bodyA->jointCount += 1;

	// Doubly linked list on bodyB
	joint->edges[1].bodyIndex = bodyB->object.index;
	joint->edges[1].prevKey = B2_NULL_INDEX;
	joint->edges[1].nextKey = bodyB->jointList;

	int32_t keyB = (jointIndex << 1) | 1;
	if (bodyB->jointList != B2_NULL_INDEX)
	{
		b2Joint* jointB = world->joints + (bodyB->jointList >> 1);
		b2JointEdge* edgeB = jointB->edges + (bodyB->jointList & 1);
		edgeB->prevKey = keyB;
	}
	bodyB->jointList = keyB;
	bodyB->jointCount += 1;

	joint->islandIndex = B2_NULL_INDEX;
	joint->islandPrev = B2_NULL_INDEX;
	joint->islandNext = B2_NULL_INDEX;

	joint->isMarked = false;

	if (bodyA->type == b2_dynamicBody || bodyB->type == b2_dynamicBody)
	{
		// Add edge to island graph
		b2LinkJoint(world, joint);
	}

	return joint;
}

static void b2DestroyContactsBetweenBodies(b2World* world, b2Body* bodyA, b2Body* bodyB)
{
	int32_t contactKey;
	int32_t otherBodyIndex;

	if (bodyA->contactCount < bodyB->contactCount)
	{
		contactKey = bodyA->contactList;
		otherBodyIndex = bodyB->object.index;
	}
	else
	{
		contactKey = bodyB->contactList;
		otherBodyIndex = bodyA->object.index;
	}

	while (contactKey != B2_NULL_INDEX)
	{
		int32_t contactIndex = contactKey >> 1;
		int32_t edgeIndex = contactKey & 1;

		b2Contact* contact = world->contacts + contactIndex;
		contactKey = contact->edges[edgeIndex].nextKey;

		int32_t otherEdgeIndex = edgeIndex ^ 1;
		if (contact->edges[otherEdgeIndex].bodyIndex == otherBodyIndex)
		{
			// Careful, this removes the contact from the current doubly linked list
			b2DestroyContact(world, contact);
		}
	}
}

b2JointId b2World_CreateMouseJoint(b2WorldId worldId, const b2MouseJointDef* def)
{
	b2World* world = b2GetWorldFromId(worldId);

	assert(world->locked == false);

	if (world->locked)
	{
		return b2_nullJointId;
	}

	assert(b2IsBodyIdValid(world, def->bodyIdA));
	assert(b2IsBodyIdValid(world, def->bodyIdB));

	b2Body* bodyA = world->bodies + def->bodyIdA.index;
	b2Body* bodyB = world->bodies + def->bodyIdB.index;

	b2Joint* joint = b2CreateJoint(world, bodyA, bodyB);

	joint->type = b2_mouseJoint;

	joint->localAnchorA = b2InvTransformPoint(bodyA->transform, def->target);
	joint->localAnchorB = b2InvTransformPoint(bodyB->transform, def->target);

	b2MouseJoint empty = {0};
	joint->mouseJoint = empty;
	joint->mouseJoint.targetA = def->target;
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

	b2Body* bodyA = world->bodies + def->bodyIdA.index;
	b2Body* bodyB = world->bodies + def->bodyIdB.index;

	b2Joint* joint = b2CreateJoint(world, bodyA, bodyB);

	joint->type = b2_revoluteJoint;

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

	// If the joint prevents collisions, then destroy all contacts between attached bodies
	if (def->collideConnected == false)
	{
		b2DestroyContactsBetweenBodies(world, bodyA, bodyB);
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

	assert(0 <= joint->edges[0].bodyIndex && joint->edges[0].bodyIndex < world->bodyPool.capacity);
	assert(0 <= joint->edges[1].bodyIndex && joint->edges[1].bodyIndex < world->bodyPool.capacity);

	b2JointEdge* edgeA = joint->edges + 0;
	b2JointEdge* edgeB = joint->edges + 1;

	b2Body* bodyA = world->bodies + edgeA->bodyIndex;
	b2Body* bodyB = world->bodies + edgeB->bodyIndex;

	// Remove from body A
	if (edgeA->prevKey != B2_NULL_INDEX)
	{
		b2Joint* prevJoint = world->joints + (edgeA->prevKey >> 1);
		b2JointEdge* prevEdge = prevJoint->edges + (edgeA->prevKey & 1);
		prevEdge->nextKey = edgeA->nextKey;
	}

	if (edgeA->nextKey != B2_NULL_INDEX)
	{
		b2Joint* nextJoint = world->joints + (edgeA->nextKey >> 1);
		b2JointEdge* nextEdge = nextJoint->edges + (edgeA->nextKey & 1);
		nextEdge->prevKey = edgeA->prevKey;
	}

	int32_t edgeKeyA = (joint->object.index << 1) | 0;
	if (bodyA->jointList == edgeKeyA)
	{
		bodyA->jointList = edgeA->nextKey;
	}

	bodyA->jointCount -= 1;

	// Remove from body B
	if (edgeB->prevKey != B2_NULL_INDEX)
	{
		b2Joint* prevJoint = world->joints + (edgeB->prevKey >> 1);
		b2JointEdge* prevEdge = prevJoint->edges + (edgeB->prevKey & 1);
		prevEdge->nextKey = edgeB->nextKey;
	}

	if (edgeB->nextKey != B2_NULL_INDEX)
	{
		b2Joint* nextJoint = world->joints + (edgeB->nextKey >> 1);
		b2JointEdge* nextEdge = nextJoint->edges + (edgeB->nextKey & 1);
		nextEdge->prevKey = edgeB->prevKey;
	}

	int32_t edgeKeyB = (joint->object.index << 1) | 1;
	if (bodyB->jointList == edgeKeyB)
	{
		bodyB->jointList = edgeB->nextKey;
	}

	bodyB->jointCount -= 1;

	b2UnlinkJoint(world, joint);

	b2FreeObject(&world->jointPool, &joint->object);
}

extern void b2InitializeMouse(b2Joint* base, b2StepContext* data);
extern void b2InitializeRevolute(b2Joint* base, b2StepContext* data);

void b2InitVelocityConstraints(b2Joint* joint, b2StepContext* data)
{
	switch (joint->type)
	{
		case b2_mouseJoint:
			b2InitializeMouse(joint, data);
			break;

		case b2_revoluteJoint:
			b2InitializeRevolute(joint, data);
			break;

		default:
			assert(false);
	}
}

extern void b2SolveMouseVelocity(b2Joint* base, b2StepContext* data);
extern void b2SolveRevoluteVelocity(b2Joint* base, b2StepContext* data);

void b2SolveVelocityConstraints(b2Joint* joint, b2StepContext* data)
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

extern bool b2SolveRevolutePosition(b2Joint* base, b2StepContext* data);

// This returns true if the position errors are within tolerance.
bool b2SolvePositionConstraints(b2Joint* joint, b2StepContext* data)
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
	b2Body* bodyA = world->bodies + joint->edges[0].bodyIndex;
	b2Body* bodyB = world->bodies + joint->edges[1].bodyIndex;

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
		b2Vec2 target = joint->mouseJoint.targetA;

		b2Color c1 = {0.0f, 1.0f, 0.0f, 1.0f};
		draw->DrawPoint(target, 4.0f, c1, draw->context);
		draw->DrawPoint(pB, 4.0f, c1, draw->context);

		b2Color c2 = {0.8f, 0.8f, 0.8f, 1.0f};
		draw->DrawSegment(target, pB, c2, draw->context);
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
