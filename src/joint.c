// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "joint.h"
#include "body.h"
#include "world.h"

void b2LinearStiffness(float* stiffness, float* damping, float frequencyHertz, float dampingRatio, const b2Body* bodyA,
					   const b2Body* bodyB)
{
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

void b2AngularStiffness(float* stiffness, float* damping, float frequencyHertz, float dampingRatio, const b2Body* bodyA,
						const b2Body* bodyB)
{
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

	int32_t index = joint->object.index;

	b2Body* bodyA = world->bodies + joint->edgeA.bodyIndex;
	b2Body* bodyB = world->bodies + joint->edgeB.bodyIndex;

	joint->edgeA.nextJointIndex = bodyA->jointIndex;
	bodyA->jointIndex = index;

	joint->edgeB.nextJointIndex = bodyB->jointIndex;
	bodyB->jointIndex = index;

	b2MouseJoint empty = {0};
	joint->mouseJoint = empty;

	joint->mouseJoint.targetA = def->target;
	joint->mouseJoint.maxForce = def->maxForce;
	joint->mouseJoint.stiffness = def->stiffness;
	joint->mouseJoint.damping = def->damping;
	joint->mouseJoint.localAnchorB = b2InvTransformPoint(bodyB->transform, def->target);

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

extern void b2MouseJoint_InitVelocityConstraints(b2World* world, b2Joint* base, b2SolverData* data);

void b2InitVelocityConstraints(b2World* world, b2Joint* joint, b2SolverData* data)
{
	switch (joint->type)
	{
		case b2_mouseJoint:
			b2MouseJoint_InitVelocityConstraints(world, joint, data);
			break;

		default:
			assert(false);
	}
}

extern void b2MouseJoint_SolveVelocityConstraints(b2Joint* base, b2SolverData* data);

void b2SolveVelocityConstraints(b2Joint* joint, b2SolverData* data)
{
	switch (joint->type)
	{
		case b2_mouseJoint:
			b2MouseJoint_SolveVelocityConstraints(joint, data);
			break;

		default:
			assert(false);
	}
}

// This returns true if the position errors are within tolerance.
bool b2SolvePositionConstraints(b2Joint* joint, b2SolverData* data)
{
	B2_MAYBE_UNUSED(data);

	switch (joint->type)
	{
		default:
			return true;
	}
}

#if 0
bool b2Joint::IsEnabled() const
{
	return m_bodyA->IsEnabled() && m_bodyB->IsEnabled();
}

void b2Joint::Draw(b2Draw* draw) const
{
	const b2Transform& xf1 = m_bodyA->GetTransform();
	const b2Transform& xf2 = m_bodyB->GetTransform();
	b2Vec2 x1 = xf1.p;
	b2Vec2 x2 = xf2.p;
	b2Vec2 p1 = GetAnchorA();
	b2Vec2 p2 = GetAnchorB();

	b2Color color(0.5f, 0.8f, 0.8f);

	switch (m_type)
	{
	case e_distanceJoint:
		draw->DrawSegment(p1, p2, color);
		break;

	case e_pulleyJoint:
	{
		b2PulleyJoint* pulley = (b2PulleyJoint*)this;
		b2Vec2 s1 = pulley->GetGroundAnchorA();
		b2Vec2 s2 = pulley->GetGroundAnchorB();
		draw->DrawSegment(s1, p1, color);
		draw->DrawSegment(s2, p2, color);
		draw->DrawSegment(s1, s2, color);
	}
	break;

	case e_mouseJoint:
	{
		b2Color c;
		c.Set(0.0f, 1.0f, 0.0f);
		draw->DrawPoint(p1, 4.0f, c);
		draw->DrawPoint(p2, 4.0f, c);

		c.Set(0.8f, 0.8f, 0.8f);
		draw->DrawSegment(p1, p2, c);

	}
	break;

	default:
		draw->DrawSegment(x1, p1, color);
		draw->DrawSegment(p1, p2, color);
		draw->DrawSegment(x2, p2, color);
	}
}
#endif
