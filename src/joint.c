// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "joint.h"
#include "body.h"
#include "world.h"

void b2LinearStiffness(float* stiffness, float* damping,
	float frequencyHertz, float dampingRatio,
	const b2Body* bodyA, const b2Body* bodyB)
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

void b2AngularStiffness(float* stiffness, float* damping,
	float frequencyHertz, float dampingRatio,
	const b2Body* bodyA, const b2Body* bodyB)
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

b2JointId b2World_CreateMouseJoint(b2WorldId worldId, const b2MouseJointDef*)
{
	b2JointId jointId;

	return jointId;
}


b2Joint::b2Joint(const b2JointDef* def)
{
	b2Assert(def->bodyA != def->bodyB);

	m_type = def->type;
	m_prev = nullptr;
	m_next = nullptr;
	m_bodyA = def->bodyA;
	m_bodyB = def->bodyB;
	m_index = 0;
	m_collideConnected = def->collideConnected;
	m_islandFlag = false;
	m_userData = def->userData;

	m_edgeA.joint = nullptr;
	m_edgeA.other = nullptr;
	m_edgeA.prev = nullptr;
	m_edgeA.next = nullptr;

	m_edgeB.joint = nullptr;
	m_edgeB.other = nullptr;
	m_edgeB.prev = nullptr;
	m_edgeB.next = nullptr;
}

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
