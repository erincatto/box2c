// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "sample.h"
#include "settings.h"
#include <stdio.h>

#include <GLFW/glfw3.h>

#if 0
void DestructionListener::SayGoodbye(b2Joint* joint)
{
	if (test->m_mouseJoint == joint)
	{
		test->m_mouseJoint = nullptr;
	}
	else
	{
		test->JointDestroyed(joint);
	}
}
#endif

Sample::Sample()
{
	b2Vec2 gravity = { 0.0f, -10.0f };
	m_world = nullptr; // new b2World(gravity);
	m_textLine = 30;
	m_textIncrement = 18;
	m_mouseJoint = nullptr;
	m_pointCount = 0;

	//m_destructionListener.test = this;
	//m_world->SetDestructionListener(&m_destructionListener);
	//m_world->SetContactListener(this);
	//m_world->SetDebugDraw(&g_debugDraw);
	
	m_stepCount = 0;

	//b2BodyDef bodyDef;
	m_groundBody = nullptr; // m_world->CreateBody(&bodyDef);

	//memset(&m_maxProfile, 0, sizeof(b2Profile));
	//memset(&m_totalProfile, 0, sizeof(b2Profile));
}

Sample::~Sample()
{
	// By deleting the world, we delete the bomb, mouse joint, etc.
	delete m_world;
	m_world = nullptr;
}

#if 0
void Sample::PreSolve(b2Contact* contact, const b2Manifold* oldManifold)
{
	const b2Manifold* manifold = contact->GetManifold();

	if (manifold->pointCount == 0)
	{
		return;
	}

	b2Fixture* fixtureA = contact->GetFixtureA();
	b2Fixture* fixtureB = contact->GetFixtureB();

	b2PointState state1[b2_maxManifoldPoints], state2[b2_maxManifoldPoints];
	b2GetPointStates(state1, state2, oldManifold, manifold);

	b2WorldManifold worldManifold;
	contact->GetWorldManifold(&worldManifold);

	for (int32_t i = 0; i < manifold->pointCount && m_pointCount < k_maxContactPoints; ++i)
	{
		ContactPoint* cp = m_points + m_pointCount;
		cp->fixtureA = fixtureA;
		cp->fixtureB = fixtureB;
		cp->position = worldManifold.points[i];
		cp->normal = worldManifold.normal;
		cp->state = state2[i];
		cp->normalImpulse = manifold->points[i].normalImpulse;
		cp->tangentImpulse = manifold->points[i].tangentImpulse;
		cp->separation = worldManifold.separations[i];
		++m_pointCount;
	}
}
#endif

void Sample::DrawTitle(const char *string)
{
    g_debugDraw.DrawString(5, 5, string);
    m_textLine = int32_t(26.0f);
}

#if 0
class QueryCallback : public b2QueryCallback
{
public:
	QueryCallback(const b2Vec2& point)
	{
		m_point = point;
		m_fixture = nullptr;
	}

	bool ReportFixture(b2Fixture* fixture) override
	{
		b2Body* body = fixture->GetBody();
		if (body->GetType() == b2_dynamicBody)
		{
			bool inside = fixture->TestPoint(m_point);
			if (inside)
			{
				m_fixture = fixture;

				// We are done, terminate the query.
				return false;
			}
		}

		// Continue the query.
		return true;
	}

	b2Vec2 m_point;
	b2Fixture* m_fixture;
};
#endif

void Sample::MouseDown(b2Vec2 p, int button, int mod)
{
	if (m_mouseJoint != nullptr)
	{
		return;
	}

	if (button == GLFW_MOUSE_BUTTON_1)
	{
	#if 0
		// Make a small box.
		b2AABB aabb;
		b2Vec2 d;
		d.Set(0.001f, 0.001f);
		aabb.lowerBound = p - d;
		aabb.upperBound = p + d;

		// Query the world for overlapping shapes.
		QueryCallback callback(p);
		m_world->QueryAABB(&callback, aabb);

		if (callback.m_fixture)
		{
			float frequencyHz = 5.0f;
			float dampingRatio = 0.7f;

			b2Body* body = callback.m_fixture->GetBody();
			b2MouseJointDef jd;
			jd.bodyA = m_groundBody;
			jd.bodyB = body;
			jd.target = p;
			jd.maxForce = 1000.0f * body->GetMass();
			b2LinearStiffness(jd.stiffness, jd.damping, frequencyHz, dampingRatio, jd.bodyA, jd.bodyB);

			m_mouseJoint = (b2MouseJoint*)m_world->CreateJoint(&jd);
			body->SetAwake(true);
		}
	#endif
	}
}

void Sample::MouseUp(b2Vec2 p, int button)
{
	if (m_mouseJoint && button == GLFW_MOUSE_BUTTON_1)
	{
		//m_world->DestroyJoint(m_mouseJoint);
		m_mouseJoint = nullptr;
	}
}

void Sample::MouseMove(b2Vec2 p)
{
	if (m_mouseJoint)
	{
		//m_mouseJoint->SetTarget(p);
	}
}

void Sample::Step(Settings& settings)
{
	float timeStep = settings.m_hertz > 0.0f ? 1.0f / settings.m_hertz : float(0.0f);

	if (settings.m_pause)
	{
		if (settings.m_singleStep)
		{
			settings.m_singleStep = 0;
		}
		else
		{
			timeStep = 0.0f;
		}

		g_debugDraw.DrawString(5, m_textLine, "****PAUSED****");
		m_textLine += m_textIncrement;
	}

	g_debugDraw.m_draw.drawShapes = settings.m_drawShapes;
	g_debugDraw.m_draw.drawJoints = settings.m_drawJoints;
	g_debugDraw.m_draw.drawAABBs = settings.m_drawAABBs;
	g_debugDraw.m_draw.drawCOMs = settings.m_drawCOMs;

	//m_world->SetAllowSleeping(settings.m_enableSleep);
	//m_world->SetWarmStarting(settings.m_enableWarmStarting);
	//m_world->SetContinuousPhysics(settings.m_enableContinuous);

	m_pointCount = 0;

	//m_world->Step(timeStep, settings.m_velocityIterations, settings.m_positionIterations);

	//m_world->DebugDraw();

	if (timeStep > 0.0f)
	{
		++m_stepCount;
	}

#if 0
	if (settings.m_drawStats)
	{
		int32_t bodyCount = m_world->GetBodyCount();
		int32_t contactCount = m_world->GetContactCount();
		int32_t jointCount = m_world->GetJointCount();
		g_debugDraw.DrawString(5, m_textLine, "bodies/contacts/joints = %d/%d/%d", bodyCount, contactCount, jointCount);
		m_textLine += m_textIncrement;

		int32_t proxyCount = m_world->GetProxyCount();
		int32_t height = m_world->GetTreeHeight();
		int32_t balance = m_world->GetTreeBalance();
		float quality = m_world->GetTreeQuality();
		g_debugDraw.DrawString(5, m_textLine, "proxies/height/balance/quality = %d/%d/%d/%g", proxyCount, height, balance, quality);
		m_textLine += m_textIncrement;
	}

	// Track maximum profile times
	{
		const b2Profile& p = m_world->GetProfile();
		m_maxProfile.step = b2Max(m_maxProfile.step, p.step);
		m_maxProfile.collide = b2Max(m_maxProfile.collide, p.collide);
		m_maxProfile.solve = b2Max(m_maxProfile.solve, p.solve);
		m_maxProfile.solveInit = b2Max(m_maxProfile.solveInit, p.solveInit);
		m_maxProfile.solveVelocity = b2Max(m_maxProfile.solveVelocity, p.solveVelocity);
		m_maxProfile.solvePosition = b2Max(m_maxProfile.solvePosition, p.solvePosition);
		m_maxProfile.solveTOI = b2Max(m_maxProfile.solveTOI, p.solveTOI);
		m_maxProfile.broadphase = b2Max(m_maxProfile.broadphase, p.broadphase);

		m_totalProfile.step += p.step;
		m_totalProfile.collide += p.collide;
		m_totalProfile.solve += p.solve;
		m_totalProfile.solveInit += p.solveInit;
		m_totalProfile.solveVelocity += p.solveVelocity;
		m_totalProfile.solvePosition += p.solvePosition;
		m_totalProfile.solveTOI += p.solveTOI;
		m_totalProfile.broadphase += p.broadphase;
	}

	if (settings.m_drawProfile)
	{
		const b2Profile& p = m_world->GetProfile();

		b2Profile aveProfile;
		memset(&aveProfile, 0, sizeof(b2Profile));
		if (m_stepCount > 0)
		{
			float scale = 1.0f / m_stepCount;
			aveProfile.step = scale * m_totalProfile.step;
			aveProfile.collide = scale * m_totalProfile.collide;
			aveProfile.solve = scale * m_totalProfile.solve;
			aveProfile.solveInit = scale * m_totalProfile.solveInit;
			aveProfile.solveVelocity = scale * m_totalProfile.solveVelocity;
			aveProfile.solvePosition = scale * m_totalProfile.solvePosition;
			aveProfile.solveTOI = scale * m_totalProfile.solveTOI;
			aveProfile.broadphase = scale * m_totalProfile.broadphase;
		}

		g_debugDraw.DrawString(5, m_textLine, "step [ave] (max) = %5.2f [%6.2f] (%6.2f)", p.step, aveProfile.step, m_maxProfile.step);
		m_textLine += m_textIncrement;
		g_debugDraw.DrawString(5, m_textLine, "collide [ave] (max) = %5.2f [%6.2f] (%6.2f)", p.collide, aveProfile.collide, m_maxProfile.collide);
		m_textLine += m_textIncrement;
		g_debugDraw.DrawString(5, m_textLine, "solve [ave] (max) = %5.2f [%6.2f] (%6.2f)", p.solve, aveProfile.solve, m_maxProfile.solve);
		m_textLine += m_textIncrement;
		g_debugDraw.DrawString(5, m_textLine, "solve init [ave] (max) = %5.2f [%6.2f] (%6.2f)", p.solveInit, aveProfile.solveInit, m_maxProfile.solveInit);
		m_textLine += m_textIncrement;
		g_debugDraw.DrawString(5, m_textLine, "solve velocity [ave] (max) = %5.2f [%6.2f] (%6.2f)", p.solveVelocity, aveProfile.solveVelocity, m_maxProfile.solveVelocity);
		m_textLine += m_textIncrement;
		g_debugDraw.DrawString(5, m_textLine, "solve position [ave] (max) = %5.2f [%6.2f] (%6.2f)", p.solvePosition, aveProfile.solvePosition, m_maxProfile.solvePosition);
		m_textLine += m_textIncrement;
		g_debugDraw.DrawString(5, m_textLine, "solveTOI [ave] (max) = %5.2f [%6.2f] (%6.2f)", p.solveTOI, aveProfile.solveTOI, m_maxProfile.solveTOI);
		m_textLine += m_textIncrement;
		g_debugDraw.DrawString(5, m_textLine, "broad-phase [ave] (max) = %5.2f [%6.2f] (%6.2f)", p.broadphase, aveProfile.broadphase, m_maxProfile.broadphase);
		m_textLine += m_textIncrement;
	}

	if (settings.m_drawContactPoints)
	{
		const float k_impulseScale = 0.1f;
		const float k_axisScale = 0.3f;

		for (int32_t i = 0; i < m_pointCount; ++i)
		{
			ContactPoint* point = m_points + i;

			if (point->separation > b2_linearSlop)
			{
				// Speculative
				g_debugDraw.DrawPoint(point->position, 5.0f, b2Color(0.3f, 0.3f, 0.3f));
			}
			else if (point->state == b2_addState)
			{
				// Add
				g_debugDraw.DrawPoint(point->position, 10.0f, b2Color(0.3f, 0.95f, 0.3f));
			}
			else if (point->state == b2_persistState)
			{
				// Persist
				g_debugDraw.DrawPoint(point->position, 5.0f, b2Color(0.3f, 0.3f, 0.95f));
			}

			if (settings.m_drawContactNormals == 1)
			{
				b2Vec2 p1 = point->position;
				b2Vec2 p2 = p1 + k_axisScale * point->normal;
				g_debugDraw.DrawSegment(p1, p2, b2Color(0.9f, 0.9f, 0.9f));
			}
			else if (settings.m_drawContactImpulse == 1)
			{
				b2Vec2 p1 = point->position;
				b2Vec2 p2 = p1 + k_impulseScale * point->normalImpulse * point->normal;
				g_debugDraw.DrawSegment(p1, p2, b2Color(0.9f, 0.9f, 0.3f));
			}

			if (settings.m_drawFrictionImpulse == 1)
			{
				b2Vec2 tangent = b2Cross(point->normal, 1.0f);
				b2Vec2 p1 = point->position;
				b2Vec2 p2 = p1 + k_impulseScale * point->tangentImpulse * tangent;
				g_debugDraw.DrawSegment(p1, p2, b2Color(0.9f, 0.9f, 0.3f));
			}
		}
	}
#endif

}

void Sample::ShiftOrigin(b2Vec2 newOrigin)
{
	//m_world->ShiftOrigin(newOrigin);
}

SampleEntry g_sampleEntries[MAX_SAMPLES] = { {nullptr} };
int g_sampleCount = 0;

int RegisterSample(const char* category, const char* name, SampleCreateFcn* fcn)
{
	int index = g_sampleCount;
	if (index < MAX_SAMPLES)
	{
		g_sampleEntries[index] = { category, name, fcn };
		++g_sampleCount;
		return index;
	}

	return -1;
}
