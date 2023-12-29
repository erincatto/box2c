// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "sample.h"

#include "settings.h"

#include "box2d/box2d.h"
#include "box2d/callbacks.h"
#include "box2d/joint_util.h"
#include "box2d/manifold.h"
#include "box2d/math.h"
#include "box2d/timer.h"

#include <GLFW/glfw3.h>
#include <stdio.h>
#include <string.h>

static void* EnqueueTask(b2TaskCallback* task, int32_t itemCount, int32_t minRange, void* taskContext, void* userContext)
{
	Sample* sample = static_cast<Sample*>(userContext);
	if (sample->m_taskCount < maxTasks)
	{
		SampleTask& sampleTask = sample->m_tasks[sample->m_taskCount];
		sampleTask.m_SetSize = itemCount;
		sampleTask.m_MinRange = minRange;
		sampleTask.m_task = task;
		sampleTask.m_taskContext = taskContext;
		sample->m_scheduler.AddTaskSetToPipe(&sampleTask);
		++sample->m_taskCount;
		return &sampleTask;
	}
	else
	{
		// This is not fatal but the maxTasks should be increased
		assert(false);
		task(0, itemCount, 0, taskContext);
		return nullptr;
	}
}

static void FinishTask(void* taskPtr, void* userContext)
{
	if (taskPtr != nullptr)
	{
		SampleTask* sampleTask = static_cast<SampleTask*>(taskPtr);
		Sample* sample = static_cast<Sample*>(userContext);
		sample->m_scheduler.WaitforTask(sampleTask);
	}
}

Sample::Sample(const Settings& settings)
{
	b2Vec2 gravity = {0.0f, -10.0f};

	m_scheduler.Initialize(settings.workerCount);
	m_taskCount = 0;

	b2WorldDef worldDef = b2_defaultWorldDef;
	worldDef.workerCount = settings.workerCount;
	worldDef.enqueueTask = &EnqueueTask;
	worldDef.finishTask = &FinishTask;
	worldDef.userTaskContext = this;
	worldDef.enableSleep = settings.enableSleep;

	// These are not ideal, but useful for testing Box2D
	worldDef.bodyCapacity = 2;
	worldDef.contactCapacity = 2;
	worldDef.arenaAllocatorCapacity = 0;

	m_worldId = b2CreateWorld(&worldDef);
	m_textLine = 30;
	m_textIncrement = 18;
	m_mouseJointId = b2_nullJointId;

	m_stepCount = 0;

	m_groundBodyId = b2_nullBodyId;

	m_maxProfile = b2_emptyProfile;
	m_totalProfile = b2_emptyProfile;
}

Sample::~Sample()
{
	// By deleting the world, we delete the bomb, mouse joint, etc.
	b2DestroyWorld(m_worldId);
}

void Sample::DrawTitle(const char* string)
{
	g_draw.DrawString(5, 5, string);
	m_textLine = int32_t(26.0f);
}

struct QueryContext
{
	b2Vec2 point;
	b2BodyId bodyId = b2_nullBodyId;
};

bool QueryCallback(b2ShapeId shapeId, void* context)
{
	QueryContext* queryContext = static_cast<QueryContext*>(context);

	b2BodyId bodyId = b2Shape_GetBody(shapeId);
	b2BodyType bodyType = b2Body_GetType(bodyId);
	if (bodyType != b2_dynamicBody)
	{
		// continue query
		return true;
	}

	bool overlap = b2Shape_TestPoint(shapeId, queryContext->point);
	if (overlap)
	{
		// found shape
		queryContext->bodyId = bodyId;
		return false;
	}

	return true;
}

void Sample::MouseDown(b2Vec2 p, int button, int mod)
{
	if (B2_NON_NULL(m_mouseJointId))
	{
		return;
	}

	if (button == GLFW_MOUSE_BUTTON_1)
	{
		// Make a small box.
		b2AABB box;
		b2Vec2 d = {0.001f, 0.001f};
		box.lowerBound = b2Sub(p, d);
		box.upperBound = b2Add(p, d);

		// Query the world for overlapping shapes.
		QueryContext queryContext = {p, b2_nullBodyId};
		b2World_QueryAABB(m_worldId, QueryCallback, box,b2_defaultQueryFilter, &queryContext);

		if (B2_NON_NULL(queryContext.bodyId))
		{
			float frequencyHz = 5.0f;
			float dampingRatio = 0.7f;
			float mass = b2Body_GetMass(queryContext.bodyId);
			
			m_groundBodyId = b2CreateBody(m_worldId, &b2_defaultBodyDef);

			b2MouseJointDef jd = b2DefaultMouseJointDef();
			jd.bodyIdA = m_groundBodyId;
			jd.bodyIdB = queryContext.bodyId;
			jd.target = p;
			jd.maxForce = 1000.0f * mass;
			b2LinearStiffness(&jd.stiffness, &jd.damping, frequencyHz, dampingRatio, m_groundBodyId, queryContext.bodyId);

			m_mouseJointId = b2CreateMouseJoint(m_worldId, &jd);

			b2Body_Wake(queryContext.bodyId);
		}
	}
}

void Sample::MouseUp(b2Vec2 p, int button)
{
	if (B2_NON_NULL(m_mouseJointId) && button == GLFW_MOUSE_BUTTON_1)
	{
		b2DestroyJoint(m_mouseJointId);
		m_mouseJointId = b2_nullJointId;

		b2DestroyBody(m_groundBodyId);
		m_groundBodyId = b2_nullBodyId;
	}
}

void Sample::MouseMove(b2Vec2 p)
{
	if (B2_NON_NULL(m_mouseJointId))
	{
		b2MouseJoint_SetTarget(m_mouseJointId, p);
		b2BodyId bodyIdB = b2Joint_GetBodyB(m_mouseJointId);
		b2Body_Wake(bodyIdB);
	}
}

void Sample::ResetProfile()
{
	m_totalProfile = b2_emptyProfile;
	m_maxProfile = b2_emptyProfile;
	m_stepCount = 0;
}

void Sample::Step(Settings& settings)
{
	float timeStep = settings.hertz > 0.0f ? 1.0f / settings.hertz : 0.0f;

	if (settings.pause)
	{
		if (settings.singleStep)
		{
			settings.singleStep = 0;
		}
		else
		{
			timeStep = 0.0f;
		}

		g_draw.DrawString(5, m_textLine, "****PAUSED****");
		m_textLine += m_textIncrement;
	}

	g_draw.m_debugDraw.drawShapes = settings.drawShapes;
	g_draw.m_debugDraw.drawJoints = settings.drawJoints;
	g_draw.m_debugDraw.drawAABBs = settings.drawAABBs;
	g_draw.m_debugDraw.drawMass = settings.drawMass;
	g_draw.m_debugDraw.drawContacts = settings.drawContactPoints;
	g_draw.m_debugDraw.drawGraphColors = settings.drawGraphColors;
	g_draw.m_debugDraw.drawContactNormals = settings.drawContactNormals;
	g_draw.m_debugDraw.drawContactImpulses = settings.drawContactImpulses;
	g_draw.m_debugDraw.drawFrictionImpulses = settings.drawFrictionImpulses;

	b2World_EnableSleeping(m_worldId, settings.enableSleep);
	b2World_EnableWarmStarting(m_worldId, settings.enableWarmStarting);
	b2World_EnableContinuous(m_worldId, settings.enableContinuous);

	for (int32_t i = 0; i < 1; ++i)
	{
		b2World_Step(m_worldId, timeStep, settings.velocityIterations, settings.relaxIterations);
		m_taskCount = 0;
	}

	b2World_Draw(m_worldId, &g_draw.m_debugDraw);

	if (timeStep > 0.0f)
	{
		++m_stepCount;
	}

	if (settings.drawCounters)
	{
		b2Counters s = b2World_GetCounters(m_worldId);

		g_draw.DrawString(5, m_textLine, "islands/bodies/contacts/joints = %d/%d/%d/%d", s.islandCount, s.bodyCount, s.contactCount,
						  s.jointCount);
		m_textLine += m_textIncrement;

		g_draw.DrawString(5, m_textLine, "pairs/proxies/height = %d/%d/%d", s.pairCount, s.proxyCount, s.treeHeight);
		m_textLine += m_textIncrement;

		int32_t totalCount = 0;
		char buffer[256] = {0};
		int32_t offset = snprintf(buffer, 256, "colors: ");
		for (int32_t i = 0; i < b2_graphColorCount; ++i)
		{
			offset += snprintf(buffer + offset, 256 - offset, "%d/", s.colorCounts[i]);
			totalCount += s.colorCounts[i];
		}
		totalCount += s.colorCounts[b2_graphColorCount];
		snprintf(buffer + offset, 256 - offset, "(%d)[%d]", s.colorCounts[b2_graphColorCount], totalCount);
		g_draw.DrawString(5, m_textLine, buffer);
		m_textLine += m_textIncrement;

		g_draw.DrawString(5, m_textLine, "tree: proxies/height = %d/%d", s.proxyCount, s.treeHeight);
		m_textLine += m_textIncrement;

		g_draw.DrawString(5, m_textLine, "stack allocator capacity/used = %d/%d", s.stackCapacity, s.stackUsed);
		m_textLine += m_textIncrement;

		g_draw.DrawString(5, m_textLine, "task count = %d", s.taskCount);
		m_textLine += m_textIncrement;

		g_draw.DrawString(5, m_textLine, "total bytes allocated = %d", s.byteCount);
		m_textLine += m_textIncrement;
	}

	// Track maximum profile times
	{
		b2Profile p = b2World_GetProfile(m_worldId);
		m_maxProfile.step = B2_MAX(m_maxProfile.step, p.step);
		m_maxProfile.pairs = B2_MAX(m_maxProfile.pairs, p.pairs);
		m_maxProfile.collide = B2_MAX(m_maxProfile.collide, p.collide);
		m_maxProfile.solve = B2_MAX(m_maxProfile.solve, p.solve);
		m_maxProfile.buildIslands = B2_MAX(m_maxProfile.buildIslands, p.buildIslands);
		m_maxProfile.solveConstraints = B2_MAX(m_maxProfile.solveConstraints, p.solveConstraints);
		m_maxProfile.broadphase = B2_MAX(m_maxProfile.broadphase, p.broadphase);
		m_maxProfile.continuous = B2_MAX(m_maxProfile.continuous, p.continuous);

		m_totalProfile.step += p.step;
		m_totalProfile.pairs += p.pairs;
		m_totalProfile.collide += p.collide;
		m_totalProfile.solve += p.solve;
		m_totalProfile.buildIslands += p.buildIslands;
		m_totalProfile.solveConstraints += p.solveConstraints;
		m_totalProfile.broadphase += p.broadphase;
		m_totalProfile.continuous += p.continuous;
	}

	if (settings.drawProfile)
	{
		b2Profile p = b2World_GetProfile(m_worldId);

		b2Profile aveProfile;
		memset(&aveProfile, 0, sizeof(b2Profile));
		if (m_stepCount > 0)
		{
			float scale = 1.0f / m_stepCount;
			aveProfile.step = scale * m_totalProfile.step;
			aveProfile.pairs = scale * m_totalProfile.pairs;
			aveProfile.collide = scale * m_totalProfile.collide;
			aveProfile.solve = scale * m_totalProfile.solve;
			aveProfile.buildIslands = scale * m_totalProfile.buildIslands;
			aveProfile.solveConstraints = scale * m_totalProfile.solveConstraints;
			aveProfile.broadphase = scale * m_totalProfile.broadphase;
			aveProfile.continuous = scale * m_totalProfile.continuous;
		}

		g_draw.DrawString(5, m_textLine, "step [ave] (max) = %5.2f [%6.2f] (%6.2f)", p.step, aveProfile.step, m_maxProfile.step);
		m_textLine += m_textIncrement;
		g_draw.DrawString(5, m_textLine, "pairs [ave] (max) = %5.2f [%6.2f] (%6.2f)", p.pairs, aveProfile.pairs, m_maxProfile.pairs);
		m_textLine += m_textIncrement;
		g_draw.DrawString(5, m_textLine, "collide [ave] (max) = %5.2f [%6.2f] (%6.2f)", p.collide, aveProfile.collide,
						  m_maxProfile.collide);
		m_textLine += m_textIncrement;
		g_draw.DrawString(5, m_textLine, "solve [ave] (max) = %5.2f [%6.2f] (%6.2f)", p.solve, aveProfile.solve, m_maxProfile.solve);
		m_textLine += m_textIncrement;
		g_draw.DrawString(5, m_textLine, "builds island [ave] (max) = %5.2f [%6.2f] (%6.2f)", p.buildIslands, aveProfile.buildIslands,
						  m_maxProfile.buildIslands);
		m_textLine += m_textIncrement;
		g_draw.DrawString(5, m_textLine, "solve constraints [ave] (max) = %5.2f [%6.2f] (%6.2f)", p.solveConstraints, aveProfile.solveConstraints,
						  m_maxProfile.solveConstraints);
		m_textLine += m_textIncrement;
		g_draw.DrawString(5, m_textLine, "broad-phase [ave] (max) = %5.2f [%6.2f] (%6.2f)", p.broadphase, aveProfile.broadphase,
						  m_maxProfile.broadphase);
		m_textLine += m_textIncrement;
		g_draw.DrawString(5, m_textLine, "continuous collision [ave] (max) = %5.2f [%6.2f] (%6.2f)", p.continuous, aveProfile.continuous,
						  m_maxProfile.continuous);
		m_textLine += m_textIncrement;
	}

	if (settings.drawContactPoints)
	{

	}
}

void Sample::ShiftOrigin(b2Vec2 newOrigin)
{
	// m_world->ShiftOrigin(newOrigin);
}

SampleEntry g_sampleEntries[MAX_SAMPLES] = {{nullptr}};
int g_sampleCount = 0;

int RegisterSample(const char* category, const char* name, SampleCreateFcn* fcn)
{
	int index = g_sampleCount;
	if (index < MAX_SAMPLES)
	{
		g_sampleEntries[index] = {category, name, fcn};
		++g_sampleCount;
		return index;
	}

	return -1;
}
