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

bool PreSolveFcn(b2ShapeId shapeIdA, b2ShapeId shapeIdB, b2Manifold* manifold, int32_t color, void* context)
{
	Sample* sample = static_cast<Sample*>(context);
	return sample->PreSolve(shapeIdA, shapeIdB, manifold, color);
}

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
		assert(false);
		task(0, itemCount, 0, taskContext);
		return nullptr;
	}
}

static void FinishTask(void* taskPtr, void* userContext)
{
	SampleTask* sampleTask = static_cast<SampleTask*>(taskPtr);
	Sample* sample = static_cast<Sample*>(userContext);
	sample->m_scheduler.WaitforTask(sampleTask);
}

static void FinishAllTasks(void* userContext)
{
	Sample* sample = static_cast<Sample*>(userContext);
	sample->m_scheduler.WaitforAll();
	sample->m_taskCount = 0;
}

Sample::Sample(const Settings& settings)
{
	b2Vec2 gravity = {0.0f, -10.0f};

	// TODO_ERIN want core count, not including hyper-threads which don't work well for physics
	uint32_t maxThreads = enki::GetNumHardwareThreads() / 2;
	m_scheduler.Initialize(maxThreads);
	m_taskCount = 0;

	b2WorldDef worldDef = b2DefaultWorldDef();
	worldDef.workerCount = maxThreads;
	worldDef.enqueueTask = &EnqueueTask;
	worldDef.finishTask = &FinishTask;
	worldDef.finishAllTasks = &FinishAllTasks;
	worldDef.bodyCapacity = 1024;
	worldDef.contactCapacity = 4 * 1024;
	worldDef.userTaskContext = this;
	worldDef.stackAllocatorCapacity = 20 * 1024;
	worldDef.enableSleep = settings.m_enableSleep;

	m_worldId = b2CreateWorld(&worldDef);
	m_textLine = 30;
	m_textIncrement = 18;
	m_mouseJointId = b2_nullJointId;
	m_pointCount = 0;

	// m_destructionListener.test = this;
	// m_world->SetDestructionListener(&m_destructionListener);
	// m_world->SetContactListener(this);

	// TODO_ERIN too expensive
	b2World_SetPreSolveCallback(m_worldId, PreSolveFcn, this);

	m_stepCount = 0;

	b2BodyDef bodyDef = b2DefaultBodyDef();
	m_groundBodyId = b2World_CreateBody(m_worldId, &bodyDef);

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
		b2World_QueryAABB(m_worldId, box, QueryCallback, &queryContext);

		if (B2_NON_NULL(queryContext.bodyId))
		{
			float frequencyHz = 5.0f;
			float dampingRatio = 0.7f;
			float mass = b2Body_GetMass(queryContext.bodyId);

			b2MouseJointDef jd;
			jd.bodyIdA = m_groundBodyId;
			jd.bodyIdB = queryContext.bodyId;
			jd.target = p;
			jd.maxForce = 1000.0f * mass;
			b2LinearStiffness(&jd.stiffness, &jd.damping, frequencyHz, dampingRatio, m_groundBodyId, queryContext.bodyId);

			m_mouseJointId = b2World_CreateMouseJoint(m_worldId, &jd);

			b2Body_Wake(queryContext.bodyId);
		}
	}
}

void Sample::MouseUp(b2Vec2 p, int button)
{
	if (B2_NON_NULL(m_mouseJointId) && button == GLFW_MOUSE_BUTTON_1)
	{
		b2World_DestroyJoint(m_mouseJointId);
		m_mouseJointId = b2_nullJointId;
	}
}

void Sample::MouseMove(b2Vec2 p)
{
	if (B2_NON_NULL(m_mouseJointId))
	{
		b2MouseJoint_SetTarget(m_mouseJointId, p);
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

		g_draw.DrawString(5, m_textLine, "****PAUSED****");
		m_textLine += m_textIncrement;
	}

	g_draw.m_debugDraw.drawShapes = settings.m_drawShapes;
	g_draw.m_debugDraw.drawJoints = settings.m_drawJoints;
	g_draw.m_debugDraw.drawAABBs = settings.m_drawAABBs;
	g_draw.m_debugDraw.drawCOMs = settings.m_drawCOMs;

	b2World_EnableSleeping(m_worldId, settings.m_enableSleep);

	// m_world->SetWarmStarting(settings.m_enableWarmStarting);
	// m_world->SetContinuousPhysics(settings.m_enableContinuous);

	m_pointCount = 0;

	for (int32_t i = 0; i < 1; ++i)
	{
		b2World_Step(m_worldId, timeStep, settings.m_velocityIterations, settings.m_positionIterations);
	}
	b2World_Draw(m_worldId, &g_draw.m_debugDraw);

	if (timeStep > 0.0f)
	{
		++m_stepCount;
	}

	if (settings.m_drawStats)
	{
		b2Statistics s = b2World_GetStatistics(m_worldId);

		g_draw.DrawString(5, m_textLine, "islands/bodies/contacts/joints = %d/%d/%d/%d", s.islandCount, s.bodyCount, s.contactCount,
						  s.jointCount);
		m_textLine += m_textIncrement;

		g_draw.DrawString(5, m_textLine, "proxies/height = %d/%d", s.proxyCount, s.treeHeight);
		m_textLine += m_textIncrement;

		g_draw.DrawString(5, m_textLine, "stack allocator capacity/used = %d/%d", s.stackCapacity, s.stackUsed);
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
		m_maxProfile.solveIslands = B2_MAX(m_maxProfile.solveIslands, p.solveIslands);
		m_maxProfile.broadphase = B2_MAX(m_maxProfile.broadphase, p.broadphase);
		m_maxProfile.continuous = B2_MAX(m_maxProfile.continuous, p.continuous);

		m_totalProfile.step += p.step;
		m_totalProfile.pairs += p.pairs;
		m_totalProfile.collide += p.collide;
		m_totalProfile.solve += p.solve;
		m_totalProfile.buildIslands += p.buildIslands;
		m_totalProfile.solveIslands += p.solveIslands;
		m_totalProfile.broadphase += p.broadphase;
		m_totalProfile.continuous += p.continuous;
	}

	if (settings.m_drawProfile)
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
			aveProfile.solveIslands = scale * m_totalProfile.solveIslands;
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
		g_draw.DrawString(5, m_textLine, "solve islands [ave] (max) = %5.2f [%6.2f] (%6.2f)", p.solveIslands, aveProfile.solveIslands,
						  m_maxProfile.solveIslands);
		m_textLine += m_textIncrement;
		g_draw.DrawString(5, m_textLine, "broad-phase [ave] (max) = %5.2f [%6.2f] (%6.2f)", p.broadphase, aveProfile.broadphase,
						  m_maxProfile.broadphase);
		m_textLine += m_textIncrement;
		g_draw.DrawString(5, m_textLine, "continuous collision [ave] (max) = %5.2f [%6.2f] (%6.2f)", p.continuous, aveProfile.continuous,
						  m_maxProfile.continuous);
		m_textLine += m_textIncrement;
	}

	if (settings.m_drawContactPoints)
	{
		const float k_impulseScale = 1.0f;
		const float k_axisScale = 0.3f;
		b2Color speculativeColor = {0.3f, 0.3f, 0.3f, 1.0f};
		b2Color addColor = {0.3f, 0.95f, 0.3f, 1.0f};
		b2Color persistColor = {0.3f, 0.3f, 0.95f, 1.0f};

		b2HexColor colors[8] = {b2_colorAquamarine, b2_colorBisque,	   b2_colorBlue,	   b2_colorBrown,
								b2_colorBurlywood,	b2_colorCadetBlue, b2_colorChartreuse, b2_colorChocolate};

		for (int32_t i = 0; i < m_pointCount; ++i)
		{
			ContactPoint* point = m_points + i;

			if (0 <= point->color && point->color < 8)
			{
				// graph color
				g_draw.DrawPoint(point->position, 5.0f, b2MakeColor(colors[point->color], 1.0f));
				g_draw.DrawString(point->position, "%d", point->color);
			}
			else if (point->separation > b2_linearSlop)
			{
				// Speculative
				g_draw.DrawPoint(point->position, 5.0f, speculativeColor);
			}
			else if (point->persisted == false)
			{
				// Add
				g_draw.DrawPoint(point->position, 10.0f, addColor);
			}
			else if (point->persisted == true)
			{
				// Persist
				g_draw.DrawPoint(point->position, 5.0f, persistColor);
			}

			if (settings.m_drawContactNormals == 1)
			{
				b2Vec2 p1 = point->position;
				b2Vec2 p2 = b2MulAdd(p1, k_axisScale, point->normal);
				g_draw.DrawSegment(p1, p2, {0.9f, 0.9f, 0.9f, 1.0f});
			}
			else if (settings.m_drawContactImpulse == 1)
			{
				b2Vec2 p1 = point->position;
				b2Vec2 p2 = b2MulAdd(p1, k_impulseScale * point->normalImpulse, point->normal);
				g_draw.DrawSegment(p1, p2, {0.9f, 0.9f, 0.3f, 1.0f});
				g_draw.DrawString(p1, "%.2f", point->normalImpulse);
			}

			if (settings.m_drawFrictionImpulse == 1)
			{
				b2Vec2 tangent = b2CrossVS(point->normal, 1.0f);
				b2Vec2 p1 = point->position;
				b2Vec2 p2 = b2MulAdd(p1, k_impulseScale * point->tangentImpulse, tangent);
				g_draw.DrawSegment(p1, p2, {0.9f, 0.9f, 0.3f, 1.0f});
				g_draw.DrawString(p1, "%.2f", point->tangentImpulse);
			}
		}
	}
}

void Sample::ShiftOrigin(b2Vec2 newOrigin)
{
	// m_world->ShiftOrigin(newOrigin);
}

// Thread-safe callback
bool Sample::PreSolve(b2ShapeId shapeIdA, b2ShapeId shapeIdB, b2Manifold* manifold, int32_t color)
{
	long startCount = m_pointCount.fetch_add(manifold->pointCount);
	if (startCount >= k_maxContactPoints)
	{
		m_pointCount.store(k_maxContactPoints);
		return true;
	}

	long endCount = B2_MIN(k_maxContactPoints, startCount + manifold->pointCount);

	int32_t j = 0;
	for (long i = startCount; i < endCount; ++i)
	{
		ContactPoint* cp = m_points + i;
		cp->shapeIdA = shapeIdA;
		cp->shapeIdB = shapeIdB;
		cp->normal = manifold->normal;
		cp->position = manifold->points[j].point;
		cp->separation = manifold->points[j].separation;
		cp->normalImpulse = manifold->points[j].normalImpulse;
		cp->tangentImpulse = manifold->points[j].tangentImpulse;
		cp->persisted = manifold->points[j].persisted;
		cp->color = color;
		++j;
	}

	return true;
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
