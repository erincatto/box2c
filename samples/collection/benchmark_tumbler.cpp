// SPDX-FileCopyrightText: 2022 Erin Catto
// SPDX-License-Identifier: MIT

#include "box2d/box2d.h"
#include "box2d/geometry.h"
#include "sample.h"

class BenchmarkTumbler : public Sample
{
public:

	BenchmarkTumbler(const Settings& settings)
		: Sample(settings)
	{
		b2BodyId groundId;
		{
			b2BodyDef bd = b2DefaultBodyDef();
			groundId = b2World_CreateBody(m_worldId, &bd);
		}

		{
			b2BodyDef bd = b2DefaultBodyDef();
			bd.type = b2_dynamicBody;
			bd.enableSleep = false;
			bd.position = {0.0f, 10.0f};
			b2BodyId bodyId = b2World_CreateBody(m_worldId, &bd);

			b2ShapeDef sd = b2DefaultShapeDef();
			sd.density = 50.0f;

			b2Polygon polygon;
			polygon = b2MakeOffsetBox(0.5f, 10.0f, {10.0f, 0.0f}, 0.0);
			b2Body_CreatePolygon(bodyId, &sd, &polygon);
			polygon = b2MakeOffsetBox(0.5f, 10.0f, {-10.0f, 0.0f}, 0.0);
			b2Body_CreatePolygon(bodyId, &sd, &polygon);
			polygon = b2MakeOffsetBox(10.0f, 0.5f, {0.0f, 10.0f}, 0.0);
			b2Body_CreatePolygon(bodyId, &sd, &polygon);
			polygon = b2MakeOffsetBox(10.0f, 0.5f, {0.0f, -10.0f}, 0.0);
			b2Body_CreatePolygon(bodyId, &sd, &polygon);

			b2RevoluteJointDef jd = b2DefaultRevoluteJointDef();
			jd.bodyIdA = groundId;
			jd.bodyIdB = bodyId;
			jd.localAnchorA = {0.0f, 10.0f};
			jd.localAnchorB = {0.0f, 0.0f};
			jd.referenceAngle = 0.0f;
			jd.motorSpeed = 0.05f * b2_pi;
			jd.maxMotorTorque = 1e8f;
			jd.enableMotor = true;

			b2World_CreateRevoluteJoint(m_worldId, &jd);
		}

		//m_maxCount = g_sampleDebug ? 500 : 2000;
		m_maxCount = g_sampleDebug ? 2000 : 2000;
		m_count = 0;
	}

	void Step(Settings& settings) override
	{
		Sample::Step(settings);

		for (int32_t i = 0; i < 10 && m_count < m_maxCount; ++i)
		{
			b2BodyDef bd = b2DefaultBodyDef();
			bd.type = b2_dynamicBody;
			bd.position = {0.25f * i, 10.0f};
			b2BodyId bodyId = b2World_CreateBody(m_worldId, &bd);

			b2ShapeDef sd = b2DefaultShapeDef();
			sd.density = 1.0f;

			b2Polygon polygon = b2MakeBox(0.125f, 0.125f);
			b2Body_CreatePolygon(bodyId, &sd, &polygon);
			++m_count;
		}
	}

	static Sample* Create(const Settings& settings)
	{
		return new BenchmarkTumbler(settings);
	}

	int32_t m_maxCount;
	int32_t m_count;
};

static int testIndex = RegisterSample("Benchmark", "Tumbler", BenchmarkTumbler::Create);
