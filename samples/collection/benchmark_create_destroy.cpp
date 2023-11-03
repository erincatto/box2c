// SPDX-FileCopyrightText: 2022 Erin Catto
// SPDX-License-Identifier: MIT

#include "box2d/box2d.h"
#include "box2d/geometry.h"
#include "sample.h"
#include "settings.h"

class BenchmarkCreateDestroy : public Sample
{
  public:
	enum
	{
		e_maxBaseCount = 100,
		e_maxBodyCount = e_maxBaseCount * (e_maxBaseCount + 1) / 2
	};

	BenchmarkCreateDestroy(const Settings& settings) : Sample(settings)
	{
		float groundSize = 100.0f;

		b2BodyDef bd = b2DefaultBodyDef();
		b2BodyId groundId = b2World_CreateBody(m_worldId, &bd);

		b2Polygon box = b2MakeBox(groundSize, 1.0f);
		b2ShapeDef sd = b2DefaultShapeDef();
		b2Body_CreatePolygon(groundId, &sd, &box);

		for (int32_t i = 0; i < e_maxBodyCount; ++i)
		{
			m_bodies[i] = b2_nullBodyId;
		}

		m_baseCount = g_sampleDebug ? 40 : 100;
		m_iterations = g_sampleDebug ? 1 : 10;
		m_bodyCount = 0;
	}

	void CreateScene()
	{
		for (int32_t i = 0; i < e_maxBodyCount; ++i)
		{
			if (B2_NON_NULL(m_bodies[i]))
			{
				b2World_DestroyBody(m_bodies[i]);
				m_bodies[i] = b2_nullBodyId;
			}
		}

		int32_t count = m_baseCount;
		float rad = 0.5f;
		float shift = rad * 2.0f;
		float centerx = shift * count / 2.0f;
		float centery = shift / 2.0f + 1.0f;
		
		b2BodyDef bd = b2DefaultBodyDef();
		bd.type = b2_dynamicBody;

		b2ShapeDef sd = b2DefaultShapeDef();
		sd.density = 1.0f;
		sd.friction = 0.5f;

		float h = 0.5f;
		b2Polygon cuboid = b2MakeRoundedBox(h, h, 0.0f);

		int32_t index = 0;

		for (int32_t i = 0; i < count; ++i)
		{
			float y = i * shift + centery;

			for (int32_t j = i; j < count; ++j)
			{
				float x = 0.5f * i * shift + (j - i) * shift - centerx;
				bd.position = {x, y};

				assert(index < e_maxBodyCount);
				m_bodies[index] = b2World_CreateBody(m_worldId, &bd);
				b2Body_CreatePolygon(m_bodies[index], &sd, &cuboid);

				index += 1;
			}
		}

		m_bodyCount = index;
	}

	void Step(Settings& settings) override
	{
		float timeStep = settings.m_hertz > 0.0f ? 1.0f / settings.m_hertz : float(0.0f);

		for (int32_t i = 0; i < m_iterations; ++i)
		{
			CreateScene();
			b2World_Step(m_worldId, timeStep, settings.m_velocityIterations, settings.m_relaxIterations);
		}

		Sample::Step(settings);
	}

	static Sample* Create(const Settings& settings)
	{
		return new BenchmarkCreateDestroy(settings);
	}

	b2BodyId m_bodies[e_maxBodyCount];
	int32_t m_bodyCount;
	int32_t m_baseCount;
	int32_t m_iterations;
};

static int sampleIndex = RegisterSample("Benchmark", "CreateDestroy", BenchmarkCreateDestroy::Create);
