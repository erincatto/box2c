// SPDX-FileCopyrightText: 2022 Erin Catto
// SPDX-License-Identifier: MIT

#include "box2d/box2d.h"
#include "box2d/geometry.h"
#include "sample.h"

// TODO_ERIN test more joint types
// TODO_ERIN try to stabilize revolute
class BenchmarkJointGrid : public Sample
{
public:
	BenchmarkJointGrid(const Settings& settings)
		: Sample(settings)
	{
		constexpr float rad = 0.4f;
		constexpr int32_t numi = g_sampleDebug ? 100 : 100;
		constexpr int32_t numk = g_sampleDebug ? 100 : 100;
		constexpr float shift = 1.0f;

		// Allocate to avoid huge stack usage
		b2BodyId* bodies = static_cast<b2BodyId*>(malloc(numi * numk * sizeof(b2BodyId)));
		int32_t index = 0;

		b2ShapeDef sd = b2DefaultShapeDef();
		sd.density = 1.0f;
		sd.filter.maskBits = 0;

		b2Circle circle = {0};
		circle.radius = rad;

		b2RevoluteJointDef jd = b2DefaultRevoluteJointDef();

		for (int32_t k = 0; k < numk; ++k)
		{
			for (int32_t i = 0; i < numi; ++i)
			{
				float fk = (float)k;
				float fi = (float)i;

				b2BodyDef bd = b2DefaultBodyDef();
				if (k >= numk / 2 - 3 && k <= numk / 2 + 3 && i == 0)
				{
					bd.type = b2_staticBody;
				}
				else
				{
					bd.type = b2_dynamicBody;
				}

				bd.position = {fk * shift, -fi * shift};

				b2BodyId body = b2World_CreateBody(m_worldId, &bd);

				b2Body_CreateCircle(body, &sd, &circle);

				if (i > 0)
				{
					jd.bodyIdA = bodies[index - 1];
					jd.bodyIdB = body;
					jd.localAnchorA = {0.0f, -0.5f * shift};
					jd.localAnchorB = {0.0f, 0.5f * shift};
					b2World_CreateRevoluteJoint(m_worldId, &jd);
				}

				if (k > 0)
				{
					jd.bodyIdA = bodies[index - numi];
					jd.bodyIdB = body;
					jd.localAnchorA = {0.5f * shift, 0.0f};
					jd.localAnchorB = {-0.5f * shift, 0.0f};
					b2World_CreateRevoluteJoint(m_worldId, &jd);
				}

				bodies[index++] = body;
			}
		}

		free(bodies);
	}

	static Sample* Create(const Settings& settings)
	{
		return new BenchmarkJointGrid(settings);
	}
};

static int sampleJointGridIndex = RegisterSample("Joints", "Joint Grid", BenchmarkJointGrid::Create);

// A suspension bridge
class Bridge : public Sample
{
  public:
	enum
	{
		e_count = 200
	};

	Bridge(const Settings& settings)
		: Sample(settings)
	{
		b2BodyId groundId = b2_nullBodyId;
		{
			b2BodyDef bd = b2DefaultBodyDef();
			bd.position = {0.0f, -1.0f};
			groundId = b2World_CreateBody(m_worldId, &bd);

			//b2Segment segment = {{-80.0f, 0.0f}, {80.0f, 0.0f}};
			//b2ShapeDef sd = b2DefaultShapeDef();
			//b2Body_CreateSegment(groundId, &sd, &segment);
		}

		{
			b2Polygon box = b2MakeBox(0.5f, 0.125f);

			b2ShapeDef sd = b2DefaultShapeDef();
			sd.density = 20.0f;

			b2RevoluteJointDef jd = b2DefaultRevoluteJointDef();

			b2BodyId prevBodyId = groundId;
			for (int32_t i = 0; i < e_count; ++i)
			{
				b2BodyDef bd = b2DefaultBodyDef();
				bd.type = b2_dynamicBody;
				bd.position = {-34.5f + 1.0f * i, 20.0f};
				b2BodyId bodyId = b2World_CreateBody(m_worldId, &bd);
				b2Body_CreatePolygon(bodyId, &sd, &box);

				b2Vec2 pivot = {-35.0f + 1.0f * i, 20.0f};
				jd.bodyIdA = prevBodyId;
				jd.bodyIdB = bodyId;
				jd.localAnchorA = b2Body_GetLocalPoint(jd.bodyIdA, pivot);
				jd.localAnchorB = b2Body_GetLocalPoint(jd.bodyIdB, pivot);
				b2World_CreateRevoluteJoint(m_worldId, &jd);

				prevBodyId = bodyId;
			}

			b2Vec2 pivot = {-35.0f + 1.0f * e_count, 20.0f};
			jd.bodyIdA = prevBodyId;
			jd.bodyIdB = groundId;
			jd.localAnchorA = b2Body_GetLocalPoint(jd.bodyIdA, pivot);
			jd.localAnchorB = b2Body_GetLocalPoint(jd.bodyIdB, pivot);
			b2World_CreateRevoluteJoint(m_worldId, &jd);
		}

#if 0
		for (int32 i = 0; i < 2; ++i)
		{
			b2Vec2 vertices[3];
			vertices[0].Set(-0.5f, 0.0f);
			vertices[1].Set(0.5f, 0.0f);
			vertices[2].Set(0.0f, 1.5f);

			b2PolygonShape shape;
			shape.Set(vertices, 3);

			b2FixtureDef fd;
			fd.shape = &shape;
			fd.density = 1.0f;

			b2BodyDef bd;
			bd.type = b2_dynamicBody;
			bd.position.Set(-8.0f + 8.0f * i, 12.0f);
			b2Body* body = m_world->CreateBody(&bd);
			body->CreateFixture(&fd);
		}

		for (int32 i = 0; i < 3; ++i)
		{
			b2CircleShape shape;
			shape.m_radius = 0.5f;

			b2FixtureDef fd;
			fd.shape = &shape;
			fd.density = 1.0f;

			b2BodyDef bd;
			bd.type = b2_dynamicBody;
			bd.position.Set(-6.0f + 6.0f * i, 10.0f);
			b2Body* body = m_world->CreateBody(&bd);
			body->CreateFixture(&fd);
		}
#endif
	}

	static Sample* Create(const Settings& settings)
	{
		return new Bridge(settings);
	}
};

static int sampleBridgeIndex = RegisterSample("Joints", "Bridge", Bridge::Create);
