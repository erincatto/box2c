// SPDX-FileCopyrightText: 2022 Erin Catto
// SPDX-License-Identifier: MIT

#include "box2d/box2d.h"
#include "box2d/geometry.h"
#include "sample.h"

class BenchmarkBoxes : public Sample
{
public:
	BenchmarkBoxes()
	{
		float groundSize = 25.0f;

		{
			b2BodyDef bd = b2DefaultBodyDef();
			b2BodyId groundId = b2World_CreateBody(m_worldId, &bd);

			b2Polygon box = b2MakeBox(groundSize, 1.2f);
			b2ShapeDef sd = b2DefaultShapeDef();
			b2Body_CreatePolygon(groundId, &sd, &box);

			bd.angle = 0.5f * b2_pi;
			bd.position = {groundSize, 2.0f * groundSize};
			groundId = b2World_CreateBody(m_worldId, &bd);

			box = b2MakeBox(2.0f * groundSize, 1.2f);
			b2Body_CreatePolygon(groundId, &sd, &box);

			bd.position = {-groundSize, 2.0f * groundSize};
			groundId = b2World_CreateBody(m_worldId, &bd);
			b2Body_CreatePolygon(groundId, &sd, &box);
		}

		int32_t num = g_sampleDebug ? 2 : 26;
		float rad = 0.5f;

		float shift = rad * 2.0f;
		float centerx = shift * num / 2.0f;
		float centery = shift / 2.0f;

		b2BodyDef bd = b2DefaultBodyDef();
		bd.type = b2_dynamicBody;

		b2ShapeDef sd = b2DefaultShapeDef();
		sd.density = 1.0f;
		sd.friction = 0.5f;

		b2Polygon cuboid = b2MakeBox(0.5f, 0.5f);
		//b2Circle circle = {0};
		//circle.radius = rad;

		int32_t numj = g_sampleDebug ? 2 : 5 * num;

		for (int32_t i = 0; i < num; ++i)
		{
			float x = i * shift - centerx;

			for (int32_t j = 0; j < numj; ++j)
			{
				float y = j * shift + centery + 2.0f;

				bd.position = {x, y};

				b2BodyId rigidBody = b2World_CreateBody(m_worldId, &bd);
				b2Body_CreatePolygon(rigidBody, &sd, &cuboid);
				//b2Body_CreateCircle(rigidBody, &sd, &circle);
			}
		}
	}

	static Sample* Create()
	{
		return new BenchmarkBoxes;
	}
};

static int sampleIndex = RegisterSample("Benchmark", "Boxes", BenchmarkBoxes::Create);
