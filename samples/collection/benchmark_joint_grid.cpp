// SPDX-FileCopyrightText: 2022 Erin Catto
// SPDX-License-Identifier: MIT

#include "box2d/allocate.h"
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
		constexpr int32_t numi = g_sampleDebug ? 10 : 100;
		constexpr int32_t numk = g_sampleDebug ? 10 : 100;
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

static int sampleIndex = RegisterSample("Benchmark", "Joint Grid", BenchmarkJointGrid::Create);
