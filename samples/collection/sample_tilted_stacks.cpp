// SPDX-FileCopyrightText: 2022 Erin Catto
// SPDX-License-Identifier: MIT

#include "box2d/box2d.h"
#include "box2d/geometry.h"
#include "sample.h"

class TiltedStacks : public Sample
{
public:

	enum
	{
		e_columns = 10,
		e_rows = 10,
	};

	TiltedStacks(const Settings& settings)
		: Sample(settings)
	{
		{
			b2BodyDef bd = b2_defaultBodyDef;
			bd.position = {0.0f, -1.0f};
			b2BodyId groundId = b2CreateBody(m_worldId, &bd);

			b2Polygon box = b2MakeBox(1000.0f, 1.0f);
			b2ShapeDef sd = b2_defaultShapeDef;
			b2CreatePolygonShape(groundId, &sd, &box);
		}

		for (int32_t i = 0; i < e_rows * e_columns; ++i)
		{
			m_bodies[i] = b2_nullBodyId;
		}

		b2Polygon box = b2MakeRoundedBox(0.45f, 0.45f, 0.05f);

		b2ShapeDef sd = b2_defaultShapeDef;
		sd.density = 1.0f;
		sd.friction = 0.3f;

		float offset = 0.2f;
		float dx = 5.0f;
		float xroot = -0.5f * dx * (e_columns - 1.0f);

		for (int32_t j = 0; j < e_columns; ++j)
		{
			float x = xroot + j * dx;

			for (int32_t i = 0; i < e_rows; ++i)
			{
				b2BodyDef bd = b2_defaultBodyDef;
				bd.type = b2_dynamicBody;

				int32_t n = j * e_rows + i;

				bd.position = {x + offset * i, 0.5f + 1.0f * i};
				b2BodyId bodyId = b2CreateBody(m_worldId, &bd);

				m_bodies[n] = bodyId;

				b2CreatePolygonShape(bodyId, &sd, &box);
			}
		}
	}

	static Sample* Create(const Settings& settings)
	{
		return new TiltedStacks(settings);
	}

	b2BodyId m_bodies[e_rows * e_columns];
};

static int sampleIndex = RegisterSample("Stacking", "Tilted Stacks", TiltedStacks::Create);
