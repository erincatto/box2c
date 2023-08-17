// SPDX-FileCopyrightText: 2022 Erin Catto
// SPDX-License-Identifier: MIT

#include "sample.h"

#include "box2d/box2d.h"
#include "box2d/geometry.h"

#include <GLFW/glfw3.h>
#include <imgui.h>

class HighMassRatio : public Sample
{
  public:
	HighMassRatio(const Settings& settings)
		: Sample(settings)
	{
		float extent = 1.0f;

		b2BodyDef bodyDef = b2DefaultBodyDef();
		b2BodyId groundId = b2World_CreateBody(m_worldId, &bodyDef);

		float groundWidth = 66.0f * extent;
		b2ShapeDef shapeDef = b2DefaultShapeDef();
		shapeDef.friction = 0.5f;

		b2Segment segment = {{-0.5f * 2.0f * groundWidth, 0.0f}, {0.5f * 2.0f * groundWidth, 0.0f}};
		b2Body_CreateSegment(groundId, &shapeDef, &segment);

		bodyDef.type = b2_dynamicBody;

		b2Polygon box = b2MakeBox(extent, extent);

		for (int j = 0; j < 1; ++j)
		{
			int count = 2;
			float offset = 2.0f * (count + 1.0f) * extent * j;
			float y = extent;
			while (count > 0)
			{
				for (int i = 0; i < count; ++i)
				{
					float coeff = i - 0.5f * count;

					bodyDef.position = {2.0f * coeff * extent + offset, y};
					b2BodyId bodyId = b2World_CreateBody(m_worldId, &bodyDef);

					shapeDef.density = count == 1 ? (j + 1.0f) * 300.0f : 1.0f;
					b2Body_CreatePolygon(bodyId, &shapeDef, &box);
				}

				--count;
				y += 2.0f * extent;
			}
		}
	}

	static Sample* Create(const Settings& settings)
	{
		return new HighMassRatio(settings);
	}
};

static int sampleIndex = RegisterSample("Behavior", "HighMassRatio", HighMassRatio::Create);
