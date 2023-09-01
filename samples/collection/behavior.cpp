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

#if 0
		//b2Circle circle = {{0.0f, 0.0f}, extent};
		int count = 2;
		for (int i = 0; i < count; ++i)
		{
			bodyDef.position = {0.0f, (2.0f * i + 1.0f) * 1.0f * extent};
			b2BodyId bodyId = b2World_CreateBody(m_worldId, &bodyDef);

			shapeDef.density = i == count - 1 ? 300.0f : 1.0f;
			//b2Body_CreateCircle(bodyId, &shapeDef, &circle);
			b2Body_CreatePolygon(bodyId, &shapeDef, &box);
		}
#else
		for (int j = 0; j < 3; ++j)
		{
			int count = 10;
			float offset = -20.0f * extent + 2.0f * (count + 1.0f) * extent * j;
			float y = extent;
			while (count > 0)
			{
				for (int i = 0; i < count; ++i)
				{
					float coeff = i - 0.5f * count;

					float yy = count == 1 ? y + 2.0f : y;
					bodyDef.position = {2.0f * coeff * extent + offset, yy};
					b2BodyId bodyId = b2World_CreateBody(m_worldId, &bodyDef);

					shapeDef.density = count == 1 ? (j + 1.0f) * 100.0f : 1.0f;
					b2Body_CreatePolygon(bodyId, &shapeDef, &box);
				}

				--count;
				y += 2.0f * extent;
			}
		}
#endif
	}

	static Sample* Create(const Settings& settings)
	{
		return new HighMassRatio(settings);
	}
};

static int sampleIndex1 = RegisterSample("Behavior", "HighMassRatio", HighMassRatio::Create);


class Friction : public Sample
{
  public:
	Friction(const Settings& settings)
		: Sample(settings)
	{

		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			b2BodyId groundId = b2World_CreateBody(m_worldId, &bodyDef);

			b2ShapeDef shapeDef = b2DefaultShapeDef();
			shapeDef.friction = 0.2f;

			b2Segment segment = {{-40.0f, 0.0f}, {40.0f, 0.0f}};
			b2Body_CreateSegment(groundId, &shapeDef, &segment);

			b2Polygon box = b2MakeOffsetBox(13.0f, 0.25f, {-4.0f, 22.0f}, -0.25f);
			b2Body_CreatePolygon(groundId, &shapeDef, &box);

			box = b2MakeOffsetBox(0.25f, 1.0f, {10.5f, 19.0f}, 0.0f);
			b2Body_CreatePolygon(groundId, &shapeDef, &box);

			box = b2MakeOffsetBox(13.0f, 0.25f, {4.0f, 14.0f}, 0.25f);
			b2Body_CreatePolygon(groundId, &shapeDef, &box);

			box = b2MakeOffsetBox(0.25f, 1.0f, {-10.5f, 11.0f}, 0.0f);
			b2Body_CreatePolygon(groundId, &shapeDef, &box);

			box = b2MakeOffsetBox(13.0f, 0.25f, {-4.0f, 6.0f}, -0.25f);
			b2Body_CreatePolygon(groundId, &shapeDef, &box);
		}

		{
			b2Polygon box = b2MakeBox(0.5f, 0.5f);

			b2ShapeDef shapeDef = b2DefaultShapeDef();
			shapeDef.density = 25.0f;

			float friction[5] = {0.75f, 0.5f, 0.35f, 0.1f, 0.0f};

			for (int i = 0; i < 5; ++i)
			{
				b2BodyDef bodyDef = b2DefaultBodyDef();
				bodyDef.type = b2_dynamicBody;
				bodyDef.position = {-15.0f + 4.0f * i, 28.0f};
				b2BodyId bodyId = b2World_CreateBody(m_worldId, &bodyDef);

				shapeDef.friction = friction[i];
				b2Body_CreatePolygon(bodyId, &shapeDef, &box);
			}
		}
	}

	static Sample* Create(const Settings& settings)
	{
		return new Friction(settings);
	}
};

static int sampleIndex2 = RegisterSample("Behavior", "Friction", Friction::Create);
