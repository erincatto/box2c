// SPDX-FileCopyrightText: 2022 Erin Catto
// SPDX-License-Identifier: MIT

#include "donut.h"
#include "draw.h"
#include "human.h"
#include "sample.h"
#include "settings.h"

#include "box2d/box2d.h"
#include "box2d/color.h"
#include "box2d/geometry.h"
#include "box2d/hull.h"
#include "box2d/math_functions.h"

#include <GLFW/glfw3.h>
#include <imgui.h>

class LargeWorld : public Sample
{
public:
	explicit LargeWorld(Settings& settings)
		: Sample(settings)
	{
		m_period = 40.0f;
		float omega = 2.0 * b2_pi / m_period;
		m_cycleCount = g_sampleDebug ? 10 : 600;
		m_gridSize = 1.0f;
		m_gridCount = (int)(m_cycleCount * m_period / m_gridSize);

		float xStart = -0.5f * (m_cycleCount * m_period);

		m_viewPosition = {xStart, 15.0f};

		if (settings.restart == false)
		{
			g_camera.m_center = m_viewPosition;
			g_camera.m_zoom = 1.0f;
			settings.drawJoints = false;
		}

		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			b2ShapeDef shapeDef = b2DefaultShapeDef();

			// Setting this to false significantly reduces the cost of creating
			// static bodies and shapes.
			shapeDef.forceContactCreation = false;

			float height = 4.0f;
			float xBody = xStart;
			float xShape = xStart;

			b2BodyId groundId;

			for (int i = 0; i < m_gridCount; ++i)
			{
				// Create a new body regularly so that shapes are not too far from the body origin.
				// Most algorithms in Box2D work in local coordinates, but contact points are computed
				// relative to the body origin.
				// This makes a noticeable improvement in stability far from the origin.
				if (i % 10 == 0)
				{
					bodyDef.position.x = xBody;
					groundId = b2CreateBody(m_worldId, &bodyDef);
					xShape = 0.0f;
				}

				float y = 0.0f;

				int ycount = (int)roundf(height * cosf(omega * xBody)) + 12;

				for (int j = 0; j < ycount; ++j)
				{
					b2Polygon square = b2MakeOffsetBox(0.4f * m_gridSize, 0.4f * m_gridSize, {xShape, y}, 0.0f);
					square.radius = 0.1f;
					b2CreatePolygonShape(groundId, &shapeDef, &square);

					y += m_gridSize;
				}

				xBody += m_gridSize;
				xShape += m_gridSize;
			}
		}

		int humanIndex = 0;
		int donutIndex = 0;
		for (int cycleIndex = 0; cycleIndex < m_cycleCount; ++cycleIndex)
		{
			float xbase = (0.5f + cycleIndex) * m_period + xStart;

			int remainder = cycleIndex % 3;
			if (remainder == 0)
			{
				b2BodyDef bodyDef = b2DefaultBodyDef();
				bodyDef.type = b2_dynamicBody;
				bodyDef.position = {xbase - 3.0f, 10.0f};

				b2ShapeDef shapeDef = b2DefaultShapeDef();
				b2Polygon box = b2MakeBox(0.3f, 0.2f);

				for (int i = 0; i < 10; ++i)
				{
					bodyDef.position.y = 10.0f;
					for (int j = 0; j < 5; ++j)
					{
						b2BodyId bodyId = b2CreateBody(m_worldId, &bodyDef);
						b2CreatePolygonShape(bodyId, &shapeDef, &box);
						bodyDef.position.y += 0.5f;
					}
					bodyDef.position.x += 0.6f;
				}
			}
			else if (remainder == 1)
			{
				b2Vec2 position = {xbase - 1.0f, 10.0f};
				for (int i = 0; i < 3; ++i)
				{
					// Abusing this class a bit since it doesn't allocate memory
					Human human;
					human.Spawn(m_worldId, position, 2.0f, humanIndex + 1, NULL);
					humanIndex += 1;
					position.x += 0.75f;
				}
			}
			else
			{
				b2Vec2 position = {xbase - 6.0f, 12.0f};

				for (int i = 0; i < 5; ++i)
				{
					Donut donut;
					donut.Spawn(m_worldId, position, donutIndex + 1, NULL);
					donutIndex += 1;
					position.x += 3.0f;
				}
			}
		}

		m_cycleIndex = 0;
		m_speed = 0.0f;
		m_explosionPosition = {(0.5f + m_cycleIndex) * m_period + xStart, 7.0f};
		m_explode = true;
	}

	void UpdateUI() override
	{
		ImGui::SetNextWindowPos(ImVec2(10.0f, 900.0f));
		ImGui::SetNextWindowSize(ImVec2(220.0f, 140.0f));
		ImGui::Begin("Large World", nullptr, ImGuiWindowFlags_NoResize);

		ImGui::SliderFloat("speed", &m_speed, -400.0f, 400.0f, "%.0f");
		if (ImGui::Button("stop"))
		{
			m_speed = 0.0f;
		}

		ImGui::Checkbox("explode", &m_explode);

		ImGui::Text("world size = %g kilometers", m_gridSize * m_gridCount / 1000.0f);
		ImGui::End();
	}

	void Step(Settings& settings) override
	{
		float span = 0.5f * (m_period * m_cycleCount);
		float timeStep = settings.hertz > 0.0f ? 1.0f / settings.hertz : 0.0f;

		if (settings.pause)
		{
			timeStep = 0.0f;
		}

		m_viewPosition.x += timeStep * m_speed;
		m_viewPosition.x = b2ClampFloat(m_viewPosition.x, -span, span);

		if (m_speed != 0.0f)
		{
			g_camera.m_center = m_viewPosition;
		}

		float radius = 2.0f;
		if ((m_stepCount & 0x1) == 0x1 && m_explode)
		{
			m_explosionPosition.x = (0.5f + m_cycleIndex) * m_period - span;
			b2World_Explode(m_worldId, m_explosionPosition, radius, 1.0f);
			m_cycleIndex = (m_cycleIndex + 1) % m_cycleCount;
		}

		if (m_explode)
		{
			g_draw.DrawCircle(m_explosionPosition, radius, b2MakeColor(b2_colorAzure3));
		}

		Sample::Step(settings);
	}

	static Sample* Create(Settings& settings)
	{
		return new LargeWorld(settings);
	}

	b2Vec2 m_viewPosition;
	float m_period;
	int m_cycleCount;
	int m_cycleIndex;
	float m_gridCount;
	float m_gridSize;
	float m_speed;

	b2Vec2 m_explosionPosition;
	bool m_explode;
};

static int sampleLargeWorld = RegisterSample("World", "Large World", LargeWorld::Create);
