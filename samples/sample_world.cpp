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
		// One cycle per 50 meters
		float period = 50.0f;
		float omega = 2.0 * b2_pi / period;
		float cycleCount = g_sampleDebug ? 10 : 300;

		m_gridSize = 1.0f;
		m_gridCount = (int)(cycleCount * period / m_gridSize);

		m_viewPosition = {m_gridCount * m_gridSize, 30.0f};

		if (settings.restart == false)
		{
			g_camera.m_center = m_viewPosition;
			g_camera.m_zoom = 2.0f;
			settings.drawJoints = false;
		}

		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			b2BodyId groundId = b2CreateBody(m_worldId, &bodyDef);

			b2ShapeDef shapeDef = b2DefaultShapeDef();

			float height = 5.0f;
			float x = 0.0f;

			for (int i = 0; i < m_gridCount; ++i)
			{
				float y = 0.0f;

				int ycount = (int)(height * cosf(omega * x) + 0.5f) + 20;

				for (int j = 0; j < ycount; ++j)
				{
					b2Polygon square = b2MakeOffsetBox(0.5f * m_gridSize, 0.5f * m_gridSize, {x, y}, 0.0f);
					b2CreatePolygonShape(groundId, &shapeDef, &square);

					y += m_gridSize;
				}

				x += m_gridSize;
			}
		}

		for (int cycleIndex = 0; cycleIndex < cycleCount; ++cycleIndex)
		{
			float xbase = (0.5f + cycleIndex) * period;

			b2BodyDef bodyDef = b2DefaultBodyDef();
			bodyDef.type = b2_dynamicBody;
			bodyDef.position = {xbase - 4.0f, 20.0f};

			b2ShapeDef shapeDef = b2DefaultShapeDef();
			b2Polygon box = b2MakeSquare(0.5f);

			for (int i = 0; i < 15; ++i)
			{
				b2BodyId bodyId = b2CreateBody(m_worldId, &bodyDef);
				b2CreatePolygonShape(bodyId, &shapeDef, &box);
				bodyDef.position.x += 0.1f;
				bodyDef.position.y += 1.0f;
			}
		}

		m_speed = -20.0f;
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

		ImGui::Text("world size = %g kilometers", m_gridSize * m_gridCount / 1000.0f);
		ImGui::End();
	}

	void Step(Settings& settings) override
	{
		float xLast = m_gridCount * m_gridSize;
		float timeStep = settings.hertz > 0.0f ? 1.0f / settings.hertz : 0.0f;

		if (settings.pause)
		{
			timeStep = 0.0f;
		}

		m_viewPosition.x += timeStep * m_speed;
		m_viewPosition.x = b2ClampFloat(m_viewPosition.x, 0.0f, xLast);

		g_camera.m_center = m_viewPosition;

		Sample::Step(settings);
	}

	static Sample* Create(Settings& settings)
	{
		return new LargeWorld(settings);
	}

	b2Vec2 m_viewPosition;
	float m_gridCount;
	float m_gridSize;
	float m_speed;
};

static int sampleLargeWorld = RegisterSample("World", "Large World", LargeWorld::Create);
