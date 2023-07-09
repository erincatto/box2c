// SPDX-FileCopyrightText: 2022 Erin Catto
// SPDX-License-Identifier: MIT

#include "box2d/box2d.h"
#include "box2d/geometry.h"
#include "sample.h"

#include <GLFW/glfw3.h>
#include <imgui.h>

class BenchmarkPyramid : public Sample
{
public:

	enum
	{
		e_maxBaseCount = 100,
		e_maxBodyCount = e_maxBaseCount * (e_maxBaseCount + 1) / 2
	};

	BenchmarkPyramid(const Settings& settings)
		: Sample(settings)
	{
		float groundSize = 100.0f;
		m_groundThickness = 1.0f;
		m_round = 0.0f;

		b2BodyDef bd = b2DefaultBodyDef();
		b2BodyId groundId = b2World_CreateBody(m_worldId, &bd);

		b2Polygon box = b2MakeBox(groundSize, m_groundThickness);
		b2ShapeDef sd = b2DefaultShapeDef();
		b2Body_CreatePolygon(groundId, &sd, &box);

		for (int32_t i = 0; i < e_maxBodyCount; ++i)
		{
			m_bodies[i] = b2_nullBodyId;
		}

		m_baseCount = g_sampleDebug ? 80 : 80;
		m_bodyCount = 0;

		CreateScene();
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
		float centery = shift / 2.0f + m_groundThickness;//		+rad * 1.5f;

		b2BodyDef bd = b2DefaultBodyDef();
		bd.type = b2_dynamicBody;

		b2ShapeDef sd = b2DefaultShapeDef();
		sd.density = 1.0f;
		sd.friction = 0.5f;

		//b2Polygon cuboid = b2MakeBox(0.5f, 0.5f);
		//b2Polygon cuboid = b2MakeRoundedBox(0.4f, 0.4f, 0.1f);
		float h = 0.5f - m_round;
		b2Polygon cuboid = b2MakeRoundedBox(h, h, m_round);

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

	void UpdateUI() override
	{
		ImGui::SetNextWindowPos(ImVec2(10.0f, 300.0f), ImGuiCond_Once);
		ImGui::SetNextWindowSize(ImVec2(240.0f, 230.0f));
		ImGui::Begin("Stacks", nullptr, ImGuiWindowFlags_NoResize);

		bool changed = false;
		changed = changed || ImGui::SliderInt("Base Count", &m_baseCount, 1, e_maxBaseCount);

		changed = changed || ImGui::SliderFloat("Round", &m_round, 0.0f, 0.4f, "%.1f");
		changed = changed || ImGui::Button("Reset Scene");

		if (ImGui::Button("Wake Top"))
		{
			b2Body_Wake(m_bodies[m_bodyCount - 1]);
		}

		if (changed)
		{
			CreateScene();
		}

		ImGui::End();
	}

	static Sample* Create(const Settings& settings)
	{
		return new BenchmarkPyramid(settings);
	}

	b2BodyId m_bodies[e_maxBodyCount];
	int32_t m_bodyCount;
	int32_t m_baseCount;
	float m_round;
	float m_groundThickness;
};

static int sampleIndex = RegisterSample("Benchmark", "Pyramid", BenchmarkPyramid::Create);
