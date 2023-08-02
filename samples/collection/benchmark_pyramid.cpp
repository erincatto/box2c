// SPDX-FileCopyrightText: 2022 Erin Catto
// SPDX-License-Identifier: MIT

#include "box2d/box2d.h"
#include "box2d/geometry.h"
#include "sample.h"

#include <GLFW/glfw3.h>
#include <imgui.h>

BOX2D_API int32_t b2_awakeContactCount;

class BenchmarkPyramid : public Sample
{
  public:
	enum
	{
		e_maxBaseCount = 100,
		e_maxBodyCount = e_maxBaseCount * (e_maxBaseCount + 1) / 2
	};

	BenchmarkPyramid(const Settings& settings) : Sample(settings)
	{
		m_extent = 0.5f;
		m_round = 0.0f;
		m_baseCount = 10;
		m_stackCount = g_sampleDebug ? 4 : 182;
		m_groundId = b2_nullBodyId;
		m_bodyIds = nullptr;
		m_bodyCount = 0;
		m_bodyIndex = 0;

		CreateScene();
	}

	~BenchmarkPyramid()
	{
		free(m_bodyIds);
	}

	void CreateStack(float centerX)
	{
		b2BodyDef bodyDef = b2DefaultBodyDef();
		bodyDef.type = b2_dynamicBody;

		b2ShapeDef shapeDef = b2DefaultShapeDef();
		shapeDef.density = 1.0f;

		float h = m_extent - m_round;
		b2Polygon cuboid = b2MakeRoundedBox(h, h, m_round);

		for (int32_t i = 0; i < m_baseCount; ++i)
		{
			float y = (2.0f * i  + 1.0f) * m_extent;

			for (int32_t j = i; j < m_baseCount; ++j)
			{
				float x = (i + 1.0f) * m_extent + 2.0f * (j - i) * m_extent + centerX;
				bodyDef.position = {x, y};

				assert(m_bodyIndex < m_bodyCount);
				m_bodyIds[m_bodyIndex] = b2World_CreateBody(m_worldId, &bodyDef);
				b2Body_CreatePolygon(m_bodyIds[m_bodyIndex], &shapeDef, &cuboid);

				m_bodyIndex += 1;
			}
		}
	}

	void CreateScene()
	{
		if (B2_NON_NULL(m_groundId))
		{
			b2World_DestroyBody(m_groundId);
		}

		for (int32_t i = 0; i < m_bodyCount; ++i)
		{
			b2World_DestroyBody(m_bodyIds[i]);
		}

		free(m_bodyIds);

		m_bodyCount = m_stackCount * m_baseCount * (m_baseCount + 1) / 2;
		m_bodyIds = (b2BodyId*)malloc(m_bodyCount * sizeof(b2BodyId));
		m_bodyIndex = 0;

		b2BodyDef bodyDef = b2DefaultBodyDef();
		b2BodyId groundId = b2World_CreateBody(m_worldId, &bodyDef);

		float groundWidth = 2.0f * m_extent * m_stackCount * (m_baseCount + 1.0f);
		b2Segment segment = {{-0.5f * groundWidth, 0.0f}, {0.5f * groundWidth, 0.0f}};

		b2ShapeDef shapeDef = b2DefaultShapeDef();
		b2Body_CreateSegment(groundId, &shapeDef, &segment);

		float baseWidth = 2.0f * m_extent * m_baseCount;

		for (int32_t i = 0; i < m_stackCount; ++i)
		{
			float centerX = -0.5f * groundWidth + i * (baseWidth + 2.0f * m_extent) + m_extent;
			CreateStack(centerX);
		}

		assert(m_bodyIndex == m_bodyCount);
	}

	void UpdateUI() override
	{
		ImGui::SetNextWindowPos(ImVec2(10.0f, 300.0f), ImGuiCond_Once);
		ImGui::SetNextWindowSize(ImVec2(240.0f, 230.0f));
		ImGui::Begin("Stacks", nullptr, ImGuiWindowFlags_NoResize);

		bool changed = false;
		changed = changed || ImGui::SliderInt("Stack Count", &m_stackCount, 1, 200);
		changed = changed || ImGui::SliderInt("Base Count", &m_baseCount, 1, 100);

		changed = changed || ImGui::SliderFloat("Round", &m_round, 0.0f, 0.4f, "%.1f");
		changed = changed || ImGui::Button("Reset Scene");

		if (changed)
		{
			CreateScene();
		}

		ImGui::End();
	}

	void Step(Settings& settings) override
	{
		Sample::Step(settings);

		g_draw.DrawString(5, m_textLine, "awake contacts = %d", b2_awakeContactCount);
		m_textLine += m_textIncrement;
	}

	static Sample* Create(const Settings& settings)
	{
		return new BenchmarkPyramid(settings);
	}

	b2BodyId m_groundId;
	b2BodyId* m_bodyIds;
	int32_t m_bodyCount;
	int32_t m_bodyIndex;
	int32_t m_baseCount;
	int32_t m_stackCount;
	float m_round;
	float m_extent;
};

static int sampleIndex = RegisterSample("Benchmark", "Pyramid", BenchmarkPyramid::Create);
