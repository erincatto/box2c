// SPDX-FileCopyrightText: 2022 Erin Catto
// SPDX-License-Identifier: MIT

#include "box2d/box2d.h"
#include "box2d/geometry.h"
#include "sample.h"

#include <GLFW/glfw3.h>
#include <imgui.h>

BOX2D_API int32_t b2_awakeContactCount;

BOX2D_API int b2_collideMinRange;
BOX2D_API int b2_islandMinRange;

class BenchmarkPyramid : public Sample
{
  public:

	BenchmarkPyramid(const Settings& settings) : Sample(settings)
	{
		m_extent = 0.5f;
		m_round = 0.0f;
		m_baseCount = 60;
		m_rowCount = g_sampleDebug ? 1 : 16;
		m_columnCount = g_sampleDebug ? 1 : 16;
		m_groundId = b2_nullBodyId;
		m_bodyIds = nullptr;
		m_bodyCount = 0;
		m_bodyIndex = 0;

		m_collideRange = 169;
		m_islandRange = 1;

		m_bestCollideRange = 1;
		m_minCollide = FLT_MAX;

		m_bestIslandRange = 1;
		m_minIsland = FLT_MAX;

		CreateScene();
	}

	~BenchmarkPyramid() override
	{
		free(m_bodyIds);
	}

	void CreateStack(float centerX, float baseY)
	{
		b2BodyDef bodyDef = b2DefaultBodyDef();
		bodyDef.type = b2_dynamicBody;

		b2ShapeDef shapeDef = b2DefaultShapeDef();
		shapeDef.density = 1.0f;

		float h = m_extent - m_round;
		b2Polygon cuboid = b2MakeRoundedBox(h, h, m_round);

		float shift = 1.0f * h;

		for (int32_t i = 0; i < m_baseCount; ++i)
		{
			float y = (2.0f * i + 1.0f) * shift + baseY;

			for (int32_t j = i; j < m_baseCount; ++j)
			{
				float x = (i + 1.0f) * shift + 2.0f * (j - i) * shift + centerX - 0.5f;

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

		m_bodyCount = m_rowCount * m_columnCount * m_baseCount * (m_baseCount + 1) / 2;
		m_bodyIds = (b2BodyId*)malloc(m_bodyCount * sizeof(b2BodyId));
		m_bodyIndex = 0;

		b2BodyDef bodyDef = b2DefaultBodyDef();
		m_groundId = b2World_CreateBody(m_worldId, &bodyDef);

		float groundDeltaY = 2.0f * m_extent * (m_baseCount + 1.0f);
		float groundWidth = 2.0f * m_extent * m_columnCount * (m_baseCount + 1.0f);
		b2ShapeDef shapeDef = b2DefaultShapeDef();

		float groundY = 0.0f;

		for (int32_t i = 0; i < m_rowCount; ++i)
		{
			//b2Segment segment = {{-0.5f * groundWidth, groundY}, {0.5f * groundWidth, groundY}};
			b2Segment segment = {{-0.5f * 2.0f * groundWidth, groundY}, {0.5f * 2.0f * groundWidth, groundY}};
			b2Body_CreateSegment(m_groundId, &shapeDef, &segment);
			groundY += groundDeltaY;
		}

		float baseWidth = 2.0f * m_extent * m_baseCount;
		float baseY = 0.0f;

		for (int32_t i = 0; i < m_rowCount; ++i)
		{
			for (int32_t j = 0; j < m_columnCount; ++j)
			{
				float centerX = -0.5f * groundWidth + j * (baseWidth + 2.0f * m_extent) + m_extent;
				CreateStack(centerX, baseY);
			}

			baseY += groundDeltaY;
		}

		assert(m_bodyIndex == m_bodyCount);
	}

	void UpdateUI() override
	{
		ImGui::SetNextWindowPos(ImVec2(10.0f, 300.0f), ImGuiCond_Once);
		ImGui::SetNextWindowSize(ImVec2(240.0f, 230.0f));
		ImGui::Begin("Stacks", nullptr, ImGuiWindowFlags_NoResize);

		bool changed = false;
		changed = changed || ImGui::SliderInt("Row Count", &m_rowCount, 1, 32);
		changed = changed || ImGui::SliderInt("Column Count", &m_columnCount, 1, 32);
		changed = changed || ImGui::SliderInt("Base Count", &m_baseCount, 1, 30);

		changed = changed || ImGui::SliderFloat("Round", &m_round, 0.0f, 0.4f, "%.1f");
		changed = changed || ImGui::Button("Reset Scene");

		ImGui::SliderInt("Collide Min", &b2_collideMinRange, 1, 200);
		ImGui::SliderInt("Island Min", &b2_islandMinRange, 1, 10);

		if (changed)
		{
			CreateScene();
		}

		ImGui::End();
	}

	void Step(Settings& settings) override
	{
		//b2_collideMinRange = m_collideRange;
		//b2_islandMinRange = m_islandRange;

		Sample::Step(settings);

		b2Profile profile = b2World_GetProfile(m_worldId);

		if (m_stepCount > 100000000)
		{
			if (profile.collide < m_minCollide)
			{
				m_minCollide = profile.collide;
				m_bestCollideRange = m_collideRange;
			}

			if (profile.solveIslands < m_minIsland)
			{
				m_minIsland = profile.solveIslands;
				m_bestIslandRange = m_islandRange;
			}

			g_draw.DrawString(5, m_textLine, "collide range (best) = %d (%d)", m_collideRange, m_bestCollideRange);
			m_textLine += m_textIncrement;

			g_draw.DrawString(5, m_textLine, "island range (best) = %d (%d)", m_islandRange, m_bestIslandRange);
			m_textLine += m_textIncrement;

			//m_collideRange += 1;
			//if (m_collideRange > 300)
			//{
			//	m_collideRange = 32;
			//}

			//m_islandRange += 1;
			//if (m_islandRange > 4)
			//{
			//	m_islandRange = 1;
			//}
		}
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
	int32_t m_rowCount;
	int32_t m_columnCount;
	float m_round;
	float m_extent;

	int m_collideRange;
	int m_islandRange;

	int32_t m_bestCollideRange;
	float m_minCollide;

	int32_t m_bestIslandRange;
	float m_minIsland;
};

static int sampleIndex = RegisterSample("Benchmark", "Pyramid", BenchmarkPyramid::Create);
