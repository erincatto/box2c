// SPDX-FileCopyrightText: 2022 Erin Catto
// SPDX-License-Identifier: MIT

#include "box2d/box2d.h"
#include "box2d/geometry.h"
#include "sample.h"

#include <GLFW/glfw3.h>
#include <imgui.h>

class BenchmarkBarrel : public Sample
{
public:
	enum ShapeType
	{
		e_circleShape = 0,
		e_boxShape
	};

	enum
	{
		e_maxColumns = 26,
		e_maxRows = 130,
	};

	BenchmarkBarrel()
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

		for (int32_t i = 0; i < e_maxRows * e_maxColumns; ++i)
		{
			m_bodies[i] = b2_nullBodyId;
		}

		m_shapeType = e_boxShape;

		CreateScene();
	}

	void CreateScene()
	{
		for (int32_t i = 0; i < e_maxRows * e_maxColumns; ++i)
		{
			if (B2_NON_NULL(m_bodies[i]))
			{
				b2World_DestroyBody(m_bodies[i]);
				m_bodies[i] = b2_nullBodyId;
			}
		}

		m_columnCount = g_sampleDebug ? 4 : e_maxColumns;
		float rad = 0.5f;

		float shift = rad * 2.0f;
		float centerx = shift * m_columnCount / 2.0f;
		float centery = shift / 2.0f;

		b2BodyDef bd = b2DefaultBodyDef();
		bd.type = b2_dynamicBody;

		b2ShapeDef sd = b2DefaultShapeDef();
		sd.density = 1.0f;
		sd.friction = 0.5f;

		b2Polygon cuboid = b2MakeBox(0.5f, 0.5f);

		b2Circle circle = {0};
		circle.radius = rad;

		m_rowCount = g_sampleDebug ? 8 : e_maxRows;

		int32_t index = 0;

		for (int32_t i = 0; i < m_columnCount; ++i)
		{
			float x = i * shift - centerx;

			for (int32_t j = 0; j < m_rowCount; ++j)
			{
				float y = j * shift + centery + 2.0f;

				bd.position = {x, y};

				m_bodies[index] = b2World_CreateBody(m_worldId, &bd);

				if (m_shapeType == e_boxShape)
				{
					b2Body_CreatePolygon(m_bodies[index], &sd, &cuboid);
				}
				else
				{
					b2Body_CreateCircle(m_bodies[index], &sd, &circle);
				}

				index += 1;
			}
		}
	}

	void UpdateUI() override
	{
		ImGui::SetNextWindowPos(ImVec2(10.0f, 300.0f), ImGuiCond_Once);
		ImGui::SetNextWindowSize(ImVec2(240.0f, 230.0f));
		ImGui::Begin("Stacks", nullptr, ImGuiWindowFlags_NoResize);

		bool changed = false;
		const char* shapeTypes[] = {"Circle", "Box"};

		int shapeType = int(m_shapeType);
		changed = changed || ImGui::Combo("Shape", &shapeType, shapeTypes, IM_ARRAYSIZE(shapeTypes));
		m_shapeType = ShapeType(shapeType);

		changed = changed || ImGui::Button("Reset Scene");

		if (changed)
		{
			CreateScene();
		}

		ImGui::End();
	}

	static Sample* Create()
	{
		return new BenchmarkBarrel;
	}

	b2BodyId m_bodies[e_maxRows * e_maxColumns];
	int32_t m_columnCount;
	int32_t m_rowCount;

	ShapeType m_shapeType;
};

static int sampleIndex = RegisterSample("Benchmark", "Barrel", BenchmarkBarrel::Create);
