// SPDX-FileCopyrightText: 2022 Erin Catto
// SPDX-License-Identifier: MIT

#include "box2d/box2d.h"
#include "box2d/geometry.h"
#include "sample.h"

#include <GLFW/glfw3.h>
#include <imgui.h>

BOX2D_API bool g_velocityBlockSolve;

class VerticalStack : public Sample
{
public:

	enum
	{
		e_maxColumns = 500,
		e_maxRows = 100,
		e_maxBullets = 20
	};

	enum ShapeType
	{
		e_circleShape = 0,
		e_boxShape
	};

	VerticalStack()
	{
		{
			b2BodyDef bd = b2DefaultBodyDef();
			bd.position = {0.0f, -1.0f};
			b2BodyId groundId = b2World_CreateBody(m_worldId, &bd);

			b2Polygon box = b2MakeBox(1000.0f, 1.0f);
			b2ShapeDef sd = b2DefaultShapeDef();
			b2Body_CreatePolygon(groundId, &sd, &box);

			//b2EdgeShape shape;
			//shape.SetTwoSided(b2Vec2(-40.0f, 0.0f), b2Vec2(40.0f, 0.0f));
			//ground->CreateFixture(&shape, 0.0f);

			//shape.SetTwoSided(b2Vec2(20.0f, 0.0f), b2Vec2(20.0f, 20.0f));
			//ground->CreateFixture(&shape, 0.0f);
		}

		for (int32_t i = 0; i < e_maxRows * e_maxColumns; ++i)
		{
			m_bodies[i] = b2_nullBodyId;
		}

		for (int32_t i = 0; i < e_maxBullets; ++i)
		{
			m_bullets[i] = b2_nullBodyId;
		}

		m_shapeType = e_boxShape;
		m_rowCount = g_sampleDebug ? 4 : e_maxRows;
		m_columnCount = g_sampleDebug ? 1 : e_maxColumns;
		m_bulletCount = 1;
		m_bulletType = e_circleShape;

		CreateStacks();
	}

	void CreateStacks()
	{
		for (int32_t i = 0; i < e_maxRows * e_maxColumns; ++i)
		{
			if (B2_NON_NULL(m_bodies[i]))
			{
				b2World_DestroyBody(m_bodies[i]);
				m_bodies[i] = b2_nullBodyId;
			}
		}

		b2Circle circle = {0};
		circle.radius = 0.5f;

		//b2Polygon box = b2MakeBox(0.5f, 0.5f);
		b2Polygon box = b2MakeRoundedBox(0.45f, 0.45f, 0.05f);

		b2ShapeDef sd = b2DefaultShapeDef();
		sd.density = 1.0f;
		sd.friction = 0.3f;

		float offset;

		if (m_shapeType == e_circleShape)
		{
			offset = 0.0f;
		}
		else
		{
			offset = 0.0f; // 0.01f;
		}

		float dx = 3.0f;
		float xroot = -0.5f * dx * (m_columnCount - 1.0f);

		for (int32_t j = 0; j < m_columnCount; ++j)
		{
			float x = xroot + j * dx;

			for (int32_t i = 0; i < m_rowCount; ++i)
			{
				b2BodyDef bd = b2DefaultBodyDef();
				bd.type = b2_dynamicBody;

				int32_t n = j * m_rowCount + i;

				float shift = (i % 2 == 0 ? -offset : offset);
				bd.position = {x + shift, 0.505f + 1.01f * i};
				b2BodyId bodyId = b2World_CreateBody(m_worldId, &bd);

				m_bodies[n] = bodyId;

				if (m_shapeType == e_circleShape)
				{
					b2Body_CreateCircle(bodyId, &sd, &circle);
				}
				else
				{
					b2Body_CreatePolygon(bodyId, &sd, &box);
				}
			}
		}
	}

	void DestroyBullets()
	{
		for (int32_t i = 0; i < e_maxBullets; ++i)
		{
			b2BodyId bullet = m_bullets[i];

			if (B2_NON_NULL(bullet))
			{
				b2World_DestroyBody(bullet);
				m_bullets[i] = b2_nullBodyId;
			}
		}
	}

	void FireBullets()
	{
		b2Circle circle;
		circle.radius = 0.25f;

		b2Polygon box = b2MakeBox(0.25f, 0.25f);

		b2ShapeDef sd = b2DefaultShapeDef();
		sd.density = 2.0f;
		sd.friction = 0.6f;

		for (int32_t i = 0; i < m_bulletCount; ++i)
		{
			b2BodyDef bd = b2DefaultBodyDef();
			bd.type = b2_dynamicBody;
			bd.position = {-25.0f - i, 5.0f};
			bd.linearVelocity = {400.0f, 0.0f};

			b2BodyId bullet = b2World_CreateBody(m_worldId, &bd);
			b2Body_CreatePolygon(bullet, &sd, &box);

			assert(B2_IS_NULL(m_bullets[i]));
			m_bullets[i] = bullet;
		}
	}

	void UpdateUI() override
	{
		ImGui::SetNextWindowPos(ImVec2(10.0f, 300.0f), ImGuiCond_Once);
		ImGui::SetNextWindowSize(ImVec2(240.0f, 230.0f));
		ImGui::Begin("Stacks", nullptr, ImGuiWindowFlags_NoResize);

		bool changed = false;
		const char* shapeTypes[] = { "Circle", "Box" };

		int shapeType = int(m_shapeType);
		changed = changed || ImGui::Combo("Shape", &shapeType, shapeTypes, IM_ARRAYSIZE(shapeTypes));
		m_shapeType = ShapeType(shapeType);

		changed = changed || ImGui::SliderInt("Rows", &m_rowCount, 1, e_maxRows);
		changed = changed || ImGui::SliderInt("Columns", &m_columnCount, 1, e_maxColumns);

		ImGui::SliderInt("Bullets", &m_bulletCount, 1, e_maxBullets);

		int bulletType = int(m_bulletType);
		ImGui::Combo("Bullet Shape", &bulletType, shapeTypes, IM_ARRAYSIZE(shapeTypes));
		m_bulletType = ShapeType(bulletType);

		if (ImGui::Button("Fire Bullets"))
		{
			DestroyBullets();
			FireBullets();
		}

		ImGui::Checkbox("Block Solve", &g_velocityBlockSolve);

		changed = changed || ImGui::Button("Reset Stack");

		if (changed)
		{
			DestroyBullets();
			CreateStacks();
		}

		ImGui::End();
	}

	static Sample* Create()
	{
		return new VerticalStack;
	}

	b2BodyId m_bullets[e_maxBullets];
	b2BodyId m_bodies[e_maxRows * e_maxColumns];
	int32_t m_columnCount;
	int32_t m_rowCount;
	int32_t m_bulletCount;
	ShapeType m_shapeType;
	ShapeType m_bulletType;
};

static int sampleIndex = RegisterSample("Stacking", "Vertical Stack", VerticalStack::Create);
