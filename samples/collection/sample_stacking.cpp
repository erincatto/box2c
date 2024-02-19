// SPDX-FileCopyrightText: 2022 Erin Catto
// SPDX-License-Identifier: MIT

#include "box2d/box2d.h"
#include "box2d/geometry.h"
#include "box2d/math_cpp.h"
#include "sample.h"
#include "settings.h"

#include <GLFW/glfw3.h>
#include <imgui.h>

class SingleBox : public Sample
{
public:
	SingleBox(const Settings& settings)
		: Sample(settings)
	{
		if (settings.restart == false)
		{
			g_camera.m_center = {0.0f, 3.0f};
			g_camera.m_zoom = 0.2f;
		}

		float extent = 1.0f;

		b2BodyDef bodyDef = b2_defaultBodyDef;
		b2BodyId groundId = b2CreateBody(m_worldId, &bodyDef);

		float groundWidth = 66.0f * extent;
		b2ShapeDef shapeDef = b2_defaultShapeDef;
		shapeDef.friction = 0.5f;

		b2Segment segment = {{-0.5f * 2.0f * groundWidth, 0.0f}, {0.5f * 2.0f * groundWidth, 0.0f}};
		b2CreateSegmentShape(groundId, &shapeDef, &segment);
		bodyDef.type = b2_dynamicBody;

		b2Polygon box = b2MakeBox(extent, extent);
		bodyDef.position = {0.0f, 4.0f};
		b2BodyId bodyId = b2CreateBody(m_worldId, &bodyDef);
		b2CreatePolygonShape(bodyId, &shapeDef, &box);
	}

	static Sample* Create(const Settings& settings)
	{
		return new SingleBox(settings);
	}
};

static int sampleSingleBox = RegisterSample("Stacking", "Single Box", SingleBox::Create);

class TiltedStack : public Sample
{
public:

	enum
	{
		e_columns = 10,
		e_rows = 10,
	};

	TiltedStack(const Settings& settings)
		: Sample(settings)
	{
		{
			b2BodyDef bodyDef = b2_defaultBodyDef;
			bodyDef.position = {0.0f, -1.0f};
			b2BodyId groundId = b2CreateBody(m_worldId, &bodyDef);

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
				b2BodyDef bodyDef = b2_defaultBodyDef;
				bodyDef.type = b2_dynamicBody;

				int32_t n = j * e_rows + i;

				bodyDef.position = {x + offset * i, 0.5f + 1.0f * i};
				b2BodyId bodyId = b2CreateBody(m_worldId, &bodyDef);

				m_bodies[n] = bodyId;

				b2CreatePolygonShape(bodyId, &sd, &box);
			}
		}
	}

	static Sample* Create(const Settings& settings)
	{
		return new TiltedStack(settings);
	}

	b2BodyId m_bodies[e_rows * e_columns];
};

static int sampleTiltedStack = RegisterSample("Stacking", "Tilted Stack", TiltedStack::Create);

class VerticalStack : public Sample
{
public:
	enum
	{
		e_maxColumns = 50,
		e_maxRows = 30,
		e_maxBullets = 20
	};

	enum ShapeType
	{
		e_circleShape = 0,
		e_boxShape
	};

	VerticalStack(const Settings& settings)
		: Sample(settings)
	{
		{
			b2BodyDef bodyDef = b2_defaultBodyDef;
			bodyDef.position = {0.0f, -1.0f};
			b2BodyId groundId = b2CreateBody(m_worldId, &bodyDef);

			b2Polygon box = b2MakeBox(1000.0f, 1.0f);
			b2ShapeDef sd = b2_defaultShapeDef;
			b2CreatePolygonShape(groundId, &sd, &box);
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
		m_rowCount = 10;
		m_columnCount = g_sampleDebug ? 4 : e_maxColumns;
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
				b2DestroyBody(m_bodies[i]);
				m_bodies[i] = b2_nullBodyId;
			}
		}

		b2Circle circle = {0};
		circle.radius = 0.5f;

		b2Polygon box = b2MakeBox(0.5f, 0.5f);
		// b2Polygon box = b2MakeRoundedBox(0.45f, 0.45f, 0.05f);

		b2ShapeDef sd = b2_defaultShapeDef;
		sd.density = 1.0f;
		sd.friction = 0.3f;

		float offset;

		if (m_shapeType == e_circleShape)
		{
			offset = 0.0f;
		}
		else
		{
			offset = 0.01f;
		}

		float dx = 3.0f;
		float xroot = -0.5f * dx * (m_columnCount - 1.0f);

		for (int32_t j = 0; j < m_columnCount; ++j)
		{
			float x = xroot + j * dx;

			for (int32_t i = 0; i < m_rowCount; ++i)
			{
				b2BodyDef bodyDef = b2_defaultBodyDef;
				bodyDef.type = b2_dynamicBody;

				int32_t n = j * m_rowCount + i;

				float shift = (i % 2 == 0 ? -offset : offset);
				bodyDef.position = {x + shift, 0.5f + 1.0f * i};
				// bodyDef.position = {x + shift, 1.0f + 1.51f * i};
				b2BodyId bodyId = b2CreateBody(m_worldId, &bodyDef);

				m_bodies[n] = bodyId;

				if (m_shapeType == e_circleShape)
				{
					b2CreateCircleShape(bodyId, &sd, &circle);
				}
				else
				{
					b2CreatePolygonShape(bodyId, &sd, &box);
				}
			}
		}
	}

	void DestroyBody()
	{
		for (int32_t j = 0; j < m_columnCount; ++j)
		{
			for (int32_t i = 0; i < m_rowCount; ++i)
			{
				int32_t n = j * m_rowCount + i;

				if (B2_NON_NULL(m_bodies[n]))
				{
					b2DestroyBody(m_bodies[n]);
					m_bodies[n] = b2_nullBodyId;
					break;
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
				b2DestroyBody(bullet);
				m_bullets[i] = b2_nullBodyId;
			}
		}
	}

	void FireBullets()
	{
		b2Circle circle;
		circle.radius = 0.25f;

		b2Polygon box = b2MakeBox(0.25f, 0.25f);

		b2ShapeDef sd = b2_defaultShapeDef;
		sd.density = 2.0f;
		sd.friction = 0.6f;

		for (int32_t i = 0; i < m_bulletCount; ++i)
		{
			b2BodyDef bodyDef = b2_defaultBodyDef;
			bodyDef.type = b2_dynamicBody;
			bodyDef.position = {-25.0f - i, 5.0f};
			bodyDef.linearVelocity = {50.0f, 0.0f};

			b2BodyId bullet = b2CreateBody(m_worldId, &bodyDef);
			b2CreatePolygonShape(bullet, &sd, &box);

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
		const char* shapeTypes[] = {"Circle", "Box"};

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

		if (ImGui::Button("Destroy Body"))
		{
			DestroyBody();
		}

		changed = changed || ImGui::Button("Reset Stack");

		if (changed)
		{
			DestroyBullets();
			CreateStacks();
		}

		ImGui::End();
	}

	static Sample* Create(const Settings& settings)
	{
		return new VerticalStack(settings);
	}

	b2BodyId m_bullets[e_maxBullets];
	b2BodyId m_bodies[e_maxRows * e_maxColumns];
	int32_t m_columnCount;
	int32_t m_rowCount;
	int32_t m_bulletCount;
	ShapeType m_shapeType;
	ShapeType m_bulletType;
};

static int sampleVerticalStack = RegisterSample("Stacking", "Vertical Stack", VerticalStack::Create);

class Cliff : public Sample
{
public:
	Cliff(const Settings& settings)
		: Sample(settings)
	{
		if (settings.restart == false)
		{
			g_camera.m_zoom = 0.5f;
			g_camera.m_center = {0.0f, 5.0f};
		}

		{
			b2BodyDef bodyDef = b2_defaultBodyDef;
			bodyDef.position = {0.0f, 0.0f};
			b2BodyId groundId = b2CreateBody(m_worldId, &bodyDef);

			b2Polygon box = b2MakeOffsetBox(100.0f, 1.0f, {0.0f, -1.0f}, 0.0f);
			b2CreatePolygonShape(groundId, &b2_defaultShapeDef, &box);

			b2Segment segment = {{-14.0f, 4.0f}, {-8.0f, 4.0f}};
			b2CreateSegmentShape(groundId, &b2_defaultShapeDef, &segment);

			box = b2MakeOffsetBox(3.0f, 0.5f, {0.0f, 4.0f}, 0.0f);
			b2CreatePolygonShape(groundId, &b2_defaultShapeDef, &box);

			b2Capsule capsule = {{8.5f, 4.0f}, {13.5f, 4.0f}, 0.5f};
			b2CreateCapsuleShape(groundId, &b2_defaultShapeDef, &capsule);
		}


		m_flip = false;

		for (int32_t i = 0; i < 9; ++i)
		{
			m_bodyIds[i] = b2_nullBodyId;
		}

		CreateBodies();

		// Unrelated testing of math_cpp.h
		b2Vec2 a = {1.0f, 2.0f};
		b2Vec2 b = {3.0f, 4.0f};
		b2Vec2 c = {-5.0f, -6.0f};

		c += a;
		b -= c;
		a = -c;
		a *= 2.0f;
		a += a + b - c;
		b = 3.0f * a - c * 2.0f;

		b.x += 0.0f;
	}

	void CreateBodies()
	{
		for (int32_t i = 0; i < 9; ++i)
		{
			if (B2_NON_NULL(m_bodyIds[i]))
			{
				b2DestroyBody(m_bodyIds[i]);
				m_bodyIds[i] = b2_nullBodyId;
			}
		}

		float sign = m_flip ? -1.0f : 1.0f;

		b2Capsule capsule = {{-0.25f, 0.0f}, {0.25f, 0.0f}, 0.25f};
		b2Circle circle = {{0.0f, 0.0f}, 0.5f};
		b2Polygon square = b2MakeSquare(0.5f);

		b2BodyDef bodyDef = b2_defaultBodyDef;
		bodyDef.type = b2_dynamicBody;

		{
			b2ShapeDef shapeDef = b2_defaultShapeDef;
			shapeDef.friction = 0.01f;
			bodyDef.linearVelocity = {2.0f * sign, 0.0f};

			float offset = m_flip ? -4.0f : 0.0f;

			bodyDef.position = {-9.0f + offset, 4.25f};
			m_bodyIds[0] = b2CreateBody(m_worldId, &bodyDef);
			b2CreateCapsuleShape(m_bodyIds[0], &shapeDef, &capsule);

			bodyDef.position = {2.0f + offset, 4.75f};
			m_bodyIds[1] = b2CreateBody(m_worldId, &bodyDef);
			b2CreateCapsuleShape(m_bodyIds[1], &shapeDef, &capsule);

			bodyDef.position = {13.0f + offset, 4.75f};
			m_bodyIds[2] = b2CreateBody(m_worldId, &bodyDef);
			b2CreateCapsuleShape(m_bodyIds[2], &shapeDef, &capsule);
		}

		{
			b2ShapeDef shapeDef = b2_defaultShapeDef;
			shapeDef.friction = 0.01f;
			bodyDef.linearVelocity = {2.5f * sign, 0.0f};

			bodyDef.position = {-11.0f, 4.5f};
			m_bodyIds[3] = b2CreateBody(m_worldId, &bodyDef);
			b2CreatePolygonShape(m_bodyIds[3], &shapeDef, &square);

			bodyDef.position = {0.0f, 5.0f};
			m_bodyIds[4] = b2CreateBody(m_worldId, &bodyDef);
			b2CreatePolygonShape(m_bodyIds[4], &shapeDef, &square);

			bodyDef.position = {11.0f, 5.0f};
			m_bodyIds[5] = b2CreateBody(m_worldId, &bodyDef);
			b2CreatePolygonShape(m_bodyIds[5], &shapeDef, &square);
		}

		{
			b2ShapeDef shapeDef = b2_defaultShapeDef;
			shapeDef.friction = 0.2f;
			bodyDef.linearVelocity = {1.5f * sign, 0.0f};

			float offset = m_flip ? 4.0f : 0.0f;

			bodyDef.position = {-13.0f + offset, 4.5f};
			m_bodyIds[6] = b2CreateBody(m_worldId, &bodyDef);
			b2CreateCircleShape(m_bodyIds[6], &shapeDef, &circle);

			bodyDef.position = {-2.0f + offset, 5.0f};
			m_bodyIds[7] = b2CreateBody(m_worldId, &bodyDef);
			b2CreateCircleShape(m_bodyIds[7], &shapeDef, &circle);

			bodyDef.position = {9.0f + offset, 5.0f};
			m_bodyIds[8] = b2CreateBody(m_worldId, &bodyDef);
			b2CreateCircleShape(m_bodyIds[8], &shapeDef, &circle);
		}
	}

	void UpdateUI() override
	{
		ImGui::SetNextWindowPos(ImVec2(10.0f, 300.0f), ImGuiCond_Once);
		ImGui::SetNextWindowSize(ImVec2(240.0f, 230.0f));
		ImGui::Begin("Cliff Options", nullptr, ImGuiWindowFlags_NoResize);

		if (ImGui::Button("Flip"))
		{
			m_flip = !m_flip;
			CreateBodies();
		}

		ImGui::End();
	}

	static Sample* Create(const Settings& settings)
	{
		return new Cliff(settings);
	}

	b2BodyId m_bodyIds[9];
	bool m_flip;
};

static int sampleShapesOnShapes = RegisterSample("Stacking", "Cliff", Cliff::Create);
