// SPDX-FileCopyrightText: 2022 Erin Catto
// SPDX-License-Identifier: MIT

#include "box2d/box2d.h"
#include "box2d/geometry.h"
#include "sample.h"

#include <GLFW/glfw3.h>
#include <imgui.h>

// Restitution is approximate since Box2D uses speculative collision
class Restitution : public Sample
{
public:

	enum
	{
		e_count = 40
	};

	enum ShapeType
	{
		e_circleShape = 0,
		e_boxShape
	};

	Restitution(const Settings& settings)
		: Sample(settings)
	{
		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			b2BodyId groundId = b2World_CreateBody(m_worldId, &bodyDef);

			float h = 1.0f * e_count;
			b2Segment segment = {{-h, 0.0f}, {h, 0.0f}};
			b2ShapeDef shapeDef = b2DefaultShapeDef();
			b2Body_CreateSegment(groundId, &shapeDef, &segment);
		}

		for (int32_t i = 0; i < e_count; ++i)
		{
			m_bodyIds[i] = b2_nullBodyId;
		}

		m_shapeType = e_circleShape;

		CreateBodies();
	}

	void CreateBodies()
	{
		for (int32_t i = 0; i < e_count; ++i)
		{
			if (B2_NON_NULL(m_bodyIds[i]))
			{
				b2World_DestroyBody(m_bodyIds[i]);
				m_bodyIds[i] = b2_nullBodyId;
			}
		}

		b2Circle circle = {0};
		circle.radius = 0.5f;

		b2Polygon box = b2MakeBox(0.5f, 0.5f);

		b2ShapeDef shapeDef = b2DefaultShapeDef();
		shapeDef.density = 1.0f;
		shapeDef.restitution = 0.0f;

		b2BodyDef bodyDef = b2DefaultBodyDef();
		bodyDef.type = b2_dynamicBody;

		float dr = 1.0f / (e_count > 1 ? e_count - 1 : 1);
		float x = -1.0f * (e_count - 1);
		float dx = 2.0f;

		for (int32_t i = 0; i < e_count; ++i)
		{
			bodyDef.position = {x, 8.0f};
			b2BodyId bodyId = b2World_CreateBody(m_worldId, &bodyDef);

			m_bodyIds[i] = bodyId;

			if (m_shapeType == e_circleShape)
			{
				b2Body_CreateCircle(bodyId, &shapeDef, &circle);
			}
			else
			{
				b2Body_CreatePolygon(bodyId, &shapeDef, &box);
			}

			shapeDef.restitution += dr;
			x += dx;
		}
	}

	void UpdateUI() override
	{
		ImGui::SetNextWindowPos(ImVec2(10.0f, 300.0f), ImGuiCond_Once);
		ImGui::SetNextWindowSize(ImVec2(240.0f, 230.0f));
		ImGui::Begin("Restitution", nullptr, ImGuiWindowFlags_NoResize);

		bool changed = false;
		const char* shapeTypes[] = { "Circle", "Box" };

		int shapeType = int(m_shapeType);
		changed = changed || ImGui::Combo("Shape", &shapeType, shapeTypes, IM_ARRAYSIZE(shapeTypes));
		m_shapeType = ShapeType(shapeType);

		changed = changed || ImGui::Button("Reset");

		if (changed)
		{
			CreateBodies();
		}

		ImGui::End();
	}

	static Sample* Create(const Settings& settings)
	{
		return new Restitution(settings);
	}

	b2BodyId m_bodyIds[e_count];
	ShapeType m_shapeType;
};

static int sampleIndex = RegisterSample("Stacking", "Restitution", Restitution::Create);
