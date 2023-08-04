// SPDX-FileCopyrightText: 2022 Erin Catto
// SPDX-License-Identifier: MIT

#include "box2d/box2d.h"
#include "box2d/geometry.h"
#include "sample.h"

#include <GLFW/glfw3.h>
#include <imgui.h>

class Continuous1 : public Sample
{
public:

	Continuous1(const Settings& settings)
		: Sample(settings)
	{
		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			b2BodyId groundId = b2World_CreateBody(m_worldId, &bodyDef);

			b2Segment segment = {{-10.0f, 0.0f}, {10.0f, 0.0f}};
			b2ShapeDef shapeDef = b2DefaultShapeDef();
			shapeDef.friction = 0.9f;
			b2Body_CreateSegment(groundId, &shapeDef, &segment);

			b2Polygon box = b2MakeOffsetBox(0.1f, 1.0f, {0.0f, 1.0f}, 0.0f);
			b2Body_CreatePolygon(groundId, &shapeDef, &box);
		}

		m_linearSpeed = 0.0f;
		m_angularSpeed = 0.0f;
		m_autoTest = false;
		m_bullet = false;

		m_bodyId = b2_nullBodyId;
		m_bulletId = b2_nullBodyId;

		Launch();
	}

	void Launch()
	{
		if (B2_NON_NULL(m_bodyId))
		{
			b2World_DestroyBody(m_bodyId);
		}

		if (B2_NON_NULL(m_bulletId))
		{
			b2World_DestroyBody(m_bulletId);
		}

		m_angularVelocity = RandomFloat(-50.0f, 50.0f);
		//m_angularVelocity = -30.6695766f;

		b2BodyDef bodyDef = b2DefaultBodyDef();
		bodyDef.type = b2_dynamicBody;
		bodyDef.position = {0.0f, 8.0f};
		bodyDef.angularVelocity = m_angularVelocity;
		bodyDef.linearVelocity = {0.0f, -100.0f};

		b2Polygon polygon = b2MakeBox(2.0f, 0.1f);

		b2ShapeDef shapeDef = b2DefaultShapeDef();
		shapeDef.density = 1.0f;
		shapeDef.friction = 0.9f;

		m_bodyId = b2World_CreateBody(m_worldId, &bodyDef);
		b2Body_CreatePolygon(m_bodyId, &shapeDef, &polygon);

		if (m_bullet)
		{
			polygon = b2MakeBox(0.25f, 0.25f);
			m_x = RandomFloat(-1.0f, 1.0f);
			bodyDef.position = {m_x, 10.0f};
			bodyDef.linearVelocity = {0.0f, -50.0f};
			m_bulletId = b2World_CreateBody(m_worldId, &bodyDef);
			b2Body_CreatePolygon(m_bulletId, &shapeDef, &polygon);
		}
	}

	void UpdateUI() override
	{
		ImGui::SetNextWindowPos(ImVec2(10.0f, 300.0f), ImGuiCond_Once);
		ImGui::SetNextWindowSize(ImVec2(240.0f, 230.0f));
		ImGui::Begin("Continuous1", nullptr, ImGuiWindowFlags_NoResize);

		ImGui::SliderFloat("Linear Speed", &m_linearSpeed, 0.0f, 200.0f);
		ImGui::SliderFloat("Angular Speed", &m_angularSpeed, 0.0f, 45.0f);
		
		if (ImGui::Button("Launch"))
		{
			Launch();
		}

		ImGui::Checkbox("Auto Test", &m_autoTest);

		ImGui::End();
	}

	void Step(Settings& settings) override
	{
		Sample::Step(settings);

		if (m_autoTest && m_stepCount % 60 == 0)
		{
			Launch();
		}
	}

	static Sample* Create(const Settings& settings)
	{
		return new Continuous1(settings);
	}

	b2BodyId m_bodyId, m_bulletId;
	float m_angularVelocity;
	float m_x;
	float m_linearSpeed;
	float m_angularSpeed;
	bool m_autoTest;
	bool m_bullet;
};

static int sampleIndex = RegisterSample("Continuous", "Continuous1", Continuous1::Create);
