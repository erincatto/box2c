// SPDX-FileCopyrightText: 2022 Erin Catto
// SPDX-License-Identifier: MIT

#include "box2d/box2d.h"
#include "box2d/geometry.h"
#include "sample.h"

#include <GLFW/glfw3.h>
#include <imgui.h>

class BenchmarkTumbler : public Sample
{
public:

	BenchmarkTumbler(const Settings& settings)
		: Sample(settings)
	{
		b2BodyId groundId;
		{
			b2BodyDef bd = b2DefaultBodyDef();
			groundId = b2World_CreateBody(m_worldId, &bd);
		}

		{
			b2BodyDef bd = b2DefaultBodyDef();
			bd.type = b2_dynamicBody;
			bd.enableSleep = false;
			bd.position = {0.0f, 10.0f};
			b2BodyId bodyId = b2World_CreateBody(m_worldId, &bd);

			b2ShapeDef sd = b2DefaultShapeDef();
			sd.density = 50.0f;

			b2Polygon polygon;
			polygon = b2MakeOffsetBox(0.5f, 10.0f, {10.0f, 0.0f}, 0.0);
			b2Body_CreatePolygon(bodyId, &sd, &polygon);
			polygon = b2MakeOffsetBox(0.5f, 10.0f, {-10.0f, 0.0f}, 0.0);
			b2Body_CreatePolygon(bodyId, &sd, &polygon);
			polygon = b2MakeOffsetBox(10.0f, 0.5f, {0.0f, 10.0f}, 0.0);
			b2Body_CreatePolygon(bodyId, &sd, &polygon);
			polygon = b2MakeOffsetBox(10.0f, 0.5f, {0.0f, -10.0f}, 0.0);
			b2Body_CreatePolygon(bodyId, &sd, &polygon);

			//m_motorSpeed = 9.0f;
			m_motorSpeed = 25.0f;

			b2RevoluteJointDef jd = b2DefaultRevoluteJointDef();
			jd.bodyIdA = groundId;
			jd.bodyIdB = bodyId;
			jd.localAnchorA = {0.0f, 10.0f};
			jd.localAnchorB = {0.0f, 0.0f};
			jd.referenceAngle = 0.0f;
			jd.motorSpeed = (b2_pi / 180.0f) * m_motorSpeed;
			jd.maxMotorTorque = 1e8f;
			jd.enableMotor = true;

			m_jointId = b2World_CreateRevoluteJoint(m_worldId, &jd);
		}

		//m_maxCount = g_sampleDebug ? 500 : 2000;
		m_maxCount = 2000;
		m_count = 0;
	}

	void UpdateUI() override
	{
		ImGui::SetNextWindowPos(ImVec2(10.0f, 300.0f), ImGuiCond_Once);
		ImGui::SetNextWindowSize(ImVec2(240.0f, 230.0f));
		ImGui::Begin("Tumbler", nullptr, ImGuiWindowFlags_NoResize);

		if (ImGui::SliderFloat("Speed", &m_motorSpeed, 0.0f, 100.0f, "%.f"))
		{
			b2RevoluteJoint_SetMotorSpeed(m_jointId, (b2_pi / 180.0f) * m_motorSpeed);
		}

		ImGui::End();
	}

	void Step(Settings& settings) override
	{
		Sample::Step(settings);

		for (int32_t i = 0; i < 5 && m_count < m_maxCount; ++i)
		{
			b2BodyDef bd = b2DefaultBodyDef();
			bd.type = b2_dynamicBody;
			bd.position = {0.25f * i, 10.0f + 1.0f * (m_stepCount & 1)};
			b2BodyId bodyId = b2World_CreateBody(m_worldId, &bd);

			b2ShapeDef sd = b2DefaultShapeDef();
			sd.density = 1.0f;

			b2Polygon polygon = b2MakeBox(0.125f, 0.125f);
			b2Body_CreatePolygon(bodyId, &sd, &polygon);
			++m_count;
		}
	}

	static Sample* Create(const Settings& settings)
	{
		return new BenchmarkTumbler(settings);
	}

	b2JointId m_jointId;
	float m_motorSpeed;
	int32_t m_maxCount;
	int32_t m_count;
};

static int testIndex = RegisterSample("Benchmark", "Tumbler", BenchmarkTumbler::Create);
