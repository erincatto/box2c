// SPDX-FileCopyrightText: 2022 Erin Catto
// SPDX-License-Identifier: MIT

#include "box2d/box2d.h"
#include "box2d/geometry.h"
#include "sample.h"

#include <GLFW/glfw3.h>
#include <imgui.h>

class BenchmarkManyTumblers : public Sample
{
public:

	BenchmarkManyTumblers(const Settings& settings)
		: Sample(settings)
	{
		b2BodyDef bd = b2DefaultBodyDef();
		m_groundId = b2World_CreateBody(m_worldId, &bd);

		m_rowCount = g_sampleDebug ? 3 : 16;
		m_columnCount = g_sampleDebug ? 3 : 16;
		m_maxTumblerBodyCount = g_sampleDebug ? 8 : 32;

		m_groundId = b2_nullBodyId;
		m_bodyIds = nullptr;
		m_bodyCount = 0;
		m_bodyIndex = 0;
		m_jointIds = nullptr;
		m_jointCount = 0;
	}

	void CreateTumbler(float x, float y, int index)
	{
		b2BodyDef bd = b2DefaultBodyDef();
		bd.type = b2_dynamicBody;
		bd.enableSleep = false;
		bd.position = {x, y};
		b2BodyId bodyId = b2World_CreateBody(m_worldId, &bd);

		b2ShapeDef sd = b2DefaultShapeDef();
		sd.density = 50.0f;

		b2Polygon polygon;
		polygon = b2MakeOffsetBox(0.25f, 2.0f, {2.0f, 0.0f}, 0.0);
		b2Body_CreatePolygon(bodyId, &sd, &polygon);
		polygon = b2MakeOffsetBox(0.25f, 2.0f, {-2.0f, 0.0f}, 0.0);
		b2Body_CreatePolygon(bodyId, &sd, &polygon);
		polygon = b2MakeOffsetBox(2.0f, 0.25f, {0.0f, 2.0f}, 0.0);
		b2Body_CreatePolygon(bodyId, &sd, &polygon);
		polygon = b2MakeOffsetBox(2.0f, 0.25f, {0.0f, -2.0f}, 0.0);
		b2Body_CreatePolygon(bodyId, &sd, &polygon);

		m_motorSpeed = 9.0f;

		b2RevoluteJointDef jd = b2DefaultRevoluteJointDef();
		jd.bodyIdA = m_groundId;
		jd.bodyIdB = bodyId;
		jd.localAnchorA = {0.0f, 10.0f};
		jd.localAnchorB = {0.0f, 0.0f};
		jd.referenceAngle = 0.0f;
		jd.motorSpeed = (b2_pi / 180.0f) * m_motorSpeed;
		jd.maxMotorTorque = 1e8f;
		jd.enableMotor = true;

		m_jointId = b2World_CreateRevoluteJoint(m_worldId, &jd);
	}

	void CreateScene()
	{
		for (int32_t i = 0; i < m_bodyCount; ++i)
		{
			b2World_DestroyBody(m_bodyIds[i]);
		}

	}

	void UpdateUI() override
	{
		ImGui::SetNextWindowPos(ImVec2(10.0f, 300.0f), ImGuiCond_Once);
		ImGui::SetNextWindowSize(ImVec2(240.0f, 230.0f));
		ImGui::Begin("Tumbler", nullptr, ImGuiWindowFlags_NoResize);

		bool changed = false;
		changed = changed || ImGui::SliderInt("Row Count", &m_rowCount, 1, 32);
		changed = changed || ImGui::SliderInt("Column Count", &m_columnCount, 1, 32);

		if (changed)
		{
			CreateScene();
		}

		if (ImGui::SliderFloat("Speed", &m_motorSpeed, 0.0f, 100.0f, "%.f"))
		{
			for (int i = 0; i < m_jointCount; ++i)
			{
				b2RevoluteJoint_SetMotorSpeed(m_jointIds[i], (b2_pi / 180.0f) * m_motorSpeed);
			}
		}

		ImGui::End();
	}

	void Step(Settings& settings) override
	{
		Sample::Step(settings);

		if (m_bodyIndex < m_bodyCount)
		{
			for (int i = 0; i < m_rowCount; ++i)
			{
				for (int j = 0; j < m_columnCount; ++j)
				{
					b2BodyDef bd = b2DefaultBodyDef();
					bd.type = b2_dynamicBody;
					bd.position = {0.25f * i, 10.0f};
					m_bodyIds[m_bodyIndex] = b2World_CreateBody(m_worldId, &bd);

					b2ShapeDef sd = b2DefaultShapeDef();
					sd.density = 1.0f;

					b2Polygon polygon = b2MakeBox(0.125f, 0.125f);
					b2Body_CreatePolygon(m_bodyIds[m_bodyIndex], &sd, &polygon);
					
					m_bodyIndex += 1;
				}
			}
		}
	}

	static Sample* Create(const Settings& settings)
	{
		return new BenchmarkManyTumblers(settings);
	}

	b2BodyId m_groundId;

	int32_t m_rowCount;
	int32_t m_columnCount;

	b2BodyId* m_tumblerIds;
	int32_t m_tumblerCount;

	b2JointId* m_jointIds;
	int32_t m_jointCount;

	b2BodyId* m_bodyIds;
	int32_t m_bodyCount;
	int32_t m_bodyIndex;

	float m_motorSpeed;
};

static int testIndex = RegisterSample("Benchmark", "Many Tumblers", BenchmarkManyTumblers::Create);
