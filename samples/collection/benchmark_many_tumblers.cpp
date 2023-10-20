// SPDX-FileCopyrightText: 2022 Erin Catto
// SPDX-License-Identifier: MIT

#include "sample.h"

#include "box2d/box2d.h"
#include "box2d/geometry.h"

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

		m_rowCount = g_sampleDebug ? 1 : 19;
		m_columnCount = g_sampleDebug ? 1 : 19;

		m_tumblerIds = nullptr;
		m_jointIds = nullptr;
		m_positions = nullptr;
		m_tumblerCount = 0;

		m_bodyIds = nullptr;
		m_bodyCount = 0;
		m_bodyIndex = 0;

		m_motorSpeed = 25.0f;
		m_shapeType = 0;

		CreateScene();
	}

	~BenchmarkManyTumblers()
	{
		free(m_jointIds);
		free(m_tumblerIds);
		free(m_positions);
		free(m_bodyIds);
	}

	void CreateTumbler(b2Vec2 position, int index)
	{
		b2BodyDef bd = b2DefaultBodyDef();
		bd.type = b2_dynamicBody;
		bd.enableSleep = false;
		bd.position = {position.x, position.y};
		b2BodyId bodyId = b2World_CreateBody(m_worldId, &bd);
		m_tumblerIds[index] = bodyId;

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

		b2RevoluteJointDef jd = b2DefaultRevoluteJointDef();
		jd.bodyIdA = m_groundId;
		jd.bodyIdB = bodyId;
		jd.localAnchorA = position;
		jd.localAnchorB = {0.0f, 0.0f};
		jd.referenceAngle = 0.0f;
		jd.motorSpeed = (b2_pi / 180.0f) * m_motorSpeed;
		jd.maxMotorTorque = 1e8f;
		jd.enableMotor = true;

		m_jointIds[index] = b2World_CreateRevoluteJoint(m_worldId, &jd);
	}

	void CreateScene()
	{
		for (int32_t i = 0; i < m_bodyCount; ++i)
		{
			if (B2_NON_NULL(m_bodyIds[i]))
			{
				b2World_DestroyBody(m_bodyIds[i]);
			}
		}

		for (int32_t i = 0; i < m_tumblerCount; ++i)
		{
			b2World_DestroyJoint(m_jointIds[i]);
			b2World_DestroyBody(m_tumblerIds[i]);
		}

		free(m_jointIds);
		free(m_tumblerIds);
		free(m_positions);

		m_tumblerCount = m_rowCount * m_columnCount;
		m_tumblerIds = static_cast<b2BodyId*>(malloc(m_tumblerCount * sizeof(b2BodyId)));
		m_jointIds = static_cast<b2JointId*>(malloc(m_tumblerCount * sizeof(b2JointId)));
		m_positions = static_cast<b2Vec2*>(malloc(m_tumblerCount * sizeof(b2Vec2)));

		int32_t index = 0;
		float x = -4.0f * m_rowCount;
		for (int32_t i = 0; i < m_rowCount; ++i)
		{
			float y = -4.0f * m_columnCount;
			for (int32_t j = 0; j < m_columnCount; ++j)
			{
				m_positions[index] = {x, y};
				CreateTumbler(m_positions[index], index);
				++index;
				y += 8.0f;
			}

			x += 8.0f;
		}

		free(m_bodyIds);

		int32_t bodiesPerTumbler = g_sampleDebug ? 1 : 50;
		m_bodyCount = bodiesPerTumbler * m_tumblerCount;

		m_bodyIds = static_cast<b2BodyId*>(malloc(m_bodyCount * sizeof(b2BodyId)));

		// 0xFF is a fast way to make all bodies satisfy B2_IS_NULL
		memset(m_bodyIds, 0XFF, m_bodyCount * sizeof(b2BodyId));
		m_bodyIndex = 0;

		m_shapeType = 0;
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
			for (int i = 0; i < m_tumblerCount; ++i)
			{
				b2RevoluteJoint_SetMotorSpeed(m_jointIds[i], (b2_pi / 180.0f) * m_motorSpeed);
			}
		}

		ImGui::End();
	}

	void Step(Settings& settings) override
	{
		Sample::Step(settings);

		if (m_bodyIndex < m_bodyCount && (m_stepCount & 0x7) == 0)
		{
			b2ShapeDef sd = b2DefaultShapeDef();
			sd.density = 1.0f;

			b2Circle circle = {{0.0f, 0.0f}, 0.125f};
			b2Polygon polygon = b2MakeBox(0.125f, 0.125f);
			b2Capsule capsule = {{-0.1f, 0.0f}, {0.1f, 0.0f}, 0.075f};
			int j = m_shapeType % 3;

			for (int i = 0; i < m_tumblerCount; ++i)
			{
				assert(m_bodyIndex < m_bodyCount);

				b2BodyDef bd = b2DefaultBodyDef();
				bd.type = b2_dynamicBody;
				bd.position = m_positions[i];
				m_bodyIds[m_bodyIndex] = b2World_CreateBody(m_worldId, &bd);

				//if (j == 0)
				//{
				//	b2Body_CreatePolygon(m_bodyIds[m_bodyIndex], &sd, &polygon);
				//}
				//else if (j == 1)
				{
					b2Body_CreateCapsule(m_bodyIds[m_bodyIndex], &sd, &capsule);
				}
				//else
				//{
				//	b2Body_CreateCircle(m_bodyIds[m_bodyIndex], &sd, &circle);
				//}

				m_bodyIndex += 1;
			}

			m_shapeType += 1;
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
	b2JointId* m_jointIds;
	b2Vec2* m_positions;
	int32_t m_tumblerCount;

	b2BodyId* m_bodyIds;
	int32_t m_bodyCount;
	int32_t m_bodyIndex;
	int32_t m_shapeType;

	float m_motorSpeed;
};

static int testIndex = RegisterSample("Benchmark", "Many Tumblers", BenchmarkManyTumblers::Create);
