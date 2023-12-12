// SPDX-FileCopyrightText: 2022 Erin Catto
// SPDX-License-Identifier: MIT

#include "sample.h"
#include "settings.h"

#include "box2d/box2d.h"
#include "box2d/geometry.h"

#include <GLFW/glfw3.h>
#include <imgui.h>

// Note: reseting the scene is non-deterministic because the world uses freelists
class BenchmarkBarrel : public Sample
{
  public:
	enum ShapeType
	{
		e_circleShape = 0,
		e_capsuleShape = 1,
		e_boxShape = 2,
	};

	enum
	{
		e_maxColumns = 26,
		e_maxRows = 130,
	};

	BenchmarkBarrel(const Settings& settings)
		: Sample(settings)
	{
		float groundSize = 25.0f;

		{
			b2BodyDef bd = b2DefaultBodyDef();
			b2BodyId groundId = b2World_CreateBody(m_worldId, &bd);

			b2Polygon box = b2MakeBox(groundSize, 1.2f);
			b2ShapeDef sd = b2DefaultShapeDef();
			b2Body_CreatePolygon(groundId, &sd, &box);

			box = b2MakeOffsetBox(1.2f, 2.0f * groundSize, {-groundSize, 2.0f * groundSize}, 0.0f);
			b2Body_CreatePolygon(groundId, &sd, &box);

			box = b2MakeOffsetBox(1.2f, 2.0f * groundSize, {groundSize, 2.0f * groundSize}, 0.0f);
			b2Body_CreatePolygon(groundId, &sd, &box);

			b2Segment segment = {{-200.0f, -40.0f}, {200.0f, -40.0f}};
			b2Body_CreateSegment(groundId, &sd, &segment);
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

		m_columnCount = g_sampleDebug ? 10 : e_maxColumns;

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
		b2Capsule capsule = {{0.0f, -0.25f}, {0.0f, 0.25f}, rad};
		b2Circle circle = {{0.0f, 0.0f}, rad};

		float extray = m_shapeType == e_capsuleShape ? rad : 0.0f;

		m_rowCount = g_sampleDebug ? 40 : e_maxRows;

		int32_t index = 0;
		float side = -0.05f;

		for (int32_t i = 0; i < m_columnCount; ++i)
		{
			float x = i * shift - centerx;

			for (int32_t j = 0; j < m_rowCount; ++j)
			{
				float y = j * (shift + extray) + centery + 2.0f;

				bd.position = {x + side, y};
				side = -side;

				m_bodies[index] = b2World_CreateBody(m_worldId, &bd);

				if (m_shapeType == e_circleShape)
				{
					b2Body_CreateCircle(m_bodies[index], &sd, &circle);
				}
				else if (m_shapeType == e_capsuleShape)
				{
					b2Body_CreateCapsule(m_bodies[index], &sd, &capsule);
				}
				else
				{
					b2Body_CreatePolygon(m_bodies[index], &sd, &cuboid);
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
		const char* shapeTypes[] = {"Circle", "Capsule", "Box"};

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

	static Sample* Create(const Settings& settings)
	{
		return new BenchmarkBarrel(settings);
	}

	b2BodyId m_bodies[e_maxRows * e_maxColumns];
	int32_t m_columnCount;
	int32_t m_rowCount;

	ShapeType m_shapeType;
};

static int benchmarkBarrel = RegisterSample("Benchmark", "Barrel", BenchmarkBarrel::Create);

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

			// m_motorSpeed = 9.0f;
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

		m_maxCount = g_sampleDebug ? 300 : 2000;
		m_count = 0;
	}

	void UpdateUI() override
	{
		ImGui::SetNextWindowPos(ImVec2(10.0f, 300.0f), ImGuiCond_Once);
		ImGui::SetNextWindowSize(ImVec2(240.0f, 80.0f));
		ImGui::Begin("Tumbler", nullptr, ImGuiWindowFlags_NoResize);

		if (ImGui::SliderFloat("Speed", &m_motorSpeed, 0.0f, 100.0f, "%.f"))
		{
			b2RevoluteJoint_SetMotorSpeed(m_jointId, (b2_pi / 180.0f) * m_motorSpeed);
		}

		ImGui::End();
	}

	void Step(Settings& settings) override
	{
		if (settings.m_pause == false || settings.m_singleStep == true)
		{
			float a = 0.125f;
			for (int32_t i = 0; i < 5 && m_count < m_maxCount; ++i)
			{
				b2BodyDef bd = b2DefaultBodyDef();
				bd.type = b2_dynamicBody;
				bd.position = {5.0f * a + 2.0f * a * i, 10.0f + 2.0f * a * (m_stepCount % 5)};
				b2BodyId bodyId = b2World_CreateBody(m_worldId, &bd);

				b2ShapeDef sd = b2DefaultShapeDef();
				sd.density = 1.0f;

				b2Polygon polygon = b2MakeBox(0.125f, 0.125f);
				b2Body_CreatePolygon(bodyId, &sd, &polygon);
				++m_count;
			}
		}

		Sample::Step(settings);
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

static int benchmarkTumbler = RegisterSample("Benchmark", "Tumbler", BenchmarkTumbler::Create);

// TODO_ERIN make these kinematic
class BenchmarkManyTumblers : public Sample
{
  public:
	BenchmarkManyTumblers(const Settings& settings)
		: Sample(settings)
	{
		b2BodyDef bd = b2DefaultBodyDef();
		m_groundId = b2World_CreateBody(m_worldId, &bd);

		m_rowCount = g_sampleDebug ? 2 : 19;
		m_columnCount = g_sampleDebug ? 2 : 19;

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

		int32_t bodiesPerTumbler = g_sampleDebug ? 8 : 50;
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
				b2Body_Wake(m_tumblerIds[i]);
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
			// sd.restitution = 0.5f;

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

				// if (j == 0)
				//{
				//	b2Body_CreatePolygon(m_bodyIds[m_bodyIndex], &sd, &polygon);
				// }
				// else if (j == 1)
				{
					b2Body_CreateCapsule(m_bodyIds[m_bodyIndex], &sd, &capsule);
				}
				// else
				//{
				//	b2Body_CreateCircle(m_bodyIds[m_bodyIndex], &sd, &circle);
				// }

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

static int benchmarkManyTumblers = RegisterSample("Benchmark", "Many Tumblers", BenchmarkManyTumblers::Create);

class BenchmarkPyramid : public Sample
{
  public:
	BenchmarkPyramid(const Settings& settings)
		: Sample(settings)
	{
		m_extent = 0.5f;
		m_round = 0.0f;
		m_baseCount = 10;
		m_rowCount = g_sampleDebug ? 4 : 14;
		m_columnCount = g_sampleDebug ? 4 : 13;
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
			// b2Segment segment = {{-0.5f * groundWidth, groundY}, {0.5f * groundWidth, groundY}};
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

		if (changed)
		{
			CreateScene();
		}

		ImGui::End();
	}

	void Step(Settings& settings) override
	{
		// b2_collideMinRange = m_collideRange;
		// b2_islandMinRange = m_islandRange;

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

			// m_collideRange += 1;
			// if (m_collideRange > 300)
			//{
			//	m_collideRange = 32;
			// }

			// m_islandRange += 1;
			// if (m_islandRange > 4)
			//{
			//	m_islandRange = 1;
			// }
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

static int benchmarkPyramid = RegisterSample("Benchmark", "Pyramid", BenchmarkPyramid::Create);

class BenchmarkCreateDestroy : public Sample
{
  public:
	enum
	{
		e_maxBaseCount = 100,
		e_maxBodyCount = e_maxBaseCount * (e_maxBaseCount + 1) / 2
	};

	BenchmarkCreateDestroy(const Settings& settings)
		: Sample(settings)
	{
		float groundSize = 100.0f;

		b2BodyDef bd = b2DefaultBodyDef();
		b2BodyId groundId = b2World_CreateBody(m_worldId, &bd);

		b2Polygon box = b2MakeBox(groundSize, 1.0f);
		b2ShapeDef sd = b2DefaultShapeDef();
		b2Body_CreatePolygon(groundId, &sd, &box);

		for (int32_t i = 0; i < e_maxBodyCount; ++i)
		{
			m_bodies[i] = b2_nullBodyId;
		}

		m_baseCount = g_sampleDebug ? 40 : 100;
		m_iterations = g_sampleDebug ? 1 : 10;
		m_bodyCount = 0;
	}

	void CreateScene()
	{
		for (int32_t i = 0; i < e_maxBodyCount; ++i)
		{
			if (B2_NON_NULL(m_bodies[i]))
			{
				b2World_DestroyBody(m_bodies[i]);
				m_bodies[i] = b2_nullBodyId;
			}
		}

		int32_t count = m_baseCount;
		float rad = 0.5f;
		float shift = rad * 2.0f;
		float centerx = shift * count / 2.0f;
		float centery = shift / 2.0f + 1.0f;

		b2BodyDef bd = b2DefaultBodyDef();
		bd.type = b2_dynamicBody;

		b2ShapeDef sd = b2DefaultShapeDef();
		sd.density = 1.0f;
		sd.friction = 0.5f;

		float h = 0.5f;
		b2Polygon cuboid = b2MakeRoundedBox(h, h, 0.0f);

		int32_t index = 0;

		for (int32_t i = 0; i < count; ++i)
		{
			float y = i * shift + centery;

			for (int32_t j = i; j < count; ++j)
			{
				float x = 0.5f * i * shift + (j - i) * shift - centerx;
				bd.position = {x, y};

				assert(index < e_maxBodyCount);
				m_bodies[index] = b2World_CreateBody(m_worldId, &bd);
				b2Body_CreatePolygon(m_bodies[index], &sd, &cuboid);

				index += 1;
			}
		}

		m_bodyCount = index;
	}

	void Step(Settings& settings) override
	{
		float timeStep = settings.m_hertz > 0.0f ? 1.0f / settings.m_hertz : float(0.0f);

		for (int32_t i = 0; i < m_iterations; ++i)
		{
			CreateScene();
			b2World_Step(m_worldId, timeStep, settings.m_velocityIterations, settings.m_relaxIterations);
		}

		Sample::Step(settings);
	}

	static Sample* Create(const Settings& settings)
	{
		return new BenchmarkCreateDestroy(settings);
	}

	b2BodyId m_bodies[e_maxBodyCount];
	int32_t m_bodyCount;
	int32_t m_baseCount;
	int32_t m_iterations;
};

static int benchmarkCreateDestroy = RegisterSample("Benchmark", "CreateDestroy", BenchmarkCreateDestroy::Create);


class BenchmarkJointGrid : public Sample
{
  public:
	BenchmarkJointGrid(const Settings& settings)
		: Sample(settings)
	{
		constexpr float rad = 0.4f;
		constexpr int32_t numi = g_sampleDebug ? 10 : 100;
		constexpr int32_t numk = g_sampleDebug ? 10 : 100;
		constexpr float shift = 1.0f;

		// Allocate to avoid huge stack usage
		b2BodyId* bodies = static_cast<b2BodyId*>(malloc(numi * numk * sizeof(b2BodyId)));
		int32_t index = 0;

		b2ShapeDef shapeDef = b2DefaultShapeDef();
		shapeDef.density = 1.0f;
		shapeDef.filter.categoryBits = 2;
		shapeDef.filter.maskBits = ~2u;

		b2Circle circle = {0};
		circle.radius = rad;

		b2RevoluteJointDef jd = b2DefaultRevoluteJointDef();

		for (int32_t k = 0; k < numk; ++k)
		{
			for (int32_t i = 0; i < numi; ++i)
			{
				float fk = (float)k;
				float fi = (float)i;

				b2BodyDef bd = b2DefaultBodyDef();
				if (k >= numk / 2 - 3 && k <= numk / 2 + 3 && i == 0)
				{
					bd.type = b2_staticBody;
				}
				else
				{
					bd.type = b2_dynamicBody;
				}

				bd.position = {fk * shift, -fi * shift};

				b2BodyId body = b2World_CreateBody(m_worldId, &bd);

				b2Body_CreateCircle(body, &shapeDef, &circle);

				if (i > 0)
				{
					jd.bodyIdA = bodies[index - 1];
					jd.bodyIdB = body;
					jd.localAnchorA = {0.0f, -0.5f * shift};
					jd.localAnchorB = {0.0f, 0.5f * shift};
					b2World_CreateRevoluteJoint(m_worldId, &jd);
				}

				if (k > 0)
				{
					jd.bodyIdA = bodies[index - numi];
					jd.bodyIdB = body;
					jd.localAnchorA = {0.5f * shift, 0.0f};
					jd.localAnchorB = {-0.5f * shift, 0.0f};
					b2World_CreateRevoluteJoint(m_worldId, &jd);
				}

				bodies[index++] = body;
			}
		}

		free(bodies);
	}

	static Sample* Create(const Settings& settings)
	{
		return new BenchmarkJointGrid(settings);
	}
};

static int benchmarkJointGridIndex = RegisterSample("Benchmark", "Joint Grid", BenchmarkJointGrid::Create);
