// SPDX-FileCopyrightText: 2022 Erin Catto
// SPDX-License-Identifier: MIT

#include "sample.h"

#include "box2d/box2d.h"
#include "box2d/geometry.h"
#include "box2d/hull.h"
#include "box2d/math.h"

#include <GLFW/glfw3.h>
#include <imgui.h>

class BallDrop : public Sample
{
public:
	BallDrop(const Settings& settings)
		: Sample(settings)
	{
		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			b2BodyId groundId = b2World_CreateBody(m_worldId, &bodyDef);

			b2Segment segment = {{-10.0f, -10.0f}, {10.0f, 10.0f}};
			b2ShapeDef shapeDef = b2DefaultShapeDef();
			b2Body_CreateSegment(groundId, &shapeDef, &segment);
		}

		m_bodyId = b2_nullBodyId;

		Launch();
	}

	void Launch()
	{
		if (B2_NON_NULL(m_bodyId))
		{
			b2World_DestroyBody(m_bodyId);
		}

		b2BodyDef bodyDef = b2DefaultBodyDef();
		bodyDef.type = b2_dynamicBody;
		bodyDef.position = {0.0f, 8.0f};
		bodyDef.linearVelocity = {0.0f, -100.0f};

		b2Circle circle = {{0.0f, 0.0f}, 0.5f};
		b2ShapeDef shapeDef = b2DefaultShapeDef();
		shapeDef.density = 1.0f;

		m_bodyId = b2World_CreateBody(m_worldId, &bodyDef);
		b2Body_CreateCircle(m_bodyId, &shapeDef, &circle);
	}

	void UpdateUI() override
	{
		ImGui::SetNextWindowPos(ImVec2(10.0f, 300.0f), ImGuiCond_Once);
		ImGui::SetNextWindowSize(ImVec2(240.0f, 230.0f));
		ImGui::Begin("Options", nullptr, ImGuiWindowFlags_NoResize);

		if (ImGui::Button("Launch"))
		{
			Launch();
		}

		ImGui::End();
	}

	static Sample* Create(const Settings& settings)
	{
		return new BallDrop(settings);
	}

	b2BodyId m_bodyId;
};

static int sampleBallDrop = RegisterSample("Continuous", "BallDrop", BallDrop::Create);

class SkinnyBox : public Sample
{
public:
	SkinnyBox(const Settings& settings)
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

		m_autoTest = false;
		m_bullet = false;
		m_capsule = false;

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
		// m_angularVelocity = -30.6695766f;

		b2BodyDef bodyDef = b2DefaultBodyDef();
		bodyDef.type = b2_dynamicBody;
		bodyDef.position = {0.0f, 8.0f};
		bodyDef.angularVelocity = m_angularVelocity;
		bodyDef.linearVelocity = {0.0f, -100.0f};

		b2Polygon polygon;

		if (m_capsule)
		{
			polygon = b2MakeCapsule({0.0f, -1.0f}, {0.0f, 1.0f}, 0.1f);
		}
		else
		{
			polygon = b2MakeBox(2.0f, 0.05f);
		}

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
		ImGui::Begin("Options", nullptr, ImGuiWindowFlags_NoResize);

		ImGui::Checkbox("Capsule", &m_capsule);

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
		return new SkinnyBox(settings);
	}

	b2BodyId m_bodyId, m_bulletId;
	float m_angularVelocity;
	float m_x;
	bool m_capsule;
	bool m_autoTest;
	bool m_bullet;
};

static int sampleSkinnyBox = RegisterSample("Continuous", "Skinny Box", SkinnyBox::Create);

// This sample shows ghost collisions
class GhostCollision : public Sample
{
public:
	enum ShapeType
	{
		e_circleShape = 0,
		e_boxShape
	};

	GhostCollision(const Settings& settings)
		: Sample(settings)
	{
		m_groundId = b2_nullBodyId;
		m_bodyId = b2_nullBodyId;
		m_shapeId = b2_nullShapeId;

		m_shapeType = e_circleShape;
		m_round = 0.0f;
		m_friction = 0.5f;
		m_bevel = 0.0f;
		m_useChain = true;

		CreateScene();
		Launch();
	}

	void CreateScene()
	{
		if (B2_NON_NULL(m_groundId))
		{
			b2World_DestroyBody(m_groundId);
		}

		b2BodyDef bodyDef = b2DefaultBodyDef();
		m_groundId = b2World_CreateBody(m_worldId, &bodyDef);


		float m = 1.0f / sqrt(2.0f);
		float mm = 2.0f * (sqrt(2.0f) - 1.0f);
		float hx = 4.0f, hy = 0.25f;

		if (m_useChain)
		{
			b2Vec2 points[20];
			points[0] = {-3.0f * hx, hy};
			points[1] = b2Add(points[0], {-2.0f * hx * m, 2.0f * hx * m});
			points[2] = b2Add(points[1], {-2.0f * hx * m, 2.0f * hx * m});
			points[3] = b2Add(points[2], {-2.0f * hx * m, 2.0f * hx * m});
			points[4] = b2Add(points[3], {-2.0f * hy * m, -2.0f * hy * m});
			points[5] = b2Add(points[4], {2.0f * hx * m, -2.0f * hx * m});
			points[6] = b2Add(points[5], {2.0f * hx * m, -2.0f * hx * m});
			points[7] = b2Add(points[6], {2.0f * hx * m + 2.0f * hy * (1.0f - m), -2.0f * hx * m - 2.0f * hy * (1.0f - m)});
			points[8] = b2Add(points[7], {2.0f * hx + hy * mm, 0.0f});
			points[9] = b2Add(points[8], {2.0f * hx, 0.0f});
			points[10] = b2Add(points[9], {2.0f * hx + hy * mm, 0.0f});
			points[11] = b2Add(points[10], {2.0f * hx * m + 2.0f * hy * (1.0f - m), 2.0f * hx * m + 2.0f * hy * (1.0f - m)});
			points[12] = b2Add(points[11], {2.0f * hx * m, 2.0f * hx * m});
			points[13] = b2Add(points[12], {2.0f * hx * m, 2.0f * hx * m});
			points[14] = b2Add(points[13], {-2.0f * hy * m, 2.0f * hy * m});
			points[15] = b2Add(points[14], {-2.0f * hx * m, -2.0f * hx * m});
			points[16] = b2Add(points[15], {-2.0f * hx * m, -2.0f * hx * m});
			points[17] = b2Add(points[16], {-2.0f * hx * m, -2.0f * hx * m});
			points[18] = b2Add(points[17], {-2.0f * hx, 0.0f});
			points[19] = b2Add(points[18], {-2.0f * hx, 0.0f});

			b2ChainDef chainDef = b2DefaultChainDef();
			chainDef.points = points;
			chainDef.count = 20;
			chainDef.loop = true;

			b2Body_CreateChain(m_groundId, &chainDef);
		}
		else
		{
			b2ShapeDef shapeDef = b2DefaultShapeDef();
			b2Hull hull = {0};

			if (m_bevel > 0.0f)
			{
				float hb = m_bevel;
				b2Vec2 vs[8] = {{hx + hb, hy - 0.05f},	 {hx, hy},	 {-hx, hy}, {-hx - hb, hy - 0.05f},
								{-hx - hb, -hy + 0.05f}, {-hx, -hy}, {hx, -hy}, {hx + hb, -hy + 0.05f}};
				hull = b2ComputeHull(vs, 8);
			}
			else
			{
				b2Vec2 vs[4] = {{hx, hy}, {-hx, hy}, {-hx, -hy}, {hx, -hy}};
				hull = b2ComputeHull(vs, 4);
			}

			b2Transform transform;
			float x, y;

			// Left slope
			x = -3.0f * hx - m * hx - m * hy;
			y = hy + m * hx - m * hy;
			transform.q = b2MakeRot(-0.25f * b2_pi);

			{
				transform.p = {x, y};
				b2Polygon polygon = b2MakeOffsetPolygon(&hull, m_round, transform);
				b2Body_CreatePolygon(m_groundId, &shapeDef, &polygon);
				x -= 2.0f * m * hx;
				y += 2.0f * m * hx;
			}
			{
				transform.p = {x, y};
				b2Polygon polygon = b2MakeOffsetPolygon(&hull, m_round, transform);
				b2Body_CreatePolygon(m_groundId, &shapeDef, &polygon);
				x -= 2.0f * m * hx;
				y += 2.0f * m * hx;
			}
			{
				transform.p = {x, y};
				b2Polygon polygon = b2MakeOffsetPolygon(&hull, m_round, transform);
				b2Body_CreatePolygon(m_groundId, &shapeDef, &polygon);
				x -= 2.0f * m * hx;
				y += 2.0f * m * hx;
			}

			x = -2.0f * hx;
			y = 0.0f;
			transform.q = b2MakeRot(0.0f);

			{
				transform.p = {x, y};
				b2Polygon polygon = b2MakeOffsetPolygon(&hull, m_round, transform);
				b2Body_CreatePolygon(m_groundId, &shapeDef, &polygon);
				x += 2.0f * hx;
			}
			{
				transform.p = {x, y};
				b2Polygon polygon = b2MakeOffsetPolygon(&hull, m_round, transform);
				b2Body_CreatePolygon(m_groundId, &shapeDef, &polygon);
				x += 2.0f * hx;
			}
			{
				transform.p = {x, y};
				b2Polygon polygon = b2MakeOffsetPolygon(&hull, m_round, transform);
				b2Body_CreatePolygon(m_groundId, &shapeDef, &polygon);
				x += 2.0f * hx;
			}

			x = 3.0f * hx + m * hx + m * hy;
			y = hy + m * hx - m * hy;
			transform.q = b2MakeRot(0.25f * b2_pi);

			{
				transform.p = {x, y};
				b2Polygon polygon = b2MakeOffsetPolygon(&hull, m_round, transform);
				b2Body_CreatePolygon(m_groundId, &shapeDef, &polygon);
				x += 2.0f * m * hx;
				y += 2.0f * m * hx;
			}
			{
				transform.p = {x, y};
				b2Polygon polygon = b2MakeOffsetPolygon(&hull, m_round, transform);
				b2Body_CreatePolygon(m_groundId, &shapeDef, &polygon);
				x += 2.0f * m * hx;
				y += 2.0f * m * hx;
			}
			{
				transform.p = {x, y};
				b2Polygon polygon = b2MakeOffsetPolygon(&hull, m_round, transform);
				b2Body_CreatePolygon(m_groundId, &shapeDef, &polygon);
				x += 2.0f * m * hx;
				y += 2.0f * m * hx;
			}
		}
	}

	void Launch()
	{
		if (B2_NON_NULL(m_bodyId))
		{
			b2World_DestroyBody(m_bodyId);
			m_shapeId = b2_nullShapeId;
		}

		b2BodyDef bodyDef = b2DefaultBodyDef();
		bodyDef.type = b2_dynamicBody;
		bodyDef.position = {-28.0f, 18.0f};
		bodyDef.linearVelocity = {0.0f, 0.0f};
		m_bodyId = b2World_CreateBody(m_worldId, &bodyDef);

		b2ShapeDef shapeDef = b2DefaultShapeDef();
		shapeDef.density = 1.0f;
		shapeDef.friction = m_friction;

		if (m_shapeType == e_circleShape)
		{
			b2Circle circle = {{0.0f, 0.0f}, 0.5f};
			m_shapeId = b2Body_CreateCircle(m_bodyId, &shapeDef, &circle);
		}
		else
		{
			b2Polygon box = b2MakeBox(0.5f, 0.5f);
			m_shapeId = b2Body_CreatePolygon(m_bodyId, &shapeDef, &box);
		}
	}

	void UpdateUI() override
	{
		ImGui::SetNextWindowPos(ImVec2(10.0f, 300.0f), ImGuiCond_Once);
		ImGui::SetNextWindowSize(ImVec2(240.0f, 230.0f));
		ImGui::Begin("Options", nullptr, ImGuiWindowFlags_NoResize);

		if (ImGui::Checkbox("Chain", &m_useChain))
		{
			CreateScene();
		}

		if (m_useChain == false)
		{
			if (ImGui::SliderFloat("Round", &m_round, 0.0f, 0.5f, "%.2f"))
			{
				CreateScene();
			}

			if (ImGui::SliderFloat("Bevel", &m_bevel, 0.0f, 1.0f, "%.2f"))
			{
				CreateScene();
			}
		}

		{
			const char* shapeTypes[] = {"Circle", "Box"};
			int shapeType = int(m_shapeType);
			ImGui::Combo("Shape", &shapeType, shapeTypes, IM_ARRAYSIZE(shapeTypes));
			m_shapeType = ShapeType(shapeType);
		}

		if (ImGui::SliderFloat("Friction", &m_friction, 0.0f, 1.0f, "%.2f"))
		{
			if (B2_NON_NULL(m_shapeId))
			{
				b2Shape_SetFriction(m_shapeId, m_friction);
			}
		}

		if (ImGui::Button("Launch"))
		{
			Launch();
		}

		ImGui::End();
	}

	static Sample* Create(const Settings& settings)
	{
		return new GhostCollision(settings);
	}

	b2BodyId m_groundId;
	b2BodyId m_bodyId;
	b2ShapeId m_shapeId;
	ShapeType m_shapeType;
	float m_round;
	float m_friction;
	float m_bevel;
	bool m_useChain;
};

static int sampleGhostCollision = RegisterSample("Continuous", "Ghost Collision", GhostCollision::Create);
