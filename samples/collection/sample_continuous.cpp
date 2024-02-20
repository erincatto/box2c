// SPDX-FileCopyrightText: 2022 Erin Catto
// SPDX-License-Identifier: MIT

#include "sample.h"
#include "settings.h"

#include "box2d/box2d.h"
#include "box2d/geometry.h"
#include "box2d/hull.h"
#include "box2d/math.h"

#include <GLFW/glfw3.h>
#include <imgui.h>

// This tests continuous collision robustness and also demonstrates the speed limits imposed
// by b2_maxTranslation and b2_maxRotation.
class BounceHouse : public Sample
{
public:
	enum ShapeType
	{
		e_circleShape = 0,
		e_capsuleShape,
		e_boxShape
	};

	BounceHouse(const Settings& settings)
		: Sample(settings)
	{
		if (settings.restart == false)
		{
			g_camera.m_center = {0.0f, 0.0f};
		}

		b2BodyDef bodyDef = b2DefaultBodyDef();
		b2BodyId groundId = b2CreateBody(m_worldId, &bodyDef);

		b2ShapeDef shapeDef = b2DefaultShapeDef();
		{
			b2Segment segment = {{-10.0f, -10.0f}, {10.0f, -10.0f}};
			b2CreateSegmentShape(groundId, &shapeDef, &segment);
		}

		{
			b2Segment segment = {{10.0f, -10.0f}, {10.0f, 10.0f}};
			b2CreateSegmentShape(groundId, &shapeDef, &segment);
		}

		{
			b2Segment segment = {{10.0f, 10.0f}, {-10.0f, 10.0f}};
			b2CreateSegmentShape(groundId, &shapeDef, &segment);
		}

		{
			b2Segment segment = {{-10.0f, 10.0f}, {-10.0f, -10.0f}};
			b2CreateSegmentShape(groundId, &shapeDef, &segment);
		}

		m_shapeType = e_circleShape;
		m_bodyId = b2_nullBodyId;

		Launch();
	}

	void Launch()
	{
		if (B2_IS_NON_NULL(m_bodyId))
		{
			b2DestroyBody(m_bodyId);
		}

		b2BodyDef bodyDef = b2DefaultBodyDef();
		bodyDef.type = b2_dynamicBody;
		bodyDef.linearVelocity = {10.0f, 20.0f};
		bodyDef.position = {0.0f, 0.0f};
		bodyDef.gravityScale = 0.0f;
		m_bodyId = b2CreateBody(m_worldId, &bodyDef);

		b2ShapeDef shapeDef = b2DefaultShapeDef();
		shapeDef.density = 1.0f;
		shapeDef.restitution = 1.2f;

		if (m_shapeType == e_circleShape)
		{
			b2Circle circle = {{0.0f, 0.0f}, 0.5f};
			b2CreateCircleShape(m_bodyId, &shapeDef, &circle);
		}
		else if (m_shapeType == e_capsuleShape)
		{
			b2Capsule capsule = {{-0.5f, 0.0f}, {0.5f, 0.0}, 0.25f};
			b2CreateCapsuleShape(m_bodyId, &shapeDef, &capsule);
		}
		else
		{
			float h = 0.5f;
			b2Polygon box = b2MakeBox(h, h);
			b2CreatePolygonShape(m_bodyId, &shapeDef, &box);
		}
	}

	void UpdateUI() override
	{
		ImGui::SetNextWindowPos(ImVec2(10.0f, 300.0f), ImGuiCond_Once);
		ImGui::SetNextWindowSize(ImVec2(240.0f, 70.0f));
		ImGui::Begin("Options", nullptr, ImGuiWindowFlags_NoResize);

		const char* shapeTypes[] = {"Circle", "Capsule", "Box"};
		int shapeType = int(m_shapeType);
		if (ImGui::Combo("Shape", &shapeType, shapeTypes, IM_ARRAYSIZE(shapeTypes)))
		{
			m_shapeType = ShapeType(shapeType);
			Launch();
		}

		ImGui::End();
	}

	static Sample* Create(const Settings& settings)
	{
		return new BounceHouse(settings);
	}

	b2BodyId m_bodyId;
	ShapeType m_shapeType;
};

static int sampleBounceHouse = RegisterSample("Continuous", "Bounce House", BounceHouse::Create);

class FastChain : public Sample
{
public:
	FastChain(const Settings& settings)
		: Sample(settings)
	{
		if (settings.restart == false)
		{
			g_camera.m_center = {0.0f, 0.0f};
			g_camera.m_zoom = 0.5f;
		}

		b2BodyDef bodyDef = b2DefaultBodyDef();
		bodyDef.position = {0.0f, -6.0f};
		b2BodyId groundId = b2CreateBody(m_worldId, &bodyDef);

		b2Vec2 points[4] = {{-10.0f, -2.0f}, {10.0f, -2.0f}, {10.0f, 1.0f}, {-10.0f, 1.0f}};

		b2ChainDef chainDef = b2_defaultChainDef;
		chainDef.points = points;
		chainDef.count = 4;
		chainDef.loop = true;

		b2CreateChain(groundId, &chainDef);

		m_bodyId = b2_nullBodyId;

		Launch();
	}

	void Launch()
	{
		if (B2_IS_NON_NULL(m_bodyId))
		{
			b2DestroyBody(m_bodyId);
		}

		b2BodyDef bodyDef = b2DefaultBodyDef();
		bodyDef.type = b2_dynamicBody;
		bodyDef.linearVelocity = {0.0f, -200.0f};
		bodyDef.position = {0.0f, 10.0f};
		bodyDef.gravityScale = 1.0f;
		m_bodyId = b2CreateBody(m_worldId, &bodyDef);

		b2Circle circle = {{0.0f, 0.0f}, 0.5f};
		b2CreateCircleShape(m_bodyId, &b2DefaultShapeDef(), &circle);
	}

	void UpdateUI() override
	{
		ImGui::SetNextWindowPos(ImVec2(10.0f, 300.0f), ImGuiCond_Once);
		ImGui::SetNextWindowSize(ImVec2(240.0f, 70.0f));
		ImGui::Begin("Fast Chain", nullptr, ImGuiWindowFlags_NoResize);

		if (ImGui::Button("Launch"))
		{
			Launch();
		}

		ImGui::End();
	}

	static Sample* Create(const Settings& settings)
	{
		return new FastChain(settings);
	}

	b2BodyId m_bodyId;
};

static int sampleFastChainHouse = RegisterSample("Continuous", "Fast Chain", FastChain::Create);

class SkinnyBox : public Sample
{
public:
	SkinnyBox(const Settings& settings)
		: Sample(settings)
	{
		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			b2BodyId groundId = b2CreateBody(m_worldId, &bodyDef);

			b2Segment segment = {{-10.0f, 0.0f}, {10.0f, 0.0f}};
			b2ShapeDef shapeDef = b2DefaultShapeDef();
			shapeDef.friction = 0.9f;
			b2CreateSegmentShape(groundId, &shapeDef, &segment);

			b2Polygon box = b2MakeOffsetBox(0.1f, 1.0f, {0.0f, 1.0f}, 0.0f);
			b2CreatePolygonShape(groundId, &shapeDef, &box);
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
		if (B2_IS_NON_NULL(m_bodyId))
		{
			b2DestroyBody(m_bodyId);
		}

		if (B2_IS_NON_NULL(m_bulletId))
		{
			b2DestroyBody(m_bulletId);
		}

		m_angularVelocity = RandomFloat(-50.0f, 50.0f);
		// m_angularVelocity = -30.6695766f;

		b2BodyDef bodyDef = b2DefaultBodyDef();
		bodyDef.type = b2_dynamicBody;
		bodyDef.position = {0.0f, 8.0f};
		bodyDef.angularVelocity = m_angularVelocity;
		bodyDef.linearVelocity = {0.0f, -100.0f};

		b2ShapeDef shapeDef = b2DefaultShapeDef();
		shapeDef.density = 1.0f;
		shapeDef.friction = 0.9f;

		m_bodyId = b2CreateBody(m_worldId, &bodyDef);

		if (m_capsule)
		{
			b2Capsule capsule = {{0.0f, -1.0f}, {0.0f, 1.0f}, 0.1f};
			b2CreateCapsuleShape(m_bodyId, &shapeDef, &capsule);
		}
		else
		{
			b2Polygon polygon = b2MakeBox(2.0f, 0.05f);
			b2CreatePolygonShape(m_bodyId, &shapeDef, &polygon);
		}

		if (m_bullet)
		{
			b2Polygon polygon = b2MakeBox(0.25f, 0.25f);
			m_x = RandomFloat(-1.0f, 1.0f);
			bodyDef.position = {m_x, 10.0f};
			bodyDef.linearVelocity = {0.0f, -50.0f};
			m_bulletId = b2CreateBody(m_worldId, &bodyDef);
			b2CreatePolygonShape(m_bulletId, &shapeDef, &polygon);
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
		e_capsuleShape,
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
		m_friction = 0.2f;
		m_bevel = 0.0f;
		m_useChain = true;

		CreateScene();
		Launch();
	}

	void CreateScene()
	{
		if (B2_IS_NON_NULL(m_groundId))
		{
			b2DestroyBody(m_groundId);
		}

		m_shapeId = b2_nullShapeId;

		b2BodyDef bodyDef = b2DefaultBodyDef();
		m_groundId = b2CreateBody(m_worldId, &bodyDef);

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

			b2ChainDef chainDef = b2_defaultChainDef;
			chainDef.points = points;
			chainDef.count = 20;
			chainDef.loop = true;
			chainDef.friction = m_friction;

			b2CreateChain(m_groundId, &chainDef);
		}
		else
		{
			b2ShapeDef shapeDef = b2DefaultShapeDef();
			shapeDef.friction = m_friction;

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
				b2Polygon polygon = b2MakeOffsetPolygon(&hull, 0.0f, transform);
				b2CreatePolygonShape(m_groundId, &shapeDef, &polygon);
				x -= 2.0f * m * hx;
				y += 2.0f * m * hx;
			}
			{
				transform.p = {x, y};
				b2Polygon polygon = b2MakeOffsetPolygon(&hull, 0.0f, transform);
				b2CreatePolygonShape(m_groundId, &shapeDef, &polygon);
				x -= 2.0f * m * hx;
				y += 2.0f * m * hx;
			}
			{
				transform.p = {x, y};
				b2Polygon polygon = b2MakeOffsetPolygon(&hull, 0.0f, transform);
				b2CreatePolygonShape(m_groundId, &shapeDef, &polygon);
				x -= 2.0f * m * hx;
				y += 2.0f * m * hx;
			}

			x = -2.0f * hx;
			y = 0.0f;
			transform.q = b2MakeRot(0.0f);

			{
				transform.p = {x, y};
				b2Polygon polygon = b2MakeOffsetPolygon(&hull, 0.0f, transform);
				b2CreatePolygonShape(m_groundId, &shapeDef, &polygon);
				x += 2.0f * hx;
			}
			{
				transform.p = {x, y};
				b2Polygon polygon = b2MakeOffsetPolygon(&hull, 0.0f, transform);
				b2CreatePolygonShape(m_groundId, &shapeDef, &polygon);
				x += 2.0f * hx;
			}
			{
				transform.p = {x, y};
				b2Polygon polygon = b2MakeOffsetPolygon(&hull, 0.0f, transform);
				b2CreatePolygonShape(m_groundId, &shapeDef, &polygon);
				x += 2.0f * hx;
			}

			x = 3.0f * hx + m * hx + m * hy;
			y = hy + m * hx - m * hy;
			transform.q = b2MakeRot(0.25f * b2_pi);

			{
				transform.p = {x, y};
				b2Polygon polygon = b2MakeOffsetPolygon(&hull, 0.0f, transform);
				b2CreatePolygonShape(m_groundId, &shapeDef, &polygon);
				x += 2.0f * m * hx;
				y += 2.0f * m * hx;
			}
			{
				transform.p = {x, y};
				b2Polygon polygon = b2MakeOffsetPolygon(&hull, 0.0f, transform);
				b2CreatePolygonShape(m_groundId, &shapeDef, &polygon);
				x += 2.0f * m * hx;
				y += 2.0f * m * hx;
			}
			{
				transform.p = {x, y};
				b2Polygon polygon = b2MakeOffsetPolygon(&hull, 0.0f, transform);
				b2CreatePolygonShape(m_groundId, &shapeDef, &polygon);
				x += 2.0f * m * hx;
				y += 2.0f * m * hx;
			}
		}
	}

	void Launch()
	{
		if (B2_IS_NON_NULL(m_bodyId))
		{
			b2DestroyBody(m_bodyId);
			m_shapeId = b2_nullShapeId;
		}

		b2BodyDef bodyDef = b2DefaultBodyDef();
		bodyDef.type = b2_dynamicBody;
		bodyDef.position = {-28.0f, 18.0f};
		bodyDef.linearVelocity = {0.0f, 0.0f};
		m_bodyId = b2CreateBody(m_worldId, &bodyDef);

		b2ShapeDef shapeDef = b2DefaultShapeDef();
		shapeDef.density = 1.0f;
		shapeDef.friction = m_friction;

		if (m_shapeType == e_circleShape)
		{
			b2Circle circle = {{0.0f, 0.0f}, 0.5f};
			m_shapeId = b2CreateCircleShape(m_bodyId, &shapeDef, &circle);
		}
		else if (m_shapeType == e_capsuleShape)
		{
			b2Capsule capsule = {{-0.5f, 0.0f}, {0.5f, 0.0}, 0.25f};
			m_shapeId = b2CreateCapsuleShape(m_bodyId, &shapeDef, &capsule);
		}
		else
		{
			float h = 0.5f - m_round;
			b2Polygon box = b2MakeRoundedBox(h, h, m_round);
			m_shapeId = b2CreatePolygonShape(m_bodyId, &shapeDef, &box);
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
			if (ImGui::SliderFloat("Bevel", &m_bevel, 0.0f, 1.0f, "%.2f"))
			{
				CreateScene();
			}
		}

		{
			const char* shapeTypes[] = {"Circle", "Capsule", "Box"};
			int shapeType = int(m_shapeType);
			ImGui::Combo("Shape", &shapeType, shapeTypes, IM_ARRAYSIZE(shapeTypes));
			m_shapeType = ShapeType(shapeType);
		}

		if (m_shapeType == e_boxShape)
		{
			ImGui::SliderFloat("Round", &m_round, 0.0f, 0.4f, "%.1f");
		}

		if (ImGui::SliderFloat("Friction", &m_friction, 0.0f, 1.0f, "%.1f"))
		{
			if (B2_IS_NON_NULL(m_shapeId))
			{
				b2Shape_SetFriction(m_shapeId, m_friction);
			}

			CreateScene();
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
