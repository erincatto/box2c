// SPDX-FileCopyrightText: 2022 Erin Catto
// SPDX-License-Identifier: MIT

#include "sample.h"
#include "settings.h"

#include "box2d/box2d.h"
#include "box2d/geometry.h"
#include "box2d/math.h"

#include <GLFW/glfw3.h>
#include <imgui.h>

struct RayCastContext
{
	b2Vec2 point;
	b2Vec2 normal;
	bool hit;
};

static float RayCastClosestCallback(b2ShapeId shapeId, b2Vec2 point, b2Vec2 normal, float fraction, void* context)
{
	RayCastContext* rayContext = (RayCastContext*)context;
	rayContext->point = point;
	rayContext->normal = normal;
	rayContext->hit = true;
	return fraction;
}

class ChainShape : public Sample
{
public:
	enum ShapeType
	{
		e_circleShape = 0,
		e_capsuleShape,
		e_boxShape
	};

	ChainShape(const Settings& settings)
		: Sample(settings)
	{
		if (settings.restart == false)
		{
			g_camera.m_center = {0.0f, 0.0f};
			g_camera.m_zoom = 1.75f;
		}

		m_groundId = b2_nullBodyId;
		m_bodyId = b2_nullBodyId;
		m_chainId = b2_nullChainId;
		m_shapeId = b2_nullShapeId;
		m_shapeType = e_circleShape;
		m_restitution = 0.0f;
		m_friction = 0.2f;
		CreateScene();
		Launch();
	}

	void CreateScene()
	{
		if (B2_NON_NULL(m_groundId))
		{
			b2World_DestroyBody(m_groundId);
		}

		// https://betravis.github.io/shape-tools/path-to-polygon/
		// b2Vec2 points[] = {{-20.58325, 14.54175}, {-21.90625, 15.8645},		 {-24.552, 17.1875},
		//				   {-27.198, 11.89575},	  {-29.84375, 15.8645},		 {-29.84375, 21.15625},
		//				   {-25.875, 23.802},	  {-20.58325, 25.125},		 {-25.875, 29.09375},
		//				   {-20.58325, 31.7395},  {-11.0089998, 23.2290001}, {-8.67700005, 21.15625},
		//				   {-6.03125, 21.15625},  {-7.35424995, 29.09375},	 {-3.38549995, 29.09375},
		//				   {1.90625, 30.41675},	  {5.875, 17.1875},			 {11.16675, 25.125},
		//				   {9.84375, 29.09375},	  {13.8125, 31.7395},		 {21.75, 30.41675},
		//				   {28.3644981, 26.448},  {25.71875, 18.5105},		 {24.3957481, 13.21875},
		//				   {17.78125, 11.89575},  {15.1355, 7.92700005},	 {5.875, 9.25},
		//				   {1.90625, 11.89575},	  {-3.25, 11.89575},		 {-3.25, 9.9375},
		//				   {-4.70825005, 9.25},	  {-8.67700005, 9.25},		 {-11.323, 11.89575},
		//				   {-13.96875, 11.89575}, {-15.29175, 14.54175},	 {-19.2605, 14.54175}};

		b2Vec2 points[] = {
			{-56.885498, 12.8985004},	{-56.885498, 16.2057495},	{56.885498, 16.2057495},   {56.885498, -16.2057514},
			{51.5935059, -16.2057514},	{43.6559982, -10.9139996},	{35.7184982, -10.9139996}, {27.7809982, -10.9139996},
			{21.1664963, -14.2212505},	{11.9059982, -16.2057514},	{0, -16.2057514},		   {-10.5835037, -14.8827496},
			{-17.1980019, -13.5597477}, {-21.1665001, -12.2370014}, {-25.1355019, -9.5909977}, {-31.75, -3.63799858},
			{-38.3644981, 6.2840004},	{-42.3334999, 9.59125137},	{-47.625, 11.5755005},	   {-56.885498, 12.8985004},
		};

		int count = sizeof(points) / sizeof(points[0]);

		// float scale = 0.25f;
		// b2Vec2 lower = {FLT_MAX, FLT_MAX};
		// b2Vec2 upper = {-FLT_MAX, -FLT_MAX};
		// for (int i = 0; i < count; ++i)
		//{
		//	points[i].x = 2.0f * scale * points[i].x;
		//	points[i].y = -scale * points[i].y;

		//	lower = b2Min(lower, points[i]);
		//	upper = b2Max(upper, points[i]);
		//}

		// b2Vec2 center = b2MulSV(0.5f, b2Add(lower, upper));
		// for (int i = 0; i < count; ++i)
		//{
		//	points[i] = b2Sub(points[i], center);
		// }

		// for (int i = 0; i < count / 2; ++i)
		//{
		//	b2Vec2 temp = points[i];
		//	points[i] = points[count - 1 - i];
		//	points[count - 1 - i] = temp;
		// }

		// printf("{");
		// for (int i = 0; i < count; ++i)
		//{
		//	printf("{%.9g, %.9g},", points[i].x, points[i].y);
		// }
		// printf("};\n");

		b2ChainDef chainDef = b2DefaultChainDef();
		chainDef.points = points;
		chainDef.count = count;
		chainDef.loop = true;
		chainDef.friction = 0.2f;

		b2BodyDef bodyDef = b2DefaultBodyDef();
		m_groundId = b2World_CreateBody(m_worldId, &bodyDef);

		m_chainId = b2Body_CreateChain(m_groundId, &chainDef);
	}

	void Launch()
	{
		if (B2_NON_NULL(m_bodyId))
		{
			b2World_DestroyBody(m_bodyId);
		}

		b2BodyDef bodyDef = b2DefaultBodyDef();
		bodyDef.type = b2_dynamicBody;
		bodyDef.position = {-55.0f, 13.5f};
		m_bodyId = b2World_CreateBody(m_worldId, &bodyDef);

		b2ShapeDef shapeDef = b2DefaultShapeDef();
		shapeDef.density = 1.0f;
		shapeDef.friction = m_friction;
		shapeDef.restitution = m_restitution;

		if (m_shapeType == e_circleShape)
		{
			b2Circle circle = {{0.0f, 0.0f}, 0.5f};
			m_shapeId = b2Body_CreateCircle(m_bodyId, &shapeDef, &circle);
		}
		else if (m_shapeType == e_capsuleShape)
		{
			b2Capsule capsule = {{-0.5f, 0.0f}, {0.5f, 0.0}, 0.25f};
			m_shapeId = b2Body_CreateCapsule(m_bodyId, &shapeDef, &capsule);
		}
		else
		{
			float h = 0.5f;
			b2Polygon box = b2MakeBox(h, h);
			m_shapeId = b2Body_CreatePolygon(m_bodyId, &shapeDef, &box);
		}
	}

	void UpdateUI() override
	{
		ImGui::SetNextWindowPos(ImVec2(10.0f, 200.0f), ImGuiCond_Once);
		ImGui::SetNextWindowSize(ImVec2(280.0f, 125.0f));
		ImGui::Begin("Options", nullptr, ImGuiWindowFlags_NoResize);

		const char* shapeTypes[] = {"Circle", "Capsule", "Box"};
		int shapeType = int(m_shapeType);
		if (ImGui::Combo("Shape", &shapeType, shapeTypes, IM_ARRAYSIZE(shapeTypes)))
		{
			m_shapeType = ShapeType(shapeType);
			Launch();
		}

		if (ImGui::SliderFloat("Friction", &m_friction, 0.0f, 1.0f, "%.2f"))
		{
			b2Shape_SetFriction(m_shapeId, m_friction);
			b2Chain_SetFriction(m_chainId, m_friction);
		}

		if (ImGui::SliderFloat("Restitution", &m_restitution, 0.0f, 2.0f, "%.1f"))
		{
			b2Shape_SetRestitution(m_shapeId, m_restitution);
		}

		if (ImGui::Button("Launch"))
		{
			Launch();
		}

		ImGui::End();
	}

	void Step(Settings& settings) override
	{
		Sample::Step(settings);

		g_draw.DrawSegment(b2Vec2_zero, {0.5f, 0.0f}, b2MakeColor(b2_colorRed, 1.0f));
		g_draw.DrawSegment(b2Vec2_zero, {0.0f, 0.5f}, b2MakeColor(b2_colorGreen, 1.0f));
	}

	static Sample* Create(const Settings& settings)
	{
		return new ChainShape(settings);
	}

	b2BodyId m_groundId;
	b2BodyId m_bodyId;
	b2ChainId m_chainId;
	ShapeType m_shapeType;
	b2ShapeId m_shapeId;
	float m_restitution;
	float m_friction;
};

static int sampleChainShape = RegisterSample("Shapes", "Chain Shape", ChainShape::Create);

class CompoundShapes : public Sample
{
public:
	CompoundShapes(const Settings& settings)
		: Sample(settings)
	{
		if (settings.restart == false)
		{
			g_camera.m_center = {0.0f, 0.0f};
			g_camera.m_zoom = 1.75f;
		}

		{
			b2BodyDef bd;
			bd.position.Set(0.0f, 0.0f);
			b2Body* body = m_world->CreateBody(&bd);

			b2EdgeShape shape;
			shape.SetTwoSided(b2Vec2(50.0f, 0.0f), b2Vec2(-50.0f, 0.0f));

			body->CreateFixture(&shape, 0.0f);
		}

		// Table 1
		{
			b2BodyDef bd;
			bd.type = b2_dynamicBody;
			bd.position.Set(-15.0f, 1.0f);
			m_table1 = m_world->CreateBody(&bd);

			b2PolygonShape top;
			top.SetAsBox(3.0f, 0.5f, b2Vec2(0.0f, 3.5f), 0.0f);

			b2PolygonShape leftLeg;
			leftLeg.SetAsBox(0.5f, 1.5f, b2Vec2(-2.5f, 1.5f), 0.0f);

			b2PolygonShape rightLeg;
			rightLeg.SetAsBox(0.5f, 1.5f, b2Vec2(2.5f, 1.5f), 0.0f);

			m_table1->CreateFixture(&top, 2.0f);
			m_table1->CreateFixture(&leftLeg, 2.0f);
			m_table1->CreateFixture(&rightLeg, 2.0f);
		}

		// Table 2
		{
			b2BodyDef bd;
			bd.type = b2_dynamicBody;
			bd.position.Set(-5.0f, 1.0f);
			m_table2 = m_world->CreateBody(&bd);

			b2PolygonShape top;
			top.SetAsBox(3.0f, 0.5f, b2Vec2(0.0f, 3.5f), 0.0f);

			b2PolygonShape leftLeg;
			leftLeg.SetAsBox(0.5f, 2.0f, b2Vec2(-2.5f, 2.0f), 0.0f);

			b2PolygonShape rightLeg;
			rightLeg.SetAsBox(0.5f, 2.0f, b2Vec2(2.5f, 2.0f), 0.0f);

			m_table2->CreateFixture(&top, 2.0f);
			m_table2->CreateFixture(&leftLeg, 2.0f);
			m_table2->CreateFixture(&rightLeg, 2.0f);
		}

		// Spaceship 1
		{
			b2BodyDef bd;
			bd.type = b2_dynamicBody;
			bd.position.Set(5.0f, 1.0f);
			m_ship1 = m_world->CreateBody(&bd);

			b2Vec2 vertices[3];

			b2PolygonShape left;
			vertices[0].Set(-2.0f, 0.0f);
			vertices[1].Set(0.0f, 4.0f / 3.0f);
			vertices[2].Set(0.0f, 4.0f);
			left.Set(vertices, 3);

			b2PolygonShape right;
			vertices[0].Set(2.0f, 0.0f);
			vertices[1].Set(0.0f, 4.0f / 3.0f);
			vertices[2].Set(0.0f, 4.0f);
			right.Set(vertices, 3);

			m_ship1->CreateFixture(&left, 2.0f);
			m_ship1->CreateFixture(&right, 2.0f);
		}

		// Spaceship 2
		{
			b2BodyDef bd;
			bd.type = b2_dynamicBody;
			bd.position.Set(15.0f, 1.0f);
			m_ship2 = m_world->CreateBody(&bd);

			b2Vec2 vertices[3];

			b2PolygonShape left;
			vertices[0].Set(-2.0f, 0.0f);
			vertices[1].Set(1.0f, 2.0f);
			vertices[2].Set(0.0f, 4.0f);
			left.Set(vertices, 3);

			b2PolygonShape right;
			vertices[0].Set(2.0f, 0.0f);
			vertices[1].Set(-1.0f, 2.0f);
			vertices[2].Set(0.0f, 4.0f);
			right.Set(vertices, 3);

			m_ship2->CreateFixture(&left, 2.0f);
			m_ship2->CreateFixture(&right, 2.0f);
		}
	}

	void Spawn()
	{
		// Table 1 obstruction
		{
			b2BodyDef bd;
			bd.type = b2_dynamicBody;
			bd.position = m_table1->GetPosition();
			bd.angle = m_table1->GetAngle();

			b2Body* body = m_world->CreateBody(&bd);

			b2PolygonShape box;
			box.SetAsBox(4.0f, 0.1f, b2Vec2(0.0f, 3.0f), 0.0f);

			body->CreateFixture(&box, 2.0f);
		}

		// Table 2 obstruction
		{
			b2BodyDef bd;
			bd.type = b2_dynamicBody;
			bd.position = m_table2->GetPosition();
			bd.angle = m_table2->GetAngle();

			b2Body* body = m_world->CreateBody(&bd);

			b2PolygonShape box;
			box.SetAsBox(4.0f, 0.1f, b2Vec2(0.0f, 3.0f), 0.0f);

			body->CreateFixture(&box, 2.0f);
		}

		// Ship 1 obstruction
		{
			b2BodyDef bd;
			bd.type = b2_dynamicBody;
			bd.position = m_ship1->GetPosition();
			bd.angle = m_ship1->GetAngle();
			bd.gravityScale = 0.0f;

			b2Body* body = m_world->CreateBody(&bd);

			b2CircleShape circle;
			circle.m_radius = 0.5f;
			circle.m_p.Set(0.0f, 2.0f);

			body->CreateFixture(&circle, 2.0f);
		}

		// Ship 2 obstruction
		{
			b2BodyDef bd;
			bd.type = b2_dynamicBody;
			bd.position = m_ship2->GetPosition();
			bd.angle = m_ship2->GetAngle();
			bd.gravityScale = 0.0f;

			b2Body* body = m_world->CreateBody(&bd);

			b2CircleShape circle;
			circle.m_radius = 0.5f;
			circle.m_p.Set(0.0f, 2.0f);

			body->CreateFixture(&circle, 2.0f);
		}
	}

	void UpdateUI() override
	{
		ImGui::SetNextWindowPos(ImVec2(10.0f, 100.0f));
		ImGui::SetNextWindowSize(ImVec2(200.0f, 100.0f));
		ImGui::Begin("Controls", nullptr, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize);

		if (ImGui::Button("Spawn"))
		{
			Spawn();
		}

		ImGui::End();
	}

	static Sample* Create(const Settings& settings)
	{
		return new CompoundShapes(settings);
	}

	b2BodyId m_table1;
	b2BodyId m_table2;
	b2BodyId m_ship1;
	b2BodyId m_ship2;
};

static int sampleCompoundShape = RegisterSample("Shapes", "Compound Shapes", CompoundShapes::Create);
