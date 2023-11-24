// SPDX-FileCopyrightText: 2022 Erin Catto
// SPDX-License-Identifier: MIT

#include "sample.h"

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

// TODO_ERIN fix bounce tunnel
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
		b2Vec2 points[] = {{-20.58325, 14.54175}, {-21.90625, 15.8645},		 {-24.552, 17.1875},
						   {-27.198, 11.89575},	  {-29.84375, 15.8645},		 {-29.84375, 21.15625},
						   {-25.875, 23.802},	  {-20.58325, 25.125},		 {-25.875, 29.09375},
						   {-20.58325, 31.7395},  {-11.0089998, 23.2290001}, {-8.67700005, 21.15625},
						   {-6.03125, 21.15625},  {-7.35424995, 29.09375},	 {-3.38549995, 29.09375},
						   {1.90625, 30.41675},	  {5.875, 17.1875},			 {11.16675, 25.125},
						   {9.84375, 29.09375},	  {13.8125, 31.7395},		 {21.75, 30.41675},
						   {28.3644981, 26.448},  {25.71875, 18.5105},		 {24.3957481, 13.21875},
						   {17.78125, 11.89575},  {15.1355, 7.92700005},	 {5.875, 9.25},
						   {1.90625, 11.89575},	  {-3.25, 11.89575},		 {-3.25, 9.9375},
						   {-4.70825005, 9.25},	  {-8.67700005, 9.25},		 {-11.323, 11.89575},
						   {-13.96875, 11.89575}, {-15.29175, 14.54175},	 {-19.2605, 14.54175}};

		b2ChainDef chainDef = b2DefaultChainDef();
		chainDef.points = points;
		chainDef.count = sizeof(points) / sizeof(points[0]);
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
		bodyDef.position = {0.0f, 20.0f};
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
		ImGui::SetNextWindowPos(ImVec2(10.0f, 300.0f), ImGuiCond_Once);
		ImGui::SetNextWindowSize(ImVec2(240.0f, 160.0f));
		ImGui::Begin("Options", nullptr, ImGuiWindowFlags_NoResize);

		const char* shapeTypes[] = {"Circle", "Capsule", "Box"};
		int shapeType = int(m_shapeType);
		if (ImGui::Combo("Shape", &shapeType, shapeTypes, IM_ARRAYSIZE(shapeTypes)))
		{
			m_shapeType = ShapeType(shapeType);
			Launch();
		}

		if (ImGui::SliderFloat("Friction", &m_friction, 0.0f, 1.0f, "%.1f"))
		{
			b2Shape_SetFriction(m_shapeId, m_friction);
			b2Chain_SetFriction(m_chainId, m_friction);
		}

		if (ImGui::SliderFloat("Restitution", &m_restitution, 0.0f, 2.0f, "%.1f"))
		{
			b2Shape_SetRestitution(m_shapeId, m_restitution);
		}

		ImGui::End();
	}

	void Step(Settings& settings) override
	{
		Sample::Step(settings);
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
