// SPDX-FileCopyrightText: 2022 Erin Catto
// SPDX-License-Identifier: MIT

#include "sample.h"

#include "box2d/box2d.h"
#include "box2d/geometry.h"
#include "box2d/hull.h"

#include <imgui.h>

class BodyType : public Sample
{
  public:
	BodyType(const Settings& settings)
		: Sample(settings)
	{
		b2BodyId groundId = b2_nullBodyId;
		{
			b2BodyDef bodyDef = b2_defaultBodyDef;
			groundId = b2World_CreateBody(m_worldId, &bodyDef);

			b2Segment segment = {{-20.0f, 0.0f}, {20.0f, 0.0f}};
			b2ShapeDef shapeDef = b2_defaultShapeDef;
			b2Body_CreateSegment(groundId, &shapeDef, &segment);
		}

		// Define attachment
		{
			b2BodyDef bodyDef = b2_defaultBodyDef;
			bodyDef.type = b2_dynamicBody;
			bodyDef.position = {0.0f, 3.0f};
			m_attachmentId = b2World_CreateBody(m_worldId, &bodyDef);

			b2Polygon box = b2MakeBox(0.5f, 2.0f);
			b2ShapeDef shapeDef = b2_defaultShapeDef;
			shapeDef.density = 1.0f;
			b2Body_CreatePolygon(m_attachmentId, &shapeDef, &box);
		}

		// Define platform
		{
			b2BodyDef bodyDef = b2_defaultBodyDef;
			bodyDef.type = b2_dynamicBody;
			bodyDef.position = {-4.0f, 5.0f};
			m_platformId = b2World_CreateBody(m_worldId, &bodyDef);

			b2Polygon box = b2MakeOffsetBox(0.5f, 4.0f, {4.0f, 0.0f}, 0.5f * b2_pi);

			b2ShapeDef shapeDef = b2_defaultShapeDef;
			shapeDef.friction = 0.6f;
			shapeDef.density = 2.0f;
			b2Body_CreatePolygon(m_platformId, &shapeDef, &box);

			b2RevoluteJointDef revoluteDef = b2DefaultRevoluteJointDef();
			b2Vec2 pivot = {0.0f, 5.0f};
			revoluteDef.bodyIdA = m_attachmentId;
			revoluteDef.bodyIdB = m_platformId;
			revoluteDef.localAnchorA = b2Body_GetLocalPoint(m_attachmentId, pivot);
			revoluteDef.localAnchorB = b2Body_GetLocalPoint(m_platformId, pivot);
			revoluteDef.maxMotorTorque = 50.0f;
			revoluteDef.enableMotor = true;
			b2World_CreateRevoluteJoint(m_worldId, &revoluteDef);

			b2PrismaticJointDef prismaticDef = b2DefaultPrismaticJointDef();
			b2Vec2 anchor = {0.0f, 5.0f};
			prismaticDef.bodyIdA = groundId;
			prismaticDef.bodyIdB = m_platformId;
			prismaticDef.localAnchorA = b2Body_GetLocalPoint(groundId, anchor);
			prismaticDef.localAnchorB = b2Body_GetLocalPoint(m_platformId, anchor);
			prismaticDef.localAxisA = {1.0f, 0.0f};
			prismaticDef.maxMotorForce = 1000.0f;
			prismaticDef.motorSpeed = 0.0f;
			prismaticDef.enableMotor = true;
			prismaticDef.lowerTranslation = -10.0f;
			prismaticDef.upperTranslation = 10.0f;
			prismaticDef.enableLimit = true;

			b2World_CreatePrismaticJoint(m_worldId, &prismaticDef);

			m_speed = 3.0f;
		}

		// Create a payload
		{
			b2BodyDef bodyDef = b2_defaultBodyDef;
			bodyDef.type = b2_dynamicBody;
			bodyDef.position = {0.0f, 8.0f};
			b2BodyId bodyId = b2World_CreateBody(m_worldId, &bodyDef);

			b2Polygon box = b2MakeBox(0.75f, 0.75f);

			b2ShapeDef shapeDef = b2_defaultShapeDef;
			shapeDef.friction = 0.6f;
			shapeDef.density = 2.0f;

			b2Body_CreatePolygon(bodyId, &shapeDef, &box);
		}
	}

	void UpdateUI() override
	{
		ImGui::SetNextWindowPos(ImVec2(10.0f, 400.0f));
		ImGui::SetNextWindowSize(ImVec2(200.0f, 150.0f));
		ImGui::Begin("Controls", nullptr, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize);

		b2BodyType bodyType = b2Body_GetType(m_platformId);

		if (ImGui::RadioButton("Static", bodyType == b2_staticBody))
		{
			b2Body_SetType(m_platformId, b2_staticBody);
		}

		if (ImGui::RadioButton("Kinematic", bodyType == b2_kinematicBody))
		{
			b2Body_SetType(m_platformId, b2_kinematicBody);
			b2Body_SetLinearVelocity(m_platformId, {-m_speed, 0.0f});
			b2Body_SetAngularVelocity(m_platformId, 0.0f);
		}
		
		if (ImGui::RadioButton("Dynamic", bodyType == b2_dynamicBody))
		{
			b2Body_SetType(m_platformId, b2_dynamicBody);
		}

		bool isEnabled = b2Body_IsEnabled(m_platformId);
		if (ImGui::Checkbox("Enable", &isEnabled))
		{
			if (isEnabled)
			{
				b2Body_Enable(m_platformId);
			}
			else
			{
				b2Body_Disable(m_platformId);
			}
		}

		ImGui::End();
	}

	void Step(Settings& settings) override
	{
		// Drive the kinematic body.
		if (b2Body_GetType(m_platformId) == b2_kinematicBody)
		{
			b2Vec2 p = b2Body_GetPosition(m_platformId);
			b2Vec2 v = b2Body_GetLinearVelocity(m_platformId);

			if ((p.x < -14.0f && v.x < 0.0f) || (p.x > 6.0f && v.x > 0.0f))
			{
				v.x = -v.x;
				b2Body_SetLinearVelocity(m_platformId, v);
			}
		}

		Sample::Step(settings);
	}

	static Sample* Create(const Settings& settings)
	{
		return new BodyType(settings);
	}

	b2BodyId m_attachmentId;
	b2BodyId m_platformId;
	float m_speed;
};

static int sampleBodyType = RegisterSample("Bodies", "Body Type", BodyType::Create);


/// This is a test of typical character collision scenarios. This does not
/// show how you should implement a character in your application.
/// Instead this is used to test smooth collision on edge chains.
class Character : public Sample
{
public:
	Character(const Settings& settings)
		: Sample(settings)
	{
		// Ground body
		{
			b2BodyId groundId = b2World_CreateBody(m_worldId, &b2_defaultBodyDef);

			b2Segment segment = {{-20.0f, 0.0f}, {20.0f, 0.0f}};
			b2Body_CreateSegment(groundId, &b2_defaultShapeDef, &segment);
		}

		// Collinear edges with no adjacency information.
		// This shows the problematic case where a box shape can hit
		// an internal vertex.
		{
			b2BodyId groundId = b2World_CreateBody(m_worldId, &b2_defaultBodyDef);
			
			b2Segment segment1 = {{-8.0f, 1.0f}, {-6.0f, 1.0f}};
			b2Body_CreateSegment(groundId, &b2_defaultShapeDef, &segment1);

			b2Segment segment2 = {{-6.0f, 1.0f}, {-4.0f, 1.0f}};
			b2Body_CreateSegment(groundId, &b2_defaultShapeDef, &segment2);

			b2Segment segment3 = {{-4.0f, 1.0f}, {-2.0f, 1.0f}};
			b2Body_CreateSegment(groundId, &b2_defaultShapeDef, &segment3);
		}

		// Chain shape
		{
			b2BodyDef bodyDef = b2_defaultBodyDef;
			bodyDef.angle = 0.25f * b2_pi;
			b2BodyId groundId = b2World_CreateBody(m_worldId, &bodyDef);

			b2Vec2 points[4] = {{8.0f, 7.0f}, {7.0f, 8.0f}, {6.0f, 8.0f}, {5.0f, 7.0f}};
			b2ChainDef chainDef = b2DefaultChainDef();
			chainDef.points = points;
			chainDef.count = 4;
			chainDef.loop = true;

			b2Body_CreateChain(groundId, &chainDef);
		}

		// Square tiles. This shows that adjacency shapes may have non-smooth collision. Box2D has no solution
		// to this problem.
		// TODO_ERIN try this: https://briansemrau.github.io/dealing-with-ghost-collisions/
		{
			b2BodyId groundId = b2World_CreateBody(m_worldId, &b2_defaultBodyDef);

			b2Polygon box = b2MakeOffsetBox(1.0f, 1.0f, {4.0f, 3.0f}, 0.0f);
			b2Body_CreatePolygon(groundId, &b2_defaultShapeDef, &box);

			box = b2MakeOffsetBox(1.0f, 1.0f, {6.0f, 3.0f}, 0.0f);
			b2Body_CreatePolygon(groundId, &b2_defaultShapeDef, &box);

			box = b2MakeOffsetBox(1.0f, 1.0f, {8.0f, 3.0f}, 0.0f);
			b2Body_CreatePolygon(groundId, &b2_defaultShapeDef, &box);
		}

		// Square made from a chain loop. Collision should be smooth.
		{
			b2BodyId groundId = b2World_CreateBody(m_worldId, &b2_defaultBodyDef);

			b2Vec2 points[4] = {{-1.0f, 3.0}, {1.0f, 3.0f}, {1.0f, 5.0f}, {-1.0f, 5.0}};
			b2ChainDef chainDef = b2DefaultChainDef();
			chainDef.points = points;
			chainDef.count = 4;
			chainDef.loop = true;
			b2Body_CreateChain(groundId, &chainDef);
		}

		// Chain loop. Collision should be smooth.
		{
			b2BodyDef bodyDef = b2_defaultBodyDef;
			bodyDef.position = {-10.0f, 4.0f};
			b2BodyId groundId = b2World_CreateBody(m_worldId, &bodyDef);

			b2Vec2 points[10] = {{0.0f, 0.0f}, {6.0f, 0.0f},  {6.0f, 2.0f},	 {4.0f, 1.0f},	{2.0f, 2.0f},
								 {0.0f, 2.0f}, {-2.0f, 2.0f}, {-4.0f, 3.0f}, {-6.0f, 2.0f}, {-6.0f, 0.0f}};
			b2ChainDef chainDef = b2DefaultChainDef();
			chainDef.points = points;
			chainDef.count = 10;
			chainDef.loop = true;
			b2Body_CreateChain(groundId, &chainDef);
		}

		// Circle character
		{
			b2BodyDef bodyDef = b2_defaultBodyDef;
			bodyDef.position = {-7.0f, 6.0f};
			bodyDef.type = b2_dynamicBody;
			bodyDef.fixedRotation = true;
			bodyDef.enableSleep = false;

			m_circleCharacterId = b2World_CreateBody(m_worldId, &bodyDef);

			b2Circle circle = {{0.0f, 0.0f}, 0.25f};

			b2ShapeDef shapeDef = b2_defaultShapeDef;
			shapeDef.density = 20.0f;
			shapeDef.friction = 0.2f;
			b2Body_CreateCircle(m_circleCharacterId, &shapeDef, &circle);
		}

		// Capsule character
		{
			b2BodyDef bodyDef = b2_defaultBodyDef;
			bodyDef.position = {3.0f, 5.0f};
			bodyDef.type = b2_dynamicBody;
			bodyDef.fixedRotation = true;
			bodyDef.enableSleep = false;

			m_capsuleCharacterId = b2World_CreateBody(m_worldId, &bodyDef);

			b2Capsule capsule = {{0.0f, 0.25f}, {0.0f, 0.75f}, 0.25f};

			b2ShapeDef shapeDef = b2_defaultShapeDef;
			shapeDef.density = 20.0f;
			shapeDef.friction = 0.2f;
			b2Body_CreateCapsule(m_capsuleCharacterId, &shapeDef, &capsule);
		}

		// Square character
		{
			b2BodyDef bodyDef = b2_defaultBodyDef;
			bodyDef.position = {-3.0f, 8.0f};
			bodyDef.type = b2_dynamicBody;
			bodyDef.fixedRotation = true;
			bodyDef.enableSleep = false;

			m_boxCharacterId = b2World_CreateBody(m_worldId, &bodyDef);

			b2Polygon box = b2MakeBox(0.4f, 0.4f);

			b2ShapeDef shapeDef = b2_defaultShapeDef;
			shapeDef.density = 20.0f;
			shapeDef.friction = 0.2f;
			b2Body_CreatePolygon(m_boxCharacterId, &shapeDef, &box);
		}
	}

	void Step(Settings& settings) override
	{
		Sample::Step(settings);

		g_draw.DrawString(5, m_textLine, "This tests various character collision shapes.");
		m_textLine += m_textIncrement;
		g_draw.DrawString(5, m_textLine, "Limitation: square and hexagon can snag on aligned boxes.");
		m_textLine += m_textIncrement;
		g_draw.DrawString(5, m_textLine, "Feature: edge chains have smooth collision inside and out.");
		m_textLine += m_textIncrement;
	}

	static Sample* Create(const Settings& settings)
	{
		return new Character(settings);
	}

	b2BodyId m_circleCharacterId;
	b2BodyId m_capsuleCharacterId;
	b2BodyId m_boxCharacterId;
};

static int sampleCharacter = RegisterSample("Bodies", "Character", Character::Create);

class Weeble : public Sample
{
public:
	Weeble(const Settings& settings)
		: Sample(settings)
	{
		b2BodyId groundId = b2_nullBodyId;
		{
			b2BodyDef bodyDef = b2_defaultBodyDef;
			groundId = b2World_CreateBody(m_worldId, &bodyDef);

			b2Segment segment = {{-20.0f, 0.0f}, {20.0f, 0.0f}};
			b2ShapeDef shapeDef = b2_defaultShapeDef;
			b2Body_CreateSegment(groundId, &shapeDef, &segment);
		}

		// Build weeble
		{
			b2BodyDef bodyDef = b2_defaultBodyDef;
			bodyDef.type = b2_dynamicBody;
			bodyDef.position = {0.0f, 3.0f};
			m_weebleId = b2World_CreateBody(m_worldId, &bodyDef);

			b2Capsule capsule = {{0.0f, -1.0f}, {0.0f, 1.0f}, 1.0f};
			b2ShapeDef shapeDef = b2_defaultShapeDef;
			shapeDef.density = 1.0f;
			b2Body_CreateCapsule(m_weebleId, &shapeDef, &capsule);

			float mass = b2Body_GetMass(m_weebleId);
			float I = b2Body_GetInertiaTensor(m_weebleId);

			b2MassData massData = {mass, {0.0f, -1.5f}, I, 2.0f, 4.0f};
			b2Body_SetMassData(m_weebleId, massData);
		}
	}

	void UpdateUI() override
	{
		ImGui::SetNextWindowPos(ImVec2(10.0f, 400.0f));
		ImGui::SetNextWindowSize(ImVec2(200.0f, 60.0f));
		ImGui::Begin("Sample Controls", nullptr, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize);

		if (ImGui::Button("Teleport"))
		{
			b2Body_SetTransform(m_weebleId, {0.0f, 5.0f}, 0.95 * b2_pi);
		}

		ImGui::End();
	}

	static Sample* Create(const Settings& settings)
	{
		return new Weeble(settings);
	}

	b2BodyId m_weebleId;
};

static int sampleWeeble = RegisterSample("Bodies", "Weeble", Weeble::Create);
