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
			b2BodyDef bodyDef = b2DefaultBodyDef();
			groundId = b2World_CreateBody(m_worldId, &bodyDef);

			b2Segment segment = {{-20.0f, 0.0f}, {20.0f, 0.0f}};
			b2ShapeDef shapeDef = b2DefaultShapeDef();
			b2Body_CreateSegment(groundId, &shapeDef, &segment);
		}

#if 1
		// Define attachment
		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			bodyDef.type = b2_dynamicBody;
			bodyDef.position = {0.0f, 3.0f};
			m_attachmentId = b2World_CreateBody(m_worldId, &bodyDef);

			b2Polygon box = b2MakeBox(0.5f, 2.0f);
			b2ShapeDef shapeDef = b2DefaultShapeDef();
			shapeDef.density = 1.0f;
			b2Body_CreatePolygon(m_attachmentId, &shapeDef, &box);
		}
#endif

		// Define platform
		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			bodyDef.type = b2_dynamicBody;
			bodyDef.position = {-4.0f, 5.0f};
			m_platformId = b2World_CreateBody(m_worldId, &bodyDef);

			b2Polygon box = b2MakeOffsetBox(0.5f, 4.0f, {4.0f, 0.0f}, 0.5f * b2_pi);

			b2ShapeDef shapeDef = b2DefaultShapeDef();
			shapeDef.friction = 0.6f;
			shapeDef.density = 2.0f;
			b2Body_CreatePolygon(m_platformId, &shapeDef, &box);

#if 1
			b2RevoluteJointDef revoluteDef = b2DefaultRevoluteJointDef();
			b2Vec2 pivot = {0.0f, 5.0f};
			revoluteDef.bodyIdA = m_attachmentId;
			revoluteDef.bodyIdB = m_platformId;
			revoluteDef.localAnchorA = b2Body_GetLocalPoint(m_attachmentId, pivot);
			revoluteDef.localAnchorB = b2Body_GetLocalPoint(m_platformId, pivot);
			revoluteDef.maxMotorTorque = 50.0f;
			revoluteDef.enableMotor = true;
			b2World_CreateRevoluteJoint(m_worldId, &revoluteDef);
#endif

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

#if 1
		// Create a payload
		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			bodyDef.type = b2_dynamicBody;
			bodyDef.position = {0.0f, 8.0f};
			b2BodyId bodyId = b2World_CreateBody(m_worldId, &bodyDef);

			b2Polygon box = b2MakeBox(0.75f, 0.75f);

			b2ShapeDef shapeDef = b2DefaultShapeDef();
			shapeDef.friction = 0.6f;
			shapeDef.density = 2.0f;

			b2Body_CreatePolygon(bodyId, &shapeDef, &box);
		}
#endif
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


// Test all these APIs:
#if 0
void b2Body_SetTransform(b2BodyId bodyId, b2Vec2 position, float angle);
float b2Body_GetMass(b2BodyId bodyId);
float b2Body_GetInertiaTensor(b2BodyId bodyId);
float b2Body_GetCenterOfMass(b2BodyId bodyId);
void b2Body_SetMassData(b2MassData massData);
void b2Body_Wake(b2BodyId bodyId);
void b2Body_Disable(b2BodyId bodyId);
void b2Body_Enable(b2BodyId bodyId);
#endif