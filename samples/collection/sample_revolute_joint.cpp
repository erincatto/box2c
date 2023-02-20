// SPDX-FileCopyrightText: 2022 Erin Catto
// SPDX-License-Identifier: MIT

#include "box2d/box2d.h"
#include "box2d/geometry.h"
#include "sample.h"
#include "settings.h"

#include <GLFW/glfw3.h>
#include <imgui.h>

class RevoluteJoint : public Sample
{
public:
	RevoluteJoint()
	{
		b2BodyId ground = b2_nullBodyId;
		{
			b2BodyDef bd = b2DefaultBodyDef();
			bd.position = {0.0f, -1.0f};
			ground = b2World_CreateBody(m_worldId, &bd);

			b2Polygon box = b2MakeBox(40.0f, 1.0f);

			b2ShapeDef sd = b2DefaultShapeDef();
			b2Body_CreatePolygon(ground, &sd, &box);
		}

		m_enableLimit = true;
		m_enableMotor = false;
		m_motorSpeed = 1.0f;

		{
			b2Polygon box = b2MakeOffsetBox(0.25f, 3.0f, {0.0f, 3.0f}, 0.0f);

			b2BodyDef bd = b2DefaultBodyDef();
			bd.type = b2_dynamicBody;
			bd.position = {-10.0f, 20.0f};
			b2BodyId body = b2World_CreateBody(m_worldId, &bd);
			
			b2ShapeDef sd = b2DefaultShapeDef();
			sd.density = 5.0f;
			b2Body_CreatePolygon(body, &sd, &box);

			b2Vec2 pivot = {-10.0f, 20.5f};
			b2RevoluteJointDef jd = b2DefaultRevoluteJointDef();
			jd.bodyIdA = ground;
			jd.bodyIdB = body;
			jd.localAnchorA = b2Body_GetLocalPoint(jd.bodyIdA, pivot);
			jd.localAnchorB = b2Body_GetLocalPoint(jd.bodyIdB, pivot);
			jd.motorSpeed = m_motorSpeed;
			jd.maxMotorTorque = 10000.0f;
			jd.enableMotor = m_enableMotor;
			jd.lowerAngle = -0.25f * b2_pi;
			jd.upperAngle = 0.5f * b2_pi;
			jd.enableLimit = m_enableLimit;

			m_joint1 = b2World_CreateRevoluteJoint(m_worldId, &jd);
		}

		{
			b2Circle circle = {0};
			circle.radius = 2.0f;

			b2BodyDef bd = b2DefaultBodyDef();
			bd.type = b2_dynamicBody;
			bd.position = {5.0f, 30.0f};
			m_ball = b2World_CreateBody(m_worldId, &bd);

			b2ShapeDef sd = b2DefaultShapeDef();
			sd.density = 5.0f;

			b2Body_CreateCircle(m_ball, &sd, &circle);
		}

		{
			b2BodyDef bd = b2DefaultBodyDef();
			bd.position = {20.0f, 10.0f};
			bd.type = b2_dynamicBody;
			b2BodyId body = b2World_CreateBody(m_worldId, &bd);

			b2Polygon box = b2MakeOffsetBox(10.0f, 0.5f, {-10.0f, 0.0f}, 0.0f);
			b2ShapeDef sd = b2DefaultShapeDef();
			sd.density = 2.0f;
			b2Body_CreatePolygon(body, &sd, &box);

			b2Vec2 pivot = {19.0f, 10.0f};
			b2RevoluteJointDef jd = b2DefaultRevoluteJointDef();
			jd.bodyIdA = ground;
			jd.bodyIdB = body;
			jd.localAnchorA = b2Body_GetLocalPoint(jd.bodyIdA, pivot);
			jd.localAnchorB = b2Body_GetLocalPoint(jd.bodyIdB, pivot);
			jd.lowerAngle = -0.25f * b2_pi;
			jd.upperAngle = 0.0f * b2_pi;
			jd.enableLimit = true;
			jd.enableMotor = true;
			jd.motorSpeed = 0.0f;
			jd.maxMotorTorque = 10000.0f;

			m_joint2 = b2World_CreateRevoluteJoint(m_worldId, &jd);
		}
	}

	void UpdateUI() override
	{
		ImGui::SetNextWindowPos(ImVec2(10.0f, 100.0f));
		ImGui::SetNextWindowSize(ImVec2(200.0f, 100.0f));
		ImGui::Begin("Joint Controls", nullptr, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize);

		if (ImGui::Checkbox("Limit", &m_enableLimit))
		{
			b2RevoluteJoint_EnableLimit(m_joint1, m_enableLimit);
		}

		if (ImGui::Checkbox("Motor", &m_enableMotor))
		{
			b2RevoluteJoint_EnableMotor(m_joint1, m_enableMotor);
		}

		if (ImGui::SliderFloat("Speed", &m_motorSpeed, -20.0f, 20.0f, "%.0f"))
		{
			b2RevoluteJoint_SetMotorSpeed(m_joint1, m_motorSpeed);
		}

		ImGui::End();
	}

	void Step(Settings& settings) override
	{
		Sample::Step(settings);
		
		float torque1 = b2RevoluteJoint_GetMotorTorque(m_joint1, settings.m_hertz);
		g_draw.DrawString(5, m_textLine, "Motor Torque 1= %4.0f", torque1);
		m_textLine += m_textIncrement;

		float torque2 = b2RevoluteJoint_GetMotorTorque(m_joint2, settings.m_hertz);
		g_draw.DrawString(5, m_textLine, "Motor Torque 2= %4.0f", torque2);
		m_textLine += m_textIncrement;
	}

	static Sample* Create()
	{
		return new RevoluteJoint;
	}

	b2BodyId m_ball;
	b2JointId m_joint1;
	b2JointId m_joint2;
	float m_motorSpeed;
	bool m_enableMotor;
	bool m_enableLimit;
};

static int sampleIndex = RegisterSample("Joints", "Revolute", RevoluteJoint::Create);
