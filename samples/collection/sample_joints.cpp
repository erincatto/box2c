// SPDX-FileCopyrightText: 2022 Erin Catto
// SPDX-License-Identifier: MIT

#include "sample.h"
#include "settings.h"

#include "box2d/box2d.h"
#include "box2d/geometry.h"
#include "box2d/hull.h"
#include "box2d/joint_util.h"

// #include <GLFW/glfw3.h>
#include "box2d/math.h"

#include <imgui.h>

class RevoluteJoint : public Sample
{
public:
	RevoluteJoint(const Settings& settings)
		: Sample(settings)
	{
		b2BodyId groundId = b2_nullBodyId;
		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			bodyDef.position = {0.0f, -1.0f};
			groundId = b2World_CreateBody(m_worldId, &bodyDef);

			b2Polygon box = b2MakeBox(40.0f, 1.0f);

			b2ShapeDef shapeDef = b2DefaultShapeDef();
			b2Body_CreatePolygon(groundId, &shapeDef, &box);
		}

		m_enableLimit = true;
		m_enableMotor = false;
		m_motorSpeed = 1.0f;
		m_motorTorque = 100.0f;

		{
			b2Polygon box = b2MakeOffsetBox(0.25f, 3.0f, {0.0f, 3.0f}, 0.0f);

			b2BodyDef bodyDef = b2DefaultBodyDef();
			bodyDef.type = b2_dynamicBody;
			bodyDef.position = {-10.0f, 20.0f};
			b2BodyId bodyId = b2World_CreateBody(m_worldId, &bodyDef);

			b2ShapeDef shapeDef = b2DefaultShapeDef();
			shapeDef.density = 5.0f;
			b2Body_CreatePolygon(bodyId, &shapeDef, &box);

			b2Vec2 pivot = {-10.0f, 20.5f};
			b2RevoluteJointDef jointDef = b2DefaultRevoluteJointDef();
			jointDef.bodyIdA = groundId;
			jointDef.bodyIdB = bodyId;
			jointDef.localAnchorA = b2Body_GetLocalPoint(jointDef.bodyIdA, pivot);
			jointDef.localAnchorB = b2Body_GetLocalPoint(jointDef.bodyIdB, pivot);
			jointDef.motorSpeed = m_motorSpeed;
			jointDef.maxMotorTorque = m_motorTorque;
			jointDef.enableMotor = m_enableMotor;
			jointDef.lowerAngle = -0.25f * b2_pi;
			jointDef.upperAngle = 0.5f * b2_pi;
			jointDef.enableLimit = m_enableLimit;

			m_jointId1 = b2World_CreateRevoluteJoint(m_worldId, &jointDef);
		}

		{
			b2Circle circle = {0};
			circle.radius = 2.0f;

			b2BodyDef bodyDef = b2DefaultBodyDef();
			bodyDef.type = b2_dynamicBody;
			bodyDef.position = {5.0f, 30.0f};
			m_ball = b2World_CreateBody(m_worldId, &bodyDef);

			b2ShapeDef shapeDef = b2DefaultShapeDef();
			shapeDef.density = 5.0f;

			b2Body_CreateCircle(m_ball, &shapeDef, &circle);
		}

		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			bodyDef.position = {20.0f, 10.0f};
			bodyDef.type = b2_dynamicBody;
			b2BodyId body = b2World_CreateBody(m_worldId, &bodyDef);

			b2Polygon box = b2MakeOffsetBox(10.0f, 0.5f, {-10.0f, 0.0f}, 0.0f);
			b2ShapeDef shapeDef = b2DefaultShapeDef();
			shapeDef.density = 2.0f;
			b2Body_CreatePolygon(body, &shapeDef, &box);

			b2Vec2 pivot = {19.0f, 10.0f};
			b2RevoluteJointDef jointDef = b2DefaultRevoluteJointDef();
			jointDef.bodyIdA = groundId;
			jointDef.bodyIdB = body;
			jointDef.localAnchorA = b2Body_GetLocalPoint(jointDef.bodyIdA, pivot);
			jointDef.localAnchorB = b2Body_GetLocalPoint(jointDef.bodyIdB, pivot);
			jointDef.lowerAngle = -0.25f * b2_pi;
			jointDef.upperAngle = 0.0f * b2_pi;
			jointDef.enableLimit = true;
			jointDef.enableMotor = true;
			jointDef.motorSpeed = 0.0f;
			jointDef.maxMotorTorque = m_motorTorque;

			m_jointId2 = b2World_CreateRevoluteJoint(m_worldId, &jointDef);
		}
	}

	void UpdateUI() override
	{
		ImGui::SetNextWindowPos(ImVec2(10.0f, 100.0f));
		ImGui::SetNextWindowSize(ImVec2(250.0f, 140.0f));
		ImGui::Begin("Joint Controls", nullptr, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize);

		if (ImGui::Checkbox("Limit", &m_enableLimit))
		{
			b2RevoluteJoint_EnableLimit(m_jointId1, m_enableLimit);
		}

		if (ImGui::Checkbox("Motor", &m_enableMotor))
		{
			b2RevoluteJoint_EnableMotor(m_jointId1, m_enableMotor);
		}

		if (ImGui::SliderFloat("Max Torque", &m_motorTorque, 0.0f, 500.0f, "%.0f"))
		{
			b2RevoluteJoint_SetMaxMotorTorque(m_jointId1, m_motorTorque);
		}

		if (ImGui::SliderFloat("Speed", &m_motorSpeed, -20.0f, 20.0f, "%.0f"))
		{
			b2RevoluteJoint_SetMotorSpeed(m_jointId1, m_motorSpeed);
		}

		ImGui::End();
	}

	void Step(Settings& settings) override
	{
		Sample::Step(settings);

		float torque1 = b2RevoluteJoint_GetMotorTorque(m_jointId1, settings.hertz);
		g_draw.DrawString(5, m_textLine, "Motor Torque 1 = %4.1f", torque1);
		m_textLine += m_textIncrement;

		float torque2 = b2RevoluteJoint_GetMotorTorque(m_jointId2, settings.hertz);
		g_draw.DrawString(5, m_textLine, "Motor Torque 2 = %4.1f", torque2);
		m_textLine += m_textIncrement;
	}

	static Sample* Create(const Settings& settings)
	{
		return new RevoluteJoint(settings);
	}

	b2BodyId m_ball;
	b2JointId m_jointId1;
	b2JointId m_jointId2;
	float m_motorSpeed;
	float m_motorTorque;
	bool m_enableMotor;
	bool m_enableLimit;
};

static int sampleRevolute = RegisterSample("Joints", "Revolute", RevoluteJoint::Create);

class PrismaticJoint : public Sample
{
public:
	PrismaticJoint(const Settings& settings)
		: Sample(settings)
	{
		if (settings.restart == false)
		{
			g_camera.m_center = {0.0f, 8.0f};
			g_camera.m_zoom = 0.5f;
		}

		b2BodyId groundId = b2World_CreateBody(m_worldId, &b2_defaultBodyDef);

		m_enableLimit = true;
		m_enableMotor = false;
		m_motorSpeed = 2.0f;
		m_motorForce = 25.0f;

		{
			b2Polygon box = b2MakeBox(0.5f, 2.0f);

			b2BodyDef bodyDef = b2_defaultBodyDef;
			bodyDef.position = {0.0f, 10.0f};
			bodyDef.type = b2_dynamicBody;
			b2BodyId bodyId = b2World_CreateBody(m_worldId, &bodyDef);

			b2Body_CreatePolygon(bodyId, &b2_defaultShapeDef, &box);

			b2Vec2 pivot = {0.0f, 9.0f};
			b2Vec2 axis = b2Normalize({1.0f, 1.0f});
			b2PrismaticJointDef jointDef = b2DefaultPrismaticJointDef();
			jointDef.bodyIdA = groundId;
			jointDef.bodyIdB = bodyId;
			jointDef.localAxisA = b2Body_GetLocalVector(jointDef.bodyIdA, axis);
			jointDef.localAnchorA = b2Body_GetLocalPoint(jointDef.bodyIdA, pivot);
			jointDef.localAnchorB = b2Body_GetLocalPoint(jointDef.bodyIdB, pivot);
			jointDef.motorSpeed = m_motorSpeed;
			jointDef.maxMotorForce = m_motorForce;
			jointDef.enableMotor = m_enableMotor;
			jointDef.lowerTranslation = -10.0f;
			jointDef.upperTranslation = 10.0f;
			jointDef.enableLimit = m_enableLimit;

			m_jointId = b2World_CreatePrismaticJoint(m_worldId, &jointDef);
		}
	}

	void UpdateUI() override
	{
		ImGui::SetNextWindowPos(ImVec2(10.0f, 100.0f));
		ImGui::SetNextWindowSize(ImVec2(250.0f, 140.0f));
		ImGui::Begin("Joint Controls", nullptr, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize);

		if (ImGui::Checkbox("Limit", &m_enableLimit))
		{
			b2PrismaticJoint_EnableLimit(m_jointId, m_enableLimit);
		}

		if (ImGui::Checkbox("Motor", &m_enableMotor))
		{
			b2PrismaticJoint_EnableMotor(m_jointId, m_enableMotor);
		}

		if (ImGui::SliderFloat("Max Force", &m_motorForce, 0.0f, 50.0f, "%.0f"))
		{
			b2PrismaticJoint_SetMaxMotorForce(m_jointId, m_motorForce);
		}

		if (ImGui::SliderFloat("Speed", &m_motorSpeed, -20.0f, 20.0f, "%.0f"))
		{
			b2PrismaticJoint_SetMotorSpeed(m_jointId, m_motorSpeed);
		}

		ImGui::End();
	}

	void Step(Settings& settings) override
	{
		Sample::Step(settings);

		float force = b2PrismaticJoint_GetMotorForce(m_jointId, settings.hertz);
		g_draw.DrawString(5, m_textLine, "Motor Force = %4.1f", force);
		m_textLine += m_textIncrement;
	}

	static Sample* Create(const Settings& settings)
	{
		return new PrismaticJoint(settings);
	}

	b2JointId m_jointId;
	float m_motorSpeed;
	float m_motorForce;
	bool m_enableMotor;
	bool m_enableLimit;
};

static int samplePrismatic = RegisterSample("Joints", "Prismatic", PrismaticJoint::Create);

class WheelJoint : public Sample
{
public:
	WheelJoint(const Settings& settings)
		: Sample(settings)
	{
		if (settings.restart == false)
		{
			g_camera.m_center = {0.0f, 10.0f};
			g_camera.m_zoom = 0.25f;
		}

		b2BodyId groundId = b2World_CreateBody(m_worldId, &b2_defaultBodyDef);

		m_enableLimit = true;
		m_enableMotor = false;
		m_motorSpeed = 2.0f;
		m_motorTorque = 5.0f;

		{
			b2Capsule capsule = {{0.0f, -0.5f}, {0.0f, 0.5f}, 0.5f};

			b2BodyDef bodyDef = b2_defaultBodyDef;
			bodyDef.position = {0.0f, 10.25f};
			bodyDef.type = b2_dynamicBody;
			b2BodyId bodyId = b2World_CreateBody(m_worldId, &bodyDef);

			b2Body_CreateCapsule(bodyId, &b2_defaultShapeDef, &capsule);

			float hertz = 1.0f;
			float dampingRatio = 0.7f;
			b2LinearStiffness(&m_stiffness, &m_damping, hertz, dampingRatio, groundId, bodyId);

			b2Vec2 pivot = {0.0f, 10.0f};
			b2Vec2 axis = b2Normalize({1.0f, 1.0f});
			b2WheelJointDef jointDef = b2DefaultWheelJointDef();
			jointDef.bodyIdA = groundId;
			jointDef.bodyIdB = bodyId;
			jointDef.localAxisA = b2Body_GetLocalVector(jointDef.bodyIdA, axis);
			jointDef.localAnchorA = b2Body_GetLocalPoint(jointDef.bodyIdA, pivot);
			jointDef.localAnchorB = b2Body_GetLocalPoint(jointDef.bodyIdB, pivot);
			jointDef.motorSpeed = m_motorSpeed;
			jointDef.maxMotorTorque = m_motorTorque;
			jointDef.enableMotor = m_enableMotor;
			jointDef.lowerTranslation = -3.0f;
			jointDef.upperTranslation = 3.0f;
			jointDef.enableLimit = m_enableLimit;
			jointDef.stiffness = m_stiffness;
			jointDef.damping = m_damping;

			m_jointId = b2World_CreateWheelJoint(m_worldId, &jointDef);
		}
	}

	void UpdateUI() override
	{
		ImGui::SetNextWindowPos(ImVec2(10.0f, 100.0f));
		ImGui::SetNextWindowSize(ImVec2(250.0f, 180.0f));
		ImGui::Begin("Wheel Joint", nullptr, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize);

		if (ImGui::Checkbox("Limit", &m_enableLimit))
		{
			b2WheelJoint_EnableLimit(m_jointId, m_enableLimit);
		}

		if (ImGui::Checkbox("Motor", &m_enableMotor))
		{
			b2WheelJoint_EnableMotor(m_jointId, m_enableMotor);
		}

		if (ImGui::SliderFloat("Torque", &m_motorTorque, 0.0f, 20.0f, "%.0f"))
		{
			b2WheelJoint_SetMaxMotorTorque(m_jointId, m_motorTorque);
		}

		if (ImGui::SliderFloat("Speed", &m_motorSpeed, -20.0f, 20.0f, "%.0f"))
		{
			b2WheelJoint_SetMotorSpeed(m_jointId, m_motorSpeed);
		}

		if (ImGui::SliderFloat("Stiffness", &m_stiffness, 0.0f, 100.0f, "%.0f"))
		{
			b2WheelJoint_SetStiffness(m_jointId, m_stiffness);
		}

		if (ImGui::SliderFloat("Damping", &m_damping, 0.0f, 50.0f, "%.0f"))
		{
			b2WheelJoint_SetDamping(m_jointId, m_damping);
		}

		ImGui::End();
	}

	void Step(Settings& settings) override
	{
		Sample::Step(settings);

		float torque = b2WheelJoint_GetMotorTorque(m_jointId, settings.hertz);
		g_draw.DrawString(5, m_textLine, "Motor Torque = %4.1f", torque);
		m_textLine += m_textIncrement;
	}

	static Sample* Create(const Settings& settings)
	{
		return new WheelJoint(settings);
	}

	b2JointId m_jointId;
	float m_stiffness;
	float m_damping;
	float m_motorSpeed;
	float m_motorTorque;
	bool m_enableMotor;
	bool m_enableLimit;
};

static int sampleWheel = RegisterSample("Joints", "Wheel", WheelJoint::Create);

// A suspension bridge
class Bridge : public Sample
{
public:
	enum
	{
		e_count = 160
	};

	Bridge(const Settings& settings)
		: Sample(settings)
	{
		b2BodyId groundId = b2_nullBodyId;
		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			groundId = b2World_CreateBody(m_worldId, &bodyDef);
		}

		{
			b2Polygon box = b2MakeBox(0.5f, 0.125f);

			b2ShapeDef shapeDef = b2DefaultShapeDef();
			shapeDef.density = 20.0f;

			b2RevoluteJointDef jointDef = b2DefaultRevoluteJointDef();
			int32_t jointIndex = 0;
			m_frictionTorque = 200.0f;

			b2BodyId prevBodyId = groundId;
			for (int32_t i = 0; i < e_count; ++i)
			{
				b2BodyDef bodyDef = b2DefaultBodyDef();
				bodyDef.type = b2_dynamicBody;
				bodyDef.position = {-34.5f + 1.0f * i, 20.0f};
				// bodyDef.linearDamping = 0.1f;
				// bodyDef.angularDamping = 0.1f;
				b2BodyId bodyId = b2World_CreateBody(m_worldId, &bodyDef);
				b2Body_CreatePolygon(bodyId, &shapeDef, &box);

				b2Vec2 pivot = {-35.0f + 1.0f * i, 20.0f};
				jointDef.bodyIdA = prevBodyId;
				jointDef.bodyIdB = bodyId;
				jointDef.localAnchorA = b2Body_GetLocalPoint(jointDef.bodyIdA, pivot);
				jointDef.localAnchorB = b2Body_GetLocalPoint(jointDef.bodyIdB, pivot);
				jointDef.enableMotor = true;
				jointDef.maxMotorTorque = m_frictionTorque;
				m_jointIds[jointIndex++] = b2World_CreateRevoluteJoint(m_worldId, &jointDef);

				prevBodyId = bodyId;
			}

			b2Vec2 pivot = {-35.0f + 1.0f * e_count, 20.0f};
			jointDef.bodyIdA = prevBodyId;
			jointDef.bodyIdB = groundId;
			jointDef.localAnchorA = b2Body_GetLocalPoint(jointDef.bodyIdA, pivot);
			jointDef.localAnchorB = b2Body_GetLocalPoint(jointDef.bodyIdB, pivot);
			jointDef.enableMotor = true;
			jointDef.maxMotorTorque = m_frictionTorque;
			m_jointIds[jointIndex++] = b2World_CreateRevoluteJoint(m_worldId, &jointDef);

			assert(jointIndex == e_count + 1);
		}

		for (int32_t i = 0; i < 2; ++i)
		{
			b2Vec2 vertices[3] = {{-0.5f, 0.0f}, {0.5f, 0.0f}, {0.0f, 1.5f}};

			b2Hull hull = b2ComputeHull(vertices, 3);
			b2Polygon triangle = b2MakePolygon(&hull, 0.0f);

			b2ShapeDef shapeDef = b2DefaultShapeDef();
			shapeDef.density = 20.0f;

			b2BodyDef bodyDef = b2DefaultBodyDef();
			bodyDef.type = b2_dynamicBody;
			bodyDef.position = {-8.0f + 8.0f * i, 22.0f};
			b2BodyId bodyId = b2World_CreateBody(m_worldId, &bodyDef);
			b2Body_CreatePolygon(bodyId, &shapeDef, &triangle);
		}

		for (int32_t i = 0; i < 3; ++i)
		{
			b2Circle circle = {{0.0f, 0.0f}, 0.5f};

			b2ShapeDef shapeDef = b2DefaultShapeDef();
			shapeDef.density = 20.0f;

			b2BodyDef bodyDef = b2DefaultBodyDef();
			bodyDef.type = b2_dynamicBody;
			bodyDef.position = {-6.0f + 6.0f * i, 25.0f};
			b2BodyId bodyId = b2World_CreateBody(m_worldId, &bodyDef);
			b2Body_CreateCircle(bodyId, &shapeDef, &circle);
		}
	}

	void UpdateUI() override
	{
		ImGui::SetNextWindowPos(ImVec2(10.0f, 300.0f), ImGuiCond_Once);

		// Automatic window size
		ImGui::Begin("Options", nullptr, ImGuiWindowFlags_AlwaysAutoResize);

		// Slider takes half the window
		ImGui::PushItemWidth(ImGui::GetWindowWidth() * 0.5f);
		bool updateFriction = ImGui::SliderFloat("Joint Friction", &m_frictionTorque, 0.0f, 1000.0f, "%2.f");
		if (updateFriction)
		{
			for (int32_t i = 0; i <= e_count; ++i)
			{
				b2RevoluteJoint_SetMaxMotorTorque(m_jointIds[i], m_frictionTorque);
			}
		}

		ImGui::End();
	}

	static Sample* Create(const Settings& settings)
	{
		return new Bridge(settings);
	}

	b2JointId m_jointIds[e_count + 1];
	float m_frictionTorque;
};

static int sampleBridgeIndex = RegisterSample("Joints", "Bridge", Bridge::Create);

class BallAndChain : public Sample
{
public:
	enum
	{
		e_count = 30
	};

	BallAndChain(const Settings& settings)
		: Sample(settings)
	{
		b2BodyId groundId = b2_nullBodyId;
		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			groundId = b2World_CreateBody(m_worldId, &bodyDef);
		}

		m_frictionTorque = 100.0f;

		{
			float hx = 0.5f;
			b2Capsule capsule = {{-hx, 0.0f}, {hx, 0.0f}, 0.125f};

			b2ShapeDef shapeDef = b2DefaultShapeDef();
			shapeDef.density = 20.0f;

			b2RevoluteJointDef jointDef = b2DefaultRevoluteJointDef();

			int32_t jointIndex = 0;

			b2BodyId prevBodyId = groundId;
			for (int32_t i = 0; i < e_count; ++i)
			{
				b2BodyDef bodyDef = b2DefaultBodyDef();
				bodyDef.type = b2_dynamicBody;
				bodyDef.position = {(1.0f + 2.0f * i) * hx, e_count * hx};
				b2BodyId bodyId = b2World_CreateBody(m_worldId, &bodyDef);
				b2Body_CreateCapsule(bodyId, &shapeDef, &capsule);

				b2Vec2 pivot = {(2.0f * i) * hx, e_count * hx};
				jointDef.bodyIdA = prevBodyId;
				jointDef.bodyIdB = bodyId;
				jointDef.localAnchorA = b2Body_GetLocalPoint(jointDef.bodyIdA, pivot);
				jointDef.localAnchorB = b2Body_GetLocalPoint(jointDef.bodyIdB, pivot);
				jointDef.enableMotor = true;
				jointDef.maxMotorTorque = m_frictionTorque;
				m_jointIds[jointIndex++] = b2World_CreateRevoluteJoint(m_worldId, &jointDef);

				prevBodyId = bodyId;
			}

			b2Circle circle = {{0.0f, 0.0f}, 4.0f};

			b2BodyDef bodyDef = b2DefaultBodyDef();
			bodyDef.type = b2_dynamicBody;
			bodyDef.position = {(1.0f + 2.0f * e_count) * hx + circle.radius - hx, e_count * hx};

			b2BodyId bodyId = b2World_CreateBody(m_worldId, &bodyDef);
			b2Body_CreateCircle(bodyId, &shapeDef, &circle);

			b2Vec2 pivot = {(2.0f * e_count) * hx, e_count * hx};
			jointDef.bodyIdA = prevBodyId;
			jointDef.bodyIdB = bodyId;
			jointDef.localAnchorA = b2Body_GetLocalPoint(jointDef.bodyIdA, pivot);
			jointDef.localAnchorB = b2Body_GetLocalPoint(jointDef.bodyIdB, pivot);
			jointDef.enableMotor = true;
			jointDef.maxMotorTorque = m_frictionTorque;
			m_jointIds[jointIndex++] = b2World_CreateRevoluteJoint(m_worldId, &jointDef);
			assert(jointIndex == e_count + 1);
		}
	}

	void UpdateUI() override
	{
		ImGui::SetNextWindowPos(ImVec2(10.0f, 300.0f), ImGuiCond_Once);
		ImGui::SetNextWindowSize(ImVec2(300.0f, 60.0f));
		ImGui::Begin("Options", nullptr, ImGuiWindowFlags_NoResize);

		bool updateFriction = ImGui::SliderFloat("Joint Friction", &m_frictionTorque, 0.0f, 1000.0f, "%2.f");
		if (updateFriction)
		{
			for (int32_t i = 0; i <= e_count; ++i)
			{
				b2RevoluteJoint_SetMaxMotorTorque(m_jointIds[i], m_frictionTorque);
			}
		}

		ImGui::End();
	}

	static Sample* Create(const Settings& settings)
	{
		return new BallAndChain(settings);
	}

	b2JointId m_jointIds[e_count + 1];
	float m_frictionTorque;
};

static int sampleBallAndChainIndex = RegisterSample("Joints", "Ball & Chain", BallAndChain::Create);

class Cantilever : public Sample
{
public:
	enum
	{
		e_count = 8
	};

	Cantilever(const Settings& settings)
		: Sample(settings)
	{
		b2BodyId groundId = b2_nullBodyId;
		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			groundId = b2World_CreateBody(m_worldId, &bodyDef);
		}

		{
			float hx = 0.5f;
			b2Polygon box = b2MakeBox(hx, 0.125f);

			b2ShapeDef shapeDef = b2DefaultShapeDef();
			shapeDef.density = 20.0f;

			b2WeldJointDef jointDef = b2DefaultWeldJointDef();

			b2BodyId prevBodyId = groundId;
			for (int32_t i = 0; i < e_count; ++i)
			{
				b2BodyDef bodyDef = b2DefaultBodyDef();
				bodyDef.type = b2_dynamicBody;
				bodyDef.position = {(1.0f + 2.0f * i) * hx, 0.0f};
				b2BodyId bodyId = b2World_CreateBody(m_worldId, &bodyDef);
				b2Body_CreatePolygon(bodyId, &shapeDef, &box);

				b2Vec2 pivot = {(2.0f * i) * hx, 0.0f};
				jointDef.bodyIdA = prevBodyId;
				jointDef.bodyIdB = bodyId;
				jointDef.localAnchorA = b2Body_GetLocalPoint(jointDef.bodyIdA, pivot);
				jointDef.localAnchorB = b2Body_GetLocalPoint(jointDef.bodyIdB, pivot);
				// jointDef.angularHertz = i == 0 ? 0.0f : 1.0f;
				// jointDef.linearHertz = 5.0f;
				b2World_CreateWeldJoint(m_worldId, &jointDef);

				prevBodyId = bodyId;
			}

			m_tipId = prevBodyId;
		}
	}

	void Step(Settings& settings) override
	{
		Sample::Step(settings);

		b2Vec2 tipPosition = b2Body_GetPosition(m_tipId);
		g_draw.DrawString(5, m_textLine, "tip-y = %.2f", tipPosition.y);
		m_textLine += m_textIncrement;
	}

	static Sample* Create(const Settings& settings)
	{
		return new Cantilever(settings);
	}

	b2BodyId m_tipId;
};

static int sampleCantileverIndex = RegisterSample("Joints", "Cantilever", Cantilever::Create);

// Test the distance joint and all options
class DistanceJoint : public Sample
{
public:
	enum
	{
		e_maxCount = 10
	};

	DistanceJoint(const Settings& settings)
		: Sample(settings)
	{
		if (settings.restart == false)
		{
			g_camera.m_zoom = 0.25f;
		}

		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			m_groundId = b2World_CreateBody(m_worldId, &bodyDef);
		}

		m_count = 0;
		m_hertz = 2.0f;
		m_dampingRatio = 0.5f;
		m_length = 1.0f;
		m_minLength = 0.5f;
		m_maxLength = 2.0f;
		m_fixedLength = false;

		for (int i = 0; i < e_maxCount; ++i)
		{
			m_bodyIds[i] = b2_nullBodyId;
			m_jointIds[i] = b2_nullJointId;
		}

		CreateScene(1);
	}

	void CreateScene(int newCount)
	{
		// Must destroy joints before bodies
		for (int i = 0; i < m_count; ++i)
		{
			b2World_DestroyJoint(m_jointIds[i]);
			m_jointIds[i] = b2_nullJointId;
		}

		for (int i = 0; i < m_count; ++i)
		{
			b2World_DestroyBody(m_bodyIds[i]);
			m_bodyIds[i] = b2_nullBodyId;
		}

		m_count = newCount;

		float radius = 0.25f;
		b2Circle circle = {{0.0f, 0.0f}, radius};

		b2ShapeDef shapeDef = b2DefaultShapeDef();
		shapeDef.density = 20.0f;

		float yOffset = 20.0f;

		b2DistanceJointDef jointDef = b2DefaultDistanceJointDef();

		b2BodyId prevBodyId = m_groundId;
		for (int32_t i = 0; i < m_count; ++i)
		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			bodyDef.type = b2_dynamicBody;
			bodyDef.position = {m_length * (i + 1.0f), yOffset};
			m_bodyIds[i] = b2World_CreateBody(m_worldId, &bodyDef);
			b2Body_CreateCircle(m_bodyIds[i], &shapeDef, &circle);

			b2Vec2 pivotA = {m_length * i, yOffset};
			b2Vec2 pivotB = {m_length * (i + 1.0f), yOffset};
			jointDef.bodyIdA = prevBodyId;
			jointDef.bodyIdB = m_bodyIds[i];
			jointDef.localAnchorA = b2Body_GetLocalPoint(jointDef.bodyIdA, pivotA);
			jointDef.localAnchorB = b2Body_GetLocalPoint(jointDef.bodyIdB, pivotB);
			jointDef.hertz = m_hertz;
			jointDef.dampingRatio = m_dampingRatio;
			jointDef.length = m_length;
			jointDef.minLength = m_minLength;
			jointDef.maxLength = m_maxLength;
			jointDef.collideConnected = true;
			m_jointIds[i] = b2World_CreateDistanceJoint(m_worldId, &jointDef);

			prevBodyId = m_bodyIds[i];
		}
	}

	void UpdateUI() override
	{
		ImGui::SetNextWindowPos(ImVec2(10.0f, 300.0f), ImGuiCond_Once);
		ImGui::SetNextWindowSize(ImVec2(300.0f, 220.0f));
		ImGui::Begin("Options", nullptr, ImGuiWindowFlags_NoResize);

		if (ImGui::SliderFloat("length", &m_length, 0.1f, 4.0f, "%3.1f"))
		{
			if (m_fixedLength)
			{
				m_minLength = m_length;
				m_maxLength = m_length;
			}

			for (int32_t i = 0; i < m_count; ++i)
			{
				b2DistanceJoint_SetLength(m_jointIds[i], m_length, m_minLength, m_maxLength);
			}
		}

		if (ImGui::Checkbox("fixed length", &m_fixedLength))
		{
			if (m_fixedLength)
			{
				m_minLength = m_length;
				m_maxLength = m_length;
				for (int32_t i = 0; i < m_count; ++i)
				{
					b2DistanceJoint_SetLength(m_jointIds[i], m_length, m_minLength, m_maxLength);
				}
			}
		}

		if (m_fixedLength == false)
		{
			if (ImGui::SliderFloat("min length", &m_minLength, 0.1f, 4.0f, "%3.1f"))
			{
				for (int32_t i = 0; i < m_count; ++i)
				{
					b2DistanceJoint_SetLength(m_jointIds[i], m_length, m_minLength, m_maxLength);
				}
			}

			if (ImGui::SliderFloat("max length", &m_maxLength, 0.1f, 4.0f, "%3.1f"))
			{
				for (int32_t i = 0; i < m_count; ++i)
				{
					b2DistanceJoint_SetLength(m_jointIds[i], m_length, m_minLength, m_maxLength);
				}
			}

			if (ImGui::SliderFloat("hertz", &m_hertz, 0.0f, 15.0f, "%3.1f"))
			{
				for (int32_t i = 0; i < m_count; ++i)
				{
					b2DistanceJoint_SetTuning(m_jointIds[i], m_hertz, m_dampingRatio);
				}
			}

			if (ImGui::SliderFloat("damping", &m_dampingRatio, 0.0f, 4.0f, "%3.1f"))
			{
				for (int32_t i = 0; i < m_count; ++i)
				{
					b2DistanceJoint_SetTuning(m_jointIds[i], m_hertz, m_dampingRatio);
				}
			}
		}

		int count = m_count;
		if (ImGui::SliderInt("count", &count, 1, e_maxCount))
		{
			CreateScene(count);
		}

		ImGui::End();
	}

	static Sample* Create(const Settings& settings)
	{
		return new DistanceJoint(settings);
	}

	b2BodyId m_groundId;
	b2BodyId m_bodyIds[e_maxCount];
	b2JointId m_jointIds[e_maxCount];
	int32_t m_count;
	float m_hertz;
	float m_dampingRatio;
	float m_length;
	float m_minLength;
	float m_maxLength;
	bool m_fixedLength;
};

static int sampleDistanceJoint = RegisterSample("Joints", "Distance Joint", DistanceJoint::Create);

class AllJoints : public Sample
{
public:
	enum
	{
		e_count = 5
	};

	AllJoints(const Settings& settings)
		: Sample(settings)
	{
		m_groundId = b2World_CreateBody(m_worldId, &b2_defaultBodyDef);
		m_fixedRotation = false;

		for (int i = 0; i < 5; ++i)
		{
			m_bodyIds[i] = b2_nullBodyId;
			m_jointIds[i] = b2_nullJointId;
		}

		CreateScene();
	}

	void CreateScene()
	{
		for (int i = 0; i < 5; ++i)
		{
			if (B2_NON_NULL(m_jointIds[i]))
			{
				b2World_DestroyJoint(m_jointIds[i]);
				m_jointIds[i] = b2_nullJointId;
			}

			if (B2_NON_NULL(m_bodyIds[i]))
			{
				b2World_DestroyBody(m_bodyIds[i]);
				m_bodyIds[i] = b2_nullBodyId;
			}
		}

		b2Vec2 position = {-20.0f, 10.0f};
		b2BodyDef bodyDef = b2_defaultBodyDef;
		bodyDef.type = b2_dynamicBody;
		bodyDef.enableSleep = false;
		bodyDef.fixedRotation = m_fixedRotation;

		b2Polygon box = b2MakeBox(1.0f, 1.0f);
		b2ShapeDef shapeDef = b2_defaultShapeDef;
		shapeDef.density = 1.0f;

		int index = 0;
		{
			bodyDef.position = position;
			m_bodyIds[index] = b2World_CreateBody(m_worldId, &bodyDef);
			b2Body_CreatePolygon(m_bodyIds[index], &shapeDef, &box);

			b2Vec2 pivot = {position.x - 1.0f, position.y};
			b2WeldJointDef jointDef = b2DefaultWeldJointDef();
			jointDef.bodyIdA = m_groundId;
			jointDef.bodyIdB = m_bodyIds[index];
			jointDef.localAnchorA = b2Body_GetLocalPoint(jointDef.bodyIdA, pivot);
			jointDef.localAnchorB = b2Body_GetLocalPoint(jointDef.bodyIdB, pivot);
			m_jointIds[index] = b2World_CreateWeldJoint(m_worldId, &jointDef);
		}

		position.x += 5.0f;
		++index;

		{
			bodyDef.position = position;
			m_bodyIds[index] = b2World_CreateBody(m_worldId, &bodyDef);
			b2Body_CreatePolygon(m_bodyIds[index], &shapeDef, &box);

			b2Vec2 pivot = {position.x - 1.0f, position.y};
			b2RevoluteJointDef jointDef = b2DefaultRevoluteJointDef();
			jointDef.bodyIdA = m_groundId;
			jointDef.bodyIdB = m_bodyIds[index];
			jointDef.localAnchorA = b2Body_GetLocalPoint(jointDef.bodyIdA, pivot);
			jointDef.localAnchorB = b2Body_GetLocalPoint(jointDef.bodyIdB, pivot);
			m_jointIds[index] = b2World_CreateRevoluteJoint(m_worldId, &jointDef);
		}

		position.x += 5.0f;
		++index;

		{
			bodyDef.position = position;
			m_bodyIds[index] = b2World_CreateBody(m_worldId, &bodyDef);
			b2Body_CreatePolygon(m_bodyIds[index], &shapeDef, &box);

			b2Vec2 pivot = {position.x - 1.0f, position.y};
			b2PrismaticJointDef jointDef = b2DefaultPrismaticJointDef();
			jointDef.bodyIdA = m_groundId;
			jointDef.bodyIdB = m_bodyIds[index];
			jointDef.localAnchorA = b2Body_GetLocalPoint(jointDef.bodyIdA, pivot);
			jointDef.localAnchorB = b2Body_GetLocalPoint(jointDef.bodyIdB, pivot);
			jointDef.localAxisA = b2Body_GetLocalVector(jointDef.bodyIdA, {1.0f, 0.0f});
			m_jointIds[index] = b2World_CreatePrismaticJoint(m_worldId, &jointDef);
		}

		position.x += 5.0f;
		++index;

		{
			bodyDef.position = position;
			m_bodyIds[index] = b2World_CreateBody(m_worldId, &bodyDef);
			b2Body_CreatePolygon(m_bodyIds[index], &shapeDef, &box);

			float length = 2.0f;
			b2Vec2 pivot1 = {position.x, position.y + 1.0f + length};
			b2Vec2 pivot2 = {position.x, position.y + 1.0f};
			b2DistanceJointDef jointDef = b2DefaultDistanceJointDef();
			jointDef.bodyIdA = m_groundId;
			jointDef.bodyIdB = m_bodyIds[index];
			jointDef.localAnchorA = b2Body_GetLocalPoint(jointDef.bodyIdA, pivot1);
			jointDef.localAnchorB = b2Body_GetLocalPoint(jointDef.bodyIdB, pivot2);
			jointDef.minLength = length;
			jointDef.maxLength = length;
			m_jointIds[index] = b2World_CreateDistanceJoint(m_worldId, &jointDef);
		}
	}

	void UpdateUI() override
	{
		ImGui::SetNextWindowPos(ImVec2(10.0f, 300.0f), ImGuiCond_Once);
		ImGui::SetNextWindowSize(ImVec2(300.0f, 60.0f));
		ImGui::Begin("Options", nullptr, ImGuiWindowFlags_NoResize);

		if (ImGui::Checkbox("fixed rotation", &m_fixedRotation))
		{
			CreateScene();
		}

		ImGui::End();
	}

	static Sample* Create(const Settings& settings)
	{
		return new AllJoints(settings);
	}

	b2BodyId m_groundId;
	b2BodyId m_bodyIds[5];
	b2JointId m_jointIds[5];
	bool m_fixedRotation;
};

static int sampleAllJoints = RegisterSample("Joints", "All Joints", AllJoints::Create);

class UserConstraint : public Sample
{
public:
	UserConstraint(const Settings& settings)
		: Sample(settings)
	{
		b2Polygon box = b2MakeBox(1.0f, 0.5f);

		b2ShapeDef shapeDef = b2DefaultShapeDef();
		shapeDef.density = 20.0f;

		b2BodyDef bodyDef = b2DefaultBodyDef();
		bodyDef.type = b2_dynamicBody;
		bodyDef.gravityScale = 1.0f;
		bodyDef.angularDamping = 0.5f;
		bodyDef.linearDamping = 0.2f;
		m_bodyId = b2World_CreateBody(m_worldId, &bodyDef);
		b2Body_CreatePolygon(m_bodyId, &shapeDef, &box);

		m_impulses[0] = 0.0f;
		m_impulses[1] = 0.0f;
	}

	void Step(Settings& settings) override
	{
		Sample::Step(settings);

		b2Transform axes = b2Transform_identity;
		g_draw.DrawTransform(axes);

		if (settings.pause)
		{
			return;
		}

		float timeStep = settings.hertz > 0.0f ? 1.0f / settings.hertz : 0.0f;
		if (timeStep == 0.0f)
		{
			return;
		}

		float invTimeStep = settings.hertz;

		static float hertz = 3.0f;
		static float zeta = 0.7f;
		static float maxForce = 1000.0f;
		float omega = 2.0f * b2_pi * hertz;
		float sigma = 2.0f * zeta + timeStep * omega;
		float s = timeStep * omega * sigma;
		float impulseCoefficient = 1.0f / (1.0f + s);
		float massCoefficient = s * impulseCoefficient;
		float biasCoefficient = omega / sigma;

		b2Vec2 localAnchors[2] = {{1.0f, -0.5f}, {1.0f, 0.5f}};
		float mass = b2Body_GetMass(m_bodyId);
		float invMass = mass < 0.0001f ? 0.0f : 1.0f / mass;
		float I = b2Body_GetInertiaTensor(m_bodyId);
		float invI = I < 0.0001f ? 0.0f : 1.0f / I;

		b2Vec2 vB = b2Body_GetLinearVelocity(m_bodyId);
		float omegaB = b2Body_GetAngularVelocity(m_bodyId);
		b2Vec2 pB = b2Body_GetWorldCenterOfMass(m_bodyId);

		for (int i = 0; i < 2; ++i)
		{
			b2Vec2 anchorA = {3.0f, 0.0f};
			b2Vec2 anchorB = b2Body_GetWorldPoint(m_bodyId, localAnchors[i]);

			b2Vec2 deltaAnchor = b2Sub(anchorB, anchorA);

			float slackLength = 1.0f;
			float length = b2Length(deltaAnchor);
			float C = length - slackLength;
			if (C < 0.0f || length < 0.001f)
			{
				g_draw.DrawSegment(anchorA, anchorB, b2MakeColor(b2_colorLightCyan, 1.0f));
				m_impulses[i] = 0.0f;
				continue;
			}

			g_draw.DrawSegment(anchorA, anchorB, b2MakeColor(b2_colorViolet, 1.0f));
			b2Vec2 axis = b2Normalize(deltaAnchor);

			b2Vec2 rB = b2Sub(anchorB, pB);
			float Jb = b2Cross(rB, axis);
			float K = invMass + Jb * invI * Jb;
			float invK = K < 0.0001f ? 0.0f : 1.0f / K;

			float Cdot = b2Dot(vB, axis) + Jb * omegaB;
			float impulse = -massCoefficient * invK * (Cdot + biasCoefficient * C);
			float appliedImpulse = B2_CLAMP(impulse, -maxForce * timeStep, 0.0f);

			vB = b2MulAdd(vB, invMass * appliedImpulse, axis);
			omegaB += appliedImpulse * invI * Jb;

			m_impulses[i] = appliedImpulse;
		}

		b2Body_SetLinearVelocity(m_bodyId, vB);
		b2Body_SetAngularVelocity(m_bodyId, omegaB);

		g_draw.DrawString(5, m_textLine, "forces = %g, %g", m_impulses[0] * invTimeStep, m_impulses[1] * invTimeStep);
		m_textLine += m_textIncrement;
	}

	static Sample* Create(const Settings& settings)
	{
		return new UserConstraint(settings);
	}

	b2BodyId m_bodyId;
	float m_impulses[2];
};

static int sampleUserConstraintIndex = RegisterSample("Joints", "User Constraint", UserConstraint::Create);
