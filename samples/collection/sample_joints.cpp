// SPDX-FileCopyrightText: 2022 Erin Catto
// SPDX-License-Identifier: MIT

#include "sample.h"

#include "box2d/box2d.h"
#include "box2d/geometry.h"
#include "box2d/hull.h"

// #include <GLFW/glfw3.h>
#include <imgui.h>

class BenchmarkJointGrid : public Sample
{
  public:
	BenchmarkJointGrid(const Settings& settings)
		: Sample(settings)
	{
		constexpr float rad = 0.4f;
		constexpr int32_t numi = g_sampleDebug ? 100 : 100;
		constexpr int32_t numk = g_sampleDebug ? 100 : 100;
		constexpr float shift = 1.0f;

		// Allocate to avoid huge stack usage
		b2BodyId* bodies = static_cast<b2BodyId*>(malloc(numi * numk * sizeof(b2BodyId)));
		int32_t index = 0;

		b2ShapeDef sd = b2DefaultShapeDef();
		sd.density = 1.0f;
		sd.filter.maskBits = 0;

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

				b2Body_CreateCircle(body, &sd, &circle);

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

static int sampleJointGridIndex = RegisterSample("Joints", "Joint Grid", BenchmarkJointGrid::Create);

// A suspension bridge
class Bridge : public Sample
{
  public:
	enum
	{
		e_count = 20
	};

	Bridge(const Settings& settings)
		: Sample(settings)
	{
		b2BodyId groundId = b2_nullBodyId;
		{
			b2BodyDef bd = b2DefaultBodyDef();
			groundId = b2World_CreateBody(m_worldId, &bd);
		}

		{
			b2Polygon box = b2MakeBox(0.5f, 0.125f);

			b2ShapeDef sd = b2DefaultShapeDef();
			sd.density = 20.0f;

			b2RevoluteJointDef jd = b2DefaultRevoluteJointDef();
			int32_t jointIndex = 0;
			m_maxMotorTorque = 0.0f;

			b2BodyId prevBodyId = groundId;
			for (int32_t i = 0; i < e_count; ++i)
			{
				b2BodyDef bd = b2DefaultBodyDef();
				bd.type = b2_dynamicBody;
				bd.position = {-34.5f + 1.0f * i, 20.0f};
				//bd.linearDamping = 0.1f;
				//bd.angularDamping = 0.1f;
				b2BodyId bodyId = b2World_CreateBody(m_worldId, &bd);
				b2Body_CreatePolygon(bodyId, &sd, &box);

				b2Vec2 pivot = {-35.0f + 1.0f * i, 20.0f};
				jd.bodyIdA = prevBodyId;
				jd.bodyIdB = bodyId;
				jd.localAnchorA = b2Body_GetLocalPoint(jd.bodyIdA, pivot);
				jd.localAnchorB = b2Body_GetLocalPoint(jd.bodyIdB, pivot);
				jd.enableMotor = true;
				jd.maxMotorTorque = m_maxMotorTorque;
				m_jointIds[jointIndex++] = b2World_CreateRevoluteJoint(m_worldId, &jd);

				prevBodyId = bodyId;
			}

			b2Vec2 pivot = {-35.0f + 1.0f * e_count, 20.0f};
			jd.bodyIdA = prevBodyId;
			jd.bodyIdB = groundId;
			jd.localAnchorA = b2Body_GetLocalPoint(jd.bodyIdA, pivot);
			jd.localAnchorB = b2Body_GetLocalPoint(jd.bodyIdB, pivot);
			jd.enableMotor = true;
			jd.maxMotorTorque = m_maxMotorTorque;
			m_jointIds[jointIndex++] = b2World_CreateRevoluteJoint(m_worldId, &jd);

			assert(jointIndex == e_count + 1);
		}

		for (int32_t i = 0; i < 2; ++i)
		{
			b2Vec2 vertices[3] = {{-0.5f, 0.0f}, {0.5f, 0.0f}, {0.0f, 1.5f}};

			b2Hull hull = b2ComputeHull(vertices, 3);
			b2Polygon triangle = b2MakePolygon(&hull, 0.0f);

			b2ShapeDef sd = b2DefaultShapeDef();
			sd.density = 20.0f;

			b2BodyDef bd = b2DefaultBodyDef();
			bd.type = b2_dynamicBody;
			bd.position = {-8.0f + 8.0f * i, 22.0f};
			b2BodyId bodyId = b2World_CreateBody(m_worldId, &bd);
			b2Body_CreatePolygon(bodyId, &sd, &triangle);
		}

		for (int32_t i = 0; i < 3; ++i)
		{
			b2Circle circle = {{0.0f, 0.0f}, 0.5f};

			b2ShapeDef sd = b2DefaultShapeDef();
			sd.density = 20.0f;

			b2BodyDef bd = b2DefaultBodyDef();
			bd.type = b2_dynamicBody;
			bd.position = {-6.0f + 6.0f * i, 25.0f};
			b2BodyId bodyId = b2World_CreateBody(m_worldId, &bd);
			b2Body_CreateCircle(bodyId, &sd, &circle);
		}
	}

	void UpdateUI() override
	{
		ImGui::SetNextWindowPos(ImVec2(10.0f, 300.0f), ImGuiCond_Once);

		// Automatic window size
		ImGui::Begin("Options", nullptr, ImGuiWindowFlags_AlwaysAutoResize);

		// Slider takes half the window
		ImGui::PushItemWidth(ImGui::GetWindowWidth() * 0.5f);
		bool updateFriction = ImGui::SliderFloat("Joint Friction", &m_maxMotorTorque, 0.0f, 10000.0f, "%2.f");
		if (updateFriction)
		{
			for (int32_t i = 0; i <= e_count; ++i)
			{
				b2RevoluteJoint_SetMaxMotorTorque(m_jointIds[i], m_maxMotorTorque);
			}
		}

		ImGui::End();
	}

	static Sample* Create(const Settings& settings)
	{
		return new Bridge(settings);
	}

	b2JointId m_jointIds[e_count + 1];
	float m_maxMotorTorque;
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
			b2BodyDef bd = b2DefaultBodyDef();
			groundId = b2World_CreateBody(m_worldId, &bd);
		}

		m_maxMotorTorque = 0.0f;

		{
			float hx = 0.5f;
			b2Polygon box = b2MakeBox(hx, 0.125f);

			b2ShapeDef sd = b2DefaultShapeDef();
			sd.density = 20.0f;

			b2RevoluteJointDef jd = b2DefaultRevoluteJointDef();

			int32_t jointIndex = 0;

			b2BodyId prevBodyId = groundId;
			for (int32_t i = 0; i < e_count; ++i)
			{
				b2BodyDef bd = b2DefaultBodyDef();
				bd.type = b2_dynamicBody;
				bd.position = {(1.0f + 2.0f * i) * hx, e_count * hx};
				b2BodyId bodyId = b2World_CreateBody(m_worldId, &bd);
				b2Body_CreatePolygon(bodyId, &sd, &box);

				b2Vec2 pivot = {(2.0f * i) * hx, e_count * hx};
				jd.bodyIdA = prevBodyId;
				jd.bodyIdB = bodyId;
				jd.localAnchorA = b2Body_GetLocalPoint(jd.bodyIdA, pivot);
				jd.localAnchorB = b2Body_GetLocalPoint(jd.bodyIdB, pivot);
				jd.enableMotor = true;
				jd.maxMotorTorque = m_maxMotorTorque;
				m_jointIds[jointIndex++] = b2World_CreateRevoluteJoint(m_worldId, &jd);

				prevBodyId = bodyId;
			}

			b2Circle circle = {{0.0f, 0.0f}, 4.0f};

			b2BodyDef bd = b2DefaultBodyDef();
			bd.type = b2_dynamicBody;
			bd.position = {(1.0f + 2.0f * e_count) * hx + circle.radius - hx, e_count * hx};
			//bd.linearDamping = 0.1f;
			//bd.angularDamping = 0.1f;

			//bd.linearVelocity = {100.0f, -100.0f};
			b2BodyId bodyId = b2World_CreateBody(m_worldId, &bd);
			b2Body_CreateCircle(bodyId, &sd, &circle);

			b2Vec2 pivot = {(2.0f * e_count) * hx, e_count * hx};
			jd.bodyIdA = prevBodyId;
			jd.bodyIdB = bodyId;
			jd.localAnchorA = b2Body_GetLocalPoint(jd.bodyIdA, pivot);
			jd.localAnchorB = b2Body_GetLocalPoint(jd.bodyIdB, pivot);
			jd.enableMotor = true;
			jd.maxMotorTorque = m_maxMotorTorque;
			m_jointIds[jointIndex++] = b2World_CreateRevoluteJoint(m_worldId, &jd);
			assert(jointIndex == e_count + 1);
		}
	}

	void UpdateUI() override
	{
		ImGui::SetNextWindowPos(ImVec2(10.0f, 300.0f), ImGuiCond_Once);
		ImGui::SetNextWindowSize(ImVec2(300.0f, 60.0f));
		ImGui::Begin("Options", nullptr, ImGuiWindowFlags_NoResize);

		bool updateFriction = ImGui::SliderFloat("Joint Friction", &m_maxMotorTorque, 0.0f, 10000.0f, "%2.f");
		if (updateFriction)
		{
			for (int32_t i = 0; i <= e_count; ++i)
			{
				b2RevoluteJoint_SetMaxMotorTorque(m_jointIds[i], m_maxMotorTorque);
			}
		}

		ImGui::End();
	}

	static Sample* Create(const Settings& settings)
	{
		return new BallAndChain(settings);
	}

	b2JointId m_jointIds[e_count + 1];
	float m_maxMotorTorque;
};

static int sampleBallAndChainIndex = RegisterSample("Joints", "BallAndChain", BallAndChain::Create);

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
			b2BodyDef bd = b2DefaultBodyDef();
			groundId = b2World_CreateBody(m_worldId, &bd);
		}

		{
			float hx = 0.5f;
			b2Polygon box = b2MakeBox(hx, 0.125f);

			b2ShapeDef sd = b2DefaultShapeDef();
			sd.density = 20.0f;

			b2WeldJointDef jd = b2DefaultWeldJointDef();

			b2BodyId prevBodyId = groundId;
			for (int32_t i = 0; i < e_count; ++i)
			{
				b2BodyDef bd = b2DefaultBodyDef();
				bd.type = b2_dynamicBody;
				bd.position = {(1.0f + 2.0f * i) * hx, 0.0f};
				b2BodyId bodyId = b2World_CreateBody(m_worldId, &bd);
				b2Body_CreatePolygon(bodyId, &sd, &box);

				b2Vec2 pivot = {(2.0f * i) * hx, 0.0f};
				jd.bodyIdA = prevBodyId;
				jd.bodyIdB = bodyId;
				jd.localAnchorA = b2Body_GetLocalPoint(jd.bodyIdA, pivot);
				jd.localAnchorB = b2Body_GetLocalPoint(jd.bodyIdB, pivot);
				//jd.linearHertz = 5.0f;
				b2World_CreateWeldJoint(m_worldId, &jd);

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
