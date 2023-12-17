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

#define SIDES 7

struct Ring
{
	b2BodyId bodyIds[SIDES];
	b2JointId jointIds[SIDES];
	bool valid;
};

class SensorEvent : public Sample
{
public:
	enum
	{
		e_count = 16
	};

	SensorEvent(const Settings& settings)
		: Sample(settings)
	{
		{
			b2BodyId groundId = b2World_CreateBody(m_worldId, &b2_defaultBodyDef);
			// b2Segment segment = {{-20.0f, 0.0f}, {20.0f, 0.0f}};
			// b2Body_CreateSegment(groundId, &b2_defaultShapeDef, &segment);

			// b2Vec2 points[] = {
			//{42.333, 44.979},	{177.271, 44.979},	{177.271, 100.542}, {142.875, 121.708}, {177.271, 121.708},
			//{177.271, 171.979}, {142.875, 193.146}, {177.271, 193.146}, {177.271, 222.250}, {124.354, 261.938},
			//{124.354, 293.688}, {95.250, 293.688},	{95.250, 261.938},	{42.333, 222.250},	{42.333, 193.146},
			//{76.729, 193.146},	{42.333, 171.979},	{42.333, 121.708},	{76.729, 121.708},	{42.333, 100.542},
			//};

			b2Vec2 points[] = {
				{-16.8672504, 31.088623},	 {16.8672485, 31.088623},	 {16.8672485, 17.1978741}, {8.26824951, 11.906374},
				{16.8672485, 11.906374},	 {16.8672485, -0.661376953}, {8.26824951, -5.953125},  {16.8672485, -5.953125},
				{16.8672485, -13.229126},	 {3.63799858, -23.151123},	 {3.63799858, -31.088623}, {-3.63800049, -31.088623},
				{-3.63800049, -23.151123},	 {-16.8672504, -13.229126},	 {-16.8672504, -5.953125}, {-8.26825142, -5.953125},
				{-16.8672504, -0.661376953}, {-16.8672504, 11.906374},	 {-8.26825142, 11.906374}, {-16.8672504, 17.1978741},
			};

			int count = sizeof(points) / sizeof(points[0]);

			// float scale = 0.25f;
			// b2Vec2 lower = {FLT_MAX, FLT_MAX};
			// b2Vec2 upper = {-FLT_MAX, -FLT_MAX};
			// for (int i = 0; i < count; ++i)
			//{
			//	points[i].x = scale * points[i].x;
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
			b2Body_CreateChain(groundId, &chainDef);

#if 0
			{
				b2ShapeDef shapeDef = b2_defaultShapeDef;
				shapeDef.friction = 0.2f;
				shapeDef.restitution = 2.0f;

				float radius = 0.75f;
				float y = 15.0f;
				for (int j = 0; j < 5; ++j)
				{
					b2Circle circle = {{0.0f, y}, radius};
					b2Body_CreateCircle(groundId, &shapeDef, &circle);
					y -= 6.0f;
				}
			}
#endif
			float sign = 1.0f;
			float y = 14.0f;
			for (int i = 0; i < 3; ++i)
			{
				b2BodyDef bodyDef = b2_defaultBodyDef;
				bodyDef.position = {0.0f, y};
				bodyDef.type = b2_dynamicBody;

				b2BodyId bodyId = b2World_CreateBody(m_worldId, &bodyDef);

				b2Polygon box = b2MakeBox(6.0f, 0.5f);
				b2ShapeDef shapeDef = b2_defaultShapeDef;
				shapeDef.friction = 0.1f;
				shapeDef.restitution = 1.0f;
				shapeDef.density = 1.0f;

				b2Body_CreatePolygon(bodyId, &shapeDef, &box);

				b2RevoluteJointDef revoluteDef = b2DefaultRevoluteJointDef();
				revoluteDef.bodyIdA = groundId;
				revoluteDef.bodyIdB = bodyId;
				revoluteDef.localAnchorA = bodyDef.position;
				revoluteDef.localAnchorB = b2Vec2_zero;
				revoluteDef.maxMotorTorque = 50.0f;
				revoluteDef.motorSpeed = 10.0f * sign;
				revoluteDef.enableMotor = true;

				b2World_CreateRevoluteJoint(m_worldId, &revoluteDef);

				y -= 14.0f;
				sign = -sign;
			}

			{
				b2Polygon box = b2MakeOffsetBox(4.0f, 1.0f, {0.0f, -30.5f}, 0.0f);
				b2ShapeDef shapeDef = b2_defaultShapeDef;
				shapeDef.isSensor = true;
				b2Body_CreatePolygon(groundId, &shapeDef, &box);
			}
		}

		for (int i = 0; i < e_count; ++i)
		{
			for (int j = 0; j < SIDES; ++j)
			{
				m_rings[i].bodyIds[j] = b2_nullBodyId;
				m_rings[i].jointIds[j] = b2_nullJointId;
			}
			m_rings[i].valid = false;
		}

		m_wait = 0.5f;
		m_side = -15.0f;
		CreateRing();
	}

	void CreateRing()
	{
		int index = -1;
		for (int i = 0; i < e_count; ++i)
		{
			if (m_rings[i].valid == false)
			{
				index = i;
				break;
			}
		}

		if (index == -1)
		{
			return;
		}

		Ring* ring = m_rings + index;

		float radius = 1.0f;
		float deltaAngle = 2.0f * b2_pi / SIDES;
		float length = 2.0f * b2_pi * radius / SIDES;

		b2Capsule capsule = {{0.0f, -0.5f * length}, {0.0f, 0.5f * length}, 0.25f};

		b2Vec2 center = {m_side, 29.5f};

		b2BodyDef bodyDef = b2_defaultBodyDef;
		bodyDef.type = b2_dynamicBody;
		bodyDef.userData = ring;

		b2ShapeDef shapeDef = b2_defaultShapeDef;
		shapeDef.density = 1.0f;

		// Create bodies
		float angle = 0.0f;
		for (int i = 0; i < SIDES; ++i)
		{
			bodyDef.position = {radius * cosf(angle) + center.x, radius * sinf(angle) + center.y};
			bodyDef.angle = angle;

			ring->bodyIds[i] = b2World_CreateBody(m_worldId, &bodyDef);
			b2Body_CreateCapsule(ring->bodyIds[i], &shapeDef, &capsule);

			angle += deltaAngle;
		}

		// Create joints
		b2WeldJointDef weldDef = b2_defaultWeldJointDef;
		weldDef.angularHertz = 5.0f;
		weldDef.angularDampingRatio = 0.0f;
		weldDef.localAnchorA = {0.0f, 0.5f * length};
		weldDef.localAnchorB = {0.0f, -0.5f * length};

		b2BodyId prevBodyId = ring->bodyIds[SIDES - 1];
		for (int i = 0; i < SIDES; ++i)
		{
			weldDef.bodyIdA = prevBodyId;
			weldDef.bodyIdB = ring->bodyIds[i];
			weldDef.referenceAngle = b2Body_GetAngle(ring->bodyIds[i]) - b2Body_GetAngle(prevBodyId);
			ring->jointIds[i] = b2World_CreateWeldJoint(m_worldId, &weldDef);
			prevBodyId = weldDef.bodyIdB;
		}

		ring->valid = true;
		m_side = -m_side;
	}

	void DestroyRing(int index)
	{
		Ring* ring = m_rings + index;
		assert(ring->valid == true);

		for (int i = 0; i < SIDES; ++i)
		{
			b2World_DestroyJoint(ring->jointIds[i]);
			ring->jointIds[i] = b2_nullJointId;
		}

		for (int i = 0; i < SIDES; ++i)
		{
			b2World_DestroyBody(ring->bodyIds[i]);
			ring->bodyIds[i] = b2_nullBodyId;
		}

		ring->valid = false;
	}

	void Step(Settings& settings) override
	{
		Sample::Step(settings);

		// Discover rings that touch the bottom sensor
		bool deferredDestructions[e_count] = {0};
		b2SensorEvents sensorEvents = b2World_GetSensorEvents(m_worldId);
		for (int i = 0; i < sensorEvents.beginCount; ++i)
		{
			b2SensorBeginTouchEvent event = sensorEvents.beginEvents[i];
			b2ShapeId visitorId = event.visitorShapeId;
			b2BodyId ringBodyId = b2Shape_GetBody(visitorId);
			Ring* ring = (Ring*)b2Body_GetUserData(ringBodyId);
			if (ring != nullptr && ring->valid)
			{
				int index = (int)(ring - m_rings);
				assert(0 <= index && index < e_count);

				// Defer destruction to avoid double destruction and event invalidation (orphaned shape ids)
				deferredDestructions[index] = true;
			}
		}

		// Safely destroy rings that hit the bottom sensor
		for (int i = 0; i < e_count; ++i)
		{
			if (deferredDestructions[i])
			{
				DestroyRing(i);
			}
		}

		if (settings.hertz > 0.0f && settings.pause == false)
		{
			m_wait -= 1.0f / settings.hertz;
			if (m_wait < 0.0f)
			{
				CreateRing();
				m_wait += 0.5f;
			}
		}
	}

	static Sample* Create(const Settings& settings)
	{
		return new SensorEvent(settings);
	}

	Ring m_rings[e_count];
	float m_wait;
	float m_side;
};

static int sampleSensorEvent = RegisterSample("Events", "Sensor", SensorEvent::Create);

struct BodyUserData
{
	int index;
};

class ContactEvent : public Sample
{
public:
	enum
	{
		e_count = 20
	};

	ContactEvent(const Settings& settings)
		: Sample(settings)
	{
		if (settings.restart == false)
		{
			g_camera.m_center = {0.0f, 0.0f};
			g_camera.m_zoom = 1.75f;
		}

		{
			b2BodyId groundId = b2World_CreateBody(m_worldId, &b2_defaultBodyDef);

			b2Vec2 points[] = {{40.0f, -40.0f}, {-40.0f, -40.0f}, {-40.0f, 40.0f}, {40.0f, 40.0f}};

			b2ChainDef chainDef = b2_defaultChainDef;
			chainDef.count = 4;
			chainDef.points = points;
			chainDef.loop = true;

			b2Body_CreateChain(groundId, &chainDef);
		}

		// Player
		{
			b2BodyDef bodyDef = b2_defaultBodyDef;
			bodyDef.type = b2_dynamicBody;
			bodyDef.gravityScale = 0.0f;
			bodyDef.linearDamping = 0.5f;
			bodyDef.angularDamping = 0.5f;
			m_playerId = b2World_CreateBody(m_worldId, &bodyDef);

			b2Circle circle = {{0.0f, 0.0f}, 1.0f};
			b2ShapeDef shapeDef = b2_defaultShapeDef;

			// Enable contact events for the player shape
			shapeDef.enableContactEvents = true;

			m_coreShapeId = b2Body_CreateCircle(m_playerId, &shapeDef, &circle);
		}

		for (int i = 0; i < e_count; ++i)
		{
			m_debrisIds[i] = b2_nullBodyId;
			m_bodyUserData[i].index = i;
		}

		m_wait = 0.5f;
		m_force = 100.0f;
	}

	void SpawnDebris()
	{
		int index = -1;
		for (int i = 0; i < e_count; ++i)
		{
			if (B2_IS_NULL(m_debrisIds[i]))
			{
				index = i;
				break;
			}
		}

		if (index == -1)
		{
			return;
		}

		// Debris
		b2BodyDef bodyDef = b2_defaultBodyDef;
		bodyDef.type = b2_dynamicBody;
		bodyDef.position = {RandomFloat(-38.0f, 38.0f), RandomFloat(-38.0f, 38.0f)};
		bodyDef.angle = RandomFloat(-b2_pi, b2_pi);
		bodyDef.gravityScale = 0.0f;
		bodyDef.linearDamping = 0.5f;
		bodyDef.angularDamping = 0.5f;
		bodyDef.userData = m_bodyUserData + index;
		m_debrisIds[index] = b2World_CreateBody(m_worldId, &bodyDef);

		b2ShapeDef shapeDef = b2_defaultShapeDef;

		// No events when debris hits debris
		shapeDef.enableContactEvents = false;

		if ((index + 1) % 3 == 0)
		{
			b2Circle circle = {{0.0f, 0.0f}, 0.5f};
			b2Body_CreateCircle(m_debrisIds[index], &shapeDef, &circle);
		}
		else if ((index + 1) % 2 == 0)
		{
			b2Capsule capsule = {{0.0f, -0.25f}, {0.0f, 0.25f}, 0.25f};
			b2Body_CreateCapsule(m_debrisIds[index], &shapeDef, &capsule);
		}
		else
		{
			b2Polygon box = b2MakeBox(0.4f, 0.6f);
			b2Body_CreatePolygon(m_debrisIds[index], &shapeDef, &box);
		}
	}

	void UpdateUI() override
	{
		ImGui::SetNextWindowPos(ImVec2(10.0f, 400.0f));
		ImGui::SetNextWindowSize(ImVec2(200.0f, 60.0f));
		ImGui::Begin("Sample Controls", nullptr, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize);

		ImGui::SliderFloat("force", &m_force, 100.0f, 500.0f, "%.1f");

		ImGui::End();
	}

	void Step(Settings& settings) override
	{
		if (glfwGetKey(g_mainWindow, GLFW_KEY_A) == GLFW_PRESS)
		{
			b2Body_ApplyForceToCenter(m_playerId, {-m_force, 0.0f}, true);
		}

		if (glfwGetKey(g_mainWindow, GLFW_KEY_D) == GLFW_PRESS)
		{
			b2Body_ApplyForceToCenter(m_playerId, {m_force, 0.0f}, true);
		}

		if (glfwGetKey(g_mainWindow, GLFW_KEY_W) == GLFW_PRESS)
		{
			b2Body_ApplyForceToCenter(m_playerId, {0.0f, m_force}, true);
		}

		if (glfwGetKey(g_mainWindow, GLFW_KEY_S) == GLFW_PRESS)
		{
			b2Body_ApplyForceToCenter(m_playerId, {0.0f, -m_force}, true);
		}

		Sample::Step(settings);

		// Discover rings that touch the bottom sensor
		int debrisToAttach[e_count] = {};
		b2ShapeId shapesToDestroy[e_count] = {b2_nullShapeId};
		int attachCount = 0;
		int destroyCount = 0;

		b2ContactEvents contactEvents = b2World_GetContactEvents(m_worldId);
		for (int i = 0; i < contactEvents.beginCount; ++i)
		{
			b2ContactBeginTouchEvent event = contactEvents.beginEvents[i];
			b2BodyId bodyIdA = b2Shape_GetBody(event.shapeIdA);
			b2BodyId bodyIdB = b2Shape_GetBody(event.shapeIdB);

			if (B2_ID_EQUALS(bodyIdA, m_playerId))
			{
				BodyUserData* userDataB = static_cast<BodyUserData*>(b2Body_GetUserData(bodyIdB));
				if (userDataB == nullptr)
				{
					if (B2_ID_EQUALS(event.shapeIdA, m_coreShapeId) == false && destroyCount < e_count)
					{
						// player non-core shape hit the wall

						bool found = false;
						for (int j = 0; j < destroyCount; ++j)
						{
							if (B2_ID_EQUALS(event.shapeIdA, shapesToDestroy[j]))
							{
								found = true;
								break;
							}
						}
						
						// avoid double deletion
						if (found == false)
						{
							shapesToDestroy[destroyCount] = event.shapeIdA;
							destroyCount += 1;
						}
					}
				}
				else if (attachCount < e_count)
				{
					debrisToAttach[attachCount] = userDataB->index;
					attachCount += 1;
				}
			}
			else
			{
				// Only expect events for the player
				assert(B2_ID_EQUALS(bodyIdB, m_playerId));
				BodyUserData* userDataA = static_cast<BodyUserData*>(b2Body_GetUserData(bodyIdA));
				if (userDataA == nullptr)
				{
					if (B2_ID_EQUALS(event.shapeIdB, m_coreShapeId) == false && destroyCount < e_count)
					{
						// player non-core shape hit the wall

						bool found = false;
						for (int j = 0; j < destroyCount; ++j)
						{
							if (B2_ID_EQUALS(event.shapeIdB, shapesToDestroy[j]))
							{
								found = true;
								break;
							}
						}

						// avoid double deletion
						if (found == false)
						{
							shapesToDestroy[destroyCount] = event.shapeIdB;
							destroyCount += 1;
						}
					}
				}
				else if (attachCount < e_count)
				{
					debrisToAttach[attachCount] = userDataA->index;
					attachCount += 1;
				}
			}
		}

		// Attach debris to player body
		for (int i = 0; i < attachCount; ++i)
		{
			int index = debrisToAttach[i];
			b2BodyId debrisId = m_debrisIds[index];
			if (B2_IS_NULL(debrisId))
			{
				continue;
			}

			b2Transform playerTransform = b2Body_GetTransform(m_playerId);
			b2Transform debrisTransform = b2Body_GetTransform(debrisId);
			b2Transform relativeTransform = b2InvMulTransforms(playerTransform, debrisTransform);

			b2ShapeId shapeId = b2Body_GetFirstShape(debrisId);
			b2ShapeType type = b2Shape_GetType(shapeId);

			b2ShapeDef shapeDef = b2_defaultShapeDef;
			shapeDef.enableContactEvents = true;
			
			switch (type)
			{
				case b2_circleShape:
				{
					b2Circle circle = *b2Shape_GetCircle(shapeId);
					circle.point = b2TransformPoint(relativeTransform, circle.point);

					b2Body_CreateCircle(m_playerId, &shapeDef, &circle);
				}
				break;

				case b2_capsuleShape:
				{
					b2Capsule capsule = *b2Shape_GetCapsule(shapeId);
					capsule.point1 = b2TransformPoint(relativeTransform, capsule.point1);
					capsule.point2 = b2TransformPoint(relativeTransform, capsule.point2);

					b2Body_CreateCapsule(m_playerId, &shapeDef, &capsule);
				}
				break;

				case b2_polygonShape:
				{
					b2Polygon polygon = b2TransformPolygon(relativeTransform, b2Shape_GetPolygon(shapeId));

					b2Body_CreatePolygon(m_playerId, &shapeDef, &polygon);
				}
				break;

				default:
					assert(false);
			}

			b2World_DestroyBody(debrisId);
			m_debrisIds[index] = b2_nullBodyId;
		}

		for (int i = 0; i < destroyCount; ++i)
		{
			b2Body_DestroyShape(shapesToDestroy[i]);
		}

		if (settings.hertz > 0.0f && settings.pause == false)
		{
			m_wait -= 1.0f / settings.hertz;
			if (m_wait < 0.0f)
			{
				SpawnDebris();
				m_wait += 0.5f;
			}
		}
	}

	static Sample* Create(const Settings& settings)
	{
		return new ContactEvent(settings);
	}

	b2BodyId m_playerId;
	b2ShapeId m_coreShapeId;
	b2BodyId m_debrisIds[e_count];
	BodyUserData m_bodyUserData[e_count];
	float m_force;
	float m_wait;
};

static int sampleWeeble = RegisterSample("Events", "Contact", ContactEvent::Create);
