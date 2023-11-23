// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "box2d/box2d.h"
#include "box2d/geometry.h"
#include "box2d/hull.h"
#include "box2d/math.h"
#include "sample.h"

#include <GLFW/glfw3.h>
#include <imgui.h>

class RayCast : public Sample
{
  public:
	RayCast(const Settings& settings) : Sample(settings)
	{
		m_circle = {{0.0f, 0.0f}, 2.0f};
		m_capsule = {{-1.0f, 1.0f}, {1.0f, -1.0f}, 1.5f};
		m_box = b2MakeBox(2.0f, 2.0f);

		b2Vec2 vertices[3] = {{-2.0f, 0.0f}, {2.0f, 0.0f}, {2.0f, 3.0f}};
		b2Hull hull = b2ComputeHull(vertices, 3);
		m_triangle = b2MakePolygon(&hull, 0.0f);

		m_segment = {{-3.0f, 0.0f}, {3.0f, 0.0}};

		m_transform = b2Transform_identity;
		m_angle = 0.0f;

		m_basePosition = {0.0f, 0.0f};
		m_baseAngle = 0.0f;
		m_startPoint = {0.0f, 0.0f};

		m_rayStart = {0.0f, 30.0f};
		m_rayEnd = {0.0f, 0.0f};
		m_rayRadius = 0.0f;

		m_rayDrag = false;
		m_translating = false;
		m_rotating = false;

		m_showFraction = false;
	}

	void UpdateUI() override
	{
		ImGui::SetNextWindowPos(ImVec2(10.0f, 100.0f));
		ImGui::SetNextWindowSize(ImVec2(250.0f, 230.0f));
		ImGui::Begin("RayCast Controls", nullptr, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize);

		if (ImGui::SliderFloat("x offset", &m_transform.p.x, -2.0f, 2.0f, "%.2f"))
		{
		}

		if (ImGui::SliderFloat("y offset", &m_transform.p.y, -2.0f, 2.0f, "%.2f"))
		{
		}

		if (ImGui::SliderFloat("angle", &m_angle, -b2_pi, b2_pi, "%.2f"))
		{
			m_transform.q = b2MakeRot(m_angle);
		}

		if (ImGui::SliderFloat("ray radius", &m_rayRadius, 0.0f, 1.0f, "%.1f"))
		{
		}

		if (ImGui::Checkbox("show fraction", &m_showFraction))
		{
		}

		if (ImGui::Button("Reset"))
		{
			m_transform = b2Transform_identity;
			m_angle = 0.0f;
		}

		ImGui::Separator();

		ImGui::Text("mouse btn 1: ray cast");
		ImGui::Text("mouse btn 1 + shft: rotate");
		ImGui::Text("mouse btn 1 + ctrl: translate");

		ImGui::End();
	}

	void MouseDown(b2Vec2 p, int button, int mods) override
	{
		if (button == GLFW_MOUSE_BUTTON_1)
		{
			m_startPoint = p;

			if (mods == 0)
			{
				m_rayStart = p;
				m_rayDrag = true;
			}
			else if (mods == GLFW_MOD_SHIFT)
			{
				m_translating = true;
				m_basePosition = m_transform.p;
			}
			else if (mods == GLFW_MOD_CONTROL)
			{
				m_rotating = true;
				m_baseAngle = m_angle;
			}
		}
	}

	void MouseUp(b2Vec2, int button) override
	{
		if (button == GLFW_MOUSE_BUTTON_1)
		{
			m_rayDrag = false;
			m_rotating = false;
			m_translating = false;
		}
	}

	void MouseMove(b2Vec2 p) override
	{
		if (m_rayDrag)
		{
			m_rayEnd = p;
		}
		else if (m_translating)
		{
			m_transform.p.x = m_basePosition.x + 0.5f * (p.x - m_startPoint.x);
			m_transform.p.y = m_basePosition.y + 0.5f * (p.y - m_startPoint.y);
		}
		else if (m_rotating)
		{
			float dx = p.x - m_startPoint.x;
			m_angle = B2_CLAMP(m_baseAngle + 0.5f * dx, -b2_pi, b2_pi);
			m_transform.q = b2MakeRot(m_angle);
		}
	}

	void DrawRay(const b2RayCastOutput* output)
	{
		b2Color white = {1.0f, 1.0f, 1.0f, 1.0f};
		b2Color green = {0.0f, 1.0f, 0.0f, 1.0f};
		b2Color red = {1.0f, 0.0f, 0.0f, 1.0f};
		b2Color violet = {1.0f, 0.0f, 1.0f, 1.0f};

		b2Vec2 p1 = m_rayStart;
		b2Vec2 p2 = m_rayEnd;
		b2Vec2 d = b2Sub(p2, p1);

		if (output->hit)
		{
			b2Vec2 p = b2MulAdd(p1, output->fraction, d);
			g_draw.DrawSegment(p1, p, white);
			g_draw.DrawPoint(p1, 5.0f, green);
			g_draw.DrawPoint(output->point, 5.0f, white);

			b2Vec2 n = b2MulAdd(p, 1.0f, output->normal);
			g_draw.DrawSegment(p, n, violet);

			if (m_rayRadius > 0.0f)
			{
				g_draw.DrawCircle(p1, m_rayRadius, green);
				g_draw.DrawCircle(p, m_rayRadius, red);
			}

			if (m_showFraction)
			{
				b2Vec2 ps = {p.x + 0.05f, p.y - 0.02f};
				g_draw.DrawString(ps, "%.2f", output->fraction);
			}
		}
		else
		{
			g_draw.DrawSegment(p1, p2, white);
			g_draw.DrawPoint(p1, 5.0f, green);
			g_draw.DrawPoint(p2, 5.0f, red);

			if (m_rayRadius > 0.0f)
			{
				g_draw.DrawCircle(p1, m_rayRadius, green);
				g_draw.DrawCircle(p2, m_rayRadius, red);
			}
		}
	}

	void Step(Settings&) override
	{
		b2Vec2 offset = {-20.0f, 20.0f};
		b2Vec2 increment = {10.0f, 0.0f};

		b2Color color1 = {0.3f, 0.8f, 0.6f, 1.0f};
		b2Color dim1 = {0.5f * color1.r, 0.5f * color1.g, 0.5f * color1.b, 1.0f};

		b2RayCastOutput output = {0};
		float maxFraction = 1.0f;

		// circle
		{
			b2Transform xf = {b2Add(m_transform.p, offset), m_transform.q};
			b2Vec2 c = b2TransformPoint(xf, m_circle.point);
			b2Vec2 axis = b2RotateVector(xf.q, {1.0f, 0.0f});
			g_draw.DrawSolidCircle(c, m_circle.radius, axis, color1);

			b2Vec2 start = b2InvTransformPoint(xf, m_rayStart);
			b2Vec2 end = b2InvTransformPoint(xf, m_rayEnd);
			b2RayCastInput input = {start, end, m_rayRadius, maxFraction};

			b2RayCastOutput localOutput = b2RayCastCircle(&input, &m_circle);
			if (localOutput.hit)
			{
				output = localOutput;
				output.point = b2TransformPoint(xf, localOutput.point);
				output.normal = b2RotateVector(xf.q, localOutput.normal);
				maxFraction = localOutput.fraction;
			}

			offset = b2Add(offset, increment);
		}

		// capsule
		{
			b2Transform xf = {b2Add(m_transform.p, offset), m_transform.q};
			b2Vec2 v1 = b2TransformPoint(xf, m_capsule.point1);
			b2Vec2 v2 = b2TransformPoint(xf, m_capsule.point2);
			g_draw.DrawSolidCapsule(v1, v2, m_capsule.radius, color1);

			b2Vec2 start = b2InvTransformPoint(xf, m_rayStart);
			b2Vec2 end = b2InvTransformPoint(xf, m_rayEnd);
			b2RayCastInput input = {start, end, m_rayRadius, maxFraction};

			b2RayCastOutput localOutput = b2RayCastCapsule(&input, &m_capsule);
			if (localOutput.hit)
			{
				output = localOutput;
				output.point = b2TransformPoint(xf, localOutput.point);
				output.normal = b2RotateVector(xf.q, localOutput.normal);
				maxFraction = localOutput.fraction;
			}

			offset = b2Add(offset, increment);
		}

		// box
		{
			b2Transform xf = {b2Add(m_transform.p, offset), m_transform.q};

			b2Vec2 vertices[b2_maxPolygonVertices];
			for (int i = 0; i < m_box.count; ++i)
			{
				vertices[i] = b2TransformPoint(xf, m_box.vertices[i]);
			}
			g_draw.DrawSolidPolygon(vertices, m_box.count, color1);

			b2Vec2 start = b2InvTransformPoint(xf, m_rayStart);
			b2Vec2 end = b2InvTransformPoint(xf, m_rayEnd);
			b2RayCastInput input = {start, end, m_rayRadius, maxFraction};

			b2RayCastOutput localOutput = b2RayCastPolygon(&input, &m_box);
			if (localOutput.hit)
			{
				output = localOutput;
				output.point = b2TransformPoint(xf, localOutput.point);
				output.normal = b2RotateVector(xf.q, localOutput.normal);
				maxFraction = localOutput.fraction;
			}

			offset = b2Add(offset, increment);
		}

		// triangle
		{
			b2Transform xf = {b2Add(m_transform.p, offset), m_transform.q};

			b2Vec2 vertices[b2_maxPolygonVertices];
			for (int i = 0; i < m_triangle.count; ++i)
			{
				vertices[i] = b2TransformPoint(xf, m_triangle.vertices[i]);
			}
			g_draw.DrawSolidPolygon(vertices, m_triangle.count, color1);

			b2Vec2 start = b2InvTransformPoint(xf, m_rayStart);
			b2Vec2 end = b2InvTransformPoint(xf, m_rayEnd);
			b2RayCastInput input = {start, end, m_rayRadius, maxFraction};

			b2RayCastOutput localOutput = b2RayCastPolygon(&input, &m_triangle);
			if (localOutput.hit)
			{
				output = localOutput;
				output.point = b2TransformPoint(xf, localOutput.point);
				output.normal = b2RotateVector(xf.q, localOutput.normal);
				maxFraction = localOutput.fraction;
			}

			offset = b2Add(offset, increment);
		}

		// segment
		{
			b2Transform xf = {b2Add(m_transform.p, offset), m_transform.q};

			b2Vec2 p1 = b2TransformPoint(xf, m_segment.point1);
			b2Vec2 p2 = b2TransformPoint(xf, m_segment.point2);
			g_draw.DrawSegment(p1, p2, color1);

			b2Vec2 start = b2InvTransformPoint(xf, m_rayStart);
			b2Vec2 end = b2InvTransformPoint(xf, m_rayEnd);
			b2RayCastInput input = {start, end, m_rayRadius, maxFraction};

			b2RayCastOutput localOutput = b2RayCastSegment(&input, &m_segment, false);
			if (localOutput.hit)
			{
				output = localOutput;
				output.point = b2TransformPoint(xf, localOutput.point);
				output.normal = b2RotateVector(xf.q, localOutput.normal);
				maxFraction = localOutput.fraction;
			}

			offset = b2Add(offset, increment);
		}

		DrawRay(&output);
	}

	static Sample* Create(const Settings& settings)
	{
		return new RayCast(settings);
	}

	b2Polygon m_box;
	b2Polygon m_triangle;
	b2Circle m_circle;
	b2Capsule m_capsule;
	b2Segment m_segment;

	b2Transform m_transform;
	float m_angle;

	b2Vec2 m_rayStart;
	b2Vec2 m_rayEnd;
	float m_rayRadius;

	b2Vec2 m_basePosition;
	float m_baseAngle;

	b2Vec2 m_startPoint;

	bool m_rayDrag;
	bool m_translating;
	bool m_rotating;
	bool m_showFraction;
};

static int sampleIndex = RegisterSample("Collision", "Ray Cast", RayCast::Create);


// This shows how to filter a specific shape using using data.
struct ShapeUserData
{
	bool ignore;
};

// Context for ray cast callbacks. Do what you want with this.
struct RayCastContext
{
	b2Vec2 points[3];
	b2Vec2 normal[3];
	int count;
};

// This callback finds the closest hit. This is the most common callback used in games.
float RayCastClosestCallback(b2ShapeId shapeId, b2Vec2 point, b2Vec2 normal, float fraction, void* context)
{
	RayCastContext rayContext = (RayCastContext)context;

	ShapeUserData* userData = (ShapeUserData*)b2Shape_GetUserData(shapeId);
	if (userData->ignore)
	{
		// By returning -1, we instruct the calling code to ignore this fixture and
		// continue the ray-cast to the next fixture.
		return -1.0f;
	}

	rayContext->points[0] = point;
	rayContext->normals[0] = normal;
	rayContext->count = 1;

	// By returning the current fraction, we instruct the calling code to clip the ray and
	// continue the ray-cast to the next fixture. WARNING: do not assume that fixtures
	// are reported in order. However, by clipping, we can always get the closest fixture.
	return fraction;
}

// This callback finds any hit. For this type of query we are usually just checking for obstruction,
// so the hit data is not relevant.
// NOTE: shape hits are not ordered, so this may not return the closest hit
float RayCastAnyCallback(b2ShapeId shapeId, b2Vec2 point, b2Vec2 normal, float fraction, void* context)
{
	RayCastContext rayContext = (RayCastContext)context;

	ShapeUserData* userData = (ShapeUserData*)b2Shape_GetUserData(shapeId);
	if (userData->ignore)
	{
		// By returning -1, we instruct the calling code to ignore this fixture and
		// continue the ray-cast to the next fixture.
		return -1.0f;
	}

	rayContext->points[0] = point;
	rayContext->normals[0] = normal;
	rayContext->count = 1;

	// At this point we have a hit, so we know the ray is obstructed.
	// By returning 0, we instruct the calling code to terminate the ray-cast.
	return 0.0f;
}

// This ray cast collects multiple hits along the ray. Polygon 0 is filtered.
// The fixtures are not necessary reported in order, so we might not capture
// the closest fixture.
// NOTE: shape hits are not ordered, so this may return hits in any order
float RayCastMultipleCallback(b2ShapeId shapeId, b2Vec2 point, b2Vec2 normal, float fraction, void* context)
{
	RayCastContext rayContext = (RayCastContext)context;

	ShapeUserData* userData = (ShapeUserData*)b2Shape_GetUserData(shapeId);
	if (userData->ignore)
	{
		// By returning -1, we instruct the calling code to ignore this fixture and
		// continue the ray-cast to the next fixture.
		return -1.0f;
	}

	count = rayContext->count;
	assert(count < 3);

	rayContext->points[count] = point;
	rayContext->normals[count] = normal;
	rayContext->count = count + 1;

	if (count == 3)
	{
		// At this point the buffer is full.
		// By returning 0, we instruct the calling code to terminate the ray-cast.
		return 0.0f;
	}

	// By returning 1, we instruct the caller to continue without clipping the ray.
	return 1.0f;
}

class RayCastWorld : public Test
{
public:
	enum Mode
	{
		e_any = 0,
		e_closest = 1,
		e_multiple = 2
	};

	enum
	{
		e_maxCount = 256
	};

	RayCast()
	{
		// Ground body
		{
			b2BodyDef bodyDef;
			b2BodyId groundId = m_world->CreateBody(&bodyDef);

			b2ShapeDef shapeDef = b2DefaultShapeDef();
			b2Segment segment = {{-40.0f, 0.0f}, {40.0f, 0.0f}};
			b2Body_CreateSegment(groundId, &shapeDef, &segment);
		}

		{
			b2Vec2 vertices[3];
			vertices[0].Set(-0.5f, 0.0f);
			vertices[1].Set(0.5f, 0.0f);
			vertices[2].Set(0.0f, 1.5f);
			b2Hull hull = b2ComputeHull(vertices, 3);
			m_polygons[0] = b2MakePolygon(&hull, 0.0f);
		}

		{
			b2Vec2 vertices[3];
			vertices[0].Set(-0.1f, 0.0f);
			vertices[1].Set(0.1f, 0.0f);
			vertices[2].Set(0.0f, 1.5f);
			b2Hull hull = b2ComputeHull(vertices, 3);
			m_polygons[1] = b2MakePolygon(&hull, 0.0f);
		}

		{
			float w = 1.0f;
			float b = w / (2.0f + b2Sqrt(2.0f));
			float s = b2Sqrt(2.0f) * b;

			b2Vec2 vertices[8];
			vertices[0].Set(0.5f * s, 0.0f);
			vertices[1].Set(0.5f * w, b);
			vertices[2].Set(0.5f * w, b + s);
			vertices[3].Set(0.5f * s, w);
			vertices[4].Set(-0.5f * s, w);
			vertices[5].Set(-0.5f * w, b + s);
			vertices[6].Set(-0.5f * w, b);
			vertices[7].Set(-0.5f * s, 0.0f);

			b2Hull hull = b2ComputeHull(vertices, 8);
			m_polygons[2] = b2MakePolygon(&hull, 0.0f);
		}

		m_polygons[3] = b2MakeBox(0.5f, 0.5f);
		m_circle = {{0.0f, 0.0f}, 0.5f};
		m_segment = {{-1.0f, 0.0f}, {1.0f, 0.0f}};

		m_bodyIndex = 0;

		for (int i = 0; i < e_maxCount; ++i)
		{
			m_bodyIds[i] = b2_nullBodyId;
		}

		m_degrees = 0.0f;
		m_mode = e_closest;
	}

	void Create(int index)
	{
		if (B2_IS_NULL(m_bodyIds[m_bodyIndex]))
		{
			b2World_DestroyBody(m_worldIds, m_bodyIds[m_bodyIndex]);
			m_bodyIds[m_bodyIndex] = b2_nullBodyId;
		}

		float x = RandomFloat(-10.0f, 10.0f);
		float y = RandomFloat(0.0f, 20.0f);

		b2BodyDef bodyDef;
		bodyDef.position = {x, y};
		bodyDef.angle = RandomFloat(-b2_pi, b2_pi);

		m_bodyIds[m_bodyIndex] = b2World_CreateBody(m_worldId, &bodyDef);

		const int ignoreIndex = 7;

		b2ShapeDef shapeDef;
		shapeDef.userData = m_userData + index;
		m_userData[index].ignore = m_bodyIndex == ignoreIndex ? true : false;

		if (index < 4)
		{
			b2Body_CreatePolygon(m_bodyIds[m_bodyIndex], &shapeDef, m_polygons + index);
		}
		else if (index < 5)
		{
			b2Body_CreateCircle(m_bodyIdes[m_bodyIndex], &shapeDef, &m_circle);
		}
		else
		{
			b2Body_CreateSegment(m_bodyIdes[m_bodyIndex], &shapeDef, &m_segment);
		}

		m_bodyIndex = (m_bodyIndex + 1) % e_maxBodies;
	}

	void DestroyBody()
	{
		for (int32 i = 0; i < e_maxBodies; ++i)
		{
			if (B2_NON_NULL(m_bodyIds[i]))
			{
				b2World_DestroyBody(m_bodyIds[i]);
				m_bodyIds[i] = b2_nullBodyId;
				return;
			}
		}
	}

	void UpdateUI() override
	{
		ImGui::SetNextWindowPos(ImVec2(10.0f, 100.0f));
		ImGui::SetNextWindowSize(ImVec2(210.0f, 300.0f));
		ImGui::Begin("Options", nullptr, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize);

		if (ImGui::Button("Shape 1"))
		{
			Create(0);
		}

		if (ImGui::Button("Shape 2"))
		{
			Create(1);
		}

		if (ImGui::Button("Shape 3"))
		{
			Create(2);
		}

		if (ImGui::Button("Shape 4"))
		{
			Create(3);
		}

		if (ImGui::Button("Shape 5"))
		{
			Create(4);
		}

		if (ImGui::Button("Shape 6"))
		{
			Create(5);
		}

		if (ImGui::Button("Destroy Shape"))
		{
			DestroyBody();
		}

		ImGui::RadioButton("Any", &m_mode, e_any);
		ImGui::RadioButton("Closest", &m_mode, e_closest);
		ImGui::RadioButton("Multiple", &m_mode, e_multiple);

		ImGui::SliderFloat("Angle", &m_degrees, 0.0f, 360.0f, "%.0f");

		ImGui::End();
	}

	void Step(Settings& settings) override
	{
		Test::Step(settings);

		g_debugDraw.DrawString(5, m_textLine, "Shape 7 is intentionally ignored by the ray");
		m_textLine += m_textIncrement;
		switch (m_mode)
		{
			case e_closest:
				g_debugDraw.DrawString(5, m_textLine, "Ray-cast mode: closest - find closest fixture along the ray");
				break;

			case e_any:
				g_debugDraw.DrawString(5, m_textLine, "Ray-cast mode: any - check for obstruction");
				break;

			case e_multiple:
				g_debugDraw.DrawString(5, m_textLine, "Ray-cast mode: multiple - gather multiple fixtures");
				break;
		}

		m_textLine += m_textIncrement;

		float angle = b2_pi * m_degrees / 180.0f;
		float L = 11.0f;
		b2Vec2 point1(0.0f, 10.0f);
		b2Vec2 d(L * cosf(angle), L * sinf(angle));
		b2Vec2 point2 = point1 + d;

		if (m_mode == e_closest)
		{
			RayCastContext context = {0};
			b2World_RayCast(m_worldId, RayCastClosestCallback, point1, point2, b2_defaultQueryFilter, &context);

			if (context.count > 0)
			{
				g_debugDraw.DrawPoint(callback.m_points[0], 5.0f, b2Color(0.4f, 0.9f, 0.4f));
				g_debugDraw.DrawSegment(point1, callback.m_points[0], b2Color(0.8f, 0.8f, 0.8f));
				b2Vec2 head = b2Lerp(callback.m_points[0], callback.m_normals[0], 0.5f);
				g_debugDraw.DrawSegment(callback.m_point, head, b2Color(0.9f, 0.9f, 0.4f));
			}
			else
			{
				g_debugDraw.DrawSegment(point1, point2, b2Color(0.8f, 0.8f, 0.8f));
			}
		}
		else if (m_mode == e_any)
		{
			RayCastContext context = {0};
			b2World_RayCast(m_worldId, RayCastAnyCallback, point1, point2, &context);

			if (callback.count > 0)
			{
				g_debugDraw.DrawPoint(callback.m_points[0], 5.0f, b2Color(0.4f, 0.9f, 0.4f));
				g_debugDraw.DrawSegment(point1, callback.m_points[0], b2Color(0.8f, 0.8f, 0.8f));
				b2Vec2 head = b2Lerp(callback.m_points[0], callback.m_normals[0], 0.5f);
				g_debugDraw.DrawSegment(callback.m_point, head, b2Color(0.9f, 0.9f, 0.4f));
			}
			else
			{
				g_debugDraw.DrawSegment(point1, point2, b2Color(0.8f, 0.8f, 0.8f));
			}
		}
		else if (m_mode == e_multiple)
		{
			RayCastContext context = {0};
			b2World_RayCast(m_worldId, RayCastMultipleCallback, point1, point2, &context;
			g_debugDraw.DrawSegment(point1, point2, b2Color(0.8f, 0.8f, 0.8f));

			for (int32 i = 0; i < callback.m_count; ++i)
			{
				b2Vec2 p = callback.m_points[i];
				b2Vec2 n = callback.m_normals[i];
				g_debugDraw.DrawPoint(p, 5.0f, b2Color(0.4f, 0.9f, 0.4f));
				g_debugDraw.DrawSegment(point1, p, b2Color(0.8f, 0.8f, 0.8f));
				b2Vec2 head = p + 0.5f * n;
				g_debugDraw.DrawSegment(p, head, b2Color(0.9f, 0.9f, 0.4f));
			}
		}
	}

	static Test* Create()
	{
		return new RayCastWorld;
	}

	int m_bodyIndex;
	b2BodyId m_bodyIds[e_maxBodies];
	ShapeUserData m_userData[e_maxBodies];
	b2Polygon m_polygons[4];
	b2Circle m_circle;
	b2Segment m_segment;
	float m_degrees;
	int m_mode;
};

static int sampleRayCastWorld = RegisterTest("Collision", "Ray Cast World", RayCastWorld::Create);
