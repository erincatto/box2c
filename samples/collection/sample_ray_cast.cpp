// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "sample.h"

#include "box2d/box2d.h"
#include "box2d/geometry.h"
#include "box2d/hull.h"
#include "box2d/math.h"

#include <GLFW/glfw3.h>
#include <imgui.h>

class RayCast : public Sample
{
public:
	RayCast(const Settings& settings)
		: Sample(settings)
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
		m_startPosition = {0.0f, 0.0f};

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
			m_startPosition = p;

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
			m_transform.p.x = m_basePosition.x + 0.5f * (p.x - m_startPosition.x);
			m_transform.p.y = m_basePosition.y + 0.5f * (p.y - m_startPosition.y);
		}
		else if (m_rotating)
		{
			float dx = p.x - m_startPosition.x;
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

	b2Vec2 m_startPosition;

	bool m_rayDrag;
	bool m_translating;
	bool m_rotating;
	bool m_showFraction;
};

static int sampleIndex = RegisterSample("Collision", "Ray Cast", RayCast::Create);

// This shows how to filter a specific shape using using data.
struct ShapeUserData
{
	int index;
	bool ignore;
};

// Context for ray cast callbacks. Do what you want with this.
struct RayCastContext
{
	b2Vec2 points[3];
	b2Vec2 normals[3];
	float fractions[3];
	int count;
};

// This callback finds the closest hit. This is the most common callback used in games.
static float RayCastClosestCallback(b2ShapeId shapeId, b2Vec2 point, b2Vec2 normal, float fraction, void* context)
{
	RayCastContext* rayContext = (RayCastContext*)context;

	ShapeUserData* userData = (ShapeUserData*)b2Shape_GetUserData(shapeId);
	if (userData != nullptr && userData->ignore)
	{
		// By returning -1, we instruct the calling code to ignore this shape and
		// continue the ray-cast to the next shape.
		return -1.0f;
	}

	rayContext->points[0] = point;
	rayContext->normals[0] = normal;
	rayContext->fractions[0] = fraction;
	rayContext->count = 1;

	// By returning the current fraction, we instruct the calling code to clip the ray and
	// continue the ray-cast to the next shape. WARNING: do not assume that shapes
	// are reported in order. However, by clipping, we can always get the closest shape.
	return fraction;
}

// This callback finds any hit. For this type of query we are usually just checking for obstruction,
// so the hit data is not relevant.
// NOTE: shape hits are not ordered, so this may not return the closest hit
static float RayCastAnyCallback(b2ShapeId shapeId, b2Vec2 point, b2Vec2 normal, float fraction, void* context)
{
	RayCastContext* rayContext = (RayCastContext*)context;

	ShapeUserData* userData = (ShapeUserData*)b2Shape_GetUserData(shapeId);
	if (userData != nullptr && userData->ignore)
	{
		// By returning -1, we instruct the calling code to ignore this shape and
		// continue the ray-cast to the next shape.
		return -1.0f;
	}

	rayContext->points[0] = point;
	rayContext->normals[0] = normal;
	rayContext->fractions[0] = fraction;
	rayContext->count = 1;

	// At this point we have a hit, so we know the ray is obstructed.
	// By returning 0, we instruct the calling code to terminate the ray-cast.
	return 0.0f;
}

// This ray cast collects multiple hits along the ray.
// The shapes are not necessary reported in order, so we might not capture
// the closest shape.
// NOTE: shape hits are not ordered, so this may return hits in any order. This means that
// if you limit the number of results, you may discard the closest hit. You can see this
// behavior in the sample.
static float RayCastMultipleCallback(b2ShapeId shapeId, b2Vec2 point, b2Vec2 normal, float fraction, void* context)
{
	RayCastContext* rayContext = (RayCastContext*)context;

	ShapeUserData* userData = (ShapeUserData*)b2Shape_GetUserData(shapeId);
	if (userData != nullptr && userData->ignore)
	{
		// By returning -1, we instruct the calling code to ignore this shape and
		// continue the ray-cast to the next shape.
		return -1.0f;
	}

	int count = rayContext->count;
	assert(count < 3);

	rayContext->points[count] = point;
	rayContext->normals[count] = normal;
	rayContext->fractions[count] = fraction;
	rayContext->count = count + 1;

	if (rayContext->count == 3)
	{
		// At this point the buffer is full.
		// By returning 0, we instruct the calling code to terminate the ray-cast.
		return 0.0f;
	}

	// By returning 1, we instruct the caller to continue without clipping the ray.
	return 1.0f;
}

// This ray cast collects multiple hits along the ray and sorts them.
static float RayCastSortedCallback(b2ShapeId shapeId, b2Vec2 point, b2Vec2 normal, float fraction, void* context)
{
	RayCastContext* rayContext = (RayCastContext*)context;

	ShapeUserData* userData = (ShapeUserData*)b2Shape_GetUserData(shapeId);
	if (userData != nullptr && userData->ignore)
	{
		// By returning -1, we instruct the calling code to ignore this shape and
		// continue the ray-cast to the next shape.
		return -1.0f;
	}

	int count = rayContext->count;
	assert(count <= 3);

	int index = 3;
	while (fraction < rayContext->fractions[index-1])
	{
		index -= 1;

		if (index == 0)
		{
			break;
		}
	}

	if (index == 3)
	{
		// not closer, continue but tell the caller not to consider fractions further than the largest fraction acquired
		// this only happens once the buffer is full
		assert(rayContext->count == 3);
		assert(rayContext->fractions[2] <= 1.0f);
		return rayContext->fractions[2];
	}

	for (int j = 2; j > index; --j)
	{
		rayContext->points[j] = rayContext->points[j - 1];
		rayContext->normals[j] = rayContext->normals[j - 1];
		rayContext->fractions[j] = rayContext->fractions[j - 1];
	}

	rayContext->points[index] = point;
	rayContext->normals[index] = normal;
	rayContext->fractions[index] = fraction;
	rayContext->count = count < 3 ? count + 1 : 3;

	if (rayContext->count == 3)
	{
		return rayContext->fractions[2];
	}

	// By returning 1, we instruct the caller to continue without clipping the ray.
	return 1.0f;
}

class RayCastWorld : public Sample
{
public:
	enum Mode
	{
		e_any = 0,
		e_closest = 1,
		e_multiple = 2,
		e_sorted = 3
	};

	enum
	{
		e_maxCount = 64
	};

	RayCastWorld(const Settings& settings)
		: Sample(settings)
	{
		// Ground body
		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			b2BodyId groundId = b2World_CreateBody(m_worldId, &bodyDef);

			b2ShapeDef shapeDef = b2DefaultShapeDef();
			b2Segment segment = {{-40.0f, 0.0f}, {40.0f, 0.0f}};
			b2Body_CreateSegment(groundId, &shapeDef, &segment);
		}

		{
			b2Vec2 vertices[3] = {{-0.5f, 0.0f}, {0.5f, 0.0f}, {0.0f, 1.5f}};
			b2Hull hull = b2ComputeHull(vertices, 3);
			m_polygons[0] = b2MakePolygon(&hull, 0.0f);
		}

		{
			b2Vec2 vertices[3] = {{-0.1f, 0.0f}, {0.1f, 0.0f}, {0.0f, 1.5f}};
			b2Hull hull = b2ComputeHull(vertices, 3);
			m_polygons[1] = b2MakePolygon(&hull, 0.0f);
			m_polygons[1].radius = 0.5f;
		}

		{
			float w = 1.0f;
			float b = w / (2.0f + sqrtf(2.0f));
			float s = sqrtf(2.0f) * b;

			b2Vec2 vertices[8] = {{0.5f * s, 0.0f}, {0.5f * w, b},		{0.5f * w, b + s}, {0.5f * s, w},
								  {-0.5f * s, w},	{-0.5f * w, b + s}, {-0.5f * w, b},	   {-0.5f * s, 0.0f}};

			b2Hull hull = b2ComputeHull(vertices, 8);
			m_polygons[2] = b2MakePolygon(&hull, 0.0f);
		}

		m_polygons[3] = b2MakeBox(0.5f, 0.5f);
		m_capsule = {{-0.5f, 0.0f}, {0.5f, 0.0f}, 0.25f};
		m_circle = {{0.0f, 0.0f}, 0.5f};
		m_segment = {{-1.0f, 0.0f}, {1.0f, 0.0f}};

		m_bodyIndex = 0;

		for (int i = 0; i < e_maxCount; ++i)
		{
			m_bodyIds[i] = b2_nullBodyId;
		}

		m_mode = e_closest;
		m_ignoreIndex = 7;

		m_rayStart = {-20.0f, 10.0f};
		m_rayEnd = {20.0f, 10.0f};
		m_rayRadius = 0.0f;
		m_rayDrag = false;
	}

	void Create(int index)
	{
		if (B2_NON_NULL(m_bodyIds[m_bodyIndex]))
		{
			b2World_DestroyBody(m_bodyIds[m_bodyIndex]);
			m_bodyIds[m_bodyIndex] = b2_nullBodyId;
		}

		float x = RandomFloat(-20.0f, 20.0f);
		float y = RandomFloat(0.0f, 20.0f);

		b2BodyDef bodyDef = b2DefaultBodyDef();
		bodyDef.position = {x, y};
		bodyDef.angle = RandomFloat(-b2_pi, b2_pi);

		m_bodyIds[m_bodyIndex] = b2World_CreateBody(m_worldId, &bodyDef);

		b2ShapeDef shapeDef = b2DefaultShapeDef();
		shapeDef.userData = m_userData + m_bodyIndex;
		m_userData[m_bodyIndex].ignore = false;
		if (m_bodyIndex == m_ignoreIndex)
		{
			m_userData[m_bodyIndex].ignore = true;
		}

		if (index < 4)
		{
			b2Body_CreatePolygon(m_bodyIds[m_bodyIndex], &shapeDef, m_polygons + index);
		}
		else if (index == 4)
		{
			b2Body_CreateCircle(m_bodyIds[m_bodyIndex], &shapeDef, &m_circle);
		}
		else if (index == 5)
		{
			b2Body_CreateCapsule(m_bodyIds[m_bodyIndex], &shapeDef, &m_capsule);
		}
		else
		{
			b2Body_CreateSegment(m_bodyIds[m_bodyIndex], &shapeDef, &m_segment);
		}

		m_bodyIndex = (m_bodyIndex + 1) % e_maxCount;
	}

	void CreateN(int index, int count)
	{
		for (int i = 0; i < count; ++i)
		{
			Create(index);
		}
	}

	void DestroyBody()
	{
		for (int i = 0; i < e_maxCount; ++i)
		{
			if (B2_NON_NULL(m_bodyIds[i]))
			{
				b2World_DestroyBody(m_bodyIds[i]);
				m_bodyIds[i] = b2_nullBodyId;
				return;
			}
		}
	}

	void MouseDown(b2Vec2 p, int button, int mods) override
	{
		if (button == GLFW_MOUSE_BUTTON_1)
		{
			m_rayStart = p;
			m_rayEnd = p;
			m_rayDrag = true;
		}
	}

	void MouseUp(b2Vec2, int button) override
	{
		m_rayDrag = false;
	}

	void MouseMove(b2Vec2 p) override
	{
		if (m_rayDrag)
		{
			m_rayEnd = p;
		}
	}

	void UpdateUI() override
	{
		ImGui::SetNextWindowPos(ImVec2(10.0f, 100.0f));
		ImGui::SetNextWindowSize(ImVec2(210.0f, 330.0f));
		ImGui::Begin("Options", nullptr, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize);

		if (ImGui::SliderFloat("radius", &m_rayRadius, 0.0f, 2.0f, "%.1f"))
		{
		}

		if (ImGui::Button("Polygon 1"))
			Create(0);
		ImGui::SameLine();
		if (ImGui::Button("10x##Poly1"))
			CreateN(0, 10);

		if (ImGui::Button("Polygon 2"))
			Create(1);
		ImGui::SameLine();
		if (ImGui::Button("10x##Poly2"))
			CreateN(1, 10);

		if (ImGui::Button("Polygon 3"))
			Create(2);
		ImGui::SameLine();
		if (ImGui::Button("10x##Poly3"))
			CreateN(2, 10);

		if (ImGui::Button("Box"))
			Create(3);
		ImGui::SameLine();
		if (ImGui::Button("10x##Box"))
			CreateN(3, 10);

		if (ImGui::Button("Circle"))
			Create(4);
		ImGui::SameLine();
		if (ImGui::Button("10x##Circle"))
			CreateN(4, 10);

		if (ImGui::Button("Capsule"))
			Create(5);
		ImGui::SameLine();
		if (ImGui::Button("10x##Capsule"))
			CreateN(5, 10);

		if (ImGui::Button("Segment"))
			Create(6);
		ImGui::SameLine();
		if (ImGui::Button("10x##Segment"))
			CreateN(6, 10);

		if (ImGui::Button("Destroy Shape"))
		{
			DestroyBody();
		}

		ImGui::RadioButton("Any", &m_mode, e_any);
		ImGui::RadioButton("Closest", &m_mode, e_closest);
		ImGui::RadioButton("Multiple", &m_mode, e_multiple);
		ImGui::RadioButton("Sorted", &m_mode, e_sorted);

		ImGui::End();
	}

	void Step(Settings& settings) override
	{
		Sample::Step(settings);

		g_draw.DrawString(5, m_textLine, "Click left mouse button and drag to modify ray cast");
		m_textLine += m_textIncrement;
		g_draw.DrawString(5, m_textLine, "Shape 7 is intentionally ignored by the ray");
		m_textLine += m_textIncrement;
		switch (m_mode)
		{
			case e_closest:
				g_draw.DrawString(5, m_textLine, "Ray-cast mode: closest - find closest shape along the ray");
				break;

			case e_any:
				g_draw.DrawString(5, m_textLine, "Ray-cast mode: any - check for obstruction - unsorted");
				break;

			case e_multiple:
				g_draw.DrawString(5, m_textLine, "Ray-cast mode: multiple - gather multiple shapes - unsorted");
				break;

			case e_sorted:
				g_draw.DrawString(5, m_textLine, "Ray-cast mode: sorted - gather multiple shapes sorted by closeness");
				break;
		}

		m_textLine += m_textIncrement;

		b2Color color1 = {0.4f, 0.9f, 0.4f, 1.0f};
		b2Color color2 = {0.8f, 0.8f, 0.8f, 1.0f};
		b2Color color3 = {0.9f, 0.9f, 0.4f, 1.0f};
		b2Color green = b2MakeColor(b2_colorGreen, 0.7f);
		b2Color yellow = b2MakeColor(b2_colorYellow, 0.7f);
		b2Color gray = b2MakeColor(b2_colorGray, 0.7f);

		b2Vec2 rayTranslation = b2Sub(m_rayEnd, m_rayStart);

		if (m_mode == e_closest)
		{
			RayCastContext context = {0};
			b2World_RayCast(m_worldId, m_rayStart, rayTranslation, m_rayRadius, b2_defaultQueryFilter, RayCastClosestCallback, &context);

			if (context.count > 0)
			{
				b2Vec2 c = b2MulAdd(m_rayStart, context.fractions[0], rayTranslation);
				g_draw.DrawPoint(context.points[0], 5.0f, color1);
				g_draw.DrawSegment(m_rayStart, c, color2);
				b2Vec2 head = b2MulAdd(context.points[0], 0.5f, context.normals[0]);
				g_draw.DrawSegment(context.points[0], head, color3);

				if (m_rayRadius > 0.0f)
				{
					g_draw.DrawCircle(b2MulAdd(m_rayStart, context.fractions[0], rayTranslation), m_rayRadius, yellow);
				}
			}
			else
			{
				g_draw.DrawSegment(m_rayStart, m_rayEnd, color2);

				if (m_rayRadius > 0.0f)
				{
					g_draw.DrawCircle(b2MulAdd(m_rayEnd, context.fractions[0], rayTranslation), m_rayRadius, gray);
				}
			}
		}
		else if (m_mode == e_any)
		{
			RayCastContext context = {0};
			b2World_RayCast(m_worldId, m_rayStart, rayTranslation, m_rayRadius, b2_defaultQueryFilter, RayCastAnyCallback, &context);

			if (context.count > 0)
			{
				b2Vec2 c = b2MulAdd(m_rayStart, context.fractions[0], rayTranslation);
				g_draw.DrawPoint(context.points[0], 5.0f, color1);
				g_draw.DrawSegment(m_rayStart, c, color2);
				b2Vec2 head = b2MulAdd(context.points[0], 0.5f, context.normals[0]);
				g_draw.DrawSegment(context.points[0], head, color3);

				if (m_rayRadius > 0.0f)
				{
					g_draw.DrawCircle(b2MulAdd(m_rayStart, context.fractions[0], rayTranslation), m_rayRadius, yellow);
				}
			}
			else
			{
				g_draw.DrawSegment(m_rayStart, m_rayEnd, color2);

				if (m_rayRadius > 0.0f)
				{
					g_draw.DrawCircle(b2MulAdd(m_rayEnd, context.fractions[0], rayTranslation), m_rayRadius, gray);
				}
			}
		}
		else if (m_mode == e_multiple)
		{
			RayCastContext context = {0};
			b2World_RayCast(m_worldId, m_rayStart, rayTranslation,  m_rayRadius, b2_defaultQueryFilter, RayCastMultipleCallback, &context);

			if (context.count > 0)
			{
				for (int i = 0; i < context.count; ++i)
				{
					b2Vec2 p = context.points[i];
					b2Vec2 n = context.normals[i];
					b2Vec2 c = b2MulAdd(m_rayStart, context.fractions[i], rayTranslation);
					g_draw.DrawPoint(p, 5.0f, color1);
					g_draw.DrawSegment(m_rayStart, c, color2);
					b2Vec2 head = b2MulAdd(p, 0.5f, n);
					g_draw.DrawSegment(p, head, color3);

					if (m_rayRadius > 0.0f)
					{
						g_draw.DrawCircle(b2MulAdd(m_rayStart, context.fractions[i], rayTranslation), m_rayRadius, yellow);
					}
				}
			}
			else
			{
				g_draw.DrawSegment(m_rayStart, m_rayEnd, color2);

				if (m_rayRadius > 0.0f)
				{
					g_draw.DrawCircle(b2MulAdd(m_rayEnd, context.fractions[0], rayTranslation), m_rayRadius, gray);
				}
			}
		}
		else if (m_mode == e_sorted)
		{
			RayCastContext context = {0};

			// Must initialize fractions for sorting
			context.fractions[0] = FLT_MAX;
			context.fractions[1] = FLT_MAX;
			context.fractions[2] = FLT_MAX;

			b2World_RayCast(m_worldId, m_rayStart, rayTranslation, m_rayRadius, b2_defaultQueryFilter, RayCastSortedCallback, &context);

			if (context.count > 0)
			{
				assert(context.count <= 3);
				b2Color colors[3] = {b2MakeColor(b2_colorRed, 1.0f), b2MakeColor(b2_colorGreen, 1.0f),
									 b2MakeColor(b2_colorBlue, 1.0f)};
				for (int i = 0; i < context.count; ++i)
				{
					b2Vec2 c = b2MulAdd(m_rayStart, context.fractions[i], rayTranslation);
					b2Vec2 p = context.points[i];
					b2Vec2 n = context.normals[i];
					g_draw.DrawPoint(p, 5.0f, colors[i]);
					g_draw.DrawSegment(m_rayStart, c, color2);
					b2Vec2 head = b2MulAdd(p, 0.5f, n);
					g_draw.DrawSegment(p, head, color3);

					if (m_rayRadius > 0.0f)
					{
						g_draw.DrawCircle(b2MulAdd(m_rayStart, context.fractions[i], rayTranslation), m_rayRadius, yellow);
					}
				}
			}
			else
			{
				g_draw.DrawSegment(m_rayStart, m_rayEnd, color2);

				if (m_rayRadius > 0.0f)
				{
					g_draw.DrawCircle(b2MulAdd(m_rayEnd, context.fractions[0], rayTranslation), m_rayRadius, gray);
				}
			}
		}

		g_draw.DrawPoint(m_rayStart, 5.0f, green);

		if (B2_NON_NULL(m_bodyIds[m_ignoreIndex]))
		{
			b2Vec2 p = b2Body_GetPosition(m_bodyIds[m_ignoreIndex]);
			p.x -= 0.2f;
			g_draw.DrawString(p, "ign");
		}
	}

	static Sample* Create(const Settings& settings)
	{
		return new RayCastWorld(settings);
	}

	int m_bodyIndex;
	b2BodyId m_bodyIds[e_maxCount];
	ShapeUserData m_userData[e_maxCount];
	b2Polygon m_polygons[4];
	b2Capsule m_capsule;
	b2Circle m_circle;
	b2Segment m_segment;
	int m_mode;
	int m_ignoreIndex;

	b2Vec2 m_rayStart;
	b2Vec2 m_rayEnd;
	float m_rayRadius;
	bool m_rayDrag;
};

static int sampleRayCastWorld = RegisterSample("Collision", "Ray Cast World", RayCastWorld::Create);


class OverlapWorld : public Sample
{
public:
	enum
	{
		e_circleShape = 0,
		e_capsuleShape = 1,
		e_boxShape = 2
	};

	enum
	{
		e_maxCount = 64,
		e_maxDoomed = 16,
	};

	static bool OverlapResultFcn(b2ShapeId shapeId, void* context)
	{
		ShapeUserData* userData = (ShapeUserData*)b2Shape_GetUserData(shapeId);
		if (userData != nullptr && userData->ignore)
		{
			// continue the query
			return true;
		}

		OverlapWorld* sample = (OverlapWorld*)context;

		if (sample->m_doomCount < e_maxDoomed)
		{
			int index = sample->m_doomCount;
			sample->m_doomIds[index] = shapeId;
			sample->m_doomCount += 1;
		}

		// continue the query
		return true;
	}

	OverlapWorld(const Settings& settings)
		: Sample(settings)
	{
		{
			b2Vec2 vertices[3] = {{-0.5f, 0.0f}, {0.5f, 0.0f}, {0.0f, 1.5f}};
			b2Hull hull = b2ComputeHull(vertices, 3);
			m_polygons[0] = b2MakePolygon(&hull, 0.0f);
		}

		{
			b2Vec2 vertices[3] = {{-0.1f, 0.0f}, {0.1f, 0.0f}, {0.0f, 1.5f}};
			b2Hull hull = b2ComputeHull(vertices, 3);
			m_polygons[1] = b2MakePolygon(&hull, 0.0f);
		}

		{
			float w = 1.0f;
			float b = w / (2.0f + sqrtf(2.0f));
			float s = sqrtf(2.0f) * b;

			b2Vec2 vertices[8] = {{0.5f * s, 0.0f}, {0.5f * w, b},		{0.5f * w, b + s}, {0.5f * s, w},
								  {-0.5f * s, w},	{-0.5f * w, b + s}, {-0.5f * w, b},	   {-0.5f * s, 0.0f}};

			b2Hull hull = b2ComputeHull(vertices, 8);
			m_polygons[2] = b2MakePolygon(&hull, 0.0f);
		}

		m_polygons[3] = b2MakeBox(0.5f, 0.5f);
		m_capsule = {{-0.5f, 0.0f}, {0.5f, 0.0f}, 0.25f};
		m_circle = {{0.0f, 0.0f}, 0.5f};
		m_segment = {{-1.0f, 0.0f}, {1.0f, 0.0f}};

		m_bodyIndex = 0;

		for (int i = 0; i < e_maxCount; ++i)
		{
			m_bodyIds[i] = b2_nullBodyId;
		}

		m_ignoreIndex = 7;

		m_shapeType = e_circleShape;

		m_queryCircle = {{0.0f, 0.0f}, 1.0f};
		m_queryCapsule = {{-1.0f, 0.0f}, {1.0f, 0.0f}, 0.5f};
		m_queryBox = b2MakeBox(2.0f, 0.5f);

		m_position = {0.0f, 10.0f};
		m_angle = 0.0f;
		m_dragging = false;
		m_rotating = false;

		m_doomCount = 0;
	}

	void Create(int index)
	{
		if (B2_NON_NULL(m_bodyIds[m_bodyIndex]))
		{
			b2World_DestroyBody(m_bodyIds[m_bodyIndex]);
			m_bodyIds[m_bodyIndex] = b2_nullBodyId;
		}

		float x = RandomFloat(-20.0f, 20.0f);
		float y = RandomFloat(0.0f, 20.0f);

		b2BodyDef bodyDef = b2DefaultBodyDef();
		bodyDef.position = {x, y};
		bodyDef.angle = RandomFloat(-b2_pi, b2_pi);

		m_bodyIds[m_bodyIndex] = b2World_CreateBody(m_worldId, &bodyDef);

		b2ShapeDef shapeDef = b2DefaultShapeDef();
		shapeDef.userData = m_userData + m_bodyIndex;
		m_userData[m_bodyIndex].index = m_bodyIndex;
		m_userData[m_bodyIndex].ignore = false;
		if (m_bodyIndex == m_ignoreIndex)
		{
			m_userData[m_bodyIndex].ignore = true;
		}

		if (index < 4)
		{
			b2Body_CreatePolygon(m_bodyIds[m_bodyIndex], &shapeDef, m_polygons + index);
		}
		else if (index == 4)
		{
			b2Body_CreateCircle(m_bodyIds[m_bodyIndex], &shapeDef, &m_circle);
		}
		else if (index == 5)
		{
			b2Body_CreateCapsule(m_bodyIds[m_bodyIndex], &shapeDef, &m_capsule);
		}
		else
		{
			b2Body_CreateSegment(m_bodyIds[m_bodyIndex], &shapeDef, &m_segment);
		}

		m_bodyIndex = (m_bodyIndex + 1) % e_maxCount;
	}

	void CreateN(int index, int count)
	{
		for (int i = 0; i < count; ++i)
		{
			Create(index);
		}
	}

	void DestroyBody()
	{
		for (int i = 0; i < e_maxCount; ++i)
		{
			if (B2_NON_NULL(m_bodyIds[i]))
			{
				b2World_DestroyBody(m_bodyIds[i]);
				m_bodyIds[i] = b2_nullBodyId;
				return;
			}
		}
	}

	void MouseDown(b2Vec2 p, int button, int mods) override
	{
		if (button == GLFW_MOUSE_BUTTON_1)
		{
			if (mods == 0 && m_rotating == false)
			{
				m_dragging = true;
				m_position = p;
			}
			else if (mods == GLFW_MOD_SHIFT && m_dragging == false)
			{
				m_rotating = true;
				m_startPosition = p;
				m_baseAngle = m_angle;
			}
		}
	}

	void MouseUp(b2Vec2, int button) override
	{
		if (button == GLFW_MOUSE_BUTTON_1)
		{
			m_dragging = false;
			m_rotating = false;
		}
	}

	void MouseMove(b2Vec2 p) override
	{
		if (m_dragging)
		{
			m_position = p;
		}
		else if (m_rotating)
		{
			float dx = p.x - m_startPosition.x;
			m_angle = m_baseAngle + 1.0f * dx;
		}
	}

	void UpdateUI() override
	{
		ImGui::SetNextWindowPos(ImVec2(10.0f, 100.0f));
		ImGui::SetNextWindowSize(ImVec2(210.0f, 310.0f));
		ImGui::Begin("Options", nullptr, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize);

		if (ImGui::Button("Polygon 1"))
			Create(0);
		ImGui::SameLine();
		if (ImGui::Button("10x##Poly1"))
			CreateN(0, 10);

		if (ImGui::Button("Polygon 2"))
			Create(1);
		ImGui::SameLine();
		if (ImGui::Button("10x##Poly2"))
			CreateN(1, 10);

		if (ImGui::Button("Polygon 3"))
			Create(2);
		ImGui::SameLine();
		if (ImGui::Button("10x##Poly3"))
			CreateN(2, 10);

		if (ImGui::Button("Box"))
			Create(3);
		ImGui::SameLine();
		if (ImGui::Button("10x##Box"))
			CreateN(3, 10);

		if (ImGui::Button("Circle"))
			Create(4);
		ImGui::SameLine();
		if (ImGui::Button("10x##Circle"))
			CreateN(4, 10);

		if (ImGui::Button("Capsule"))
			Create(5);
		ImGui::SameLine();
		if (ImGui::Button("10x##Capsule"))
			CreateN(5, 10);

		if (ImGui::Button("Segment"))
			Create(6);
		ImGui::SameLine();
		if (ImGui::Button("10x##Segment"))
			CreateN(6, 10);

		if (ImGui::Button("Destroy Shape"))
		{
			DestroyBody();
		}

		ImGui::Separator();
		ImGui::Text("Overlap Shape");
		ImGui::RadioButton("Circle##Overlap", &m_shapeType, e_circleShape);
		ImGui::RadioButton("Capsule##Overlap", &m_shapeType, e_capsuleShape);
		ImGui::RadioButton("Box##Overlap", &m_shapeType, e_boxShape);

		ImGui::End();
	}

	void Step(Settings& settings) override
	{
		Sample::Step(settings);

		g_draw.DrawString(5, m_textLine, "left mouse button: drag query shape");
		m_textLine += m_textIncrement;
		g_draw.DrawString(5, m_textLine, "left moust button + shift: rotate query shape");
		m_textLine += m_textIncrement;

		m_doomCount = 0;

		b2Color color = b2MakeColor(b2_colorWhite, 1.0f);
		b2Transform transform = {m_position, b2MakeRot(m_angle)};

		if (m_shapeType == e_circleShape)
		{
			b2World_OverlapCircle(m_worldId, OverlapWorld::OverlapResultFcn, &m_queryCircle, transform, b2_defaultQueryFilter, this);
			g_draw.DrawCircle(transform.p, m_queryCircle.radius, color);
		}
		else if (m_shapeType == e_capsuleShape)
		{
			b2World_OverlapCapsule(m_worldId, OverlapWorld::OverlapResultFcn, &m_queryCapsule, transform, b2_defaultQueryFilter, this);
			b2Vec2 p1 = b2TransformPoint(transform, m_queryCapsule.point1);
			b2Vec2 p2 = b2TransformPoint(transform, m_queryCapsule.point2);
			g_draw.DrawCapsule(p1, p2, m_queryCapsule.radius, color);
		}
		else if (m_shapeType == e_boxShape)
		{
			b2World_OverlapPolygon(m_worldId, OverlapWorld::OverlapResultFcn, &m_queryBox, transform, b2_defaultQueryFilter, this);
			b2Vec2 points[b2_maxPolygonVertices] = {0};
			for (int i = 0; i < m_queryBox.count; ++i)
			{
				points[i] = b2TransformPoint(transform, m_queryBox.vertices[i]);
			}
			g_draw.DrawPolygon(points, m_queryBox.count, color);
		}

		if (B2_NON_NULL(m_bodyIds[m_ignoreIndex]))
		{
			b2Vec2 p = b2Body_GetPosition(m_bodyIds[m_ignoreIndex]);
			p.x -= 0.2f;
			g_draw.DrawString(p, "ign");
		}

		for (int i = 0; i < m_doomCount; ++i)
		{
			b2ShapeId shapeId = m_doomIds[i];
			ShapeUserData* userData = (ShapeUserData*)b2Shape_GetUserData(shapeId);
			if (userData == nullptr)
			{
				continue;
			}

			int index = userData->index;
			assert(0 <= index && index < e_maxCount);
			assert(B2_NON_NULL(m_bodyIds[index]));

			b2World_DestroyBody(m_bodyIds[index]);
			m_bodyIds[index] = b2_nullBodyId;
		}
	}

	static Sample* Create(const Settings& settings)
	{
		return new OverlapWorld(settings);
	}

	int m_bodyIndex;
	b2BodyId m_bodyIds[e_maxCount];
	ShapeUserData m_userData[e_maxCount];
	b2Polygon m_polygons[4];
	b2Capsule m_capsule;
	b2Circle m_circle;
	b2Segment m_segment;
	int m_ignoreIndex;

	b2ShapeId m_doomIds[e_maxDoomed];
	int m_doomCount;

	b2Circle m_queryCircle;
	b2Capsule m_queryCapsule;
	b2Polygon m_queryBox;

	int m_shapeType;
	b2Transform m_transform;

	b2Vec2 m_startPosition;

	b2Vec2 m_position;
	b2Vec2 m_basePosition;
	float m_angle;
	float m_baseAngle;
	
	bool m_dragging;
	bool m_rotating;
};

static int sampleOverlapWorld = RegisterSample("Collision", "Overlap World", OverlapWorld::Create);
