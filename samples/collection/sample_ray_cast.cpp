// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "sample.h"

#include "box2d/allocate.h"
#include "box2d/hull.h"
#include "box2d/math.h"
#include "box2d/geometry.h"

#include <GLFW/glfw3.h>
#include <imgui.h>


class RayCast : public Sample
{
public:
	RayCast()
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

			b2RayCastOutput localOutput = b2RayCastSegment(&input, &m_segment);
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

	static Sample* Create()
	{
		return new RayCast;
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
