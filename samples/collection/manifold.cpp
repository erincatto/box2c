// SPDX-FileCopyrightText: 2022 Erin Catto
// SPDX-License-Identifier: MIT

#include "box2d/manifold.h"
#include "box2d/math.h"
#include "box2d/shapes.h"
#include "sample.h"

#include <GLFW/glfw3.h>
#include <imgui.h>

// Tests manifolds and contact points
class Manifold : public Sample
{
public:
	Manifold()
	{
		m_circle1 = {{0.0f, 0.0f}, 0.5f};
		m_circle2 = {{0.0f, 0.0f}, 1.0f};
		m_box = b2MakeBox(0.5f, 0.5f, {0.0f, 0.0f}, 0.0f);

		m_segment = {{-1.0f, 0.0f}, {1.0f, 0.0}};
		m_smoothSegment = {{2.0f, 1.0f}, {1.0f, 0.0f}, {-1.0f, 0.0}, {-2.0f, -1.0f}};

		m_transform = b2Transform_identity;
		m_angle = 0.0f;

		m_startPoint = {0.0f, 0.0f};
		m_basePosition = {0.0f, 0.0f};
		m_baseAngle = 0.0f;

		m_dragging = false;
		m_rotating = false;
		m_showIds = false;
		m_showSeparation = false;
	}

	void UpdateUI() override
	{
		ImGui::SetNextWindowPos(ImVec2(10.0f, 100.0f));
		ImGui::SetNextWindowSize(ImVec2(230.0f, 210.0f));
		ImGui::Begin("Manifold Controls", nullptr, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize);

		if (ImGui::SliderFloat("x offset", &m_transform.p.x, -2.0f, 2.0f, "%.2f"))
		{
		}

		if (ImGui::SliderFloat("y offset", &m_transform.p.y, -2.0f, 2.0f, "%.2f"))
		{
		}

		if (ImGui::SliderFloat("angle", &m_angle, -b2_pi, b2_pi, "%.2f"))
		{
			m_transform.q = b2Rot_Set(m_angle);
		}

		if (ImGui::Checkbox("show ids", &m_showIds))
		{
		}

		if (ImGui::Checkbox("show separation", &m_showSeparation))
		{
		}

		if (ImGui::Button("Reset"))
		{
			m_transform = b2Transform_identity;
			m_angle = 0.0f;
		}

		ImGui::Separator();

		ImGui::Text("mouse button 1: drag");
		ImGui::Text("mouse button 1 + shift: rotate");

		ImGui::End();
	}

	void MouseDown(b2Vec2 p, int button, int mods) override
	{
		if (button == GLFW_MOUSE_BUTTON_1)
		{
			if (mods == 0 && m_rotating == false)
			{
				m_dragging = true;
				m_startPoint = p;
				m_basePosition = m_transform.p;
			}
			else if (mods == GLFW_MOD_SHIFT && m_dragging == false)
			{
				m_rotating = true;
				m_startPoint = p;
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
			m_transform.p.x = m_basePosition.x + 0.5f * (p.x - m_startPoint.x);
			m_transform.p.y = m_basePosition.y + 0.5f * (p.y - m_startPoint.y);
		}
		else if (m_rotating)
		{
			float dx = p.x - m_startPoint.x;
			m_angle = B2_CLAMP(m_baseAngle + 1.0f * dx, -b2_pi, b2_pi);
			m_transform.q = b2Rot_Set(m_angle);
		}
	}

	void DrawManifold(const b2Manifold* m, const b2WorldManifold* wm)
	{
		b2Color white = {1.0f, 1.0f, 1.0f, 1.0f};
		b2Color green = {0.0f, 1.0f, 0.0f, 1.0f};

		for (int i = 0; i < m->pointCount; ++i)
		{
			b2Vec2 p1 = wm->points[i];
			b2Vec2 p2 = b2MulAdd(p1, 0.5f, wm->normal);
			g_debugDraw.DrawSegment(p1, p2, white);
			g_debugDraw.DrawPoint(p1, 5.0f, green);

			if (m_showIds)
			{
				b2Vec2 p = {p1.x + 0.05f, p1.y - 0.02f};
				g_debugDraw.DrawString(p, "%x", m->points[i].id.key);
			}

			if (m_showSeparation)
			{
				b2Vec2 p = {p1.x + 0.05f, p1.y + 0.03f};
				g_debugDraw.DrawString(p, "%.3f", wm->separations[i]);
			}
		}
	}

	void Step(Settings&) override
	{
		b2Vec2 offset = {-20.0f, 10.0f};
		b2Vec2 increment = {6.0f, 0.0f};

		b2Color color1 = {0.3f, 0.8f, 0.6f, 1.0f};
		b2Color color2 = {0.8f, 0.6f, 0.3f, 1.0f};
		b2Color dim1 = {0.5f * color1.r, 0.5f * color1.g, 0.5f * color1.b, 1.0f};

		// circle-circle
		{
			b2Transform xf1 = {offset, b2Rot_identity};
			b2Transform xf2 = {b2Add(m_transform.p, offset), m_transform.q};

			b2Manifold m = b2CollideCircles(&m_circle1, &m_circle2);
			b2WorldManifold wm = b2ComputeWorldManifold(&m, xf1, m_circle1.radius, xf2, m_circle2.radius);

			b2Vec2 c1 = b2TransformPoint(xf1, m_circle1.point);
			b2Vec2 c2 = b2TransformPoint(xf2, m_circle2.point);
			b2Vec2 axis1 = b2RotateVector(xf1.q, {1.0f, 0.0f});
			b2Vec2 axis2 = b2RotateVector(xf2.q, {1.0f, 0.0f});
			g_debugDraw.DrawSolidCircle(c1, m_circle1.radius, axis1, color1);
			g_debugDraw.DrawSolidCircle(c2, m_circle2.radius, axis2, color2);

			DrawManifold(&m, &wm);

			offset = b2Add(offset, increment);
		}

		// box-circle
		{
			b2Transform xf1 = {offset, b2Rot_identity};
			b2Transform xf2 = {b2Add(m_transform.p, offset), m_transform.q};

			b2Manifold m = b2CollidePolygonAndCircle(&m_box, xf1, &m_circle1, xf2);
			b2WorldManifold wm = b2ComputeWorldManifold(&m, xf1, 0.0f, xf2, m_circle1.radius);

			b2Vec2 vertices[b2_maxPolygonVertices];
			for (int i = 0; i < m_box.count; ++i)
			{
				vertices[i] = b2TransformPoint(xf1, m_box.vertices[i]);
			}
			g_debugDraw.DrawPolygon(vertices, m_box.count, color1);

			b2Vec2 c2 = b2TransformPoint(xf2, m_circle1.point);
			b2Vec2 axis2 = b2RotateVector(xf2.q, {1.0f, 0.0f});
			g_debugDraw.DrawSolidCircle(c2, m_circle1.radius, axis2, color2);

			DrawManifold(&m, &wm);

			offset = b2Add(offset, increment);
		}

		// box-box
		{
			b2Transform xf1 = {offset, b2Rot_identity};
			b2Transform xf2 = {b2Add(m_transform.p, offset), m_transform.q};

			b2Manifold m = b2CollidePolygons(&m_box, xf1, &m_box, xf2);
			b2WorldManifold wm = b2ComputeWorldManifold(&m, xf1, 0.0f, xf2, 0.0f);

			b2Vec2 vertices[b2_maxPolygonVertices];
			for (int i = 0; i < m_box.count; ++i)
			{
				vertices[i] = b2TransformPoint(xf1, m_box.vertices[i]);
			}
			g_debugDraw.DrawPolygon(vertices, m_box.count, color1);

			for (int i = 0; i < m_box.count; ++i)
			{
				vertices[i] = b2TransformPoint(xf2, m_box.vertices[i]);
			}
			g_debugDraw.DrawPolygon(vertices, m_box.count, color2);

			DrawManifold(&m, &wm);

			offset = b2Add(offset, increment);
		}

		// segment-circle
		{
			b2Transform xf1 = {offset, b2Rot_identity};
			b2Transform xf2 = {b2Add(m_transform.p, offset), m_transform.q};

			b2Manifold m = b2CollideSegmentAndCircle(&m_segment, xf1, &m_circle1, xf2);
			b2WorldManifold wm = b2ComputeWorldManifold(&m, xf1, 0.0f, xf2, m_circle1.radius);

			b2Vec2 p1 = b2TransformPoint(xf1, m_segment.point1);
			b2Vec2 p2 = b2TransformPoint(xf1, m_segment.point2);
			g_debugDraw.DrawSegment(p1, p2, color1);

			b2Vec2 c2 = b2TransformPoint(xf2, m_circle1.point);
			b2Vec2 axis2 = b2RotateVector(xf2.q, {1.0f, 0.0f});
			g_debugDraw.DrawSolidCircle(c2, m_circle1.radius, axis2, color2);

			DrawManifold(&m, &wm);

			offset = b2Add(offset, increment);
		}

		// smooth segment-circle
		{
			b2Transform xf1 = {offset, b2Rot_identity};
			b2Transform xf2 = {b2Add(m_transform.p, offset), m_transform.q};

			b2Manifold m = b2CollideSmoothSegmentAndCircle(&m_smoothSegment, xf1, &m_circle1, xf2);
			b2WorldManifold wm = b2ComputeWorldManifold(&m, xf1, 0.0f, xf2, m_circle1.radius);

			b2Vec2 p1 = b2TransformPoint(xf1, m_smoothSegment.point1);
			b2Vec2 p2 = b2TransformPoint(xf1, m_smoothSegment.point2);
			g_debugDraw.DrawSegment(p1, p2, color1);

			p1 = b2TransformPoint(xf1, m_smoothSegment.ghost1);
			p2 = b2TransformPoint(xf1, m_smoothSegment.point1);
			g_debugDraw.DrawSegment(p1, p2, dim1);

			p1 = b2TransformPoint(xf1, m_smoothSegment.point2);
			p2 = b2TransformPoint(xf1, m_smoothSegment.ghost2);
			g_debugDraw.DrawSegment(p1, p2, dim1);

			b2Vec2 c2 = b2TransformPoint(xf2, m_circle1.point);
			b2Vec2 axis2 = b2RotateVector(xf2.q, {1.0f, 0.0f});
			g_debugDraw.DrawSolidCircle(c2, m_circle1.radius, axis2, color2);

			DrawManifold(&m, &wm);

			offset = b2Add(offset, increment);
		}

		// segment-box
		{
			b2Transform xf1 = {offset, b2Rot_identity};
			b2Transform xf2 = {b2Add(m_transform.p, offset), m_transform.q};

			b2Manifold m = b2CollideSegmentAndPolygon(&m_segment, xf1, &m_box, xf2);
			b2WorldManifold wm = b2ComputeWorldManifold(&m, xf1, 0.0f, xf2, 0.0f);

			b2Vec2 p1 = b2TransformPoint(xf1, m_segment.point1);
			b2Vec2 p2 = b2TransformPoint(xf1, m_segment.point2);
			g_debugDraw.DrawSegment(p1, p2, color1);

			b2Vec2 vertices[b2_maxPolygonVertices];
			for (int i = 0; i < m_box.count; ++i)
			{
				vertices[i] = b2TransformPoint(xf2, m_box.vertices[i]);
			}
			g_debugDraw.DrawPolygon(vertices, m_box.count, color2);

			DrawManifold(&m, &wm);

			offset = b2Add(offset, increment);
		}

		// smooth segment-box
		{
			b2Transform xf1 = {offset, b2Rot_identity};
			b2Transform xf2 = {b2Add(m_transform.p, offset), m_transform.q};

			b2Manifold m = b2CollideSmoothSegmentAndPolygon(&m_smoothSegment, xf1, &m_box, xf2);
			b2WorldManifold wm = b2ComputeWorldManifold(&m, xf1, 0.0f, xf2, 0.0f);

			b2Vec2 p1 = b2TransformPoint(xf1, m_smoothSegment.point1);
			b2Vec2 p2 = b2TransformPoint(xf1, m_smoothSegment.point2);
			g_debugDraw.DrawSegment(p1, p2, color1);

			p1 = b2TransformPoint(xf1, m_smoothSegment.ghost1);
			p2 = b2TransformPoint(xf1, m_smoothSegment.point1);
			g_debugDraw.DrawSegment(p1, p2, dim1);

			p1 = b2TransformPoint(xf1, m_smoothSegment.point2);
			p2 = b2TransformPoint(xf1, m_smoothSegment.ghost2);
			g_debugDraw.DrawSegment(p1, p2, dim1);

			b2Vec2 vertices[b2_maxPolygonVertices];
			for (int i = 0; i < m_box.count; ++i)
			{
				vertices[i] = b2TransformPoint(xf2, m_box.vertices[i]);
			}
			g_debugDraw.DrawPolygon(vertices, m_box.count, color2);

			DrawManifold(&m, &wm);

			offset = b2Add(offset, increment);
		}
	}

	static Sample* Create()
	{
		return new Manifold;
	}

	b2PolygonShape m_box;
	b2CircleShape m_circle1;
	b2CircleShape m_circle2;
	b2SegmentShape m_segment;
	b2SmoothSegmentShape m_smoothSegment;

	b2Transform m_transform;
	float m_angle;

	b2Vec2 m_basePosition;
	b2Vec2 m_startPoint;
	float m_baseAngle;

	bool m_dragging;
	bool m_rotating;
	bool m_showIds;
	bool m_showSeparation;
};

static int sampleIndex = RegisterSample("Collision", "Manifold", Manifold::Create);
