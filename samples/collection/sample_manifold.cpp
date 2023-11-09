// SPDX-FileCopyrightText: 2022 Erin Catto
// SPDX-License-Identifier: MIT

#include "box2d/distance.h"
#include "box2d/hull.h"
#include "box2d/manifold.h"
#include "box2d/math.h"
#include "box2d/geometry.h"
#include "sample.h"

#include <GLFW/glfw3.h>
#include <imgui.h>

// Tests manifolds and contact points
class Manifold : public Sample
{
public:
	Manifold(const Settings& settings)
		: Sample(settings)
	{
		m_capcapCache = b2_emptyDistanceCache;
		m_capboxCache = b2_emptyDistanceCache;
		m_boxboxCache = b2_emptyDistanceCache;
		m_boxroxCache = b2_emptyDistanceCache;
		m_roxroxCache = b2_emptyDistanceCache;
		m_segroxCache = b2_emptyDistanceCache;
		m_segcapCache = b2_emptyDistanceCache;
		m_woxwoxCache = b2_emptyDistanceCache;

		m_transform = b2Transform_identity;
		m_angle = 0.0f;
		m_round = 0.1f;

		m_startPoint = {0.0f, 0.0f};
		m_basePosition = {0.0f, 0.0f};
		m_baseAngle = 0.0f;

		m_dragging = false;
		m_rotating = false;
		m_showIds = false;
		m_showSeparation = false;
		m_enableCaching = true;

		b2Vec2 points[3] = {{-0.1f, -0.5f}, {0.1f, -0.5f}, {0.0f, 0.5f}};
		m_wedge = b2ComputeHull(points, 3);
	}

	void UpdateUI() override
	{
		ImGui::SetNextWindowPos(ImVec2(10.0f, 100.0f));
		ImGui::SetNextWindowSize(ImVec2(230.0f, 260.0f));
		ImGui::Begin("Manifold Controls", nullptr, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize);

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

		if (ImGui::SliderFloat("round", &m_round, 0.0f, 0.4f, "%.1f"))
		{
		}

		if (ImGui::Checkbox("show ids", &m_showIds))
		{
		}

		if (ImGui::Checkbox("show separation", &m_showSeparation))
		{
		}

		if (ImGui::Checkbox("enable caching", &m_enableCaching))
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
			m_transform.q = b2MakeRot(m_angle);
		}
	}

	void DrawManifold(const b2Manifold* manifold)
	{
		b2Color white = {1.0f, 1.0f, 1.0f, 1.0f};
		b2Color green = {0.0f, 1.0f, 0.0f, 1.0f};

		for (int i = 0; i < manifold->pointCount; ++i)
		{
			const b2ManifoldPoint* mp = manifold->points + i;

			b2Vec2 p1 = mp->point;
			b2Vec2 p2 = b2MulAdd(p1, 0.5f, manifold->normal);
			g_draw.DrawSegment(p1, p2, white);
			g_draw.DrawPoint(p1, 5.0f, green);

			if (m_showIds)
			{
				//uint32_t indexA = mp->id >> 8;
				//uint32_t indexB = 0xFF & mp->id;
				b2Vec2 p = {p1.x + 0.05f, p1.y - 0.02f};
				g_draw.DrawString(p, "0x%04x", mp->id);
			}

			if (m_showSeparation)
			{
				b2Vec2 p = {p1.x + 0.05f, p1.y + 0.03f};
				g_draw.DrawString(p, "%.3f", mp->separation);
			}
		}
	}

	void Step(Settings&) override
	{
		b2Vec2 offset = {-10.0f, 10.0f};
		b2Vec2 increment = {4.0f, 0.0f};

		b2Color color1 = {0.3f, 0.8f, 0.6f, 1.0f};
		b2Color color2 = {0.8f, 0.6f, 0.3f, 1.0f};
		b2Color fillColor1 = {0.5f * color1.r, 0.5f * color1.g, 0.5f * color1.b, 0.5f};
		b2Color fillColor2 = {0.5f * color2.r, 0.5f * color2.g, 0.5f * color2.b, 0.5f};

		b2Color dim1 = {0.5f * color1.r, 0.5f * color1.g, 0.5f * color1.b, 1.0f};

		//box = b2MakeRoundedBox(10.0f, 10.0f, 10.0f);
		//box = b2MakeRoundedBox(0.4f, 0.4f, 0.1f);

		//b2Color fill = {0.345098048f, 0.431372553f, 0.458823532f, 1.0f};
		//b2Color outline = {0.933333337f, 0.909803927f, 0.835294127f, 1.0f};
		//g_draw.DrawRoundedPolygon(box.vertices, box.count, box.radius, fill, outline);

		if (m_enableCaching == false)
		{
			m_capcapCache = b2_emptyDistanceCache;
			m_capboxCache = b2_emptyDistanceCache;
			m_segcapCache = b2_emptyDistanceCache;
			m_boxboxCache = b2_emptyDistanceCache;
			m_boxroxCache = b2_emptyDistanceCache;
			m_roxroxCache = b2_emptyDistanceCache;
			m_segroxCache = b2_emptyDistanceCache;
			m_woxwoxCache = b2_emptyDistanceCache;
		}

		// circle-circle
		{
			b2Circle circle1 = {{0.0f, 0.0f}, 0.5f};
			b2Circle circle2 = {{0.0f, 0.0f}, 1.0f};

			b2Transform xf1 = {offset, b2Rot_identity};
			b2Transform xf2 = {b2Add(m_transform.p, offset), m_transform.q};

			b2Manifold m = b2CollideCircles(&circle1, xf1, &circle2, xf2);

			b2Vec2 c1 = b2TransformPoint(xf1, circle1.point);
			b2Vec2 c2 = b2TransformPoint(xf2, circle2.point);
			b2Vec2 axis1 = b2RotateVector(xf1.q, {1.0f, 0.0f});
			b2Vec2 axis2 = b2RotateVector(xf2.q, {1.0f, 0.0f});
			g_draw.DrawSolidCircle(c1, circle1.radius, axis1, color1);
			g_draw.DrawSolidCircle(c2, circle2.radius, axis2, color2);

			DrawManifold(&m);

			offset = b2Add(offset, increment);
		}

		// capsule-circle
		{
			b2Capsule capsule = {{-0.5f, 0.0f}, {0.5f, 0.0}, 0.25f};
			b2Circle circle = {{0.0f, 0.0f}, 0.5f};

			b2Transform xf1 = {offset, b2Rot_identity};
			b2Transform xf2 = {b2Add(m_transform.p, offset), m_transform.q};

			b2Manifold m = b2CollideCapsuleAndCircle(&capsule, xf1, &circle, xf2);

			b2Vec2 v1 = b2TransformPoint(xf1, capsule.point1);
			b2Vec2 v2 = b2TransformPoint(xf1, capsule.point2);
			g_draw.DrawSolidCapsule(v1, v2, capsule.radius, color1);

			b2Vec2 c1 = b2TransformPoint(xf2, circle.point);
			b2Vec2 axis1 = b2RotateVector(xf2.q, {1.0f, 0.0f});
			g_draw.DrawSolidCircle(c1, circle.radius, axis1, color2);

			DrawManifold(&m);

			offset = b2Add(offset, increment);
		}

		// segment-circle
		{
			b2Segment segment = {{-1.0f, 0.0f}, {1.0f, 0.0}};
			b2Circle circle = {{0.0f, 0.0f}, 0.5f};

			b2Transform xf1 = {offset, b2Rot_identity};
			b2Transform xf2 = {b2Add(m_transform.p, offset), m_transform.q};

			b2Manifold m = b2CollideSegmentAndCircle(&segment, xf1, &circle, xf2);

			b2Vec2 p1 = b2TransformPoint(xf1, segment.point1);
			b2Vec2 p2 = b2TransformPoint(xf1, segment.point2);
			g_draw.DrawSegment(p1, p2, color1);

			b2Vec2 c2 = b2TransformPoint(xf2, circle.point);
			b2Vec2 axis2 = b2RotateVector(xf2.q, {1.0f, 0.0f});
			g_draw.DrawSolidCircle(c2, circle.radius, axis2, color2);

			DrawManifold(&m);

			offset = b2Add(offset, increment);
		}

		// box-circle
		{
			b2Circle circle = {{0.0f, 0.0f}, 0.5f};
			b2Polygon box = b2MakeSquare(0.5f);
			box.radius = m_round;

			b2Transform xf1 = {offset, b2Rot_identity};
			b2Transform xf2 = {b2Add(m_transform.p, offset), m_transform.q};

			b2Manifold m = b2CollidePolygonAndCircle(&box, xf1, &circle, xf2);

			b2Vec2 vertices[b2_maxPolygonVertices];
			for (int i = 0; i < box.count; ++i)
			{
				vertices[i] = b2TransformPoint(xf1, box.vertices[i]);
			}
			g_draw.DrawRoundedPolygon(vertices, box.count, m_round, fillColor1, color1);

			b2Vec2 c2 = b2TransformPoint(xf2, circle.point);
			b2Vec2 axis2 = b2RotateVector(xf2.q, {1.0f, 0.0f});
			g_draw.DrawSolidCircle(c2, circle.radius, axis2, color2);

			DrawManifold(&m);

			offset = b2Add(offset, increment);
		}

		// capsule-capsule
		{
			b2Capsule capsule = {{-0.5f, 0.0f}, {0.5f, 0.0}, 0.25f};

			b2Transform xf1 = {offset, b2Rot_identity};
			b2Transform xf2 = {b2Add(m_transform.p, offset), m_transform.q};

			if (m_enableCaching == false)
			{
				m_capcapCache = b2_emptyDistanceCache;
			}

			b2Manifold m = b2CollideCapsules(&capsule, xf1, &capsule, xf2, &m_capcapCache);

			b2Vec2 v1 = b2TransformPoint(xf1, capsule.point1);
			b2Vec2 v2 = b2TransformPoint(xf1, capsule.point2);
			g_draw.DrawSolidCapsule(v1, v2, capsule.radius, color1);

			v1 = b2TransformPoint(xf2, capsule.point1);
			v2 = b2TransformPoint(xf2, capsule.point2);
			g_draw.DrawSolidCapsule(v1, v2, capsule.radius, color2);

			DrawManifold(&m);

			offset = b2Add(offset, increment);
		}

		// box-capsule
		{
			b2Capsule capsule = {{-0.1f, 0.0f}, {0.1f, 0.0f}, 0.075f};
			b2Polygon box = b2MakeBox(2.0f, 0.25f);

			b2Transform xf1 = {offset, b2Rot_identity};
			b2Transform xf2 = {b2Add(m_transform.p, offset), m_transform.q};

			b2DistanceCache cache = b2_emptyDistanceCache;
			b2Manifold m = b2CollidePolygonAndCapsule(&box, xf1, &capsule, xf2, &cache);

			b2Vec2 vertices[b2_maxPolygonVertices];
			for (int i = 0; i < box.count; ++i)
			{
				vertices[i] = b2TransformPoint(xf1, box.vertices[i]);
			}
			g_draw.DrawPolygon(vertices, box.count, color1);

			b2Vec2 v1 = b2TransformPoint(xf2, capsule.point1);
			b2Vec2 v2 = b2TransformPoint(xf2, capsule.point2);
			g_draw.DrawSolidCapsule(v1, v2, capsule.radius, color2);

			DrawManifold(&m);

			offset = b2Add(offset, increment);
		}

		// segment-capsule
		{
			b2Segment segment = {{-1.0f, 0.0f}, {1.0f, 0.0}};
			b2Capsule capsule = {{-0.5f, 0.0f}, {0.5f, 0.0}, 0.25f};

			b2Transform xf1 = {offset, b2Rot_identity};
			b2Transform xf2 = {b2Add(m_transform.p, offset), m_transform.q};

			b2Manifold m = b2CollideSegmentAndCapsule(&segment, xf1, &capsule, xf2, &m_segcapCache);

			b2Vec2 p1 = b2TransformPoint(xf1, segment.point1);
			b2Vec2 p2 = b2TransformPoint(xf1, segment.point2);
			g_draw.DrawSegment(p1, p2, color1);

			p1 = b2TransformPoint(xf2, capsule.point1);
			p2 = b2TransformPoint(xf2, capsule.point2);
			g_draw.DrawSolidCapsule(p1, p2, capsule.radius, color2);

			DrawManifold(&m);

			offset = b2Add(offset, increment);
		}

		offset = {-10.0f, 15.0f};

		// box-box
		{
			b2Polygon box = b2MakeSquare(0.5f);

			b2Transform xf1 = {offset, b2Rot_identity};
			b2Transform xf2 = {b2Add(m_transform.p, offset), m_transform.q};
			//b2Transform xf2 = {b2Add({0.0f, -0.1f}, offset), {0.0f, 1.0f}};

			b2Manifold m = b2CollidePolygons(&box, xf1, &box, xf2, &m_boxboxCache);

			b2Vec2 vertices[b2_maxPolygonVertices];
			for (int i = 0; i < box.count; ++i)
			{
				vertices[i] = b2TransformPoint(xf1, box.vertices[i]);
			}
			g_draw.DrawSolidPolygon(vertices, box.count, color1);

			for (int i = 0; i < box.count; ++i)
			{
				vertices[i] = b2TransformPoint(xf2, box.vertices[i]);
			}
			g_draw.DrawSolidPolygon(vertices, box.count, color2);

			DrawManifold(&m);

			offset = b2Add(offset, increment);
		}

		// box-rox
		{
			b2Polygon box = b2MakeSquare(0.5f);
			float h = 0.5f - m_round;
			b2Polygon rox = b2MakeRoundedBox(h, h, m_round);

			b2Transform xf1 = {offset, b2Rot_identity};
			b2Transform xf2 = {b2Add(m_transform.p, offset), m_transform.q};
			//b2Transform xf2 = {b2Add({0.0f, -0.1f}, offset), {0.0f, 1.0f}};

			b2Manifold m = b2CollidePolygons(&box, xf1, &rox, xf2, &m_boxroxCache);

			b2Vec2 vertices[b2_maxPolygonVertices];
			for (int i = 0; i < box.count; ++i)
			{
				vertices[i] = b2TransformPoint(xf1, box.vertices[i]);
			}
			g_draw.DrawSolidPolygon(vertices, box.count, color1);

			for (int i = 0; i < rox.count; ++i)
			{
				vertices[i] = b2TransformPoint(xf2, rox.vertices[i]);
			}
			g_draw.DrawRoundedPolygon(vertices, rox.count, rox.radius, fillColor2, color2);

			DrawManifold(&m);

			offset = b2Add(offset, increment);
		}

		// rox-rox
		{
			float h = 0.5f - m_round;
			b2Polygon rox = b2MakeRoundedBox(h, h, m_round);

			b2Transform xf1 = {offset, b2Rot_identity};
			b2Transform xf2 = {b2Add(m_transform.p, offset), m_transform.q};
			//b2Transform xf1 = {{6.48024225f, 2.07872653f}, {-0.938356698f, 0.345668465f}};
			//b2Transform xf2 = {{5.52862263f, 2.51146317f}, {-0.859374702f, -0.511346340f}};

			b2Manifold m = b2CollidePolygons(&rox, xf1, &rox, xf2, &m_roxroxCache);

			b2Vec2 vertices[b2_maxPolygonVertices];
			for (int i = 0; i < rox.count; ++i)
			{
				vertices[i] = b2TransformPoint(xf1, rox.vertices[i]);
			}
			g_draw.DrawRoundedPolygon(vertices, rox.count, rox.radius, fillColor1, color1);

			for (int i = 0; i < rox.count; ++i)
			{
				vertices[i] = b2TransformPoint(xf2, rox.vertices[i]);
			}
			g_draw.DrawRoundedPolygon(vertices, rox.count, rox.radius, fillColor2, color2);

			DrawManifold(&m);

			offset = b2Add(offset, increment);
		}

		// segment-rox
		{
			b2Segment segment = {{-1.0f, 0.0f}, {1.0f, 0.0}};
			float h = 0.5f - m_round;
			b2Polygon rox = b2MakeRoundedBox(h, h, m_round);

			b2Transform xf1 = {offset, b2Rot_identity};
			b2Transform xf2 = {b2Add(m_transform.p, offset), m_transform.q};
			//b2Transform xf2 = {b2Add({-1.44583416f, 0.397352695f}, offset), m_transform.q};

			b2Manifold m = b2CollideSegmentAndPolygon(&segment, xf1, &rox, xf2, &m_segroxCache);

			b2Vec2 p1 = b2TransformPoint(xf1, segment.point1);
			b2Vec2 p2 = b2TransformPoint(xf1, segment.point2);
			g_draw.DrawSegment(p1, p2, color1);

			b2Vec2 vertices[b2_maxPolygonVertices];
			for (int i = 0; i < rox.count; ++i)
			{
				vertices[i] = b2TransformPoint(xf2, rox.vertices[i]);
			}

			if (m_round > 0.0f)
			{
				g_draw.DrawRoundedPolygon(vertices, rox.count, rox.radius, fillColor2, color2);
			}
			else
			{
				g_draw.DrawSolidPolygon(vertices, rox.count, color2);
			}

			DrawManifold(&m);

			offset = b2Add(offset, increment);
		}

		// wox-wox
		{
			b2Polygon wox = b2MakePolygon(&m_wedge, m_round);

			b2Transform xf1 = {offset, b2Rot_identity};
			b2Transform xf2 = {b2Add(m_transform.p, offset), m_transform.q};
			// b2Transform xf2 = {b2Add({0.0f, -0.1f}, offset), {0.0f, 1.0f}};

			b2Manifold m = b2CollidePolygons(&wox, xf1, &wox, xf2, &m_woxwoxCache);

			b2Vec2 vertices[b2_maxPolygonVertices];
			for (int i = 0; i < wox.count; ++i)
			{
				vertices[i] = b2TransformPoint(xf1, wox.vertices[i]);
			}
			g_draw.DrawRoundedPolygon(vertices, wox.count, wox.radius, fillColor1, color1);

			for (int i = 0; i < wox.count; ++i)
			{
				vertices[i] = b2TransformPoint(xf2, wox.vertices[i]);
			}
			g_draw.DrawRoundedPolygon(vertices, wox.count, wox.radius, fillColor2, color2);

			DrawManifold(&m);

			offset = b2Add(offset, increment);
		}
	}

	static Sample* Create(const Settings& settings)
	{
		return new Manifold(settings);
	}

	b2DistanceCache m_capcapCache;
	b2DistanceCache m_capboxCache;
	b2DistanceCache m_boxboxCache;
	b2DistanceCache m_boxroxCache;
	b2DistanceCache m_roxroxCache;
	b2DistanceCache m_segcapCache;
	b2DistanceCache m_segroxCache;
	b2DistanceCache m_woxwoxCache;

	b2Hull m_wedge;

	b2Transform m_transform;
	float m_angle;
	float m_round;

	b2Vec2 m_basePosition;
	b2Vec2 m_startPoint;
	float m_baseAngle;

	bool m_dragging;
	bool m_rotating;
	bool m_showIds;
	bool m_showSeparation;
	bool m_enableCaching;
};

static int sampleIndex = RegisterSample("Collision", "Manifold", Manifold::Create);
