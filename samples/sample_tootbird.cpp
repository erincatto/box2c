// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "draw.h"
#include "sample.h"
#include "settings.h"

#include "box2d/box2d.h"
#include "box2d/collision.h"
#include "box2d/math_functions.h"

#include <GLFW/glfw3.h>
#include <imgui.h>

struct SimplexVertex
{
	b2Vec2 p;
	float alpha;
	int index;
};

struct Simplex
{
	SimplexVertex v1, v2, v3;
	int count;
};

static void ReduceSimplex2(Simplex* s, b2Vec2 tootBird)
{
	b2Vec2 w1 = s->v1.p - tootBird;
	b2Vec2 w2 = s->v2.p - tootBird;
	b2Vec2 e12 = b2Sub(w2, w1);

	// w1 region
	float d12_2 = -b2Dot(w1, e12);
	if (d12_2 <= 0.0f)
	{
		// a2 <= 0, so we clamp it to 0
		s->v1.alpha = 1.0f;
		s->count = 1;
		return;
	}

	// w2 region
	float d12_1 = b2Dot(w2, e12);
	if (d12_1 <= 0.0f)
	{
		// a1 <= 0, so we clamp it to 0
		s->v2.alpha = 1.0f;
		s->count = 1;
		s->v1 = s->v2;
		return;
	}

	// Must be in e12 region.
	float inv_d12 = 1.0f / (d12_1 + d12_2);
	s->v1.alpha = d12_1 * inv_d12;
	s->v2.alpha = d12_2 * inv_d12;
	s->count = 2;
}

static void ReduceSimplex3(Simplex* s, b2Vec2 tootBird)
{
	b2Vec2 w1 = s->v1.p - tootBird;
	b2Vec2 w2 = s->v2.p - tootBird;
	b2Vec2 w3 = s->v3.p - tootBird;

	// Edge12
	// [1      1     ][a1] = [1]
	// [w1.e12 w2.e12][a2] = [0]
	// a3 = 0
	b2Vec2 e12 = b2Sub(w2, w1);
	float w1e12 = b2Dot(w1, e12);
	float w2e12 = b2Dot(w2, e12);
	float d12_1 = w2e12;
	float d12_2 = -w1e12;

	// Edge13
	// [1      1     ][a1] = [1]
	// [w1.e13 w3.e13][a3] = [0]
	// a2 = 0
	b2Vec2 e13 = b2Sub(w3, w1);
	float w1e13 = b2Dot(w1, e13);
	float w3e13 = b2Dot(w3, e13);
	float d13_1 = w3e13;
	float d13_2 = -w1e13;

	// Edge23
	// [1      1     ][a2] = [1]
	// [w2.e23 w3.e23][a3] = [0]
	// a1 = 0
	b2Vec2 e23 = b2Sub(w3, w2);
	float w2e23 = b2Dot(w2, e23);
	float w3e23 = b2Dot(w3, e23);
	float d23_1 = w3e23;
	float d23_2 = -w2e23;

	// Triangle123
	float n123 = b2Cross(e12, e13);

	float d123_1 = n123 * b2Cross(w2, w3);
	float d123_2 = n123 * b2Cross(w3, w1);
	float d123_3 = n123 * b2Cross(w1, w2);

	// w1 region
	if (d12_2 <= 0.0f && d13_2 <= 0.0f)
	{
		s->v1.alpha = 1.0f;
		s->count = 1;
		return;
	}

	// e12
	if (d12_1 > 0.0f && d12_2 > 0.0f && d123_3 <= 0.0f)
	{
		float inv_d12 = 1.0f / (d12_1 + d12_2);
		s->v1.alpha = d12_1 * inv_d12;
		s->v2.alpha = d12_2 * inv_d12;
		s->count = 2;
		return;
	}

	// e13
	if (d13_1 > 0.0f && d13_2 > 0.0f && d123_2 <= 0.0f)
	{
		float inv_d13 = 1.0f / (d13_1 + d13_2);
		s->v1.alpha = d13_1 * inv_d13;
		s->v3.alpha = d13_2 * inv_d13;
		s->count = 2;
		s->v2 = s->v3;
		return;
	}

	// w2 region
	if (d12_1 <= 0.0f && d23_2 <= 0.0f)
	{
		s->v2.alpha = 1.0f;
		s->count = 1;
		s->v1 = s->v2;
		return;
	}

	// w3 region
	if (d13_1 <= 0.0f && d23_1 <= 0.0f)
	{
		s->v3.alpha = 1.0f;
		s->count = 1;
		s->v1 = s->v3;
		return;
	}

	// e23
	if (d23_1 > 0.0f && d23_2 > 0.0f && d123_1 <= 0.0f)
	{
		float inv_d23 = 1.0f / (d23_1 + d23_2);
		s->v2.alpha = d23_1 * inv_d23;
		s->v3.alpha = d23_2 * inv_d23;
		s->count = 2;
		s->v1 = s->v3;
		return;
	}

	// The tootBird cannot be contained
}

static b2Vec2 ComputeClosestPoint(const Simplex* s)
{
	switch (s->count)
	{
		case 1:
			return s->v1.p;

		case 2:
		{
			b2Vec2 p1 = s->v1.p;
			b2Vec2 p2 = s->v2.p;
			float a1 = s->v1.alpha;
			float a2 = s->v2.alpha;
			return {a1 * p1.x + a2 * p2.x, a1 * p1.y + a2 * p2.y};
		}

		default:
			assert(false);
			return b2Vec2_zero;
	}
}

class SampleTootbird : public Sample
{
public:
	explicit SampleTootbird(Settings& settings)
		: Sample(settings)
	{
		if (settings.restart == false)
		{
			g_camera.m_center = {0.0f, -1.2f};
			g_camera.m_zoom = 25.0f * 0.1f;
		}

		m_box = b2MakeSquare(0.5f);

		m_transform.p = {0.55f, -0.70f};
		m_transform.q = b2Rot_identity;
		m_angle = 0.0f;

		m_maxIterations = 10;

		m_startPoint = {0.0f, 0.0f};
		m_basePosition = {0.0f, 0.0f};
		m_baseAngle = 0.0f;

		m_dragging = false;
		m_rotating = false;
		m_showIndices = false;
	}

	void UpdateUI() override
	{
		float height = 230.0f;
		ImGui::SetNextWindowPos(ImVec2(10.0f, g_camera.m_height - height - 50.0f), ImGuiCond_Once);
		ImGui::SetNextWindowSize(ImVec2(240.0f, height));

		ImGui::Begin("Tootbird", nullptr, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize);

		ImGui::SliderFloat("x offset", &m_transform.p.x, -2.0f, 2.0f, "%.2f");
		ImGui::SliderFloat("y offset", &m_transform.p.y, -2.0f, 2.0f, "%.2f");

		if (ImGui::SliderFloat("angle", &m_angle, -b2_pi, b2_pi, "%.2f"))
		{
			m_transform.q = b2MakeRot(m_angle);
		}

		ImGui::SliderInt("max iters", &m_maxIterations, 0, 20);
		ImGui::Checkbox("show indices", &m_showIndices);

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
			m_angle = b2ClampFloat(m_baseAngle + 1.0f * dx, -b2_pi, b2_pi);
			m_transform.q = b2MakeRot(m_angle);
		}
	}

	void DrawDistance(const b2DistanceInput* input, const b2DistanceCache* cache, const b2DistanceOutput* output)
	{
		g_draw.DrawSegment(output->pointA, output->pointB, b2_colorWhite);

		if (m_showIndices)
		{
			for (int32_t i = 0; i < cache->count; ++i)
			{
				b2Vec2 pointA = b2TransformPoint(input->transformA, input->proxyA.points[cache->indexA[i]]);
				b2Vec2 pointB = b2TransformPoint(input->transformB, input->proxyB.points[cache->indexB[i]]);
				g_draw.DrawPoint(pointA, 5.0f, b2_colorGreen);
				g_draw.DrawPoint(pointB, 5.0f, b2_colorRed);
			}
			b2Vec2 m = b2Lerp(output->pointA, output->pointB, 0.5f);
			g_draw.DrawString(m, " %d", cache->count);
		}
		else
		{
			g_draw.DrawPoint(output->pointA, 5.0f, b2_colorGreen);
			g_draw.DrawPoint(output->pointB, 5.0f, b2_colorRed);
		}
	}

	static int FindSupport(const b2Vec2* points, int count, b2Vec2 direction)
	{
		int bestIndex = 0;
		float bestValue = b2Dot(points[0], direction);
		for (int i = 1; i < count; ++i)
		{
			float value = b2Dot(points[i], direction);
			if (value > bestValue)
			{
				bestIndex = i;
				bestValue = value;
			}
		}

		return bestIndex;
	}

	static b2Vec2 ClosestPointOnSegment(b2Vec2 q, b2Vec2 p1, b2Vec2 p2)
	{
		// compute distance to line segment
		b2Vec2 e12 = b2Sub(p2, p1);

		// p1 region
		float d12_2 = b2Dot(q - p1, e12);
		if (d12_2 <= 0.0f)
		{
			return p1;
		}

		// p2 region
		float d12_1 = b2Dot(p2 - q, e12);
		if (d12_1 <= 0.0f)
		{
			return p2;
		}

		// Must be in e12 region.
		float inv_d12 = 1.0f / (d12_1 + d12_2);
		b2Vec2 cp = inv_d12 * (d12_1 * p1 + d12_2 * p2);
		return cp;
	}

	void Step(Settings&) override
	{
		b2Vec2 points[b2_maxPolygonVertices];
		int count = m_box.count;
		for (int i = 0; i < count; ++i)
		{
			points[i] = b2TransformPoint(m_transform, m_box.vertices[i]);
		}

		Simplex simplex = {};

		// Initial search direction
		b2Vec2 center = b2TransformPoint(m_transform, m_box.centroid);
		b2Vec2 direction = -center;
		if (b2LengthSquared(direction) < 0.00001f)
		{
			direction = {1.0f, 0.0f};
		}
		b2Vec2 normal = b2Normalize(direction);

		SimplexVertex* vertices[] = {&simplex.v1, &simplex.v2, &simplex.v3};

		// Initialize simplex and toot bird
		int index = FindSupport(points, count, direction);
		SimplexVertex* vertex = vertices[0];
		*vertex = {points[index], 1.0f, index};
		simplex.count = 1;

		// Initial toot bird is origin projected onto support plane
		float tootBirdSeparation = -b2Dot(points[index], normal);
		b2Vec2 tootBird = -tootBirdSeparation * normal;

		int iter;
		for (iter = 0; iter < m_maxIterations; ++iter)
		{
			// Search direction is from current simplex to tootBird
			if (simplex.count == 1)
			{
				direction = tootBird - simplex.v1.p;
			}
			else
			{
				assert(simplex.count == 2);
				b2Vec2 cp = ClosestPointOnSegment(tootBird, simplex.v1.p, simplex.v2.p);
				direction = tootBird - cp;
			}

			if (b2LengthSquared(direction) < 0.00001f)
			{
				// converged
				break;
			}

			normal = b2Normalize(direction);

			// Use search direction and support point to grow simplex
			vertex = vertices[simplex.count];
			index = FindSupport(points, count, direction);
			*vertex = {points[index], 1.0f, index};
			simplex.count += 1;

			// dot(origin + b * normal - points[index], normal) = 0
			// b = dot(points[index], normal)
			// tb = b * normal
			// vector from simplex vertex to tootBird
			// d = tb - points[index]
			//   = dot(points[index], normal) * normal - points[index]

			// Reduce simplex
			if (simplex.count == 2)
			{
				ReduceSimplex2(&simplex, tootBird);
			}
			else if (simplex.count == 3)
			{
				ReduceSimplex3(&simplex, tootBird);
				if (simplex.count == 3)
				{
					// Something is wrong
					break;
				}
			}

			// Project origin onto support to get candidate toot bird
			float separation = -b2Dot(points[index], normal);

			// Is the candidate an improvement?
			if (separation > tootBirdSeparation)
			{
				tootBird = -separation * normal;
			}
		}
		
		g_draw.DrawPolygon(points, count, b2_colorMagenta);

		b2HexColor simplexColors[3] = {b2_colorRed, b2_colorGreen, b2_colorBlue};
		for (int i = 0; i < simplex.count; ++i)
		{
			g_draw.DrawPoint(vertices[i]->p, 5.0f, simplexColors[i]);
		}

		g_draw.DrawPoint(tootBird, 3.0f, b2_colorPaleTurquoise);
		g_draw.DrawTransform(b2Transform_identity);


		g_draw.DrawString(5, m_textLine, "iterations = %d", iter);
		m_textLine += m_textIncrement;
	}

	static Sample* Create(Settings& settings)
	{
		return new SampleTootbird(settings);
	}

	b2Polygon m_box;

	b2Transform m_transform;
	float m_angle;

	int m_maxIterations;

	b2Vec2 m_basePosition;
	b2Vec2 m_startPoint;
	float m_baseAngle;

	bool m_dragging;
	bool m_rotating;
	bool m_showIndices;
};

static int sampleTootbird = RegisterSample("Toot bird", "Toot bird", SampleTootbird::Create);
