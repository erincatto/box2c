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

struct DepthResult
{
	b2Vec2 normal;
	b2Vec2 witness;
	float depth;
	int iterations;
};

static void ReduceSimplex2(Simplex* s, b2Vec2 target)
{
	b2Vec2 w1 = s->v1.p - target;
	b2Vec2 w2 = s->v2.p - target;
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

static void ReduceSimplex3(Simplex* simplex, b2Vec2 target)
{
	b2Vec2 v1 = simplex->v1.p;
	b2Vec2 v2 = simplex->v2.p;
	b2Vec2 v3 = simplex->v3.p;

	b2Vec2 e21 = b2Sub(v2, v1);

	// v3 should always be between v1 and v2
	// Validate this by projecting v3 onto the axis between v1 and v2
	assert(b2Dot(e21, b2Sub(v3, v1)) >= 0.0f && b2Dot(e21, b2Sub(v2, v3)) >= 0.0f);

	// dot(t - v0, v0 - v1) < dot(v0 - v2, v0 - v1)
	// dot(t - v0 - (v0 - v2), v0 - v1) < 0
	// dot(t - v2, v0 - v1) < 0
	float side = b2Dot(b2Sub(target, v3), e21);

	if (side <= 0.0f)
	{
		simplex->v2 = simplex->v3;
		simplex->count = 2;
		// retain v1 and v3
	}
	else
	{
		// retain v2 and v3
		simplex->v1 = simplex->v3;
		simplex->count = 2;
	}

	ReduceSimplex2(simplex, target);
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

struct NormalResult
{
	b2Vec2 nextNormal;
	bool converged;
};

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

		m_box = b2MakeBox(10.0f, 0.25f);

		m_transform.p = {6.0f, -0.1f};
		//m_transform.p = {0.201851815f, -0.125462919f};
		//m_transform.p = {0.138889074f, -0.138889074f};
		m_transform.q = b2Rot_identity;
		m_angle = 0.0f;

		m_normal = b2Vec2_zero;
		m_support = b2Vec2_zero;
		m_target = b2Vec2_zero;

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

		ImGui::SliderFloat("x offset", &m_transform.p.x, -5.0f, 5.0f, "%.2f");
		ImGui::SliderFloat("y offset", &m_transform.p.y, -1.0f, 1.0f, "%.2f");

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

	static int FindSupport2(const b2Vec2* points, int count, b2Vec2 direction)
	{
		// support(N, A) - support(-N, B)
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

	// More accurate than using the closest point
	static b2Vec2 DirectionSegmentToTarget(b2Vec2 target, b2Vec2 p1, b2Vec2 p2, float* distanceSqr)
	{
		b2Vec2 e12 = b2Sub(p2, p1);

		// p1 region
		b2Vec2 tp1 = b2Sub(target, p1);
		float v = b2Dot(tp1, e12);
		if (v <= 0.0f)
		{
			*distanceSqr = b2LengthSquared(tp1);
			return tp1;
		}

		// p2 region
		b2Vec2 tp2 = b2Sub(target, p2);
		float u = -b2Dot(tp2, e12);
		if (u <= 0.0f)
		{
			*distanceSqr = b2LengthSquared(tp2);
			return tp2;
		}

		float scale = 1.0f / (u + v);
		b2Vec2 closestPoint = scale * (u * p1 + v * p2);
		b2Vec2 tcp = b2Sub(target, closestPoint);
		*distanceSqr = b2LengthSquared(tcp);

		float sign = b2Cross(tp1, e12);
		b2Vec2 direction = sign >= 0.0f ? b2RightPerp(e12) : b2LeftPerp(e12);

		assert(b2Dot(direction, tcp) >= 0.0f || *distanceSqr < FLT_EPSILON);
		return direction;
	}

	NormalResult GetNextNormal(Simplex* simplex, b2Vec2 support, b2Vec2 bestNormal, float bestDepth, int supportIndex)
	{
		// if depth > 0 then search target is origin projected on the best plane (toot bird)
		// else the search target is the origin (GJK)
		b2Vec2 searchTarget = b2MaxFloat(0.0f, bestDepth) * bestNormal;
		m_target = searchTarget;

		if (supportIndex >= 0)
		{
			// add support point to simplex
			assert(simplex->count < 3);
			SimplexVertex* vertices = &simplex->v1;
			vertices[simplex->count] = {support, 1.0f, supportIndex};
			simplex->count += 1;
		}

		// reduce simplex
		// dot(origin + b * normal - points[index], normal) = 0
		// b = dot(points[index], normal)
		// tb = b * normal
		// vector from simplex vertex to tootBird
		// d = tb - points[index]
		//   = dot(points[index], normal) * normal - points[index]

		// Reduce simplex
		if (simplex->count == 2)
		{
			ReduceSimplex2(simplex, searchTarget);
		}
		else if (simplex->count == 3)
		{
			ReduceSimplex3(simplex, searchTarget);
			if (simplex->count == 3)
			{
				// Something is wrong
				assert(false);
				return {b2Vec2_zero, true};
			}
		}

		// Search direction is from current simplex to tootBird
		b2Vec2 direction;
		float distanceSqr;
		if (simplex->count == 1)
		{
			direction = b2Sub(searchTarget, simplex->v1.p);
			distanceSqr = b2LengthSquared(direction);
		}
		else
		{
			assert(simplex->count == 2);
			direction = DirectionSegmentToTarget(searchTarget, simplex->v1.p, simplex->v2.p, &distanceSqr);
		}

		// Termination condition
		float tol = FLT_EPSILON;
		bool converged;
		if (bestDepth < 0.0f)
		{
			// separation converged?
			float bestSeparation = -bestDepth;
			converged = distanceSqr < (bestSeparation + tol) * (bestSeparation + tol);
		}
		else
		{
			converged = distanceSqr < tol;
		}

		b2Vec2 nextNormal = b2Normalize(direction);
		NormalResult result = {nextNormal, converged};
		return result;
	}

	DepthResult FindMinimumDepth(b2Vec2* points, int count, b2Vec2 initialNormal, Simplex* simplex, int maximumIterations)
	{
		int supportIndex = FindSupport2(points, count, initialNormal);
		b2Vec2 initialSupport = points[supportIndex];
		float initialDepth = b2Dot(initialSupport, initialNormal);
		m_support = initialSupport;

		// Initialize simplex
		simplex->count = 1;
		simplex->v1.alpha = 1.0f;
		simplex->v1.index = supportIndex;
		simplex->v1.p = initialSupport;

		DepthResult result = {initialNormal, b2Vec2_zero, initialDepth, 0};

		// speculative margin
		float depthThreshold = -100.0f;
		if (initialDepth < depthThreshold)
		{
			return result;
		}

		b2Vec2 refinedNormal = initialNormal;
		float refinedDepth = initialDepth;

		NormalResult normalResult = GetNextNormal(simplex, b2Vec2_zero, refinedNormal, refinedDepth, -1);

		int iteration = 0;
		for (; iteration < maximumIterations; ++iteration)
		{
			if (normalResult.converged)
			{
				break;
			}

			supportIndex = FindSupport2(points, count, normalResult.nextNormal);
			b2Vec2 support = points[supportIndex];

			m_normal = normalResult.nextNormal;
			m_support = support;

			float depth = b2Dot(support, normalResult.nextNormal);
			if (depth < refinedDepth)
			{
				refinedDepth = depth;
				refinedNormal = normalResult.nextNormal;
			}

			// todo redundant?
			//if (normalResult.converged)
			//{
			//	break;
			//}

			normalResult = GetNextNormal(simplex, support, refinedNormal, refinedDepth, supportIndex);
		}

		result.normal = refinedNormal;
		result.witness = ComputeClosestPoint(simplex);
		result.depth = refinedDepth;
		result.iterations = iteration;
		return result;
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
		m_normal = -b2Normalize(m_transform.p);
		DepthResult result = FindMinimumDepth(points, count, m_normal, &simplex, m_maxIterations);

		g_draw.DrawPolygon(points, count, b2_colorMagenta);

		SimplexVertex* vertices = &simplex.v1;
		b2HexColor simplexColors[3] = {b2_colorRed, b2_colorGreen, b2_colorBlue};
		for (int i = 0; i < simplex.count; ++i)
		{
			g_draw.DrawPoint(vertices[i].p, 8.0f, simplexColors[i]);
		}

		if (m_showIndices)
		{
			for (int i = 0; i < count; ++i)
			{
				g_draw.DrawString(points[i], " %d", i);
			}
		}

		g_draw.DrawPoint(result.witness, 5.0f, b2_colorYellow);
		g_draw.DrawTransform(b2Transform_identity);

		g_draw.DrawPoint(m_support, 5.0f, b2_colorLime);
		g_draw.DrawSegment(m_support, m_support + 0.1f * m_normal, b2_colorLime);
		g_draw.DrawPoint(m_target, 5.0f, b2_colorBlanchedAlmond);

		g_draw.DrawString(5, m_textLine, "iterations = %d, separation = %.3f", result.iterations, -result.depth);
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

	b2Vec2 m_normal;
	b2Vec2 m_support;
	b2Vec2 m_target;

	b2Vec2 m_basePosition;
	b2Vec2 m_startPoint;
	float m_baseAngle;

	bool m_dragging;
	bool m_rotating;
	bool m_showIndices;
};

static int sampleTootbird = RegisterSample("Toot bird", "Toot bird", SampleTootbird::Create);

