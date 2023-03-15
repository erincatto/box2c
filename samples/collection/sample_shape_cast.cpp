// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "sample.h"
#include "box2d/constants.h"
#include "box2d/distance.h"
#include "box2d/math.h"

class ShapeCast : public Sample
{
public:
	enum
	{
		e_vertexCount = 8
	};

	ShapeCast()
	{
#if 1
		m_vAs[0] = { -0.5f, 1.0f };
		m_vAs[1] = { 0.5f, 1.0f };
		m_vAs[2] = { 0.0f, 0.0f };
		m_countA = 3;
		m_radiusA = 0.0f;

		m_vBs[0] = { -0.5f, -0.5f };
		m_vBs[1] = { 0.5f, -0.5f };
		m_vBs[2] = { 0.5f, 0.5f };
		m_vBs[3] = { -0.5f, 0.5f };
		m_countB = 4;
		m_radiusB = 0.0f;

		m_transformA.p = { 0.0f, 0.25f };
		m_transformA.q = b2Rot_identity;
		m_transformB.p = { -4.0f, 0.0f };
		m_transformB.q = b2Rot_identity;
		m_translationB = { 8.0f, 0.0f };
#elif 0
		m_vAs[0].Set(0.0f, 0.0f);
		m_countA = 1;
		m_radiusA = 0.5f;

		m_vBs[0].Set(0.0f, 0.0f);
		m_countB = 1;
		m_radiusB = 0.5f;

		m_transformA.p.Set(0.0f, 0.25f);
		m_transformA.q.SetIdentity();
		m_transformB.p.Set(-4.0f, 0.0f);
		m_transformB.q.SetIdentity();
		m_translationB.Set(8.0f, 0.0f);
#else
		m_vAs[0].Set(0.0f, 0.0f);
		m_vAs[1].Set(2.0f, 0.0f);
		m_countA = 2;
		m_radiusA = b2_polygonRadius;

		m_vBs[0].Set(0.0f, 0.0f);
		m_countB = 1;
		m_radiusB = 0.25f;

		// Initial overlap
		m_transformA.p.Set(0.0f, 0.0f);
		m_transformA.q.SetIdentity();
		m_transformB.p.Set(-0.244360745f, 0.05999358f);
		m_transformB.q.SetIdentity();
		m_translationB.Set(0.0f, 0.0399999991f);
#endif
	}

	static Sample* Create()
	{
		return new ShapeCast;
	}

	void Step(Settings& settings) override
	{
		Sample::Step(settings);

		b2ShapeCastInput input;
		input.proxyA = b2MakeProxy(m_vAs, m_countA, m_radiusA);
		input.proxyB = b2MakeProxy(m_vBs, m_countB, m_radiusB);
		input.transformA = m_transformA;
		input.transformB = m_transformB;
		input.translationB = m_translationB;

		b2RayCastOutput output = b2ShapeCast(&input);

		b2Transform transformB2;
		transformB2.q = m_transformB.q;
		transformB2.p = b2MulAdd(m_transformB.p, output.fraction, input.translationB);

		b2DistanceInput distanceInput;
		distanceInput.proxyA = b2MakeProxy(m_vAs, m_countA, m_radiusA);
		distanceInput.proxyB = b2MakeProxy(m_vBs, m_countB, m_radiusB);
		distanceInput.transformA = m_transformA;
		distanceInput.transformB = transformB2;
		distanceInput.useRadii = false;
		b2DistanceCache distanceCache;
		distanceCache.count = 0;
		b2DistanceOutput distanceOutput = b2ShapeDistance(&distanceCache, &distanceInput);

		g_draw.DrawString(5, m_textLine, "hit = %s, iters = %d, lambda = %g, distance = %g",
			output.hit ? "true" : "false", output.iterations, output.fraction, distanceOutput.distance);
		m_textLine += m_textIncrement;

		b2Vec2 vertices[b2_maxPolygonVertices];

		for (int32_t i = 0; i < m_countA; ++i)
		{
			vertices[i] = b2TransformPoint(m_transformA, m_vAs[i]);
		}

		if (m_countA == 1)
		{
			g_draw.DrawCircle(vertices[0], m_radiusA, { 0.9f, 0.9f, 0.9f, 1.0f });
		}
		else
		{
			g_draw.DrawPolygon(vertices, m_countA, { 0.9f, 0.9f, 0.9f, 1.0f });
		}

		for (int32_t i = 0; i < m_countB; ++i)
		{
			vertices[i] = b2TransformPoint(m_transformB, m_vBs[i]);
		}

		if (m_countB == 1)
		{
			g_draw.DrawCircle(vertices[0], m_radiusB, { 0.5f, 0.9f, 0.5f, 1.0f });
		}
		else
		{
			g_draw.DrawPolygon(vertices, m_countB, { 0.5f, 0.9f, 0.5f, 1.0f });
		}

		for (int32_t i = 0; i < m_countB; ++i)
		{
			vertices[i] = b2TransformPoint(transformB2, m_vBs[i]);
		}

		if (m_countB == 1)
		{
			g_draw.DrawCircle(vertices[0], m_radiusB, { 0.5f, 0.7f, 0.9f, 1.0f });
		}
		else
		{
			g_draw.DrawPolygon(vertices, m_countB, { 0.5f, 0.7f, 0.9f, 1.0f });
		}

		if (output.hit)
		{
			b2Vec2 p1 = output.point;
			g_draw.DrawPoint(p1, 10.0f, { 0.9f, 0.3f, 0.3f, 1.0f });
			b2Vec2 p2 = b2MulAdd(p1, 1.0f, output.normal);
			g_draw.DrawSegment(p1, p2, { 0.9f, 0.3f, 0.3f, 1.0f });
		}
	}

	b2Vec2 m_vAs[b2_maxPolygonVertices];
	int32_t m_countA;
	float m_radiusA;

	b2Vec2 m_vBs[b2_maxPolygonVertices];
	int32_t m_countB;
	float m_radiusB;

	b2Transform m_transformA;
	b2Transform m_transformB;
	b2Vec2 m_translationB;
};

static int sampleIndex = RegisterSample("Collision", "Shape Cast", ShapeCast::Create);
