// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "box2d/constants.h"
#include "box2d/distance.h"
#include "box2d/math.h"
#include "sample.h"

extern "C"
{
int32_t b2_toiMaxIters, b2_toiMaxRootIters;
}

class TimeOfImpact : public Sample
{
public:
	TimeOfImpact(const Settings& settings)
		: Sample(settings)
	{
	}

	static Sample* Create(const Settings& settings)
	{
		return new TimeOfImpact(settings);
	}

	void Step(Settings& settings) override
	{
		Sample::Step(settings);

		b2Sweep sweepA = {b2Vec2_zero, {0.0f, 0.0f}, {0.0f, 0.0f}, 0.0f, 0.0f};
		b2Sweep sweepB = {b2Vec2_zero, {2.0f, 4.0f}, {2.0f, 4.0f}, 0.0f, -0.25f * b2_pi};

		b2TOIInput input;
		input.proxyA = b2MakeProxy(m_verticesA, m_countA, 0.0f);
		input.proxyB = b2MakeProxy(m_verticesB, m_countB, 0.0f);
		input.sweepA = sweepA;
		input.sweepB = sweepB;
		input.tMax = 1.0f;

		b2TOIOutput output;

		b2TimeOfImpact(&output, &input);

		g_draw.DrawString(5, m_textLine, "toi = %g", output.t);
		m_textLine += m_textIncrement;

		g_draw.DrawString(5, m_textLine, "max toi iters = %d, max root iters = %d", b2_toiMaxIters,
		                       b2_toiMaxRootIters);
		m_textLine += m_textIncrement;

		b2Vec2 vertices[b2_maxPolygonVertices];

		// Draw A
		b2Transform transformA = b2GetSweepTransform(&sweepA, 0.0f);
		for (int32_t i = 0; i < m_countA; ++i)
		{
			vertices[i] = b2TransformPoint(transformA, m_verticesA[i]);
		}
		g_draw.DrawPolygon(vertices, m_countA, {0.9f, 0.9f, 0.9f, 1.0f});

		// Draw B at t = 0
		b2Transform transformB = b2GetSweepTransform(&sweepB, 0.0f);
		for (int32_t i = 0; i < m_countB; ++i)
		{
			vertices[i] = b2TransformPoint(transformB, m_verticesB[i]);
		}
		g_draw.DrawPolygon(vertices, m_countB, {0.5f, 0.9f, 0.5f, 1.0f});

		// Draw B at t = hit_time
		transformB = b2GetSweepTransform(&sweepB, output.t);
		for (int32_t i = 0; i < m_countB; ++i)
		{
			vertices[i] = b2TransformPoint(transformB, m_verticesB[i]);
		}
		g_draw.DrawPolygon(vertices, m_countB, {0.5f, 0.7f, 0.9f, 1.0f});

		// Draw B at t = 1
		transformB = b2GetSweepTransform(&sweepB, 1.0f);
		for (int32_t i = 0; i < m_countB; ++i)
		{
			vertices[i] = b2TransformPoint(transformB, m_verticesB[i]);
		}
		g_draw.DrawPolygon(vertices, m_countB, {0.9f, 0.5f, 0.5f, 1.0f});

		if (output.state == b2_toiStateHit)
		{
			b2DistanceInput dinput;
			dinput.proxyA = input.proxyA;
			dinput.proxyB = input.proxyB;
			dinput.transformA = b2GetSweepTransform(&sweepA, output.t);
			dinput.transformB = b2GetSweepTransform(&sweepB, output.t);
			dinput.useRadii = false;
			b2DistanceCache cache = {0};
			b2DistanceOutput doutput = b2ShapeDistance(&cache, &dinput);
			g_draw.DrawString(5, m_textLine, "distance = %g", doutput.distance);
			m_textLine += m_textIncrement;
		}

#if 0
		for (float t = 0.0f; t < 1.0f; t += 0.1f)
		{
			transformB = b2GetSweepTransform(&sweepB, t);
			for (int32_t i = 0; i < m_countB; ++i)
			{
				vertices[i] = b2TransformPoint(transformB, m_verticesB[i]);
			}
			g_draw.DrawPolygon(vertices, m_countB, {0.3f, 0.3f, 0.3f});
		}
#endif
	}

	b2Vec2 m_verticesA[4] = {{-1.0f, -1.0f}, {1.0f, -1.0f}, {1.0f, 5.0f}, {-1.0f, 5.0f}};
	b2Vec2 m_verticesB[4] = {{-0.5f, -4.0f}, {0.0f, -4.0f}, {0.0f, 0.0f}, {-0.5f, 0.0f}};
	int32_t m_countA = B2_ARRAY_COUNT(m_verticesA);
	int32_t m_countB = B2_ARRAY_COUNT(m_verticesB);
};

static int sampleIndex = RegisterSample("Collision", "Time of Impact", TimeOfImpact::Create);
