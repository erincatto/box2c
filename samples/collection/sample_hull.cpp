// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "sample.h"
#include "box2d/hull.h"
#include "box2d/math.h"

#include <GLFW/glfw3.h>

class ConvexHull : public Sample
{
public:
	enum
	{
		e_count = b2_maxPolygonVertices
	};

	ConvexHull(const Settings& settings)
		: Sample(settings)
	{
		m_generation = 0;
		m_auto = false;
		m_bulk = false;
		Generate();
	}

	void Generate()
	{
#if 0
		m_points[0] = { 5.65314484f, 0.204832315f };
		m_points[1] = {-5.65314484f, -0.204832315f };
		m_points[2] = {2.34463644f, 1.15731204f };
		m_points[3] = {0.0508846045f, 3.23230696f };
		m_points[4] = {-5.65314484f, -0.204832315f };
		m_points[5] = {-5.65314484f, -0.204832315f };
		m_points[6] = {3.73758054f, -1.11098099f };
		m_points[7] = {1.33504069f, -4.43795443f };

		m_count = e_count;
#else

		float angle = b2_pi * RandomFloat();
		b2Rot r = b2MakeRot(angle);

		b2Vec2 lowerBound = {-4.0f, -4.0f};
		b2Vec2 upperBound = {4.0f, 4.0f};

		for (int32_t i = 0; i < e_count; ++i)
		{
			float x = 10.0f * RandomFloat();
			float y = 10.0f * RandomFloat();

			// Clamp onto a square to help create collinearities.
			// This will stress the convex hull algorithm.
			b2Vec2 v = b2Clamp({x, y}, lowerBound, upperBound);
			m_points[i] = b2RotateVector(r, v);
		}

		m_count = e_count;
#endif

		m_generation += 1;
	}

	void Keyboard(int key) override
	{
		switch (key)
		{
		case GLFW_KEY_A:
			m_auto = !m_auto;
			break;

		case GLFW_KEY_B:
			m_bulk = !m_bulk;
			break;

		case GLFW_KEY_G:
			Generate();
			break;
		}
	}

	void Step(Settings& settings) override
	{
		Sample::Step(settings);

		g_draw.DrawString(5, m_textLine, "Options: generate(g), auto(a), bulk(b)");
		m_textLine += m_textIncrement;
		
		b2Hull hull;
		bool valid = false;
		float milliseconds = 0.0f;

		if (m_bulk)
		{
#if 1
			// defect hunting
			for (int32_t i = 0; i < 10000; ++i)
			{
				Generate();
				hull = b2ComputeHull(m_points, m_count);
				if (hull.count == 0)
				{
					//m_bulk = false;
					//break;
					continue;
				}

				valid = b2ValidateHull(&hull);
				if (valid == false || m_bulk == false)
				{
					m_bulk = false;
					break;
				}
			}
#else
			// performance
			Generate();
			b2Timer timer;
			for (int32_t i = 0; i < 1000000; ++i)
			{
				hull = b2ComputeHull(m_points, m_count);
			}
			valid = hull.count > 0;
			milliseconds = timer.GetMilliseconds();
#endif
		}
		else
		{
			if (m_auto)
			{
				Generate();
			}

			hull = b2ComputeHull(m_points, m_count);
			if (hull.count > 0)
			{
				valid = b2ValidateHull(&hull);
				if (valid == false)
				{
					m_auto = false;
				}
			}
		}

		if (valid == false)
		{
			g_draw.DrawString(5, m_textLine, "generation = %d, FAILED", m_generation);
			m_textLine += m_textIncrement;
		}
		else
		{
			g_draw.DrawString(5, m_textLine, "generation = %d, count = %d", m_generation, hull.count);
			m_textLine += m_textIncrement;
		}

		if (milliseconds > 0.0f)
		{
			g_draw.DrawString(5, m_textLine, "milliseconds = %g", milliseconds);
			m_textLine += m_textIncrement;
		}

		m_textLine += m_textIncrement;

		g_draw.DrawPolygon(hull.points, hull.count, {0.9f, 0.9f, 0.9f, 1.0f});

		for (int32_t i = 0; i < m_count; ++i)
		{
			g_draw.DrawPoint(m_points[i], 5.0f, {0.3f, 0.3f, 0.9f, 1.0f});
			g_draw.DrawString(b2Add(m_points[i], {0.1f, 0.1f}), "%d", i);
		}

		for (int32_t i = 0; i < hull.count; ++i)
		{
			g_draw.DrawPoint(hull.points[i], 6.0f, {0.3f, 0.7f, 0.3f, 1.0f});
		}
	}

	static Sample* Create(const Settings& settings)
	{
		return new ConvexHull(settings);
	}

	b2Vec2 m_points[b2_maxPolygonVertices];
	int32_t m_count;
	int32_t m_generation;
	bool m_auto;
	bool m_bulk;
};

static int sampleIndex = RegisterSample("Geometry", "Convex Hull", ConvexHull::Create);
