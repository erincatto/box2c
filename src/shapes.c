// SPDX-FileCopyrightText: 2022 Erin Catto
// SPDX-License-Identifier: MIT

#include "shapes.h"
#include "box2d/collision.h"
#include "box2d/hull.h"
#include "box2d/vec_math.h"

#include <assert.h>
#include <float.h>

b2PolygonShape b2MakePolygon(const b2Hull* hull)
{
	assert(hull->count >= 3);

	b2PolygonShape s;
	s.count = hull->count;

	// Copy vertices
	for (int32_t i = 0; i < s.count; ++i)
	{
		s.vertices[i] = hull->points[i];
	}

	// Compute normals. Ensure the edges have non-zero length.
	for (int32_t i = 0; i < s.count; ++i)
	{
		int32_t i1 = i;
		int32_t i2 = i + 1 < s.count ? i + 1 : 0;
		b2Vec2 edge = b2Sub(s.vertices[i2], s.vertices[i1]);
		assert(b2Dot(edge, edge) > FLT_EPSILON * FLT_EPSILON);
		s.normals[i] = b2Normalize(b2CrossVS(edge, 1.0f));
	}

	return s;
}

b2PolygonShape b2MakeBox(float hx, float hy, b2Vec2 center, float angle)
{
	b2Transform xf;
	xf.p = center;
	xf.q = b2Rot_Set(angle);

	b2PolygonShape s;
	s.count = 4;
	s.vertices[0] = b2TransformPoint(xf, (b2Vec2){-hx, -hy});
	s.vertices[1] = b2TransformPoint(xf, (b2Vec2){hx, -hy});
	s.vertices[2] = b2TransformPoint(xf, (b2Vec2){hx, hy});
	s.vertices[3] = b2TransformPoint(xf, (b2Vec2){-hx, hy});
	s.normals[0] = b2RotateVector(xf.q, (b2Vec2){0.0f, -1.0f});
	s.normals[1] = b2RotateVector(xf.q, (b2Vec2){1.0f, 0.0f});
	s.normals[2] = b2RotateVector(xf.q, (b2Vec2){0.0f, 1.0f});
	s.normals[3] = b2RotateVector(xf.q, (b2Vec2){-1.0f, 0.0f});

	return s;
}

//b2MassData b2ComputeCircleMass(const b2CircleShape* shape);
//b2MassData b2ComputeCapsuleMass(const b2CapsuleShape* shape);
//b2MassData b2ComputePolygonMass(const b2PolygonShape* shape);
//
//b2AABB b2ComputeCircleAABB(const b2CircleShape* shape, b2Transform transform);
//b2AABB b2ComputeCapsuleAABB(const b2CapsuleShape* shape, b2Transform transform);
//b2AABB b2ComputePolygonAABB(const b2PolygonShape* shape, b2Transform transform);
//b2AABB b2ComputeSegmentAABB(const b2SegmentShape* shape, b2Transform transform);
//
//bool b2PointTestCircle(b2RayCastOutput* output, const b2RayCastInput* input, b2CircleShape* shape, b2Transform transform);
//bool b2PointTestCapsule(b2RayCastOutput* output, const b2RayCastInput* input, b2CapsuleShape* shape, b2Transform transform);
//bool b2PointTestPolygon(b2RayCastOutput* output, const b2RayCastInput* input, b2PolygonShape* shape, b2Transform transform);
//
//bool b2RayCastCircle(b2RayCastOutput* output, const b2RayCastInput* input, b2CircleShape* shape, b2Transform transform);
//bool b2RayCastCapsule(b2RayCastOutput* output, const b2RayCastInput* input, b2CapsuleShape* shape, b2Transform transform);
//bool b2RayCastSegment(b2RayCastOutput* output, const b2RayCastInput* input, b2SegmentShape* shape, b2Transform transform);
//bool b2RayCastPolygon(b2RayCastOutput* output, const b2RayCastInput* input, b2PolygonShape* shape, b2Transform transform);

#if 0
static b2Vec2 ComputeCentroid(const b2Vec2* vs, int32_t count)
{
	b2Assert(count >= 3);

	b2Vec2 c(0.0f, 0.0f);
	float area = 0.0f;

	// Get a reference point for forming triangles.
	// Use the first vertex to reduce round-off errors.
	b2Vec2 s = vs[0];

	const float inv3 = 1.0f / 3.0f;

	for (int32_t i = 0; i < count; ++i)
	{
		// Triangle vertices.
		b2Vec2 p1 = vs[0] - s;
		b2Vec2 p2 = vs[i] - s;
		b2Vec2 p3 = i + 1 < count ? vs[i+1] - s : vs[0] - s;

		b2Vec2 e1 = p2 - p1;
		b2Vec2 e2 = p3 - p1;

		float D = b2Cross(e1, e2);

		float triangleArea = 0.5f * D;
		area += triangleArea;

		// Area weighted centroid
		c += triangleArea * inv3 * (p1 + p2 + p3);
	}

	// Centroid
	b2Assert(area > b2_epsilon);
	c = (1.0f / area) * c + s;
	return c;
}
#endif

#if 0
bool b2PolygonShape::TestPoint(const b2Transform& xf, const b2Vec2& p) const
{
	b2Vec2 pLocal = b2MulT(xf.q, p - xf.p);

	for (int32_t i = 0; i < m_count; ++i)
	{
		float dot = b2Dot(m_normals[i], pLocal - m_vertices[i]);
		if (dot > 0.0f)
		{
			return false;
		}
	}

	return true;
}

bool b2PolygonShape::RayCast(b2RayCastOutput* output, const b2RayCastInput& input,
								const b2Transform& xf, int32_t childIndex) const
{
	B2_NOT_USED(childIndex);

	// Put the ray into the polygon's frame of reference.
	b2Vec2 p1 = b2MulT(xf.q, input.p1 - xf.p);
	b2Vec2 p2 = b2MulT(xf.q, input.p2 - xf.p);
	b2Vec2 d = p2 - p1;

	float lower = 0.0f, upper = input.maxFraction;

	int32_t index = -1;

	for (int32_t i = 0; i < m_count; ++i)
	{
		// p = p1 + a * d
		// dot(normal, p - v) = 0
		// dot(normal, p1 - v) + a * dot(normal, d) = 0
		float numerator = b2Dot(m_normals[i], m_vertices[i] - p1);
		float denominator = b2Dot(m_normals[i], d);

		if (denominator == 0.0f)
		{	
			if (numerator < 0.0f)
			{
				return false;
			}
		}
		else
		{
			// Note: we want this predicate without division:
			// lower < numerator / denominator, where denominator < 0
			// Since denominator < 0, we have to flip the inequality:
			// lower < numerator / denominator <==> denominator * lower > numerator.
			if (denominator < 0.0f && numerator < lower * denominator)
			{
				// Increase lower.
				// The segment enters this half-space.
				lower = numerator / denominator;
				index = i;
			}
			else if (denominator > 0.0f && numerator < upper * denominator)
			{
				// Decrease upper.
				// The segment exits this half-space.
				upper = numerator / denominator;
			}
		}

		// The use of epsilon here causes the assert on lower to trip
		// in some cases. Apparently the use of epsilon was to make edge
		// shapes work, but now those are handled separately.
		//if (upper < lower - b2_epsilon)
		if (upper < lower)
		{
			return false;
		}
	}

	b2Assert(0.0f <= lower && lower <= input.maxFraction);

	if (index >= 0)
	{
		output->fraction = lower;
		output->normal = b2Mul(xf.q, m_normals[index]);
		return true;
	}

	return false;
}

void b2PolygonShape::ComputeAABB(b2AABB* aabb, const b2Transform& xf, int32_t childIndex) const
{
	B2_NOT_USED(childIndex);

	b2Vec2 lower = b2Mul(xf, m_vertices[0]);
	b2Vec2 upper = lower;

	for (int32_t i = 1; i < m_count; ++i)
	{
		b2Vec2 v = b2Mul(xf, m_vertices[i]);
		lower = b2Min(lower, v);
		upper = b2Max(upper, v);
	}

	b2Vec2 r(m_radius, m_radius);
	aabb->lowerBound = lower - r;
	aabb->upperBound = upper + r;
}

void b2PolygonShape::ComputeMass(b2MassData* massData, float density) const
{
	// Polygon mass, centroid, and inertia.
	// Let rho be the polygon density in mass per unit area.
	// Then:
	// mass = rho * int(dA)
	// centroid.x = (1/mass) * rho * int(x * dA)
	// centroid.y = (1/mass) * rho * int(y * dA)
	// I = rho * int((x*x + y*y) * dA)
	//
	// We can compute these integrals by summing all the integrals
	// for each triangle of the polygon. To evaluate the integral
	// for a single triangle, we make a change of variables to
	// the (u,v) coordinates of the triangle:
	// x = x0 + e1x * u + e2x * v
	// y = y0 + e1y * u + e2y * v
	// where 0 <= u && 0 <= v && u + v <= 1.
	//
	// We integrate u from [0,1-v] and then v from [0,1].
	// We also need to use the Jacobian of the transformation:
	// D = cross(e1, e2)
	//
	// Simplification: triangle centroid = (1/3) * (p1 + p2 + p3)
	//
	// The rest of the derivation is handled by computer algebra.

	b2Assert(m_count >= 3);

	b2Vec2 center(0.0f, 0.0f);
	float area = 0.0f;
	float I = 0.0f;

	// Get a reference point for forming triangles.
	// Use the first vertex to reduce round-off errors.
	b2Vec2 s = m_vertices[0];

	const float k_inv3 = 1.0f / 3.0f;

	for (int32_t i = 0; i < m_count; ++i)
	{
		// Triangle vertices.
		b2Vec2 e1 = m_vertices[i] - s;
		b2Vec2 e2 = i + 1 < m_count ? m_vertices[i+1] - s : m_vertices[0] - s;

		float D = b2Cross(e1, e2);

		float triangleArea = 0.5f * D;
		area += triangleArea;

		// Area weighted centroid
		center += triangleArea * k_inv3 * (e1 + e2);

		float ex1 = e1.x, ey1 = e1.y;
		float ex2 = e2.x, ey2 = e2.y;

		float intx2 = ex1*ex1 + ex2*ex1 + ex2*ex2;
		float inty2 = ey1*ey1 + ey2*ey1 + ey2*ey2;

		I += (0.25f * k_inv3 * D) * (intx2 + inty2);
	}

	// Total mass
	massData->mass = density * area;

	// Center of mass
	b2Assert(area > b2_epsilon);
	center *= 1.0f / area;
	massData->center = center + s;

	// Inertia tensor relative to the local origin (point s).
	massData->I = density * I;
	
	// Shift to center of mass then to original body origin.
	massData->I += massData->mass * (b2Dot(massData->center, massData->center) - b2Dot(center, center));
}
#endif
