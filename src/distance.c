// SPDX-FileCopyrightText: 2022 Erin Catto
// SPDX-License-Identifier: MIT

#include "box2d/constants.h"
#include "box2d/distance.h"
#include "box2d/vec_math.h"

#include <assert.h>
#include <float.h>

// GJK using Voronoi regions (Christer Ericson) and Barycentric coordinates.
int32_t b2_gjkCalls, b2_gjkIters, b2_gjkMaxIters;

static b2Vec2 b2Weight2(float a1, b2Vec2 w1, float a2, b2Vec2 w2)
{
	return (b2Vec2) { a1* w1.x + a2 * w2.x, a1* w1.y + a2 * w2.y };
}

static b2Vec2 b2Weight3(float a1, b2Vec2 w1, float a2, b2Vec2 w2, float a3, b2Vec2 w3)
{
	return (b2Vec2) { a1* w1.x + a2 * w2.x + a3 * w3.x, a1* w1.y + a2 * w2.y + a3 * w3.y };
}

static int32_t b2GetSupportIndex(const b2DistanceProxy* proxy, b2Vec2 direction)
{
	int32_t bestIndex = 0;
	float bestValue = b2Dot(proxy->vertices[0], direction);
	for (int32_t i = 1; i < proxy->count; ++i)
	{
		float value = b2Dot(proxy->vertices[i], direction);
		if (value > bestValue)
		{
			bestIndex = i;
			bestValue = value;
		}
	}

	return bestIndex;
}

static b2Vec2 b2GetSupportVertex(const b2DistanceProxy* proxy, b2Vec2 direction)
{
	int32_t bestIndex = 0;
	float bestValue = b2Dot(proxy->vertices[0], direction);
	for (int32_t i = 1; i < proxy->count; ++i)
	{
		float value = b2Dot(proxy->vertices[i], direction);
		if (value > bestValue)
		{
			bestIndex = i;
			bestValue = value;
		}
	}

	return proxy->vertices[bestIndex];
}

#if 0
void b2DistanceProxy::Set(const b2Shape* shape, int32_t index)
{
	switch (shape->GetType())
	{
	case b2Shape::e_circle:
	{
		const b2CircleShape* circle = static_cast<const b2CircleShape*>(shape);
		m_vertices = &circle->m_p;
		m_count = 1;
		m_radius = circle->m_radius;
	}
	break;

	case b2Shape::e_polygon:
	{
		const b2PolygonShape* polygon = static_cast<const b2PolygonShape*>(shape);
		m_vertices = polygon->m_vertices;
		m_count = polygon->m_count;
		m_radius = polygon->m_radius;
	}
	break;

	case b2Shape::e_chain:
	{
		const b2ChainShape* chain = static_cast<const b2ChainShape*>(shape);
		assert(0 <= index && index < chain->m_count);

		m_buffer[0] = chain->m_vertices[index];
		if (index + 1 < chain->m_count)
		{
			m_buffer[1] = chain->m_vertices[index + 1];
		}
		else
		{
			m_buffer[1] = chain->m_vertices[0];
		}

		m_vertices = m_buffer;
		m_count = 2;
		m_radius = chain->m_radius;
	}
	break;

	case b2Shape::e_edge:
	{
		const b2EdgeShape* edge = static_cast<const b2EdgeShape*>(shape);
		m_vertices = &edge->m_vertex1;
		m_count = 2;
		m_radius = edge->m_radius;
	}
	break;

	default:
		assert(false);
	}
}
#endif

typedef struct b2SimplexVertex
{
	b2Vec2 wA;		// support point in proxyA
	b2Vec2 wB;		// support point in proxyB
	b2Vec2 w;		// wB - wA
	float a;		// barycentric coordinate for closest point
	int32_t indexA;	// wA index
	int32_t indexB;	// wB index
} b2SimplexVertex;

typedef struct b2Simplex
{
	b2SimplexVertex v1, v2, v3;
	int32_t count;
} b2Simplex;

static float b2Simplex_Metric(const b2Simplex* s)
{
	switch (s->count)
	{
	case 0:
		assert(false);
		return 0.0f;

	case 1:
		return 0.0f;

	case 2:
		return b2Distance(s->v1.w, s->v2.w);

	case 3:
		return b2Cross(b2Sub(s->v2.w, s->v1.w), b2Sub(s->v3.w, s->v1.w));

	default:
		assert(false);
		return 0.0f;
	}
}

static b2Simplex b2Simplex_ReadCache(const b2DistanceCache* cache,
	const b2DistanceProxy* proxyA, b2Transform transformA,
	const b2DistanceProxy* proxyB, b2Transform transformB)
{
	assert(cache->count <= 3);
	b2Simplex s;

	// Copy data from cache.
	s.count = cache->count;
	b2SimplexVertex* vertices[] = { &s.v1, &s.v2, &s.v3 };
	for (int32_t i = 0; i < s.count; ++i)
	{
		b2SimplexVertex* v = vertices[i];
		v->indexA = cache->indexA[i];
		v->indexB = cache->indexB[i];
		b2Vec2 wALocal = proxyA->vertices[v->indexA];
		b2Vec2 wBLocal = proxyB->vertices[v->indexB];
		v->wA = b2TransformPoint(transformA, wALocal);
		v->wB = b2TransformPoint(transformB, wBLocal);
		v->w = b2Sub(v->wB, v->wA);
		v->a = 0.0f;
	}

	// Compute the new simplex metric, if it is substantially different than
	// old metric then flush the simplex.
	if (s.count > 1)
	{
		float metric1 = cache->metric;
		float metric2 = b2Simplex_Metric(&s);
		if (metric2 < 0.5f * metric1 || 2.0f * metric1 < metric2 || metric2 < FLT_EPSILON)
		{
			// Reset the simplex.
			s.count = 0;
		}
	}

	// If the cache is empty or invalid ...
	if (s.count == 0)
	{
		b2SimplexVertex* v = vertices[0];
		v->indexA = 0;
		v->indexB = 0;
		b2Vec2 wALocal = proxyA->vertices[0];
		b2Vec2 wBLocal = proxyB->vertices[0];
		v->wA = b2TransformPoint(transformA, wALocal);
		v->wB = b2TransformPoint(transformB, wBLocal);
		v->w = b2Sub(v->wB, v->wA);
		v->a = 1.0f;
		s.count = 1;
	}

	return s;
}

static void b2Simplex_WriteCache(b2DistanceCache* cache, const b2Simplex* simplex)
{
	cache->metric = b2Simplex_Metric(simplex);
	cache->count = (uint16_t)simplex->count;
	const b2SimplexVertex* vertices[] = { &simplex->v1, &simplex->v2, &simplex->v3 };
	for (int32_t i = 0; i < simplex->count; ++i)
	{
		cache->indexA[i] = (uint8_t)vertices[i]->indexA;
		cache->indexB[i] = (uint8_t)vertices[i]->indexB;
	}
}

b2Vec2 b2Simplex_SearchDirection(const b2Simplex* simplex)
{
	switch (simplex->count)
	{
	case 1:
		return b2Neg(simplex->v1.w);

	case 2:
	{
		b2Vec2 e12 = b2Sub(simplex->v2.w, simplex->v1.w);
		float sgn = b2Cross(e12, b2Neg(simplex->v1.w));
		if (sgn > 0.0f)
		{
			// Origin is left of e12.
			return b2CrossSV(1.0f, e12);
		}
		else
		{
			// Origin is right of e12.
			return b2CrossVS(e12, 1.0f);
		}
	}

	default:
		assert(false);
		return b2Vec2_zero;
	}
}

b2Vec2 b2Simplex_ClosestPoint(const b2Simplex* s)
{
	switch (s->count)
	{
	case 0:
		assert(false);
		return b2Vec2_zero;

	case 1:
		return s->v1.w;

	case 2:
		return b2Weight2(s->v1.a, s->v1.w, s->v2.a, s->v2.w);

	case 3:
		return b2Vec2_zero;

	default:
		assert(false);
		return b2Vec2_zero;
	}
}

void b2Simplex_WitnessPoints(b2Vec2* a, b2Vec2* b, const b2Simplex* s)
{
	switch (s->count)
	{
	case 0:
		assert(false);
		break;

	case 1:
		*a = s->v1.wA;
		*b = s->v1.wB;
		break;

	case 2:
		*a = b2Weight2(s->v1.a, s->v1.wA, s->v2.a, s->v2.wA);
		*b = b2Weight2(s->v1.a, s->v1.wB, s->v2.a, s->v2.wB);
		break;

	case 3:
		*a = b2Weight3(s->v1.a, s->v1.wA, s->v2.a, s->v2.wA, s->v3.a, s->v3.wA);
		*b = b2Weight3(s->v1.a, s->v1.wB, s->v2.a, s->v2.wB, s->v3.a, s->v3.wB);
		break;

	default:
		assert(false);
		break;
	}
}

// Solve a line segment using barycentric coordinates.
//
// p = a1 * w1 + a2 * w2
// a1 + a2 = 1
//
// The vector from the origin to the closest point on the line is
// perpendicular to the line.
// e12 = w2 - w1
// dot(p, e) = 0
// a1 * dot(w1, e) + a2 * dot(w2, e) = 0
//
// 2-by-2 linear system
// [1      1     ][a1] = [1]
// [w1.e12 w2.e12][a2] = [0]
//
// Define
// d12_1 =  dot(w2, e12)
// d12_2 = -dot(w1, e12)
// d12 = d12_1 + d12_2
//
// Solution
// a1 = d12_1 / d12
// a2 = d12_2 / d12
void b2Simplex_Solve2(b2Simplex* s)
{
	b2Vec2 w1 = s->v1.w;
	b2Vec2 w2 = s->v2.w;
	b2Vec2 e12 = b2Sub(w2, w1);

	// w1 region
	float d12_2 = -b2Dot(w1, e12);
	if (d12_2 <= 0.0f)
	{
		// a2 <= 0, so we clamp it to 0
		s->v1.a = 1.0f;
		s->count = 1;
		return;
	}

	// w2 region
	float d12_1 = b2Dot(w2, e12);
	if (d12_1 <= 0.0f)
	{
		// a1 <= 0, so we clamp it to 0
		s->v2.a = 1.0f;
		s->count = 1;
		s->v1 = s->v2;
		return;
	}

	// Must be in e12 region.
	float inv_d12 = 1.0f / (d12_1 + d12_2);
	s->v1.a = d12_1 * inv_d12;
	s->v2.a = d12_2 * inv_d12;
	s->count = 2;
}

// Possible regions:
// - points[2]
// - edge points[0]-points[2]
// - edge points[1]-points[2]
// - inside the triangle
void b2Simplex_Solve3(b2Simplex* s)
{
	b2Vec2 w1 = s->v1.w;
	b2Vec2 w2 = s->v2.w;
	b2Vec2 w3 = s->v3.w;

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
		s->v1.a = 1.0f;
		s->count = 1;
		return;
	}

	// e12
	if (d12_1 > 0.0f && d12_2 > 0.0f && d123_3 <= 0.0f)
	{
		float inv_d12 = 1.0f / (d12_1 + d12_2);
		s->v1.a = d12_1 * inv_d12;
		s->v2.a = d12_2 * inv_d12;
		s->count = 2;
		return;
	}

	// e13
	if (d13_1 > 0.0f && d13_2 > 0.0f && d123_2 <= 0.0f)
	{
		float inv_d13 = 1.0f / (d13_1 + d13_2);
		s->v1.a = d13_1 * inv_d13;
		s->v3.a = d13_2 * inv_d13;
		s->count = 2;
		s->v2 = s->v3;
		return;
	}

	// w2 region
	if (d12_1 <= 0.0f && d23_2 <= 0.0f)
	{
		s->v2.a = 1.0f;
		s->count = 1;
		s->v1 = s->v2;
		return;
	}

	// w3 region
	if (d13_1 <= 0.0f && d23_1 <= 0.0f)
	{
		s->v3.a = 1.0f;
		s->count = 1;
		s->v1 = s->v3;
		return;
	}

	// e23
	if (d23_1 > 0.0f && d23_2 > 0.0f && d123_1 <= 0.0f)
	{
		float inv_d23 = 1.0f / (d23_1 + d23_2);
		s->v2.a = d23_1 * inv_d23;
		s->v3.a = d23_2 * inv_d23;
		s->count = 2;
		s->v1 = s->v3;
		return;
	}

	// Must be in triangle123
	float inv_d123 = 1.0f / (d123_1 + d123_2 + d123_3);
	s->v1.a = d123_1 * inv_d123;
	s->v2.a = d123_2 * inv_d123;
	s->v3.a = d123_3 * inv_d123;
	s->count = 3;
}

void b2ShapeDistance(b2DistanceOutput* output, b2DistanceCache* cache, const b2DistanceInput* input)
{
	++b2_gjkCalls;

	const b2DistanceProxy* proxyA = &input->proxyA;
	const b2DistanceProxy* proxyB = &input->proxyB;

	b2Transform transformA = input->transformA;
	b2Transform transformB = input->transformB;

	// Initialize the simplex.
	b2Simplex simplex = b2Simplex_ReadCache(cache, proxyA, transformA, proxyB, transformB);

	// Get simplex vertices as an array.
	b2SimplexVertex* vertices[] = { &simplex.v1, &simplex.v2, &simplex.v3 };
	const int32_t k_maxIters = 20;

	// These store the vertices of the last simplex so that we
	// can check for duplicates and prevent cycling.
	int32_t saveA[3], saveB[3];
	int32_t saveCount = 0;

	// Main iteration loop.
	int32_t iter = 0;
	while (iter < k_maxIters)
	{
		// Copy simplex so we can identify duplicates.
		saveCount = simplex.count;
		for (int32_t i = 0; i < saveCount; ++i)
		{
			saveA[i] = vertices[i]->indexA;
			saveB[i] = vertices[i]->indexB;
		}

		switch (simplex.count)
		{
		case 1:
			break;

		case 2:
			b2Simplex_Solve2(&simplex);
			break;

		case 3:
			b2Simplex_Solve3(&simplex);
			break;

		default:
			assert(false);
		}

		// If we have 3 points, then the origin is in the corresponding triangle.
		if (simplex.count == 3)
		{
			break;
		}

		// Get search direction.
		b2Vec2 d = b2Simplex_SearchDirection(&simplex);

		// Ensure the search direction is numerically fit.
		if (b2Dot(d, d) < FLT_EPSILON * FLT_EPSILON)
		{
			// The origin is probably contained by a line segment
			// or triangle. Thus the shapes are overlapped.

			// We can't return zero here even though there may be overlap.
			// In case the simplex is a point, segment, or triangle it is difficult
			// to determine if the origin is contained in the CSO or very close to it.
			break;
		}

		// Compute a tentative new simplex vertex using support points.
		b2SimplexVertex* vertex = vertices[simplex.count];
		vertex->indexA = b2GetSupportIndex(proxyA, b2InvRotVec(transformA.q, b2Neg(d)));
		vertex->wA = b2TransformPoint(transformA, proxyA->vertices[vertex->indexA]);
		vertex->indexB = b2GetSupportIndex(proxyB, b2InvRotVec(transformB.q, d));
		vertex->wB = b2TransformPoint(transformB, proxyB->vertices[vertex->indexB]);
		vertex->w = b2Sub(vertex->wB, vertex->wA);

		// Iteration count is equated to the number of support point calls.
		++iter;
		++b2_gjkIters;

		// Check for duplicate support points. This is the main termination criteria.
		bool duplicate = false;
		for (int32_t i = 0; i < saveCount; ++i)
		{
			if (vertex->indexA == saveA[i] && vertex->indexB == saveB[i])
			{
				duplicate = true;
				break;
			}
		}

		// If we found a duplicate support point we must exit to avoid cycling.
		if (duplicate)
		{
			break;
		}

		// New vertex is ok and needed.
		++simplex.count;
	}

	b2_gjkMaxIters = B2_MAX(b2_gjkMaxIters, iter);

	// Prepare output
	b2Simplex_WitnessPoints(&output->pointA, &output->pointB, &simplex);
	output->distance = b2Distance(output->pointA, output->pointB);
	output->iterations = iter;

	// Cache the simplex
	b2Simplex_WriteCache(cache, &simplex);

	// Apply radii if requested
	if (input->useRadii)
	{
		if (output->distance < FLT_EPSILON)
		{
			// Shapes are too close to safely compute normal
			b2Vec2 p = (b2Vec2){ 0.5f * (output->pointA.x + output->pointB.x),
								0.5f * (output->pointA.y + output->pointB.y) };
			output->pointA = p;
			output->pointB = p;
			output->distance = 0.0f;
		}
		else
		{
			// Keep closest points on perimeter even if overlapped, this way
			// the points move smoothly.
			float rA = proxyA->radius;
			float rB = proxyB->radius;
			output->distance = B2_MAX(0.0f, output->distance - rA - rB);
			b2Vec2 normal = b2Normalize(b2Sub(output->pointB, output->pointA));
			b2Vec2 offsetA = (b2Vec2){ rA * normal.x, rA * normal.y };
			b2Vec2 offsetB = (b2Vec2){ rB * normal.x, rB * normal.y };
			output->pointA = b2Add(output->pointA, offsetA);
			output->pointB = b2Sub(output->pointB, offsetB);
		}
	}
}

// GJK-raycast
// Algorithm by Gino van den Bergen.
// "Smooth Mesh Contacts with GJK" in Game Physics Pearls. 2010
bool b2ShapeCast(b2ShapeCastOutput* output, const b2ShapeCastInput* input)
{
	output->iterations = 0;
	output->lambda = 1.0f;
	output->normal = b2Vec2_zero;
	output->point = b2Vec2_zero;

	const b2DistanceProxy* proxyA = &input->proxyA;
	const b2DistanceProxy* proxyB = &input->proxyB;

	float radius = proxyA->radius + proxyB->radius;

	b2Transform xfA = input->transformA;
	b2Transform xfB = input->transformB;

	b2Vec2 r = input->translationB;
	b2Vec2 n = b2Vec2_zero;
	float lambda = 0.0f;

	// Initial simplex
	b2Simplex simplex;
	simplex.count = 0;

	// Get simplex vertices as an array.
	b2SimplexVertex* vertices[] = { &simplex.v1, &simplex.v2, &simplex.v3 };

	// Get support point in -r direction
	int32_t indexA = b2GetSupportIndex(proxyA, b2InvRotVec(xfA.q, b2Neg(r)));
	b2Vec2 wA = b2TransformPoint(xfA, proxyA->vertices[indexA]);
	int32_t indexB = b2GetSupportIndex(proxyB, b2InvRotVec(xfB.q, r));
	b2Vec2 wB = b2TransformPoint(xfB, proxyB->vertices[indexB]);
	b2Vec2 v = b2Sub(wA, wB);

	// Sigma is the target distance between proxies
	const float sigma = B2_MAX(b2_linearSlop, radius - b2_linearSlop);

	// Main iteration loop.
	const int32_t k_maxIters = 20;
	int32_t iter = 0;
	while (iter < k_maxIters && b2Length(v) > sigma)
	{
		assert(simplex.count < 3);

		output->iterations += 1;

		// Support in direction -v (A - B)
		indexA = b2GetSupportIndex(proxyA, b2InvRotVec(xfA.q, b2Neg(v)));
		wA = b2TransformPoint(xfA, proxyA->vertices[indexA]);
		indexB = b2GetSupportIndex(proxyB, b2InvRotVec(xfB.q, v));
		wB = b2TransformPoint(xfB, proxyB->vertices[indexB]);
		b2Vec2 p = b2Sub(wA, wB);

		// -v is a normal at p
		v = b2Normalize(v);

		// Intersect ray with plane
		float vp = b2Dot(v, p);
		float vr = b2Dot(v, r);
		if (vp - sigma > lambda * vr)
		{
			if (vr <= 0.0f)
			{
				return false;
			}

			lambda = (vp - sigma) / vr;
			if (lambda > 1.0f)
			{
				return false;
			}

			n = (b2Vec2){ -v.x, -v.y };
			simplex.count = 0;
		}

		// Reverse simplex since it works with B - A.
		// Shift by lambda * r because we want the closest point to the current clip point.
		// Note that the support point p is not shifted because we want the plane equation
		// to be formed in unshifted space.
		b2SimplexVertex* vertex = vertices[simplex.count];
		vertex->indexA = indexB;
		vertex->wA = (b2Vec2){ wB.x + lambda * r.x, wB.y + lambda * r.y };
		vertex->indexB = indexA;
		vertex->wB = wA;
		vertex->w = b2Sub(vertex->wB, vertex->wA);
		vertex->a = 1.0f;
		simplex.count += 1;

		switch (simplex.count)
		{
		case 1:
			break;

		case 2:
			b2Simplex_Solve2(&simplex);
			break;

		case 3:
			b2Simplex_Solve3(&simplex);
			break;

		default:
			assert(false);
		}

		// If we have 3 points, then the origin is in the corresponding triangle.
		if (simplex.count == 3)
		{
			// Overlap
			return false;
		}

		// Get search direction.
		v = b2Simplex_ClosestPoint(&simplex);

		// Iteration count is equated to the number of support point calls.
		++iter;
	}

	if (iter == 0)
	{
		// Initial overlap
		return false;
	}

	// Prepare output.
	b2Vec2 pointA, pointB;
	b2Simplex_WitnessPoints(&pointB, &pointA, &simplex);

	if (b2Dot(v, v) > 0.0f)
	{
		n = b2Normalize(b2Neg(v));
	}

	float radiusA = proxyA->radius;
	output->point = (b2Vec2){ pointA.x + radiusA * n.x, pointA.y + radiusA * n.y };
	output->normal = n;
	output->lambda = lambda;
	output->iterations = iter;
	return true;
}
