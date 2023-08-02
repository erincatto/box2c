
// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "box2d/distance.h"

#include "core.h"

#include "box2d/constants.h"
#include "box2d/math.h"
#include "box2d/timer.h"

#include <float.h>

b2Transform b2GetSweepTransform(const b2Sweep* sweep, float time)
{
	// https://fgiesen.wordpress.com/2012/08/15/linear-interpolation-past-present-and-future/
	b2Transform xf;
	xf.p = b2Add(b2MulSV(1.0f - time, sweep->c1), b2MulSV(time, sweep->c2));
	float angle = (1.0f - time) * sweep->a1 + time * sweep->a2;
	xf.q = b2MakeRot(angle);

	// Shift to origin
	xf.p = b2Sub(xf.p, b2RotateVector(xf.q, sweep->localCenter));
	return xf;
}

/// Follows Ericson 5.1.9 Closest Points of Two Line Segments
b2SegmentDistanceResult b2SegmentDistance(b2Vec2 p1, b2Vec2 q1, b2Vec2 p2, b2Vec2 q2)
{
	b2SegmentDistanceResult result = {0};

	b2Vec2 d1 = b2Sub(q1, p1);
	b2Vec2 d2 = b2Sub(q2, p2);
	b2Vec2 r = b2Sub(p1, p2);
	float dd1 = b2Dot(d1, d1);
	float dd2 = b2Dot(d2, d2);
	float rd2 = b2Dot(r, d2);
	float rd1 = b2Dot(r, d1);

	const float epsSqr = FLT_EPSILON * FLT_EPSILON;

	if (dd1 < epsSqr || dd2 < epsSqr)
	{
		// Handle all degeneracies
		if (dd1 >= epsSqr)
		{
			// Segment 2 is degenerate
			result.fraction1 = B2_CLAMP(-rd1 / dd1, 0.0f, 1.0f);
			result.fraction2 = 0.0f;
		}
		else if (dd2 >= epsSqr)
		{
			// Segment 1 is degenerate
			result.fraction1 = 0.0f;
			result.fraction2 = B2_CLAMP(rd2 / dd2, 0.0f, 1.0f);
		}
		else
		{
			result.fraction1 = 0.0f;
			result.fraction2 = 0.0f;
		}
	}
	else
	{
		// Non-degenerate segments
		float d12 = b2Dot(d1, d2);

		float denom = dd1 * dd2 - d12 * d12;

		// Fraction on segment 1
		float f1 = 0.0f;
		if (denom != 0.0f)
		{
			// not parallel
			f1 = B2_CLAMP((d12 * rd2 - rd1 * dd2) / denom, 0.0f, 1.0f);
		}

		// Compute point on segment 2 closest to p1 + f1 * d1
		float f2 = (d12 * f1 + rd2) / dd2;

		// Clamping of segment 2 requires a do over on segment 1
		if (f2 < 0.0f)
		{
			f2 = 0.0f;
			f1 = B2_CLAMP(-rd1 / dd1, 0.0f, 1.0f);
		}
		else if (f2 > 1.0f)
		{
			f2 = 1.0f;
			f1 = B2_CLAMP((d12 - rd1) / dd1, 0.0f, 1.0f);
		}

		result.fraction1 = f1;
		result.fraction2 = f2;
	}

	result.closest1 = b2MulAdd(p1, result.fraction1, d1);
	result.closest2 = b2MulAdd(p2, result.fraction2, d2);
	result.distanceSquared = b2DistanceSquared(result.closest1, result.closest2);
	return result;
}

// GJK using Voronoi regions (Christer Ericson) and Barycentric coordinates.
int32_t b2_gjkCalls, b2_gjkIters, b2_gjkMaxIters;

b2DistanceProxy b2MakeProxy(const b2Vec2* vertices, int32_t count, float radius)
{
	count = B2_MIN(count, b2_maxPolygonVertices);
	b2DistanceProxy proxy;
	for (int32_t i = 0; i < count; ++i)
	{
		proxy.vertices[i] = vertices[i];
	}
	proxy.count = count;
	proxy.radius = radius;
	return proxy;
}

static b2Vec2 b2Weight2(float a1, b2Vec2 w1, float a2, b2Vec2 w2)
{
	return (b2Vec2){a1 * w1.x + a2 * w2.x, a1 * w1.y + a2 * w2.y};
}

static b2Vec2 b2Weight3(float a1, b2Vec2 w1, float a2, b2Vec2 w2, float a3, b2Vec2 w3)
{
	return (b2Vec2){a1 * w1.x + a2 * w2.x + a3 * w3.x, a1 * w1.y + a2 * w2.y + a3 * w3.y};
}

static int32_t b2FindSupport(const b2DistanceProxy* proxy, b2Vec2 direction)
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

typedef struct b2SimplexVertex
{
	b2Vec2 wA;		// support point in proxyA
	b2Vec2 wB;		// support point in proxyB
	b2Vec2 w;		// wB - wA
	float a;		// barycentric coordinate for closest point
	int32_t indexA; // wA index
	int32_t indexB; // wB index
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
			B2_ASSERT(false);
			return 0.0f;

		case 1:
			return 0.0f;

		case 2:
			return b2Distance(s->v1.w, s->v2.w);

		case 3:
			return b2Cross(b2Sub(s->v2.w, s->v1.w), b2Sub(s->v3.w, s->v1.w));

		default:
			B2_ASSERT(false);
			return 0.0f;
	}
}

#if 0
static b2Simplex b2MakeSimplexFromCache(const b2DistanceCache* cache, const b2DistanceProxy* proxyA, b2Transform transformA,
										const b2DistanceProxy* proxyB, b2Transform transformB)
{
	B2_ASSERT(cache->count <= 3);
	b2Simplex s;

	// Copy data from cache.
	s.count = cache->count;

	b2SimplexVertex* vertices[] = {&s.v1, &s.v2, &s.v3};
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

		// invalid
		v->a = -1.0f;
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
#elseif
static b2Simplex b2MakeSimplexFromCache(const b2DistanceCache* restrict cache, const b2DistanceProxy* restrict proxyA,
										b2Transform transformA, const b2DistanceProxy* restrict proxyB, b2Transform transformB)
{
	B2_ASSERT(cache->count <= 3);
	b2Simplex s;

	// Copy data from cache.
	s.count = cache->count;

	switch (s.count)
	{
		case 3:
		{
			s.v3.indexA = cache->indexA[2];
			s.v3.indexB = cache->indexB[2];
			b2Vec2 wALocal = proxyA->vertices[s.v3.indexA];
			b2Vec2 wBLocal = proxyB->vertices[s.v3.indexB];
			s.v3.wA = b2TransformPoint(transformA, wALocal);
			s.v3.wB = b2TransformPoint(transformB, wBLocal);
			s.v3.w = b2Sub(s.v3.wB, s.v3.wA);

			// invalid
			s.v3.a = -1.0f;
		}
		// fall through

		case 2:
		{
			s.v2.indexA = cache->indexA[1];
			s.v2.indexB = cache->indexB[1];
			b2Vec2 wALocal = proxyA->vertices[s.v2.indexA];
			b2Vec2 wBLocal = proxyB->vertices[s.v2.indexB];
			s.v2.wA = b2TransformPoint(transformA, wALocal);
			s.v2.wB = b2TransformPoint(transformB, wBLocal);
			s.v2.w = b2Sub(s.v2.wB, s.v2.wA);

			// invalid
			s.v2.a = -1.0f;

		}
		// fall through

		case 1:
		{
			s.v1.indexA = cache->indexA[0];
			s.v1.indexB = cache->indexB[0];
			b2Vec2 wALocal = proxyA->vertices[s.v1.indexA];
			b2Vec2 wBLocal = proxyB->vertices[s.v1.indexB];
			s.v1.wA = b2TransformPoint(transformA, wALocal);
			s.v1.wB = b2TransformPoint(transformB, wBLocal);
			s.v1.w = b2Sub(s.v1.wB, s.v1.wA);

			// invalid
			s.v1.a = -1.0f;

		}
		break;

		default:
		{
			s.v1.indexA = 0;
			s.v1.indexB = 0;
			b2Vec2 wALocal = proxyA->vertices[0];
			b2Vec2 wBLocal = proxyB->vertices[0];
			s.v1.wA = b2TransformPoint(transformA, wALocal);
			s.v1.wB = b2TransformPoint(transformB, wBLocal);
			s.v1.w = b2Sub(s.v1.wB, s.v1.wA);
			s.v1.a = 1.0f;
			s.count = 1;
		}
		break;
	}

	return s;
}

#else
static void b2MakeSimplexFromCache(b2Simplex* restrict simplex, const b2DistanceCache* restrict cache, const b2DistanceProxy* restrict proxyA,
										b2Transform transformA, const b2DistanceProxy* restrict proxyB, b2Transform transformB)
{
	B2_ASSERT(cache->count <= 3);

	// Copy data from cache.
	int32_t count = cache->count;
	simplex->count = count;

	switch (count)
	{
		case 3:
		{
			simplex->v3.indexA = cache->indexA[2];
			simplex->v3.indexB = cache->indexB[2];
			b2Vec2 wALocal = proxyA->vertices[simplex->v3.indexA];
			b2Vec2 wBLocal = proxyB->vertices[simplex->v3.indexB];
			simplex->v3.wA = b2TransformPoint(transformA, wALocal);
			simplex->v3.wB = b2TransformPoint(transformB, wBLocal);
			simplex->v3.w = b2Sub(simplex->v3.wB, simplex->v3.wA);

			// invalid
			simplex->v3.a = -1.0f;
		}
			// fall through

		case 2:
		{
			simplex->v2.indexA = cache->indexA[1];
			simplex->v2.indexB = cache->indexB[1];
			b2Vec2 wALocal = proxyA->vertices[simplex->v2.indexA];
			b2Vec2 wBLocal = proxyB->vertices[simplex->v2.indexB];
			simplex->v2.wA = b2TransformPoint(transformA, wALocal);
			simplex->v2.wB = b2TransformPoint(transformB, wBLocal);
			simplex->v2.w = b2Sub(simplex->v2.wB, simplex->v2.wA);

			// invalid
			simplex->v2.a = -1.0f;
		}
			// fall through

		case 1:
		{
			simplex->v1.indexA = cache->indexA[0];
			simplex->v1.indexB = cache->indexB[0];
			b2Vec2 wALocal = proxyA->vertices[simplex->v1.indexA];
			b2Vec2 wBLocal = proxyB->vertices[simplex->v1.indexB];
			simplex->v1.wA = b2TransformPoint(transformA, wALocal);
			simplex->v1.wB = b2TransformPoint(transformB, wBLocal);
			simplex->v1.w = b2Sub(simplex->v1.wB, simplex->v1.wA);

			// invalid
			simplex->v1.a = -1.0f;
		}
		break;

		default:
		{
			simplex->v1.indexA = 0;
			simplex->v1.indexB = 0;
			b2Vec2 wALocal = proxyA->vertices[0];
			b2Vec2 wBLocal = proxyB->vertices[0];
			simplex->v1.wA = b2TransformPoint(transformA, wALocal);
			simplex->v1.wB = b2TransformPoint(transformB, wBLocal);
			simplex->v1.w = b2Sub(simplex->v1.wB, simplex->v1.wA);
			simplex->v1.a = 1.0f;
			simplex->count = 1;
		}
		break;
	}
}
#endif

static void b2MakeSimplexCache(b2DistanceCache* cache, const b2Simplex* simplex)
{
	cache->metric = b2Simplex_Metric(simplex);
	cache->count = (uint16_t)simplex->count;
	const b2SimplexVertex* vertices[] = {&simplex->v1, &simplex->v2, &simplex->v3};
	for (int32_t i = 0; i < simplex->count; ++i)
	{
		cache->indexA[i] = (uint8_t)vertices[i]->indexA;
		cache->indexB[i] = (uint8_t)vertices[i]->indexB;
	}
}

b2Vec2 b2ComputeSimplexSearchDirection(const b2Simplex* simplex)
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
			B2_ASSERT(false);
			return b2Vec2_zero;
	}
}

b2Vec2 b2ComputeSimplexClosestPoint(const b2Simplex* s)
{
	switch (s->count)
	{
		case 0:
			B2_ASSERT(false);
			return b2Vec2_zero;

		case 1:
			return s->v1.w;

		case 2:
			return b2Weight2(s->v1.a, s->v1.w, s->v2.a, s->v2.w);

		case 3:
			return b2Vec2_zero;

		default:
			B2_ASSERT(false);
			return b2Vec2_zero;
	}
}

void b2ComputeSimplexWitnessPoints(b2Vec2* a, b2Vec2* b, const b2Simplex* s)
{
	switch (s->count)
	{
		case 0:
			B2_ASSERT(false);
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
			// TODO_ERIN why are these not equal?
			//*b = b2Weight3(s->v1.a, s->v1.wB, s->v2.a, s->v2.wB, s->v3.a, s->v3.wB);
			*b = *a;
			break;

		default:
			B2_ASSERT(false);
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
void b2SolveSimplex2(b2Simplex* restrict s)
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

void b2SolveSimplex3(b2Simplex* restrict s)
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

b2DistanceOutput b2ShapeDistance(b2DistanceCache* restrict cache, const b2DistanceInput* restrict input)
{
	//++b2_gjkCalls;

	b2DistanceOutput output = {0};

	const b2DistanceProxy* proxyA = &input->proxyA;
	const b2DistanceProxy* proxyB = &input->proxyB;

	b2Transform transformA = input->transformA;
	b2Transform transformB = input->transformB;

	// Initialize the simplex.
	b2Simplex simplex;
	b2MakeSimplexFromCache(&simplex, cache, proxyA, transformA, proxyB, transformB);

	// Get simplex vertices as an array.
	b2SimplexVertex* vertices[] = {&simplex.v1, &simplex.v2, &simplex.v3};
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
				b2SolveSimplex2(&simplex);
				break;

			case 3:
				b2SolveSimplex3(&simplex);
				break;

			default:
				B2_ASSERT(false);
		}

		// If we have 3 points, then the origin is in the corresponding triangle.
		if (simplex.count == 3)
		{
			break;
		}

		// Get search direction.
		b2Vec2 d = b2ComputeSimplexSearchDirection(&simplex);

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
		vertex->indexA = b2FindSupport(proxyA, b2InvRotateVector(transformA.q, b2Neg(d)));
		vertex->wA = b2TransformPoint(transformA, proxyA->vertices[vertex->indexA]);
		vertex->indexB = b2FindSupport(proxyB, b2InvRotateVector(transformB.q, d));
		vertex->wB = b2TransformPoint(transformB, proxyB->vertices[vertex->indexB]);
		vertex->w = b2Sub(vertex->wB, vertex->wA);

		// Iteration count is equated to the number of support point calls.
		++iter;
		//++b2_gjkIters;

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

	//b2_gjkMaxIters = B2_MAX(b2_gjkMaxIters, iter);

	// Prepare output
	b2ComputeSimplexWitnessPoints(&output.pointA, &output.pointB, &simplex);
	output.distance = b2Distance(output.pointA, output.pointB);
	output.iterations = iter;

	// Cache the simplex
	b2MakeSimplexCache(cache, &simplex);

	// Apply radii if requested
	if (input->useRadii)
	{
		if (output.distance < FLT_EPSILON)
		{
			// Shapes are too close to safely compute normal
			b2Vec2 p = (b2Vec2){0.5f * (output.pointA.x + output.pointB.x), 0.5f * (output.pointA.y + output.pointB.y)};
			output.pointA = p;
			output.pointB = p;
			output.distance = 0.0f;
		}
		else
		{
			// Keep closest points on perimeter even if overlapped, this way
			// the points move smoothly.
			float rA = proxyA->radius;
			float rB = proxyB->radius;
			output.distance = B2_MAX(0.0f, output.distance - rA - rB);
			b2Vec2 normal = b2Normalize(b2Sub(output.pointB, output.pointA));
			b2Vec2 offsetA = (b2Vec2){rA * normal.x, rA * normal.y};
			b2Vec2 offsetB = (b2Vec2){rB * normal.x, rB * normal.y};
			output.pointA = b2Add(output.pointA, offsetA);
			output.pointB = b2Sub(output.pointB, offsetB);
		}
	}

	return output;
}

// GJK-raycast
// Algorithm by Gino van den Bergen.
// "Smooth Mesh Contacts with GJK" in Game Physics Pearls. 2010
// TODO_ERIN this is failing when used to raycast a box
b2RayCastOutput b2ShapeCast(const b2ShapeCastInput* input)
{
	b2RayCastOutput output = {0};

	const b2DistanceProxy* proxyA = &input->proxyA;
	const b2DistanceProxy* proxyB = &input->proxyB;

	float radius = proxyA->radius + proxyB->radius;

	b2Transform xfA = input->transformA;
	b2Transform xfB = input->transformB;

	b2Vec2 r = input->translationB;
	b2Vec2 n = b2Vec2_zero;
	float lambda = 0.0f;
	float maxFraction = input->maxFraction;

	// Initial simplex
	b2Simplex simplex;
	simplex.count = 0;

	// Get simplex vertices as an array.
	b2SimplexVertex* vertices[] = {&simplex.v1, &simplex.v2, &simplex.v3};

	// Get support point in -r direction
	int32_t indexA = b2FindSupport(proxyA, b2InvRotateVector(xfA.q, b2Neg(r)));
	b2Vec2 wA = b2TransformPoint(xfA, proxyA->vertices[indexA]);
	int32_t indexB = b2FindSupport(proxyB, b2InvRotateVector(xfB.q, r));
	b2Vec2 wB = b2TransformPoint(xfB, proxyB->vertices[indexB]);
	b2Vec2 v = b2Sub(wA, wB);

	// Sigma is the target distance between proxies
	const float sigma = B2_MAX(b2_linearSlop, radius - b2_linearSlop);

	// Main iteration loop.
	const int32_t k_maxIters = 20;
	int32_t iter = 0;
	while (iter < k_maxIters && b2Length(v) > sigma)
	{
		B2_ASSERT(simplex.count < 3);

		output.iterations += 1;

		// Support in direction -v (A - B)
		indexA = b2FindSupport(proxyA, b2InvRotateVector(xfA.q, b2Neg(v)));
		wA = b2TransformPoint(xfA, proxyA->vertices[indexA]);
		indexB = b2FindSupport(proxyB, b2InvRotateVector(xfB.q, v));
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
				return output;
			}

			lambda = (vp - sigma) / vr;
			if (lambda > maxFraction)
			{
				return output;
			}

			n = (b2Vec2){-v.x, -v.y};
			simplex.count = 0;
		}

		// Reverse simplex since it works with B - A.
		// Shift by lambda * r because we want the closest point to the current clip point.
		// Note that the support point p is not shifted because we want the plane equation
		// to be formed in unshifted space.
		b2SimplexVertex* vertex = vertices[simplex.count];
		vertex->indexA = indexB;
		vertex->wA = (b2Vec2){wB.x + lambda * r.x, wB.y + lambda * r.y};
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
				b2SolveSimplex2(&simplex);
				break;

			case 3:
				b2SolveSimplex3(&simplex);
				break;

			default:
				B2_ASSERT(false);
		}

		// If we have 3 points, then the origin is in the corresponding triangle.
		if (simplex.count == 3)
		{
			// Overlap
			return output;
		}

		// Get search direction.
		v = b2ComputeSimplexClosestPoint(&simplex);

		// Iteration count is equated to the number of support point calls.
		++iter;
	}

	if (iter == 0)
	{
		// Initial overlap
		return output;
	}

	// Prepare output.
	b2Vec2 pointA, pointB;
	b2ComputeSimplexWitnessPoints(&pointB, &pointA, &simplex);

	if (b2Dot(v, v) > 0.0f)
	{
		n = b2Normalize(b2Neg(v));
	}

	float radiusA = proxyA->radius;
	output.point = (b2Vec2){pointA.x + radiusA * n.x, pointA.y + radiusA * n.y};
	output.normal = n;
	output.fraction = lambda;
	output.iterations = iter;
	output.hit = true;
	return output;
}

float b2_toiTime, b2_toiMaxTime;
int32_t b2_toiCalls, b2_toiIters, b2_toiMaxIters;
int32_t b2_toiRootIters, b2_toiMaxRootIters;

typedef enum b2SeparationType
{
	b2_pointsType,
	b2_faceAType,
	b2_faceBType
} b2SeparationType;

typedef struct b2SeparationFunction
{
	const b2DistanceProxy* proxyA;
	const b2DistanceProxy* proxyB;
	b2Sweep sweepA, sweepB;
	b2Vec2 localPoint;
	b2Vec2 axis;
	b2SeparationType type;
} b2SeparationFunction;

b2SeparationFunction b2MakeSeparationFunction(const b2DistanceCache* cache, const b2DistanceProxy* proxyA, const b2Sweep* sweepA,
											  const b2DistanceProxy* proxyB, const b2Sweep* sweepB, float t1)
{
	b2SeparationFunction f;

	f.proxyA = proxyA;
	f.proxyB = proxyB;
	int32_t count = cache->count;
	B2_ASSERT(0 < count && count < 3);

	f.sweepA = *sweepA;
	f.sweepB = *sweepB;

	b2Transform xfA = b2GetSweepTransform(sweepA, t1);
	b2Transform xfB = b2GetSweepTransform(sweepB, t1);

	if (count == 1)
	{
		f.type = b2_pointsType;
		b2Vec2 localPointA = proxyA->vertices[cache->indexA[0]];
		b2Vec2 localPointB = proxyB->vertices[cache->indexB[0]];
		b2Vec2 pointA = b2TransformPoint(xfA, localPointA);
		b2Vec2 pointB = b2TransformPoint(xfB, localPointB);
		f.axis = b2Normalize(b2Sub(pointB, pointA));
		f.localPoint = b2Vec2_zero;
		return f;
	}

	if (cache->indexA[0] == cache->indexA[1])
	{
		// Two points on B and one on A.
		f.type = b2_faceBType;
		b2Vec2 localPointB1 = proxyB->vertices[cache->indexB[0]];
		b2Vec2 localPointB2 = proxyB->vertices[cache->indexB[1]];

		f.axis = b2CrossVS(b2Sub(localPointB2, localPointB1), 1.0f);
		f.axis = b2Normalize(f.axis);
		b2Vec2 normal = b2RotateVector(xfB.q, f.axis);

		f.localPoint = (b2Vec2){0.5f * (localPointB1.x + localPointB2.x), 0.5f * (localPointB1.y + localPointB2.y)};
		b2Vec2 pointB = b2TransformPoint(xfB, f.localPoint);

		b2Vec2 localPointA = proxyA->vertices[cache->indexA[0]];
		b2Vec2 pointA = b2TransformPoint(xfA, localPointA);

		float s = b2Dot(b2Sub(pointA, pointB), normal);
		if (s < 0.0f)
		{
			f.axis = b2Neg(f.axis);
		}
		return f;
	}

	// Two points on A and one or two points on B.
	f.type = b2_faceAType;
	b2Vec2 localPointA1 = proxyA->vertices[cache->indexA[0]];
	b2Vec2 localPointA2 = proxyA->vertices[cache->indexA[1]];

	f.axis = b2CrossVS(b2Sub(localPointA2, localPointA1), 1.0f);
	f.axis = b2Normalize(f.axis);
	b2Vec2 normal = b2RotateVector(xfA.q, f.axis);

	f.localPoint = (b2Vec2){0.5f * (localPointA1.x + localPointA2.x), 0.5f * (localPointA1.y + localPointA2.y)};
	b2Vec2 pointA = b2TransformPoint(xfA, f.localPoint);

	b2Vec2 localPointB = proxyB->vertices[cache->indexB[0]];
	b2Vec2 pointB = b2TransformPoint(xfB, localPointB);

	float s = b2Dot(b2Sub(pointB, pointA), normal);
	if (s < 0.0f)
	{
		f.axis = b2Neg(f.axis);
	}
	return f;
}

float b2FindMinSeparation(const b2SeparationFunction* f, int32_t* indexA, int32_t* indexB, float t)
{
	b2Transform xfA = b2GetSweepTransform(&f->sweepA, t);
	b2Transform xfB = b2GetSweepTransform(&f->sweepB, t);

	switch (f->type)
	{
		case b2_pointsType:
		{
			b2Vec2 axisA = b2InvRotateVector(xfA.q, f->axis);
			b2Vec2 axisB = b2InvRotateVector(xfB.q, b2Neg(f->axis));

			*indexA = b2FindSupport(f->proxyA, axisA);
			*indexB = b2FindSupport(f->proxyB, axisB);

			b2Vec2 localPointA = f->proxyA->vertices[*indexA];
			b2Vec2 localPointB = f->proxyB->vertices[*indexB];

			b2Vec2 pointA = b2TransformPoint(xfA, localPointA);
			b2Vec2 pointB = b2TransformPoint(xfB, localPointB);

			float separation = b2Dot(b2Sub(pointB, pointA), f->axis);
			return separation;
		}

		case b2_faceAType:
		{
			b2Vec2 normal = b2RotateVector(xfA.q, f->axis);
			b2Vec2 pointA = b2TransformPoint(xfA, f->localPoint);

			b2Vec2 axisB = b2InvRotateVector(xfB.q, b2Neg(normal));

			*indexA = -1;
			*indexB = b2FindSupport(f->proxyB, axisB);

			b2Vec2 localPointB = f->proxyB->vertices[*indexB];
			b2Vec2 pointB = b2TransformPoint(xfB, localPointB);

			float separation = b2Dot(b2Sub(pointB, pointA), normal);
			return separation;
		}

		case b2_faceBType:
		{
			b2Vec2 normal = b2RotateVector(xfB.q, f->axis);
			b2Vec2 pointB = b2TransformPoint(xfB, f->localPoint);

			b2Vec2 axisA = b2InvRotateVector(xfA.q, b2Neg(normal));

			*indexB = -1;
			*indexA = b2FindSupport(f->proxyA, axisA);

			b2Vec2 localPointA = f->proxyA->vertices[*indexA];
			b2Vec2 pointA = b2TransformPoint(xfA, localPointA);

			float separation = b2Dot(b2Sub(pointA, pointB), normal);
			return separation;
		}

		default:
			B2_ASSERT(false);
			*indexA = -1;
			*indexB = -1;
			return 0.0f;
	}
}

//
float b2EvaluateSeparation(const b2SeparationFunction* f, int32_t indexA, int32_t indexB, float t)
{
	b2Transform xfA = b2GetSweepTransform(&f->sweepA, t);
	b2Transform xfB = b2GetSweepTransform(&f->sweepB, t);

	switch (f->type)
	{
		case b2_pointsType:
		{
			b2Vec2 localPointA = f->proxyA->vertices[indexA];
			b2Vec2 localPointB = f->proxyB->vertices[indexB];

			b2Vec2 pointA = b2TransformPoint(xfA, localPointA);
			b2Vec2 pointB = b2TransformPoint(xfB, localPointB);

			float separation = b2Dot(b2Sub(pointB, pointA), f->axis);
			return separation;
		}

		case b2_faceAType:
		{
			b2Vec2 normal = b2RotateVector(xfA.q, f->axis);
			b2Vec2 pointA = b2TransformPoint(xfA, f->localPoint);

			b2Vec2 localPointB = f->proxyB->vertices[indexB];
			b2Vec2 pointB = b2TransformPoint(xfB, localPointB);

			float separation = b2Dot(b2Sub(pointB, pointA), normal);
			return separation;
		}

		case b2_faceBType:
		{
			b2Vec2 normal = b2RotateVector(xfB.q, f->axis);
			b2Vec2 pointB = b2TransformPoint(xfB, f->localPoint);

			b2Vec2 localPointA = f->proxyA->vertices[indexA];
			b2Vec2 pointA = b2TransformPoint(xfA, localPointA);

			float separation = b2Dot(b2Sub(pointA, pointB), normal);
			return separation;
		}

		default:
			B2_ASSERT(false);
			return 0.0f;
	}
}

// CCD via the local separating axis method. This seeks progression
// by computing the largest time at which separation is maintained.
b2TOIOutput b2TimeOfImpact(const b2TOIInput* input)
{
	b2Timer timer = b2CreateTimer();
	++b2_toiCalls;

	b2TOIOutput output;
	output.state = b2_toiStateUnknown;
	output.t = input->tMax;

	const b2DistanceProxy* proxyA = &input->proxyA;
	const b2DistanceProxy* proxyB = &input->proxyB;

	b2Sweep sweepA = input->sweepA;
	b2Sweep sweepB = input->sweepB;

	// Large rotations can make the root finder fail, so normalize the sweep angles.
	float twoPi = 2.0f * b2_pi;
	{
		float d = twoPi * floorf(sweepA.a1 / twoPi);
		sweepA.a1 -= d;
		sweepA.a2 -= d;
	}
	{
		float d = twoPi * floorf(sweepB.a1 / twoPi);
		sweepB.a1 -= d;
		sweepB.a2 -= d;
	}

	float tMax = input->tMax;

	float totalRadius = proxyA->radius + proxyB->radius;
	float target = B2_MAX(b2_linearSlop, totalRadius + b2_linearSlop);
	float tolerance = 0.25f * b2_linearSlop;
	B2_ASSERT(target > tolerance);

	float t1 = 0.0f;
	const int32_t k_maxIterations = 20;
	int32_t iter = 0;

	// Prepare input for distance query.
	b2DistanceCache cache = {0};
	b2DistanceInput distanceInput;
	distanceInput.proxyA = input->proxyA;
	distanceInput.proxyB = input->proxyB;
	distanceInput.useRadii = false;

	// The outer loop progressively attempts to compute new separating axes.
	// This loop terminates when an axis is repeated (no progress is made).
	for (;;)
	{
		b2Transform xfA = b2GetSweepTransform(&sweepA, t1);
		b2Transform xfB = b2GetSweepTransform(&sweepB, t1);

		// Get the distance between shapes. We can also use the results
		// to get a separating axis.
		distanceInput.transformA = xfA;
		distanceInput.transformB = xfB;
		b2DistanceOutput distanceOutput = b2ShapeDistance(&cache, &distanceInput);

		// If the shapes are overlapped, we give up on continuous collision.
		if (distanceOutput.distance <= 0.0f)
		{
			// Failure!
			output.state = b2_toiStateOverlapped;
			output.t = 0.0f;
			break;
		}

		if (distanceOutput.distance < target + tolerance)
		{
			// Victory!
			output.state = b2_toiStateHit;
			output.t = t1;
			break;
		}

		// Initialize the separating axis.
		b2SeparationFunction fcn = b2MakeSeparationFunction(&cache, proxyA, &sweepA, proxyB, &sweepB, t1);
#if 0
		// Dump the curve seen by the root finder
		{
			const int32_t N = 100;
			float dx = 1.0f / N;
			float xs[N + 1];
			float fs[N + 1];

			float x = 0.0f;

			for (int32_t i = 0; i <= N; ++i)
			{
				sweepA.GetTransform(&xfA, x);
				sweepB.GetTransform(&xfB, x);
				float f = fcn.Evaluate(xfA, xfB) - target;

				printf("%g %g\n", x, f);

				xs[i] = x;
				fs[i] = f;

				x += dx;
			}
		}
#endif

		// Compute the TOI on the separating axis. We do this by successively
		// resolving the deepest point. This loop is bounded by the number of vertices.
		bool done = false;
		float t2 = tMax;
		int32_t pushBackIter = 0;
		for (;;)
		{
			// Find the deepest point at t2. Store the witness point indices.
			int32_t indexA, indexB;
			float s2 = b2FindMinSeparation(&fcn, &indexA, &indexB, t2);

			// Is the final configuration separated?
			if (s2 > target + tolerance)
			{
				// Victory!
				output.state = b2_toiStateSeparated;
				output.t = tMax;
				done = true;
				break;
			}

			// Has the separation reached tolerance?
			if (s2 > target - tolerance)
			{
				// Advance the sweeps
				t1 = t2;
				break;
			}

			// Compute the initial separation of the witness points.
			float s1 = b2EvaluateSeparation(&fcn, indexA, indexB, t1);

			// Check for initial overlap. This might happen if the root finder
			// runs out of iterations.
			if (s1 < target - tolerance)
			{
				output.state = b2_toiStateFailed;
				output.t = t1;
				done = true;
				break;
			}

			// Check for touching
			if (s1 <= target + tolerance)
			{
				// Victory! t1 should hold the TOI (could be 0.0).
				output.state = b2_toiStateHit;
				output.t = t1;
				done = true;
				break;
			}

			// Compute 1D root of: f(x) - target = 0
			int32_t rootIterCount = 0;
			float a1 = t1, a2 = t2;
			for (;;)
			{
				// Use a mix of the secant rule and bisection.
				float t;
				if (rootIterCount & 1)
				{
					// Secant rule to improve convergence.
					t = a1 + (target - s1) * (a2 - a1) / (s2 - s1);
				}
				else
				{
					// Bisection to guarantee progress.
					t = 0.5f * (a1 + a2);
				}

				++rootIterCount;
				++b2_toiRootIters;

				float s = b2EvaluateSeparation(&fcn, indexA, indexB, t);

				if (B2_ABS(s - target) < tolerance)
				{
					// t2 holds a tentative value for t1
					t2 = t;
					break;
				}

				// Ensure we continue to bracket the root.
				if (s > target)
				{
					a1 = t;
					s1 = s;
				}
				else
				{
					a2 = t;
					s2 = s;
				}

				if (rootIterCount == 50)
				{
					break;
				}
			}

			b2_toiMaxRootIters = B2_MAX(b2_toiMaxRootIters, rootIterCount);

			++pushBackIter;

			if (pushBackIter == b2_maxPolygonVertices)
			{
				break;
			}
		}

		++iter;
		++b2_toiIters;

		if (done)
		{
			break;
		}

		if (iter == k_maxIterations)
		{
			// Root finder got stuck. Semi-victory.
			output.state = b2_toiStateFailed;
			output.t = t1;
			break;
		}
	}

	b2_toiMaxIters = B2_MAX(b2_toiMaxIters, iter);

	float time = b2GetMilliseconds(&timer);
	b2_toiMaxTime = B2_MAX(b2_toiMaxTime, time);
	b2_toiTime += time;

	return output;
}
