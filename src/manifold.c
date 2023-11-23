// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "box2d/manifold.h"

#include "core.h"

#include "box2d/distance.h"
#include "box2d/geometry.h"
#include "box2d/math.h"

#include <float.h>
#include <stdatomic.h>
#include <string.h>

#if 0
b2WorldManifold b2ComputeWorldManifold(const b2Manifold* manifold, b2Transform xfA, float radiusA, b2Transform xfB,
									   float radiusB)
{
	b2WorldManifold wm = {{0.0f, 0.0f}, {{0.0f, 0.0f}, {0.0f, 0.0f}}, {0.0f, 0.0f}};

	if (manifold->pointCount == 0)
	{
		return wm;
	}

	switch (manifold->type)
	{
		case b2_manifoldCircles:
		{
			wm.normal = (b2Vec2){1.0f, 0.0f};
			b2Vec2 pointA = b2TransformPoint(xfA, manifold->localPoint);
			b2Vec2 pointB = b2TransformPoint(xfB, manifold->points[0].localPoint);
			if (b2DistanceSquared(pointA, pointB) > FLT_EPSILON * FLT_EPSILON)
			{
				wm.normal = b2Normalize(b2Sub(pointB, pointA));
			}

			b2Vec2 cA = b2MulAdd(pointA, radiusA, wm.normal);
			b2Vec2 cB = b2MulAdd(pointB, -radiusB, wm.normal);
			wm.points[0] = b2MulSV(0.5f, b2Add(cA, cB));
			wm.separations[0] = b2Dot(b2Sub(cB, cA), wm.normal);
		}
		break;

		case b2_manifoldFaceA:
		{
			wm.normal = b2RotateVector(xfA.q, manifold->localNormal);
			b2Vec2 planePoint = b2TransformPoint(xfA, manifold->localPoint);

			for (int32_t i = 0; i < manifold->pointCount; ++i)
			{
				b2Vec2 clipPoint = b2TransformPoint(xfB, manifold->points[i].localPoint);
				b2Vec2 cA = b2MulAdd(clipPoint, (radiusA - b2Dot(b2Sub(clipPoint, planePoint), wm.normal)), wm.normal);
				b2Vec2 cB = b2MulAdd(clipPoint, -radiusB, wm.normal);
				wm.points[i] = b2MulSV(0.5f, b2Add(cA, cB));
				wm.separations[i] = b2Dot(b2Sub(cB, cA), wm.normal);
			}
		}
		break;

		case b2_manifoldFaceB:
		{
			wm.normal = b2RotateVector(xfB.q, manifold->localNormal);
			b2Vec2 planePoint = b2TransformPoint(xfB, manifold->localPoint);

			for (int32_t i = 0; i < manifold->pointCount; ++i)
			{
				b2Vec2 clipPoint = b2TransformPoint(xfA, manifold->points[i].localPoint);
				b2Vec2 cB = b2MulAdd(clipPoint, (radiusB - b2Dot(b2Sub(clipPoint, planePoint), wm.normal)), wm.normal);
				b2Vec2 cA = b2MulAdd(clipPoint, -radiusA, wm.normal);
				wm.points[i] = b2MulSV(0.5f, b2Add(cA, cB));
				wm.separations[i] = b2Dot(b2Sub(cA, cB), wm.normal);
			}

			// Ensure normal points from A to B.
			wm.normal = b2Neg(wm.normal);
		}
		break;
	}

	return wm;
}
#endif

b2Manifold b2CollideCircles(const b2Circle* circleA, b2Transform xfA, const b2Circle* circleB, b2Transform xfB)
{
	b2Manifold manifold = {0};

	b2Vec2 pointA = b2TransformPoint(xfA, circleA->point);
	b2Vec2 pointB = b2TransformPoint(xfB, circleB->point);

	float distance;
	b2Vec2 normal = b2GetLengthAndNormalize(&distance, b2Sub(pointB, pointA));

	float radiusA = circleA->radius;
	float radiusB = circleB->radius;

	float separation = distance - radiusA - radiusB;
	if (separation > b2_speculativeDistance)
	{
		return manifold;
	}

	b2Vec2 cA = b2MulAdd(pointA, radiusA, normal);
	b2Vec2 cB = b2MulAdd(pointB, -radiusB, normal);
	manifold.normal = normal;
	manifold.points[0].point = b2Lerp(cA, cB, 0.5f);
	manifold.points[0].separation = separation;
	manifold.points[0].id = 0;
	manifold.pointCount = 1;
	return manifold;
}

/// Compute the collision manifold between a capsule and circle
b2Manifold b2CollideCapsuleAndCircle(const b2Capsule* capsuleA, b2Transform xfA, const b2Circle* circleB, b2Transform xfB)
{
	b2Manifold manifold = {0};

	// Compute circle position in the frame of the capsule.
	b2Vec2 pB = b2InvTransformPoint(xfA, b2TransformPoint(xfB, circleB->point));

	// Compute closest point
	b2Vec2 p1 = capsuleA->point1;
	b2Vec2 p2 = capsuleA->point2;

	b2Vec2 e = b2Sub(p2, p1);

	// dot(p - pA, e) = 0
	// dot(p - (p1 + s1 * e), e) = 0
	// s1 = dot(p - p1, e)
	b2Vec2 pA;
	float s1 = b2Dot(b2Sub(pB, p1), e);
	float s2 = b2Dot(b2Sub(p2, pB), e);
	if (s1 < 0.0f)
	{
		// p1 region
		pA = p1;
	}
	else if (s2 < 0.0f)
	{
		// p2 region
		pA = p2;
	}
	else
	{
		// circle colliding with segment interior
		float s = s1 / b2Dot(e, e);
		pA = b2MulAdd(p1, s, e);
	}

	float distance;
	b2Vec2 normal = b2GetLengthAndNormalize(&distance, b2Sub(pB, pA));

	float radiusA = capsuleA->radius;
	float radiusB = circleB->radius;
	float separation = distance - radiusA - radiusB;
	if (separation > b2_speculativeDistance)
	{
		return manifold;
	}

	b2Vec2 cA = b2MulAdd(pA, radiusA, normal);
	b2Vec2 cB = b2MulAdd(pB, -radiusB, normal);
	manifold.normal = b2RotateVector(xfA.q, normal);
	manifold.points[0].point = b2TransformPoint(xfA, b2Lerp(cA, cB, 0.5f));
	manifold.points[0].separation = separation;
	manifold.points[0].id = 0;
	manifold.pointCount = 1;
	return manifold;
}

b2Manifold b2CollidePolygonAndCircle(const b2Polygon* polygonA, b2Transform xfA, const b2Circle* circleB, b2Transform xfB)
{
	b2Manifold manifold = {0};

	// Compute circle position in the frame of the polygon.
	b2Vec2 c = b2InvTransformPoint(xfA, b2TransformPoint(xfB, circleB->point));
	float radiusA = polygonA->radius;
	float radiusB = circleB->radius;
	float radius = radiusA + radiusB;

	// Find the min separating edge.
	int32_t normalIndex = 0;
	float separation = -FLT_MAX;
	int32_t vertexCount = polygonA->count;
	const b2Vec2* vertices = polygonA->vertices;
	const b2Vec2* normals = polygonA->normals;

	for (int32_t i = 0; i < vertexCount; ++i)
	{
		float s = b2Dot(normals[i], b2Sub(c, vertices[i]));
		if (s > separation)
		{
			separation = s;
			normalIndex = i;
		}
	}

	if (separation > radius + b2_speculativeDistance)
	{
		return manifold;
	}

	// Vertices of the reference edge.
	int32_t vertIndex1 = normalIndex;
	int32_t vertIndex2 = vertIndex1 + 1 < vertexCount ? vertIndex1 + 1 : 0;
	b2Vec2 v1 = vertices[vertIndex1];
	b2Vec2 v2 = vertices[vertIndex2];

	// Compute barycentric coordinates
	float u1 = b2Dot(b2Sub(c, v1), b2Sub(v2, v1));
	float u2 = b2Dot(b2Sub(c, v2), b2Sub(v1, v2));

	if (u1 < 0.0f && separation > FLT_EPSILON)
	{
		// Circle center is closest to v1 and safely outside the polygon
		b2Vec2 normal = b2Normalize(b2Sub(c, v1));
		separation = b2Dot(b2Sub(c, v1), normal);
		if (separation > radius + b2_speculativeDistance)
		{
			return manifold;
		}

		b2Vec2 cA = b2MulAdd(v1, radiusA, normal);
		b2Vec2 cB = b2MulSub(c, radiusB, normal);
		manifold.normal = b2RotateVector(xfA.q, normal);
		manifold.points[0].point = b2TransformPoint(xfA, b2Lerp(cA, cB, 0.5f));
		manifold.points[0].separation = separation;
		manifold.points[0].id = 0;
		manifold.pointCount = 1;
	}
	else if (u2 < 0.0f && separation > FLT_EPSILON)
	{
		// Circle center is closest to v2 and safely outside the polygon
		b2Vec2 normal = b2Normalize(b2Sub(c, v2));
		separation = b2Dot(b2Sub(c, v2), normal);
		if (separation > radius + b2_speculativeDistance)
		{
			return manifold;
		}

		b2Vec2 cA = b2MulAdd(v2, radiusA, normal);
		b2Vec2 cB = b2MulSub(c, radiusB, normal);
		manifold.normal = b2RotateVector(xfA.q, normal);
		manifold.points[0].point = b2TransformPoint(xfA, b2Lerp(cA, cB, 0.5f));
		manifold.points[0].separation = b2Dot(b2Sub(cB, cA), normal);
		manifold.points[0].id = 0;
		manifold.pointCount = 1;
	}
	else
	{
		// Circle center is between v1 and v2. Center may be inside polygon
		b2Vec2 normal = normals[normalIndex];
		manifold.normal = b2RotateVector(xfA.q, normal);

		// cA is the projection of the circle center onto to the reference edge
		b2Vec2 cA = b2MulAdd(c, radiusA - b2Dot(b2Sub(c, v1), normal), normal);

		// cB is the deepest point on the circle with respect to the reference edge
		b2Vec2 cB = b2MulSub(c, radiusB, normal);

		// The contact point is the midpoint in world space
		manifold.points[0].point = b2TransformPoint(xfA, b2Lerp(cA, cB, 0.5f));
		manifold.points[0].separation = separation - radius;
		manifold.points[0].id = 0;
		manifold.pointCount = 1;
	}

	return manifold;
}

b2Manifold b2CollideCapsules(const b2Capsule* capsuleA, b2Transform xfA, const b2Capsule* capsuleB, b2Transform xfB,
							 b2DistanceCache* cache)
{
	b2Polygon polyA = b2MakeCapsule(capsuleA->point1, capsuleA->point2, capsuleA->radius);
	b2Polygon polyB = b2MakeCapsule(capsuleB->point1, capsuleB->point2, capsuleB->radius);
	return b2CollidePolygons(&polyA, xfA, &polyB, xfB, cache);
}

b2Manifold b2CollideSegmentAndCapsule(const b2Segment* segmentA, b2Transform xfA, const b2Capsule* capsuleB, b2Transform xfB,
									  b2DistanceCache* cache)
{
	b2Polygon polyA = b2MakeCapsule(segmentA->point1, segmentA->point2, 0.0f);
	b2Polygon polyB = b2MakeCapsule(capsuleB->point1, capsuleB->point2, capsuleB->radius);
	return b2CollidePolygons(&polyA, xfA, &polyB, xfB, cache);
}

b2Manifold b2CollidePolygonAndCapsule(const b2Polygon* polygonA, b2Transform xfA, const b2Capsule* capsuleB, b2Transform xfB,
									  b2DistanceCache* cache)
{
	b2Polygon polyB = b2MakeCapsule(capsuleB->point1, capsuleB->point2, capsuleB->radius);
	return b2CollidePolygons(polygonA, xfA, &polyB, xfB, cache);
}

// Polygon clipper used by GJK and SAT to compute contact points when there are potentially two contact points.
static b2Manifold b2ClipPolygons(const b2Polygon* polyA, b2Transform xfA, const b2Polygon* polyB, b2Transform xfB, int32_t edgeA,
								 int32_t edgeB, bool flip)
{
	b2Manifold manifold = {0};

	// reference polygon
	const b2Polygon* poly1;
	int32_t i11, i12;

	// incident polygon
	const b2Polygon* poly2;
	int32_t i21, i22;

	b2Transform xf;

	if (flip)
	{
		poly1 = polyB;
		poly2 = polyA;
		// take points in frame A into frame B
		xf = b2InvMulTransforms(xfB, xfA);
		i11 = edgeB;
		i12 = edgeB + 1 < polyB->count ? edgeB + 1 : 0;
		i21 = edgeA;
		i22 = edgeA + 1 < polyA->count ? edgeA + 1 : 0;
	}
	else
	{
		poly1 = polyA;
		poly2 = polyB;
		// take points in frame B into frame A
		xf = b2InvMulTransforms(xfA, xfB);
		i11 = edgeA;
		i12 = edgeA + 1 < polyA->count ? edgeA + 1 : 0;
		i21 = edgeB;
		i22 = edgeB + 1 < polyB->count ? edgeB + 1 : 0;
	}

	b2Vec2 normal = poly1->normals[i11];

	// Reference edge vertices
	b2Vec2 v11 = poly1->vertices[i11];
	b2Vec2 v12 = poly1->vertices[i12];

	// Incident edge vertices
	b2Vec2 v21 = b2TransformPoint(xf, poly2->vertices[i21]);
	b2Vec2 v22 = b2TransformPoint(xf, poly2->vertices[i22]);

	b2Vec2 tangent = b2CrossSV(1.0f, normal);

	float lower1 = 0.0f;
	float upper1 = b2Dot(b2Sub(v12, v11), tangent);

	// Incident edge points opposite of tangent due to CCW winding
	float upper2 = b2Dot(b2Sub(v21, v11), tangent);
	float lower2 = b2Dot(b2Sub(v22, v11), tangent);

	// This check can fail slightly due to mismatch with GJK code.
	// Perhaps fallback to a single point here? Otherwise we get two coincident points.
	// if (upper2 < lower1 || upper1 < lower2)
	//{
	//	// numeric failure
	//	B2_ASSERT(false);
	//	return manifold;
	//}

	b2Vec2 vLower;
	if (lower2 < lower1 && upper2 - lower2 > FLT_EPSILON)
	{
		vLower = b2Lerp(v22, v21, (lower1 - lower2) / (upper2 - lower2));
	}
	else
	{
		vLower = v22;
	}

	b2Vec2 vUpper;
	if (upper2 > upper1 && upper2 - lower2 > FLT_EPSILON)
	{
		vUpper = b2Lerp(v22, v21, (upper1 - lower2) / (upper2 - lower2));
	}
	else
	{
		vUpper = v21;
	}

	// TODO_ERIN vLower can be very close to vUpper, reduce to one point?

	float separationLower = b2Dot(b2Sub(vLower, v11), normal);
	float separationUpper = b2Dot(b2Sub(vUpper, v11), normal);

	float r1 = poly1->radius;
	float r2 = poly2->radius;

	// Put contact points at midpoint, accounting for radii
	vLower = b2MulAdd(vLower, 0.5f * (r1 - r2 - separationLower), normal);
	vUpper = b2MulAdd(vUpper, 0.5f * (r1 - r2 - separationUpper), normal);

	float radius = r1 + r2;

	if (flip == false)
	{
		manifold.normal = b2RotateVector(xfA.q, normal);
		b2ManifoldPoint* cp = manifold.points + 0;

		{
			cp->point = b2TransformPoint(xfA, vLower);
			cp->separation = separationLower - radius;
			cp->id = B2_MAKE_ID(i11, i22);
			manifold.pointCount += 1;
			cp += 1;
		}

		{
			cp->point = b2TransformPoint(xfA, vUpper);
			cp->separation = separationUpper - radius;
			cp->id = B2_MAKE_ID(i12, i21);
			manifold.pointCount += 1;
		}
	}
	else
	{
		manifold.normal = b2RotateVector(xfB.q, b2Neg(normal));
		b2ManifoldPoint* cp = manifold.points + 0;

		{
			cp->point = b2TransformPoint(xfB, vUpper);
			cp->separation = separationUpper - radius;
			cp->id = B2_MAKE_ID(i21, i12);
			manifold.pointCount += 1;
			cp += 1;
		}

		{
			cp->point = b2TransformPoint(xfB, vLower);
			cp->separation = separationLower - radius;
			cp->id = B2_MAKE_ID(i22, i11);
			manifold.pointCount += 1;
		}
	}

	return manifold;
}

// Find the max separation between poly1 and poly2 using edge normals from poly1.
static float b2FindMaxSeparation(int32_t* edgeIndex, const b2Polygon* poly1, b2Transform xf1, const b2Polygon* poly2,
								 b2Transform xf2)
{
	int32_t count1 = poly1->count;
	int32_t count2 = poly2->count;
	const b2Vec2* n1s = poly1->normals;
	const b2Vec2* v1s = poly1->vertices;
	const b2Vec2* v2s = poly2->vertices;
	b2Transform xf = b2InvMulTransforms(xf2, xf1);

	int32_t bestIndex = 0;
	float maxSeparation = -FLT_MAX;
	for (int32_t i = 0; i < count1; ++i)
	{
		// Get poly1 normal in frame2.
		b2Vec2 n = b2RotateVector(xf.q, n1s[i]);
		b2Vec2 v1 = b2TransformPoint(xf, v1s[i]);

		// Find deepest point for normal i.
		float si = FLT_MAX;
		for (int32_t j = 0; j < count2; ++j)
		{
			float sij = b2Dot(n, b2Sub(v2s[j], v1));
			if (sij < si)
			{
				si = sij;
			}
		}

		if (si > maxSeparation)
		{
			maxSeparation = si;
			bestIndex = i;
		}
	}

	*edgeIndex = bestIndex;
	return maxSeparation;
}

// This function assumes there is overlap
static b2Manifold b2PolygonSAT(const b2Polygon* polyA, b2Transform xfA, const b2Polygon* polyB, b2Transform xfB)
{
	int32_t edgeA = 0;
	float separationA = b2FindMaxSeparation(&edgeA, polyA, xfA, polyB, xfB);

	int32_t edgeB = 0;
	float separationB = b2FindMaxSeparation(&edgeB, polyB, xfB, polyA, xfA);

	bool flip;

	if (separationB > separationA)
	{
		flip = true;
		b2Vec2 normal = b2RotateVector(xfB.q, polyB->normals[edgeB]);
		b2Vec2 searchDirection = b2InvRotateVector(xfA.q, normal);

		// Find the incident edge on polyA
		int32_t count = polyA->count;
		const b2Vec2* normals = polyA->normals;
		edgeA = 0;
		float minDot = FLT_MAX;
		for (int32_t i = 0; i < count; ++i)
		{
			float dot = b2Dot(searchDirection, normals[i]);
			if (dot < minDot)
			{
				minDot = dot;
				edgeA = i;
			}
		}
	}
	else
	{
		flip = false;
		b2Vec2 normal = b2RotateVector(xfA.q, polyA->normals[edgeA]);
		b2Vec2 searchDirection = b2InvRotateVector(xfB.q, normal);

		// Find the incident edge on polyB
		int32_t count = polyB->count;
		const b2Vec2* normals = polyB->normals;
		edgeB = 0;
		float minDot = FLT_MAX;
		for (int32_t i = 0; i < count; ++i)
		{
			float dot = b2Dot(searchDirection, normals[i]);
			if (dot < minDot)
			{
				minDot = dot;
				edgeB = i;
			}
		}
	}

	return b2ClipPolygons(polyA, xfA, polyB, xfB, edgeA, edgeB, flip);
}

// Due to speculation, every polygon is rounded
// Algorithm:
// compute distance
// if distance <= 0.1f * b2_linearSlop
//   SAT
// else
//   find closest features from GJK
//   expect 2-1 or 1-1 or 1-2 features
//   if 2-1 or 1-2
//     clip
//   else
//     vertex-vertex
//   end
// end
b2Manifold b2CollidePolygons(const b2Polygon* polyA, b2Transform xfA, const b2Polygon* polyB, b2Transform xfB,
							 b2DistanceCache* cache)
{
	b2Manifold manifold = {0};
	float radius = polyA->radius + polyB->radius;

	b2DistanceInput input;
	input.proxyA = b2MakeProxy(polyA->vertices, polyA->count, 0.0f);
	input.proxyB = b2MakeProxy(polyB->vertices, polyB->count, 0.0f);
	input.transformA = xfA;
	input.transformB = xfB;
	input.useRadii = false;

	b2DistanceOutput output = b2ShapeDistance(cache, &input);

	if (output.distance > radius + b2_speculativeDistance)
	{
		return manifold;
	}

	if (output.distance < 0.1f * b2_linearSlop)
	{
		// distance is small or zero, fallback to SAT
		return b2PolygonSAT(polyA, xfA, polyB, xfB);
	}

	if (cache->count == 1)
	{
		// vertex-vertex collision
		b2Vec2 pA = output.pointA;
		b2Vec2 pB = output.pointB;

		float distance = output.distance;
		manifold.normal = b2Normalize(b2Sub(pB, pA));
		b2ManifoldPoint* cp = manifold.points + 0;
		cp->point = b2MulAdd(pB, 0.5f * (polyA->radius - polyB->radius - distance), manifold.normal);
		cp->separation = distance - radius;
		cp->id = B2_MAKE_ID(cache->indexA[0], cache->indexB[0]);
		manifold.pointCount = 1;
		return manifold;
	}

	// vertex-edge collision
	B2_ASSERT(cache->count == 2);
	bool flip;
	int32_t countA = polyA->count;
	int32_t countB = polyB->count;
	int32_t edgeA, edgeB;

	int32_t a1 = cache->indexA[0];
	int32_t a2 = cache->indexA[1];
	int32_t b1 = cache->indexB[0];
	int32_t b2 = cache->indexB[1];

	if (a1 == a2)
	{
		// 1 point on A, expect 2 points on B
		B2_ASSERT(b1 != b2);

		// Find reference edge that most aligns with vector between closest points.
		// This works for capsules and polygons
		b2Vec2 axis = b2InvRotateVector(xfB.q, b2Sub(output.pointA, output.pointB));
		float dot1 = b2Dot(axis, polyB->normals[b1]);
		float dot2 = b2Dot(axis, polyB->normals[b2]);
		edgeB = dot1 > dot2 ? b1 : b2;

		flip = true;

		// Get the normal of the reference edge in polyA's frame.
		axis = b2InvRotateVector(xfA.q, b2RotateVector(xfB.q, polyB->normals[edgeB]));

		// Find the incident edge on polyA
		// Limit search to edges adjacent to closest vertex on A
		int32_t edgeA1 = a1;
		int32_t edgeA2 = edgeA1 == 0 ? countA - 1 : edgeA1 - 1;
		dot1 = b2Dot(axis, polyA->normals[edgeA1]);
		dot2 = b2Dot(axis, polyA->normals[edgeA2]);
		edgeA = dot1 < dot2 ? edgeA1 : edgeA2;
	}
	else
	{
		// Find reference edge that most aligns with vector between closest points.
		// This works for capsules and polygons
		b2Vec2 axis = b2InvRotateVector(xfA.q, b2Sub(output.pointB, output.pointA));
		float dot1 = b2Dot(axis, polyA->normals[a1]);
		float dot2 = b2Dot(axis, polyA->normals[a2]);
		edgeA = dot1 > dot2 ? a1 : a2;

		flip = false;

		// Get the normal of the reference edge in polyB's frame.
		axis = b2InvRotateVector(xfB.q, b2RotateVector(xfA.q, polyA->normals[edgeA]));

		// Find the incident edge on polyB
		// Limit search to edges adjacent to closest vertex
		int32_t edgeB1 = b1;
		int32_t edgeB2 = edgeB1 == 0 ? countB - 1 : edgeB1 - 1;
		dot1 = b2Dot(axis, polyB->normals[edgeB1]);
		dot2 = b2Dot(axis, polyB->normals[edgeB2]);
		edgeB = dot1 < dot2 ? edgeB1 : edgeB2;
	}

	return b2ClipPolygons(polyA, xfA, polyB, xfB, edgeA, edgeB, flip);
}

b2Manifold b2CollideSegmentAndCircle(const b2Segment* segmentA, b2Transform xfA, const b2Circle* circleB, b2Transform xfB)
{
	b2Capsule capsuleA = {segmentA->point1, segmentA->point2, 0.0f};
	return b2CollideCapsuleAndCircle(&capsuleA, xfA, circleB, xfB);
}

b2Manifold b2CollideSegmentAndPolygon(const b2Segment* segmentA, b2Transform xfA, const b2Polygon* polygonB, b2Transform xfB,
									  b2DistanceCache* cache)
{
	b2Polygon polygonA = b2MakeCapsule(segmentA->point1, segmentA->point2, 0.0f);
	return b2CollidePolygons(&polygonA, xfA, polygonB, xfB, cache);
}

b2Manifold b2CollideSmoothSegmentAndCircle(const b2SmoothSegment* segmentA, b2Transform xfA, const b2Circle* circleB,
										   b2Transform xfB)
{
	b2Manifold manifold = {0};

	// Compute circle in frame of segment
	b2Vec2 pB = b2InvTransformPoint(xfA, b2TransformPoint(xfB, circleB->point));

	b2Vec2 p1 = segmentA->segment.point1;
	b2Vec2 p2 = segmentA->segment.point2;
	b2Vec2 e = b2Sub(p2, p1);

	// Normal points to the right
	float offset = b2Dot(b2RightPerp(e), b2Sub(pB, p1));
	if (offset < 0.0f)
	{
		// collision is one-sided
		return manifold;
	}

	// Barycentric coordinates
	float u = b2Dot(e, b2Sub(p2, pB));
	float v = b2Dot(e, b2Sub(pB, p1));

	b2Vec2 pA;

	if (v <= 0.0f)
	{
		// Behind point1?
		// Is pB in the voronoi region of the previous edge?
		b2Vec2 prevEdge = b2Sub(p1, segmentA->ghost1);
		float uPrev = b2Dot(prevEdge, b2Sub(pB, p1));
		if (uPrev <= 0.0f)
		{
			return manifold;
		}

		pA = p1;
	}
	else if (u <= 0.0f)
	{
		// Ahead of point2?
		b2Vec2 nextEdge = b2Sub(segmentA->ghost2, p2);
		float vNext = b2Dot(nextEdge, b2Sub(pB, p2));

		// Is pB in the voronoi region of the next edge?
		if (vNext > 0.0f)
		{
			return manifold;
		}

		pA = p2;
	}
	else
	{
		float ee = b2Dot(e, e);
		pA = (b2Vec2){u * p1.x + v * p2.x, u * p1.y + v * p2.y};
		pA = ee > 0.0f ? b2MulSV(1.0f / ee, pA) : p1;
	}

	float distance;
	b2Vec2 normal = b2GetLengthAndNormalize(&distance, b2Sub(pB, pA));

	float radius = circleB->radius;
	float separation = distance - radius;
	if (separation > b2_speculativeDistance)
	{
		return manifold;
	}

	b2Vec2 cA = pA;
	b2Vec2 cB = b2MulAdd(pB, -radius, normal);
	manifold.normal = b2RotateVector(xfA.q, normal);
	manifold.points[0].point = b2TransformPoint(xfA, b2Lerp(cA, cB, 0.5f));
	manifold.points[0].separation = separation;
	manifold.points[0].id = 0;
	manifold.pointCount = 1;
	return manifold;
}

b2Manifold b2CollideSmoothSegmentAndCapsule(const b2SmoothSegment* segmentA, b2Transform xfA, const b2Capsule* capsuleB,
											b2Transform xfB, b2DistanceCache* cache)
{
	b2Polygon polyB = b2MakeCapsule(capsuleB->point1, capsuleB->point2, capsuleB->radius);
	return b2CollideSmoothSegmentAndPolygon(segmentA, xfA, &polyB, xfB, cache);
}

static b2Manifold b2ClipSegments(b2Vec2 a1, b2Vec2 a2, b2Vec2 b1, b2Vec2 b2, b2Vec2 normal, float ra, float rb, uint16_t id1,
								 uint16_t id2)
{
	b2Manifold manifold = {0};

	b2Vec2 tangent = b2LeftPerp(normal);

	// Barycentric coordinates of each point relative to a1 along tangent
	float lower1 = 0.0f;
	float upper1 = b2Dot(b2Sub(a2, a1), tangent);

	// Incident edge points opposite of tangent due to CCW winding
	float upper2 = b2Dot(b2Sub(b1, a1), tangent);
	float lower2 = b2Dot(b2Sub(b2, a1), tangent);

	// Do segments overlap?
	if (upper2 < lower1 || upper1 < lower2)
	{
		return manifold;
	}

	b2Vec2 vLower;
	if (lower2 < lower1 && upper2 - lower2 > FLT_EPSILON)
	{
		vLower = b2Lerp(b2, b1, (lower1 - lower2) / (upper2 - lower2));
	}
	else
	{
		vLower = b2;
	}

	b2Vec2 vUpper;
	if (upper2 > upper1 && upper2 - lower2 > FLT_EPSILON)
	{
		vUpper = b2Lerp(b2, b1, (upper1 - lower2) / (upper2 - lower2));
	}
	else
	{
		vUpper = b1;
	}

	// TODO_ERIN vLower can be very close to vUpper, reduce to one point?

	float separationLower = b2Dot(b2Sub(vLower, a1), normal);
	float separationUpper = b2Dot(b2Sub(vUpper, a1), normal);

	// Put contact points at midpoint, accounting for radii
	vLower = b2MulAdd(vLower, 0.5f * (ra - rb - separationLower), normal);
	vUpper = b2MulAdd(vUpper, 0.5f * (ra - rb - separationUpper), normal);

	float radius = ra + rb;

	manifold.normal = normal;
	{
		b2ManifoldPoint* cp = manifold.points + 0;
		cp->point = vLower;
		cp->separation = separationLower - radius;
		cp->id = id1;
	}

	{
		b2ManifoldPoint* cp = manifold.points + 1;
		cp->point = vUpper;
		cp->separation = separationUpper - radius;
		cp->id = id2;
	}

	manifold.pointCount = 2;

	return manifold;
}

enum b2NormalType
{
	// This means the normal points in a direction that is non-smooth relative to a convex vertex and should be skipped
	b2_normalSkip,

	// This means the normal points in a direction that is smooth relative to a convex vertex and should be used for collision
	b2_normalAdmit,

	// This means the normal is in a region of a concave vertex and should be snapped to the smooth segment normal
	b2_normalSnap
};

struct b2SmoothSegmentParams
{
	b2Vec2 edge1;
	b2Vec2 normal0;
	b2Vec2 normal2;
	bool convex1;
	bool convex2;
};

// Evaluate Guass map
// See https://box2d.org/posts/2020/06/ghost-collisions/
static enum b2NormalType b2ClassifyNormal(struct b2SmoothSegmentParams params, b2Vec2 normal)
{
	const float sinTol = 0.01f;

	if (b2Dot(normal, params.edge1) <= 0.0f)
	{
		// Normal points towards the segment tail
		if (params.convex1)
		{
			if (b2Cross(normal, params.normal0) > sinTol)
			{
				return b2_normalSkip;
			}

			return b2_normalAdmit;
		}
		else
		{
			return b2_normalSnap;
		}
	}
	else
	{
		// Normal points towards segment head
		if (params.convex2)
		{
			if (b2Cross(params.normal2, normal) > sinTol)
			{
				return b2_normalSkip;
			}

			return b2_normalAdmit;
		}
		else
		{
			return b2_normalSnap;
		}
	}
}

b2Manifold b2CollideSmoothSegmentAndPolygon(const b2SmoothSegment* segmentA, b2Transform xfA, const b2Polygon* polygonB,
											b2Transform xfB, b2DistanceCache* cache)
{
	b2Manifold manifold = {0};

	b2Transform xf = b2InvMulTransforms(xfA, xfB);

	b2Vec2 centroidB = b2TransformPoint(xf, polygonB->centroid);
	float radiusB = polygonB->radius;

	b2Vec2 p1 = segmentA->segment.point1;
	b2Vec2 p2 = segmentA->segment.point2;

	b2Vec2 edge1 = b2Normalize(b2Sub(p2, p1));

	struct b2SmoothSegmentParams smoothParams;
	smoothParams.edge1 = edge1;

	const float convexTol = 0.01f;
	b2Vec2 edge0 = b2Normalize(b2Sub(p1, segmentA->ghost1));
	smoothParams.normal0 = b2RightPerp(edge0);
	smoothParams.convex1 = b2Cross(edge0, edge1) >= convexTol;

	b2Vec2 edge2 = b2Normalize(b2Sub(segmentA->ghost2, p2));
	smoothParams.normal2 = b2RightPerp(edge2);
	smoothParams.convex2 = b2Cross(edge1, edge2) >= convexTol;

	// Normal points to the right
	b2Vec2 normal1 = b2RightPerp(edge1);
	bool behind1 = b2Dot(normal1, b2Sub(centroidB, p1)) < 0.0f;
	bool behind0 = true;
	bool behind2 = true;
	if (smoothParams.convex1)
	{
		behind0 = b2Dot(smoothParams.normal0, b2Sub(centroidB, p1)) < 0.0f;
	}

	if (smoothParams.convex2)
	{
		behind2 = b2Dot(smoothParams.normal2, b2Sub(centroidB, p2)) < 0.0f;
	}

	if (behind1 && behind0 && behind2)
	{
		// one-sided collision
		return manifold;
	}

	// Get polygonB in frameA
	int32_t count = polygonB->count;
	b2Vec2 vertices[b2_maxPolygonVertices];
	b2Vec2 normals[b2_maxPolygonVertices];
	//b2Vec2 sum = b2Vec2_zero;
	for (int32_t i = 0; i < count; ++i)
	{
		vertices[i] = b2TransformPoint(xf, polygonB->vertices[i]);
		normals[i] = b2RotateVector(xf.q, polygonB->normals[i]);

		//sum = b2Add(sum, b2Sub(vertices[i], centroidB));
	}

	//float sumLength = b2Length(sum);

	// Distance doesn't work correctly with partial polygons
	b2DistanceInput input;
	input.proxyA = b2MakeProxy(&segmentA->segment.point1, 2, 0.0f);
	input.proxyB = b2MakeProxy(vertices, count, 0.0f);
	input.transformA = b2Transform_identity;
	input.transformB = b2Transform_identity;
	input.useRadii = false;

	b2DistanceOutput output = b2ShapeDistance(cache, &input);

	if (output.distance > radiusB + b2_speculativeDistance)
	{
		return manifold;
	}

	// Snap concave normals for partial polygon
	b2Vec2 n0 = smoothParams.convex1 ? smoothParams.normal0 : normal1;
	b2Vec2 n2 = smoothParams.convex2 ? smoothParams.normal2 : normal1;

	// Index of incident vertex on polygon
	int32_t incidentIndex = -1;
	int32_t incidentNormal = -1;

	if (behind1 == false && output.distance > 0.1f * b2_linearSlop)
	{
		// The closest features may be two vertices or an edge and a vertex even when there should
		// be face contact

		if (cache->count == 1)
		{
			// vertex-vertex collision
			b2Vec2 pA = output.pointA;
			b2Vec2 pB = output.pointB;

			b2Vec2 normal = b2Normalize(b2Sub(pB, pA));

			enum b2NormalType type = b2ClassifyNormal(smoothParams, normal);
			if (type == b2_normalSkip)
			{
				return manifold;
			}
			else if (type == b2_normalAdmit)
			{
				manifold.normal = b2RotateVector(xfA.q, normal);
				b2ManifoldPoint* cp = manifold.points + 0;
				cp->point = b2TransformPoint(xfA, pA);
				cp->separation = output.distance - radiusB;
				cp->id = B2_MAKE_ID(cache->indexA[0], cache->indexB[0]);
				manifold.pointCount = 1;
				return manifold;
			}

			// fall through b2_normalSnap
			incidentIndex = cache->indexB[0];
		}
		else
		{
			// vertex-edge collision
			B2_ASSERT(cache->count == 2);

			int32_t ia1 = cache->indexA[0];
			int32_t ia2 = cache->indexA[1];
			int32_t ib1 = cache->indexB[0];
			int32_t ib2 = cache->indexB[1];

			if (ia1 == ia2)
			{
				// 1 point on A, expect 2 points on B
				B2_ASSERT(ib1 != ib2);

				// Find polygon normal most aligned with vector between closest points.
				// This effectively sorts ib1 and ib2
				b2Vec2 normalB = b2Sub(output.pointA, output.pointB);
				float dot1 = b2Dot(normalB, normals[ib1]);
				float dot2 = b2Dot(normalB, normals[ib2]);
				int32_t ib = dot1 > dot2 ? ib1 : ib2;

				// Use accurate normal
				normalB = normals[ib];

				enum b2NormalType type = b2ClassifyNormal(smoothParams, b2Neg(normalB));
				if (type == b2_normalSkip)
				{
					return manifold;
				}
				else if (type == b2_normalAdmit)
				{
					// Get polygon edge associated with normal
					ib1 = ib;
					ib2 = ib < count - 1 ? ib + 1 : 0;

					b2Vec2 b1 = vertices[ib1];
					b2Vec2 b2 = vertices[ib2];

					// Find incident segment vertex
					dot1 = b2Dot(normalB, b2Sub(p1, b1));
					dot2 = b2Dot(normalB, b2Sub(p2, b1));

					if (dot1 < dot2)
					{
						if (b2Dot(n0, normalB) < b2Dot(normal1, normalB))
						{
							// Neighbor is incident
							return manifold;
						}
					}
					else
					{
						if (b2Dot(n2, normalB) < b2Dot(normal1, normalB))
						{
							// Neighbor is incident
							return manifold;
						}
					}

					manifold = b2ClipSegments(b1, b2, p1, p2, normalB, radiusB, 0.0f, B2_MAKE_ID(ib1, 1), B2_MAKE_ID(ib2, 0));
					manifold.normal = b2RotateVector(xfA.q, b2Neg(normalB));
					manifold.points[0].point = b2TransformPoint(xfA, manifold.points[0].point);
					manifold.points[1].point = b2TransformPoint(xfA, manifold.points[1].point);
					return manifold;
				}

				// fall through b2_normalSnap
				incidentNormal = ib;
			}
			else
			{
				// Get index of incident polygonB vertex
				float dot1 = b2Dot(normal1, b2Sub(vertices[ib1], p1));
				float dot2 = b2Dot(normal1, b2Sub(vertices[ib2], p2));
				incidentIndex = dot1 < dot2 ? ib1 : ib2;
			}
		}
	}
	else
	{
		// SAT edge normal
		float edgeSeparation = FLT_MAX;

		for (int32_t i = 0; i < count; ++i)
		{
			float s = b2Dot(normal1, b2Sub(vertices[i], p1));
			if (s < edgeSeparation)
			{
				edgeSeparation = s;
				incidentIndex = i;
			}
		}

		// Check convex neighbor for edge separation
		if (smoothParams.convex1)
		{
			float s0 = FLT_MAX;

			for (int32_t i = 0; i < count; ++i)
			{
				float s = b2Dot(smoothParams.normal0, b2Sub(vertices[i], p1));
				if (s < s0)
				{
					s0 = s;
				}
			}

			if (s0 > edgeSeparation)
			{
				edgeSeparation = s0;

				// Indicate neighbor owns edge separation
				incidentIndex = -1;
			}
		}

		// Check convex neighbor for edge separation
		if (smoothParams.convex2)
		{
			float s2 = FLT_MAX;

			for (int32_t i = 0; i < count; ++i)
			{
				float s = b2Dot(smoothParams.normal2, b2Sub(vertices[i], p2));
				if (s < s2)
				{
					s2 = s;
				}
			}

			if (s2 > edgeSeparation)
			{
				edgeSeparation = s2;

				// Indicate neighbor owns edge separation
				incidentIndex = -1;
			}
		}

		// SAT polygon normals
		float polygonSeparation = -FLT_MAX;
		int32_t referenceIndex = -1;

		for (int32_t i = 0; i < count; ++i)
		{
			b2Vec2 n = normals[i];

			// Check the infinite sides of the partial polygon
			if ((smoothParams.convex1 && b2Cross(n0, n) > 0.0f) || (smoothParams.convex2 && b2Cross(n, n2) > 0.0f))
			{
				continue;
			}

			b2Vec2 p = vertices[i];
			float s = B2_MIN(b2Dot(n, b2Sub(p2, p)), b2Dot(n, b2Sub(p1, p)));

			if (s > polygonSeparation)
			{
				polygonSeparation = s;
				referenceIndex = i;
			}
		}

		if (polygonSeparation > edgeSeparation)
		{
			int32_t ia1 = referenceIndex;
			int32_t ia2 = ia1 < count - 1 ? ia1 + 1 : 0;
			b2Vec2 a1 = vertices[ia1];
			b2Vec2 a2 = vertices[ia2];

			b2Vec2 n = normals[ia1];

			float dot1 = b2Dot(n, b2Sub(p1, a1));
			float dot2 = b2Dot(n, b2Sub(p2, a1));

			if (dot1 < dot2)
			{
				if (b2Dot(n0, n) < b2Dot(normal1, n))
				{
					// Neighbor is incident
					return manifold;
				}
			}
			else
			{
				if (b2Dot(n2, n) < b2Dot(normal1, n))
				{
					// Neighbor is incident
					return manifold;
				}
			}

			manifold = b2ClipSegments(a1, a2, p1, p2, normals[ia1], radiusB, 0.0f, B2_MAKE_ID(ia1, 1), B2_MAKE_ID(ia2, 0));
			manifold.normal = b2RotateVector(xfA.q, b2Neg(normals[ia1]));
			manifold.points[0].point = b2TransformPoint(xfA, manifold.points[0].point);
			manifold.points[1].point = b2TransformPoint(xfA, manifold.points[1].point);
			return manifold;
		}

		if (incidentIndex == -1)
		{
			// neighboring segment is the separating axis
			return manifold;
		}

		// fall through segment normal axis
	}

	B2_ASSERT(incidentNormal != -1 || incidentIndex != -1);

	// Segment normal

	// Find incident polygon normal: normal adjacent to deepest vertex that is most anti-parallel to segment normal
	b2Vec2 b1, b2;
	int32_t ib1, ib2;

	if (incidentNormal != -1)
	{
		ib1 = incidentNormal;
		ib2 = ib1 < count - 1 ? ib1 + 1 : 0;
		b1 = vertices[ib1];
		b2 = vertices[ib2];
	}
	else
	{
		int32_t i2 = incidentIndex;
		int32_t i1 = i2 > 0 ? i2 - 1 : count - 1;
		float d1 = b2Dot(normal1, normals[i1]);
		float d2 = b2Dot(normal1, normals[i2]);
		if (d1 < d2)
		{
			ib1 = i1, ib2 = i2;
			b1 = vertices[ib1];
			b2 = vertices[ib2];
		}
		else
		{
			ib1 = i2, ib2 = i2 < count - 1 ? i2 + 1 : 0;
			b1 = vertices[ib1];
			b2 = vertices[ib2];
		}
	}

	manifold = b2ClipSegments(p1, p2, b1, b2, normal1, 0.0f, radiusB, B2_MAKE_ID(0, ib2), B2_MAKE_ID(1, ib1));
	manifold.normal = b2RotateVector(xfA.q, manifold.normal);
	manifold.points[0].point = b2TransformPoint(xfA, manifold.points[0].point);
	manifold.points[1].point = b2TransformPoint(xfA, manifold.points[1].point);

	return manifold;
}
