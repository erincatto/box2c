// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "box2d/manifold.h"

#include "core.h"

#include "box2d/distance.h"
#include "box2d/geometry.h"
#include "box2d/math.h"

#include <float.h>
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

b2Manifold b2CollideCircles(const b2Circle* circleA, b2Transform xfA, const b2Circle* circleB, b2Transform xfB, float maxDistance)
{
	b2Manifold manifold = {0};

	b2Vec2 pointA = b2TransformPoint(xfA, circleA->point);
	b2Vec2 pointB = b2TransformPoint(xfB, circleB->point);

	float distance;
	b2Vec2 normal = b2GetLengthAndNormalize(&distance, b2Sub(pointB, pointA));

	float rA = circleA->radius;
	float rB = circleB->radius;

	float separation = distance - rA - rB;
	if (separation > maxDistance)
	{
		return manifold;
	}

	b2Vec2 cA = b2MulAdd(pointA, rA, normal);
	b2Vec2 cB = b2MulAdd(pointB, -rB, normal);
	manifold.normal = normal;
	manifold.points[0].point = b2Lerp(cA, cB, 0.5f);
	manifold.points[0].separation = separation;
	manifold.points[0].id = 0;
	manifold.pointCount = 1;
	return manifold;
}

/// Compute the collision manifold between a capsule and circle
b2Manifold b2CollideCapsuleAndCircle(const b2Capsule* capsuleA, b2Transform xfA, const b2Circle* circleB, b2Transform xfB,
									 float maxDistance)
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

	float rA = capsuleA->radius;
	float rB = circleB->radius;
	float separation = distance - rA - rB;
	if (separation > maxDistance)
	{
		return manifold;
	}

	b2Vec2 cA = b2MulAdd(pA, rA, normal);
	b2Vec2 cB = b2MulAdd(pB, -rB, normal);
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
	float radius = polygonA->radius + circleB->radius;

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

		b2Vec2 cA = b2MulAdd(v1, polygonA->radius, normal);
		b2Vec2 cB = b2MulSub(c, circleB->radius, normal);
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

		b2Vec2 cA = b2MulAdd(v2, polygonA->radius, normal);
		b2Vec2 cB = b2MulSub(c, circleB->radius, normal);
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
		b2Vec2 cA = b2MulAdd(c, polygonA->radius - b2Dot(b2Sub(c, v1), normal), normal);

		// cB is the deepest point on the circle with respect to the reference edge
		b2Vec2 cB = b2MulSub(c, circleB->radius, normal);

		// The contact point is the midpoint in world space
		manifold.points[0].point = b2TransformPoint(xfA, b2Lerp(cA, cB, 0.5f));
		manifold.points[0].separation = separation - radius;
		manifold.points[0].id = 0;
		manifold.pointCount = 1;
	}

	return manifold;
}

b2Manifold b2CollideCapsules(const b2Capsule* capsuleA, b2Transform xfA, const b2Capsule* capsuleB, b2Transform xfB, float maxDistance,
							 b2DistanceCache* cache)
{
	b2Polygon polyA = b2MakeCapsule(capsuleA->point1, capsuleA->point2, capsuleA->radius);
	b2Polygon polyB = b2MakeCapsule(capsuleB->point1, capsuleB->point2, capsuleB->radius);
	return b2CollidePolygons(&polyA, xfA, &polyB, xfB, maxDistance, cache);
}

b2Manifold b2CollideSegmentAndCapsule(const b2Segment* segmentA, b2Transform xfA, const b2Capsule* capsuleB, b2Transform xfB,
									  float maxDistance, b2DistanceCache* cache)
{
	b2Polygon polyA = b2MakeCapsule(segmentA->point1, segmentA->point2, 0.0f);
	b2Polygon polyB = b2MakeCapsule(capsuleB->point1, capsuleB->point2, capsuleB->radius);
	return b2CollidePolygons(&polyA, xfA, &polyB, xfB, maxDistance, cache);
}

b2Manifold b2CollidePolygonAndCapsule(const b2Polygon* polygonA, b2Transform xfA, const b2Capsule* capsuleB, b2Transform xfB,
									  float maxDistance, b2DistanceCache* cache)
{
	b2Polygon polyB = b2MakeCapsule(capsuleB->point1, capsuleB->point2, capsuleB->radius);
	return b2CollidePolygons(polygonA, xfA, &polyB, xfB, maxDistance, cache);
}

// Polygon clipper used by GJK and SAT to compute contact points when there are potentially two contact points.
static b2Manifold b2PolygonClip(const b2Polygon* polyA, b2Transform xfA, const b2Polygon* polyB, b2Transform xfB, int32_t edgeA,
								int32_t edgeB, float maxDistance, bool flip)
{
	B2_MAYBE_UNUSED(maxDistance);

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
static float b2FindMaxSeparation(int32_t* edgeIndex, const b2Polygon* poly1, b2Transform xf1, const b2Polygon* poly2, b2Transform xf2)
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
static b2Manifold b2PolygonSAT(const b2Polygon* polyA, b2Transform xfA, const b2Polygon* polyB, b2Transform xfB, float maxDistance)
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

	return b2PolygonClip(polyA, xfA, polyB, xfB, edgeA, edgeB, maxDistance, flip);
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
b2Manifold b2CollidePolygons(const b2Polygon* polyA, b2Transform xfA, const b2Polygon* polyB, b2Transform xfB, float maxDistance,
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

	if (output.distance > radius + maxDistance)
	{
		return manifold;
	}

	if (output.distance < 0.1f * b2_linearSlop)
	{
		// distance is small or zero, fallback to SAT
		return b2PolygonSAT(polyA, xfA, polyB, xfB, maxDistance);
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

	return b2PolygonClip(polyA, xfA, polyB, xfB, edgeA, edgeB, maxDistance, flip);
}

b2Manifold b2CollideSegmentAndCircle(const b2Segment* segmentA, b2Transform xfA, const b2Circle* circleB, b2Transform xfB, float maxDistance)
{
	b2Capsule capsuleA = {segmentA->point1, segmentA->point2, 0.0f};
	return b2CollideCapsuleAndCircle(&capsuleA, xfA, circleB, xfB, maxDistance);
}

#if 0
b2Manifold b2CollideSmoothSegmentAndCircle(const b2SmoothSegment* segmentA, b2Transform xfA, const b2Circle* circleB,
										   b2Transform xfB)
{
	b2Manifold manifold = b2EmptyManifold();

	// Compute circle in frame of segment
	b2Vec2 Q = b2InvTransformPoint(xfA, b2TransformPoint(xfB, circleB->point));

	b2Vec2 A = segmentA->point1, B = segmentA->point2;
	b2Vec2 e = b2Sub(B, A);

	// Normal points to the right for a CCW winding
	b2Vec2 n = {e.y, -e.x};
	float offset = b2Dot(n, b2Sub(Q, A));
	if (offset < 0.0f)
	{
		return manifold;
	}

	// Barycentric coordinates
	float u = b2Dot(e, b2Sub(B, Q));
	float v = b2Dot(e, b2Sub(Q, A));

	b2ContactFeature cf;
	cf.indexB = 0;
	cf.typeB = b2_vertexFeature;

	// Region A
	if (v <= 0.0f)
	{
		b2Vec2 A1 = segmentA->ghost1;
		b2Vec2 B1 = A;
		b2Vec2 e1 = b2Sub(B1, A1);
		float u1 = b2Dot(e1, b2Sub(B1, Q));

		// Is the circle in Region AB of the previous edge?
		if (u1 > 0.0f)
		{
			return manifold;
		}

		cf.indexA = 0;
		cf.typeA = b2_vertexFeature;
		manifold.pointCount = 1;
		manifold.type = b2_manifoldCircles;
		manifold.localNormal = b2Vec2_zero;
		manifold.localPoint = A;
		manifold.points[0].id.key = 0;
		manifold.points[0].id.cf = cf;
		manifold.points[0].localPoint = circleB->point;
		return manifold;
	}

	// Region B
	if (u <= 0.0f)
	{
		b2Vec2 B2 = segmentA->ghost2;
		b2Vec2 A2 = B;
		b2Vec2 e2 = b2Sub(B2, A2);
		float v2 = b2Dot(e2, b2Sub(Q, A2));

		// Is the circle in Region AB of the next edge?
		if (v2 > 0.0f)
		{
			return manifold;
		}

		cf.indexA = 1;
		cf.typeA = b2_vertexFeature;
		manifold.pointCount = 1;
		manifold.type = b2_manifoldCircles;
		manifold.localNormal = b2Vec2_zero;
		manifold.localPoint = B;
		manifold.points[0].id.key = 0;
		manifold.points[0].id.cf = cf;
		manifold.points[0].localPoint = circleB->point;
		return manifold;
	}

	// Region AB
	n = b2Normalize(n);

	cf.indexA = 0;
	cf.typeA = b2_faceFeature;
	manifold.pointCount = 1;
	manifold.type = b2_manifoldFaceA;
	manifold.localNormal = n;
	manifold.localPoint = A;
	manifold.points[0].id.key = 0;
	manifold.points[0].id.cf = cf;
	manifold.points[0].localPoint = circleB->point;
	return manifold;
}

// Separating axis between segment and polygon
typedef enum b2SPType
{
	b2_sp_unknown,
	b2_sp_segmentA,
	b2_sp_polygonB
} b2SPType;

typedef struct b2SPAxis
{
	b2Vec2 normal;
	b2SPType type;
	int32_t index;
	float separation;
} b2SPAxis;

// This holds polygon B expressed in frame A.
typedef struct b2TempPolygon
{
	b2Vec2 vertices[b2_maxPolygonVertices];
	b2Vec2 normals[b2_maxPolygonVertices];
	int32_t count;
} b2TempPolygon;

// Reference face used for clipping
typedef struct b2ReferenceFace
{
	int32_t i1, i2;
	b2Vec2 v1, v2;
	b2Vec2 normal;

	b2Vec2 sideNormal1;
	float sideOffset1;

	b2Vec2 sideNormal2;
	float sideOffset2;
} b2ReferenceFace;

static b2SPAxis b2ComputeEdgeSeparation(const b2TempPolygon* polygonB, b2Vec2 v1, b2Vec2 normal1)
{
	b2SPAxis axis;
	axis.type = b2_sp_segmentA;
	axis.index = -1;
	axis.separation = -FLT_MAX;
	axis.normal = b2Vec2_zero;

	b2Vec2 axes[2] = {normal1, b2Neg(normal1)};

	// Find axis with least overlap (min-max problem)
	for (int32_t j = 0; j < 2; ++j)
	{
		float sj = FLT_MAX;

		// Find deepest polygon vertex along axis j
		for (int32_t i = 0; i < polygonB->count; ++i)
		{
			float si = b2Dot(axes[j], b2Sub(polygonB->vertices[i], v1));
			if (si < sj)
			{
				sj = si;
			}
		}

		if (sj > axis.separation)
		{
			axis.index = j;
			axis.separation = sj;
			axis.normal = axes[j];
		}
	}

	return axis;
}

static b2SPAxis b2ComputePolygonSeparation(const b2TempPolygon* polygonB, b2Vec2 v1, b2Vec2 v2)
{
	b2SPAxis axis;
	axis.type = b2_sp_unknown;
	axis.index = -1;
	axis.separation = -FLT_MAX;
	axis.normal = b2Vec2_zero;

	for (int32_t i = 0; i < polygonB->count; ++i)
	{
		b2Vec2 n = b2Neg(polygonB->normals[i]);

		float s1 = b2Dot(n, b2Sub(polygonB->vertices[i], v1));
		float s2 = b2Dot(n, b2Sub(polygonB->vertices[i], v2));
		float s = B2_MIN(s1, s2);

		if (s > axis.separation)
		{
			axis.type = b2_sp_polygonB;
			axis.index = i;
			axis.separation = s;
			axis.normal = n;
		}
	}

	return axis;
}
#endif

b2Manifold b2CollideSegmentAndPolygon(const b2Segment* segmentA, b2Transform xfA, const b2Polygon* polygonB, b2Transform xfB,
									  float maxDistance, b2DistanceCache* cache)
{
	b2Polygon polygonA = b2MakeCapsule(segmentA->point1, segmentA->point2, 0.0f);
	return b2CollidePolygons(&polygonA, xfA, polygonB, xfB, maxDistance, cache);
}

#if 0
b2Manifold b2CollideSmoothSegmentAndPolygon(const b2SmoothSegment* segmentA, b2Transform xfA, const b2Polygon* polygonB,
											b2Transform xfB)
{
	b2Manifold manifold = b2EmptyManifold();

	b2Transform xf = b2InvMulTransforms(xfA, xfB);

	b2Vec2 centroidB = b2TransformPoint(xf, polygonB->centroid);

	b2Vec2 v1 = segmentA->point1;
	b2Vec2 v2 = segmentA->point2;

	b2Vec2 edge1 = b2Normalize(b2Sub(v2, v1));

	// Normal points to the right for a CCW winding
	b2Vec2 normal1 = {edge1.y, -edge1.x};
	float offset = b2Dot(normal1, b2Sub(centroidB, v1));
	if (offset < 0.0f)
	{
		return manifold;
	}

	// Get polygonB in frameA
	b2TempPolygon tempPolygonB;
	tempPolygonB.count = polygonB->count;
	for (int32_t i = 0; i < polygonB->count; ++i)
	{
		tempPolygonB.vertices[i] = b2TransformPoint(xf, polygonB->vertices[i]);
		tempPolygonB.normals[i] = b2RotateVector(xf.q, polygonB->normals[i]);
	}

	b2SPAxis edgeAxis = b2ComputeEdgeSeparation(&tempPolygonB, v1, normal1);
	b2SPAxis polygonAxis = b2ComputePolygonSeparation(&tempPolygonB, v1, v2);

	// Use hysteresis for jitter reduction.
	const float k_relativeTol = 0.98f;
	const float k_absoluteTol = 0.001f;

	b2SPAxis primaryAxis;
	if (polygonAxis.separation > k_relativeTol * edgeAxis.separation + k_absoluteTol)
	{
		primaryAxis = polygonAxis;
	}
	else
	{
		primaryAxis = edgeAxis;
	}

	// Smooth collision
	// See https://box2d.org/posts/2020/06/ghost-collisions/

	b2Vec2 edge0 = b2Normalize(b2Sub(v1, segmentA->ghost1));
	b2Vec2 normal0 = {edge0.y, -edge0.x};
	bool convex1 = b2Cross(edge0, edge1) >= 0.0f;

	b2Vec2 edge2 = b2Normalize(b2Sub(segmentA->ghost2, v2));
	b2Vec2 normal2 = {edge2.y, -edge2.x};
	bool convex2 = b2Cross(edge1, edge2) >= 0.0f;

	const float sinTol = 0.1f;
	bool side1 = b2Dot(primaryAxis.normal, edge1) <= 0.0f;

	// Check Gauss Map
	if (side1)
	{
		if (convex1)
		{
			if (b2Cross(primaryAxis.normal, normal0) > sinTol)
			{
				// Skip region
				return manifold;
			}

			// Admit region
		}
		else
		{
			// Snap region
			primaryAxis = edgeAxis;
		}
	}
	else
	{
		if (convex2)
		{
			if (b2Cross(normal2, primaryAxis.normal) > sinTol)
			{
				// Skip region
				return manifold;
			}

			// Admit region
		}
		else
		{
			// Snap region
			primaryAxis = edgeAxis;
		}
	}

	b2ClipVertex clipPoints[2];
	b2ReferenceFace ref;
	if (primaryAxis.type == b2_sp_segmentA)
	{
		manifold.type = b2_manifoldFaceA;

		// Search for the polygon normal that is most anti-parallel to the edge normal.
		int32_t bestIndex = 0;
		float bestValue = b2Dot(primaryAxis.normal, tempPolygonB.normals[0]);
		for (int32_t i = 1; i < tempPolygonB.count; ++i)
		{
			float value = b2Dot(primaryAxis.normal, tempPolygonB.normals[i]);
			if (value < bestValue)
			{
				bestValue = value;
				bestIndex = i;
			}
		}

		int32_t i1 = bestIndex;
		int32_t i2 = i1 + 1 < tempPolygonB.count ? i1 + 1 : 0;

		clipPoints[0].v = tempPolygonB.vertices[i1];
		clipPoints[0].id.cf.indexA = 0;
		clipPoints[0].id.cf.indexB = (uint8_t)i1;
		clipPoints[0].id.cf.typeA = b2_faceFeature;
		clipPoints[0].id.cf.typeB = b2_vertexFeature;

		clipPoints[1].v = tempPolygonB.vertices[i2];
		clipPoints[1].id.cf.indexA = 0;
		clipPoints[1].id.cf.indexB = (uint8_t)i2;
		clipPoints[1].id.cf.typeA = b2_faceFeature;
		clipPoints[1].id.cf.typeB = b2_vertexFeature;

		ref.i1 = 0;
		ref.i2 = 1;
		ref.v1 = v1;
		ref.v2 = v2;
		ref.normal = primaryAxis.normal;
		ref.sideNormal1 = b2Neg(edge1);
		ref.sideNormal2 = edge1;
	}
	else
	{
		manifold.type = b2_manifoldFaceB;

		clipPoints[0].v = v2;
		clipPoints[0].id.cf.indexA = 1;
		clipPoints[0].id.cf.indexB = (uint8_t)primaryAxis.index;
		clipPoints[0].id.cf.typeA = b2_vertexFeature;
		clipPoints[0].id.cf.typeB = b2_faceFeature;

		clipPoints[1].v = v1;
		clipPoints[1].id.cf.indexA = 0;
		clipPoints[1].id.cf.indexB = (uint8_t)primaryAxis.index;
		clipPoints[1].id.cf.typeA = b2_vertexFeature;
		clipPoints[1].id.cf.typeB = b2_faceFeature;

		ref.i1 = primaryAxis.index;
		ref.i2 = ref.i1 + 1 < tempPolygonB.count ? ref.i1 + 1 : 0;
		ref.v1 = tempPolygonB.vertices[ref.i1];
		ref.v2 = tempPolygonB.vertices[ref.i2];
		ref.normal = tempPolygonB.normals[ref.i1];

		// CCW winding
		ref.sideNormal1 = (b2Vec2){ref.normal.y, -ref.normal.x};
		ref.sideNormal2 = b2Neg(ref.sideNormal1);
	}

	ref.sideOffset1 = b2Dot(ref.sideNormal1, ref.v1);
	ref.sideOffset2 = b2Dot(ref.sideNormal2, ref.v2);

	// Clip incident edge against reference face side planes
	b2ClipVertex clipPoints1[2];
	b2ClipVertex clipPoints2[2];
	int32_t np;

	// Clip to side 1
	np = b2ClipSegmentToLine(clipPoints1, clipPoints, ref.sideNormal1, ref.sideOffset1, ref.i1);

	if (np < b2_maxManifoldPoints)
	{
		return manifold;
	}

	// Clip to side 2
	np = b2ClipSegmentToLine(clipPoints2, clipPoints1, ref.sideNormal2, ref.sideOffset2, ref.i2);

	if (np < b2_maxManifoldPoints)
	{
		return manifold;
	}

	// Now clipPoints2 contains the clipped points.
	if (primaryAxis.type == b2_sp_segmentA)
	{
		manifold.localNormal = ref.normal;
		manifold.localPoint = ref.v1;
	}
	else
	{
		manifold.localNormal = polygonB->normals[ref.i1];
		manifold.localPoint = polygonB->vertices[ref.i1];
	}

	for (int32_t i = 0; i < b2_maxManifoldPoints; ++i)
	{
		b2ManifoldPoint* cp = manifold.points + i;

		if (primaryAxis.type == b2_sp_segmentA)
		{
			cp->localPoint = b2InvTransformPoint(xf, clipPoints2[i].v);
			cp->id = clipPoints2[i].id;
		}
		else
		{
			cp->localPoint = clipPoints2[i].v;
			cp->id.cf.typeA = clipPoints2[i].id.cf.typeB;
			cp->id.cf.typeB = clipPoints2[i].id.cf.typeA;
			cp->id.cf.indexA = clipPoints2[i].id.cf.indexB;
			cp->id.cf.indexB = clipPoints2[i].id.cf.indexA;
		}
	}

	manifold.pointCount = b2_maxManifoldPoints;
	return manifold;
}
#endif
