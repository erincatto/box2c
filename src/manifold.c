// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "box2d/manifold.h"
#include "box2d/distance.h"
#include "box2d/geometry.h"
#include "box2d/math.h"

#include <assert.h>
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


/// Used for computing contact manifolds.
typedef struct b2ClipVertex
{
	b2Vec2 v;
	int32_t indexA;
	int32_t indexB;
} b2ClipVertex;

// Sutherland-Hodgman clipping.
// TODO clip both sides in one function
int32_t b2ClipSegmentToLine(b2ClipVertex vOut[2], const b2ClipVertex vIn[2], b2Vec2 normal, float offset,
							int32_t vertexIndexA)
{
	// Start with no output points
	int32_t count = 0;

	// Calculate the distance of end points to the line
	float distance0 = b2Dot(normal, vIn[0].v) - offset;
	float distance1 = b2Dot(normal, vIn[1].v) - offset;

	// If the points are behind the plane
	if (distance0 <= 0.0f)
		vOut[count++] = vIn[0];
	if (distance1 <= 0.0f)
		vOut[count++] = vIn[1];

	// If the points are on different sides of the plane
	if (distance0 * distance1 < 0.0f)
	{
		// Find intersection point of edge and plane
		float interp = distance0 / (distance0 - distance1);
		vOut[count].v = b2Lerp(vIn[0].v, vIn[1].v, interp);

		// VertexA is hitting edgeB.
		vOut[count].indexA = (uint8_t)vertexIndexA;
		vOut[count].indexB = vIn[0].indexB;

		++count;

		assert(count == 2);
	}

	return count;
}

int32_t b2ClipSegmentToLine2(b2Vec2 vOut[2], b2Vec2 vIn[2], b2Vec2 normal, float offset)
{
	// Start with no output points
	int32_t count = 0;

	// Calculate the distance of end points to the line
	float distance0 = b2Dot(normal, vIn[0]) - offset;
	float distance1 = b2Dot(normal, vIn[1]) - offset;

	// If the points are behind the plane
	if (distance0 <= 0.0f)
	{
		vOut[count++] = vIn[0];
	}

	if (distance1 <= 0.0f)
	{
		vOut[count++] = vIn[1];
	}

	// If the points are on different sides of the plane
	if (distance0 * distance1 < 0.0f)
	{
		assert(count == 1);

		// Find intersection point of edge and plane
		float interp = distance0 / (distance0 - distance1);
		vOut[1] = b2Lerp(vIn[0], vIn[1], interp);
		++count;
	}

	return count;
}

b2Manifold b2CollideCircles(const b2Circle* circleA, b2Transform xfA, const b2Circle* circleB, b2Transform xfB)
{
	b2Manifold manifold = b2EmptyManifold();
	
	b2Vec2 pointA = b2TransformPoint(xfA, circleA->point);
	b2Vec2 pointB = b2TransformPoint(xfB, circleB->point);

	if (b2DistanceSquared(pointA, pointB) > FLT_EPSILON * FLT_EPSILON)
	{
		manifold.normal = b2Normalize(b2Sub(pointB, pointA));
	}
	else
	{
		manifold.normal = (b2Vec2){1.0f, 0.0f};
	}

	b2Vec2 cA = b2MulAdd(pointA, circleA->radius, manifold.normal);
	b2Vec2 cB = b2MulAdd(pointB, -circleB->radius, manifold.normal);
	manifold.points[0].point = b2Lerp(cA, cB, 0.5f);
	manifold.points[0].separation = b2Dot(b2Sub(cB, cA), manifold.normal);
	manifold.points[0].id = 0;
	manifold.pointCount = 1;
	return manifold;
}

#if 0
/// Compute the collision manifold between a capulse and circle
b2Manifold b2CollideCapsuleAndCircle(const b2Capsule* capsuleA, b2Transform xfA, const b2Circle* circleB,
									 b2Transform xfB)
{
	b2Manifold manifold = b2EmptyManifold();

	manifold.pointCount = 1;
	manifold.points[0].localPoint = circleB->point;
	manifold.points[0].id.key = 0;

	// Compute circle position in the frame of the capsule.
	b2Vec2 p = b2InvTransformPoint(xfA, b2TransformPoint(xfB, circleB->point));

	// Compute closest point
	b2Vec2 v1 = capsuleA->point1;
	b2Vec2 v2 = capsuleA->point2;

	float length;
	b2Vec2 eUnit = b2GetLengthAndNormalize(&length, b2Sub(v2, v1));

	// dot(p - closest, eUnit) = 0
	// dot(p - (v1 + s * eUnit), eUnit) = 0
	// s = dot(p - v1, eUnit)
	float s = b2Dot(b2Sub(p, v1), eUnit);
	if (s < 0.0f)
	{
		// Circle colliding with v1
		manifold.type = b2_manifoldCircles;
		manifold.localPoint = v1;
		manifold.localNormal = (b2Vec2){0.0f, 0.0f};
	}
	else if (s > length)
	{
		// Circle colliding with v2
		manifold.type = b2_manifoldCircles;
		manifold.localPoint = v2;
		manifold.localNormal = (b2Vec2){0.0f, 0.0f};
	}
	else
	{
		// Circle colliding with segment interior
		b2Vec2 closest = b2MulAdd(v1, s, eUnit);

		// normal points right of v1->v2
		b2Vec2 normal = {eUnit.y, -eUnit.x};
		float side = b2Dot(b2Sub(p, closest), normal);
		if (side < 0.0f)
		{
			normal = b2Neg(normal);
		}

		manifold.type = b2_manifoldFaceA;
		manifold.localPoint = closest;
		manifold.localNormal = normal;
	}

	return manifold;
}
#endif

b2Manifold b2CollidePolygonAndCircle(const b2Polygon* polygonA, b2Transform xfA, const b2Circle* circleB,
									 b2Transform xfB)
{
	b2Manifold manifold = b2EmptyManifold();

	// Compute circle position in the frame of the polygon.
	b2Vec2 c = b2InvTransformPoint(xfA, b2TransformPoint(xfB, circleB->point));
	float radius = circleB->radius;

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
		manifold.pointCount = 1;
		manifold.normal = b2RotateVector(xfA.q, normal);
		b2Vec2 cA = v1;
		b2Vec2 cB = b2MulAdd(c, -radius, normal);
		manifold.points[0].point = b2TransformPoint(xfA, b2Lerp(cA, cB, 0.5f));
		manifold.points[0].separation = b2Dot(b2Sub(cB, cA), normal);
		manifold.points[0].id = 0;
	}
	else if (u2 < 0.0f && separation > FLT_EPSILON)
	{
		// Circle center is closest to v2 and safely outside the polygon
		b2Vec2 normal = b2Normalize(b2Sub(c, v2));
		manifold.pointCount = 1;
		manifold.normal = b2RotateVector(xfA.q, normal);
		b2Vec2 cA = v2;
		b2Vec2 cB = b2MulAdd(c, -radius, normal);
		manifold.points[0].point = b2TransformPoint(xfA, b2Lerp(cA, cB, 0.5f));
		manifold.points[0].separation = b2Dot(b2Sub(cB, cA), normal);
		manifold.points[0].id = 0;
	}
	else
	{
		// Circle center is between v1 and v2. Center may be inside polygon
		manifold.pointCount = 1;

		b2Vec2 normal = normals[normalIndex];
		manifold.normal = b2RotateVector(xfA.q, normal);

		// cA is the projection of the circle center onto to the reference edge
		b2Vec2 cA = b2MulAdd(c, -b2Dot(b2Sub(c, v1), normal), normal);

		// cB is the deepest point on the circle with respect to the reference edge
		b2Vec2 cB = b2MulAdd(c, -radius, normal);

		// The contact point is the midpoint in world space
		manifold.points[0].point = b2TransformPoint(xfA, b2Lerp(cA, cB, 0.5f));
		manifold.points[0].separation = separation - radius;
		manifold.points[0].id = 0;
		return manifold;
	}

	return manifold;
}

// Algorithm
// - find best edge separating axis
// - clip incident edge
// - if there are no points, then find best vertex-vertex
// Ideas
// - clip to the average axis to avoid normal pops

#if 0
b2Manifold b2CollideCapsules(const b2Capsule* capsuleA, b2Transform xfA, const b2Capsule* capsuleB, b2Transform xfB)
{
	b2Vec2 pA = b2TransformPoint(xfA, capsuleA->point1);
	b2Vec2 qA = b2TransformPoint(xfA, capsuleA->point2);
	b2Vec2 dA = b2Sub(qA, pA);

	b2Vec2 pB = b2TransformPoint(xfB, capsuleB->point1);
	b2Vec2 qB = b2TransformPoint(xfB, capsuleB->point2);
	b2Vec2 dB = b2Sub(qB, pB);

	b2Vec2 tangentA = b2Normalize(dA);
	b2Vec2 normalA = {tangentA.y, -tangentA.x};
	b2Vec2 tangentB = b2Normalize(dB);
	b2Vec2 normalB = {tangentB.y, -tangentB.x};

	// face A
	float sepA;
	{
		float sepP = b2Dot(b2Sub(pB, pA), normalA);
		float sepQ = b2Dot(b2Sub(qB, pA), normalA);
		float sepPos = B2_MIN(sepP, sepQ);
		float sepNeg = B2_MIN(-sepP, -sepQ);
		if (sepNeg > sepPos)
		{
			sepA = sepNeg;
			normalA = b2Neg(normalA);
		}
		else
		{
			sepA = sepPos;
		}
	}

	// face B
	float sepB;
	{
		float sepP = b2Dot(b2Sub(pA, pB), normalB);
		float sepQ = b2Dot(b2Sub(qA, pB), normalB);
		float sepPos = B2_MIN(sepP, sepQ);
		float sepNeg = B2_MIN(-sepP, -sepQ);
		if (sepNeg > sepPos)
		{
			sepB = sepNeg;
			normalB = b2Neg(normalB);
		}
		else
		{
			sepB = sepPos;
		}
	}

	b2Manifold manifold = b2EmptyManifold();

	// reference edge
	b2Vec2 pR, qR;

	// incident edge
	b2Vec2 pI, qI;

	b2Transform xfR, xfI;

	const b2Capsule* capsuleR;

	// Tolerance to stabilize contact points during stacking scenarios
	const float k_faceTol = 0.5f * b2_linearSlop;
	if (sepB > sepA + k_faceTol)
	{
		capsuleR = capsuleB;

		// segment B is the reference face
		manifold.type = b2_manifoldFaceB;

		xfR = xfB;
		xfI = xfA;
		pR = pB;
		qR = qB;
		pI = pA;
		qI = qA;
	}
	else
	{
		capsuleR = capsuleA;

		// segment A is the reference face
		manifold.type = b2_manifoldFaceA;

		xfR = xfA;
		xfI = xfB;
		pR = pA;
		qR = qA;
		pI = pB;
		qI = qB;
	}

	float length;
	b2Vec2 localAxis = b2GetLengthAndNormalize(&length, b2Sub(capsuleR->point2, capsuleR->point1));
	b2Vec2 localNormal = {localAxis.y, -localAxis.x};
	b2Vec2 axis = b2RotateVector(xfR.q, localAxis);

	b2Vec2 centerI = b2Lerp(pI, qI, 0.5f);
	float side = b2Cross(b2Sub(centerI, pR), axis);
	if (side < 0.0f)
	{
		localNormal = b2Neg(localNormal);
	}

	// Side offsets for clipping
	float sideOffset1 = -b2Dot(axis, pR);
	float sideOffset2 = b2Dot(axis, qR);

	// Clip incident edge against extruded edge1 side edges.
	b2Vec2 incidentEdge[2] = {pI, qI};
	b2Vec2 clipPoints1[2];
	b2Vec2 clipPoints2[2];
	int32_t np;

	static_assert(b2_maxManifoldPoints == 2, "b2_maxManifoldPoints != 2");

	// First side edge
	np = b2ClipSegmentToLine2(clipPoints1, incidentEdge, b2Neg(axis), sideOffset1);

	if (np == 2)
	{
		// Second side edge
		np = b2ClipSegmentToLine2(clipPoints2, clipPoints1, axis, sideOffset2);
	}

	if (np == 2)
	{
		// Now clipPoints2 contains the clipped points.
		manifold.localNormal = localNormal;
		manifold.localPoint = capsuleR->point1;

		manifold.points[0].localPoint = b2InvTransformPoint(xfI, clipPoints2[0]);
		manifold.points[1].localPoint = b2InvTransformPoint(xfI, clipPoints2[1]);

		float dist1 = b2DistanceSquared(clipPoints2[0], pA);
		float dist2 = b2DistanceSquared(clipPoints2[1], pA);

		if (dist1 < dist2)
		{
			manifold.points[0].id.key = 0;
			manifold.points[1].id.key = 1;
		}
		else
		{
			manifold.points[0].id.key = 1;
			manifold.points[1].id.key = 0;
		}

		manifold.pointCount = 2;
		return manifold;
	}

	// All points clipped away, so use vertex-vertex contact

	manifold.type = b2_manifoldCircles;
	manifold.localNormal = b2Vec2_zero;
	manifold.pointCount = 1;

	// Find closest vertex pair

	float distPP = b2DistanceSquared(pB, pA);
	float minDist = distPP;
	manifold.localPoint = capsuleA->point1;
	manifold.points[0].localPoint = capsuleB->point1;
	manifold.points[0].id.key = 0;

	float distPQ = b2DistanceSquared(qB, pA);
	if (distPQ < minDist)
	{
		minDist = distPQ;
		manifold.localPoint = capsuleA->point1;
		manifold.points[0].localPoint = capsuleB->point2;
		manifold.points[0].id.key = 0;
	}

	float distQP = b2DistanceSquared(pB, qA);
	if (distQP < minDist)
	{
		minDist = distQP;
		manifold.localPoint = capsuleA->point2;
		manifold.points[0].localPoint = capsuleB->point1;
		manifold.points[0].id.key = 1;
	}

	float distQQ = b2DistanceSquared(qB, qA);
	if (distQQ < minDist)
	{
		minDist = distQQ;
		manifold.localPoint = capsuleA->point2;
		manifold.points[0].localPoint = capsuleB->point2;
		manifold.points[0].id.key = 1;
	}

	return manifold;
}
#endif

#if 0
b2Manifold b2CollideCapsules2(const b2Capsule* capsuleA, b2Transform xfA, const b2Capsule* capsuleB,
							 b2Transform xfB)
{
	b2Vec2 pA = b2TransformPoint(xfA, capsuleA->point1);
	b2Vec2 qA = b2TransformPoint(xfA, capsuleA->point2);
	b2Vec2 dA = b2Sub(qA, pA);

	b2Vec2 pB = b2TransformPoint(xfB, capsuleB->point1);
	b2Vec2 qB = b2TransformPoint(xfB, capsuleB->point2);
	b2Vec2 dB = b2Sub(qB, pB);

	float lengthA;
	b2Vec2 tangentA = b2GetLengthAndNormalize(&lengthA, dA);
	b2Vec2 normalA = {tangentA.y, -tangentA.x};

	float lengthB;
	b2Vec2 tangentB = b2GetLengthAndNormalize(&lengthB, dB);
	b2Vec2 normalB = {tangentB.y, -tangentB.x};

	//const float cosTol = 0.7071067811865475f;
	//float cosAngle = B2_ABS(b2Dot(tangentA, tangentB));
	//bool canClip = (cosAngle < -cosTol || cosTol < cosAngle);

	// compute closest points
	b2SegmentDistanceResult result = b2SegmentDistance(pA, qA, pB, qB);
	if (result.distanceSquared > b2_linearSlop * b2_linearSlop)
	{

		b2Vec2 localA = b2Lerp(capsuleA->point1, capsuleA->point2, result.fraction1);
		b2Vec2 localB = b2Lerp(capsuleB->point1, capsuleB->point2, result.fraction2);
		b2Manifold manifold = b2EmptyManifold();
		manifold.pointCount = 1;

		if (result.fraction1 == 0.0f || result.fraction1 == 1.0f)
		{
			if (result.fraction2 == 0.0f || result.fraction2 == 1.0f)
			{
				manifold.type == b2_manifoldCircles;
				manifold.localPoint = localA;
				manifold.points[0].localPoint = localB;
				manifold.points[0].id.key = result.fraction1 < 0.5f ? 0 : 1;
			}
			else
			{
				b2Vec2 localNormal = b2InvRotateVector(xfB.q, normalB);

				if (b2Dot(normalB, b2Sub(result.closest2, result.closest1)) < 0.0f)
				{
					localNormal = b2Neg(localNormal);
				}

				manifold.type == b2_manifoldFaceB;
				manifold.localPoint = localB;
				manifold.localNormal = 
				manifold.points[0].localPoint = localA;
				manifold.points[0].id.key = result.fraction1 < 0.5f ? 0 : 1;
			}
		}
	}

	// face A separation
	float sepA;
	{
		float sepP = b2Dot(b2Sub(pB, pA), normalA);
		float sepQ = b2Dot(b2Sub(qB, pA), normalA);
		float sepPos = B2_MIN(sepP, sepQ);
		float sepNeg = B2_MIN(-sepP, -sepQ);
		if (sepNeg > sepPos)
		{
			sepA = sepNeg;
			normalA = b2Neg(normalA);
		}
		else
		{
			sepA = sepPos;
		}
	}

	// face B separation
	float sepB;
	{
		float sepP = b2Dot(b2Sub(pA, pB), normalB);
		float sepQ = b2Dot(b2Sub(qA, pB), normalB);
		float sepPos = B2_MIN(sepP, sepQ);
		float sepNeg = B2_MIN(-sepP, -sepQ);
		if (sepNeg > sepPos)
		{
			sepB = sepNeg;
			normalB = b2Neg(normalB);
		}
		else
		{
			sepB = sepPos;
		}
	}

	b2Manifold manifold = b2EmptyManifold();

	float bound_pA, bound_qA;
	float bound_pB, bound_qB;
	float lowerA, upperA;
	float lowerB, upperB;

	// Find reference face (edge)
	// Tolerance to stabilize contact points during stacking scenarios
	const float k_faceTol = 0.5f * b2_linearSlop;
	if (sepB > sepA + k_faceTol)
	{
		// segment B is the reference face
		manifold.type = b2_manifoldFaceB;
		manifold.localPoint = capsuleB->point1;
		manifold.localNormal = b2InvRotateVector(xfB.q, normalB);

		bound_pA = b2Dot(b2Sub(pA, pB), tangentB);
		bound_qA = b2Dot(b2Sub(qA, pB), tangentB);

		if (bound_pA < bound_qA)
		{
			lowerA = bound_pA;
			upperA = bound_qA;
		}
		else
		{
			lowerA = bound_qA;
			upperA = bound_pA;
		}

		lowerB = bound_pB = 0.0f;
		upperB = bound_qB = lengthB;
	}
	else
	{
		// segment A is the reference face
		manifold.type = b2_manifoldFaceA;
		manifold.localPoint = capsuleA->point1;
		manifold.localNormal = b2InvRotateVector(xfA.q, normalA);

		lowerA = bound_pA = 0.0f;
		upperA = bound_qA = lengthA;

		bound_pB = b2Dot(b2Sub(pB, pA), tangentA);
		bound_qB = b2Dot(b2Sub(qB, pA), tangentA);

		if (bound_pB < bound_qB)
		{
			lowerB = bound_pB;
			upperB = bound_qB;
		}
		else
		{
			lowerB = bound_qB;
			upperB = bound_pB;
		}
	}

	// Do intervals overlap?
	if (upperB >= lowerA && upperA >= lowerB)
	{
		float lower = B2_MAX(lowerA, lowerB);
		float upper = B2_MIN(upperA, upperB);

		if (manifold.type == b2_manifoldFaceB)
		{
			float fraction1, fraction2;
			float denA = bound_qA - bound_pA;
			if (-FLT_EPSILON < denA && denA < FLT_EPSILON)
			{
				fraction1 = 0.0f;
				fraction2 = 1.0f;
			}
			else
			{
				denA = 1.0f / denA;
				fraction1 = (lower - bound_pA) * denA;
				fraction2 = (upper - bound_pA) * denA;
			}

			if (tryClip == false)
			{
				b2Vec2 a1 = b2MulAdd(pA, fraction1, tangentA);
				b2Vec2 a2 = b2MulAdd(pA, fraction2, tangentA);

				float sep1 = b2Dot(b2Sub(a1, pB), normalB);
				float sep2 = b2Dot(b2Sub(a2, pB), normalB);

				if (sep1 < sep2)
				{
					manifold.points[0].localPoint = b2Lerp(capsuleA->point1, capsuleA->point2, fraction1);

					if (fraction1 < fraction2)
					{
						manifold.points[0].id.key = 0;
					}
					else
					{
						manifold.points[0].id.key = 1;
					}
				}
				else
				{
					manifold.points[0].localPoint = b2Lerp(capsuleA->point1, capsuleA->point2, fraction2);

					if (fraction2 < fraction1)
					{
						manifold.points[0].id.key = 0;
					}
					else
					{
						manifold.points[0].id.key = 1;
					}
				}
			}
			else
			{
				manifold.points[0].localPoint = b2Lerp(capsuleA->point1, capsuleA->point2, fraction1);
				manifold.points[1].localPoint = b2Lerp(capsuleA->point1, capsuleA->point2, fraction2);

				if (fraction1 < fraction2)
				{
					manifold.points[0].id.key = 0;
					manifold.points[1].id.key = 1;
				}
				else
				{
					manifold.points[0].id.key = 1;
					manifold.points[1].id.key = 0;
				}
			}
		}
		else
		{
			float fraction1, fraction2;
			float denB = bound_qB - bound_pB;
			if (-FLT_EPSILON < denB && denB < FLT_EPSILON)
			{
				fraction1 = 0.0f;
				fraction2 = 1.0f;
			}
			else
			{
				denB = 1.0f / denB;
				fraction1 = (lower - bound_pB) * denB;
				fraction2 = (upper - bound_pB) * denB;
			}

			b2Vec2 b1 = b2Lerp(pB, qB, fraction1);
			b2Vec2 b2 = b2Lerp(pB, qB, fraction2);

			float dist1 = b2DistanceSquared(b1, pA);
			float dist2 = b2DistanceSquared(b2, pA);

			if (tryClip == false)
			{
				float sep1 = b2Dot(b2Sub(b1, pA), normalA);
				float sep2 = b2Dot(b2Sub(b2, pA), normalA);

				if (sep1 < sep2)
				{
					manifold.points[0].localPoint = b2Lerp(capsuleB->point1, capsuleB->point2, fraction1);

					if (dist1 < dist2)
					{
						manifold.points[0].id.key = 0;
					}
					else
					{
						manifold.points[0].id.key = 1;
					}
				}
				else
				{
					manifold.points[0].localPoint = b2Lerp(capsuleB->point1, capsuleB->point2, fraction2);

					if (dist2 < dist1)
					{
						manifold.points[0].id.key = 0;
					}
					else
					{
						manifold.points[0].id.key = 1;
					}
				}
			}
			else
			{
				manifold.points[0].localPoint = b2Lerp(capsuleB->point1, capsuleB->point2, fraction1);
				manifold.points[1].localPoint = b2Lerp(capsuleB->point1, capsuleB->point2, fraction2);

				if (dist1 < dist2)
				{
					manifold.points[0].id.key = 0;
					manifold.points[1].id.key = 1;
				}
				else
				{
					manifold.points[0].id.key = 1;
					manifold.points[1].id.key = 0;
				}
			}
		}

		manifold.pointCount = tryClip ? 2 : 1;
		return manifold;
	}

	// All points clipped away, so use vertex-vertex contact

	manifold = b2EmptyManifold();
	manifold.type = b2_manifoldCircles;
	manifold.localNormal = b2Vec2_zero;
	manifold.pointCount = 1;

	// Find closest vertex pair

	float distPP = b2DistanceSquared(pB, pA);
	float minDist = distPP;
	manifold.localPoint = capsuleA->point1;
	manifold.points[0].localPoint = capsuleB->point1;
	manifold.points[0].id.key = 0;

	float distPQ = b2DistanceSquared(qB, pA);
	if (distPQ < minDist)
	{
		minDist = distPQ;
		manifold.localPoint = capsuleA->point1;
		manifold.points[0].localPoint = capsuleB->point2;
		manifold.points[0].id.key = 0;
	}

	float distQP = b2DistanceSquared(pB, qA);
	if (distQP < minDist)
	{
		minDist = distQP;
		manifold.localPoint = capsuleA->point2;
		manifold.points[0].localPoint = capsuleB->point1;
		manifold.points[0].id.key = 1;
	}

	float distQQ = b2DistanceSquared(qB, qA);
	if (distQQ < minDist)
	{
		minDist = distQQ;
		manifold.localPoint = capsuleA->point2;
		manifold.points[0].localPoint = capsuleB->point2;
		manifold.points[0].id.key = 1;
	}

	return manifold;
}

b2Manifold b2CollideCapsules3(const b2Capsule* capsuleA, b2Transform xfA, const b2Capsule* capsuleB,
							  b2Transform xfB)
{
	b2Vec2 pA = b2TransformPoint(xfA, capsuleA->point1);
	b2Vec2 qA = b2TransformPoint(xfA, capsuleA->point2);
	b2Vec2 dA = b2Sub(qA, pA);

	b2Vec2 pB = b2TransformPoint(xfB, capsuleB->point1);
	b2Vec2 qB = b2TransformPoint(xfB, capsuleB->point2);
	b2Vec2 dB = b2Sub(qB, pB);

	float lengthA;
	b2Vec2 tangentA = b2GetLengthAndNormalize(&lengthA, dA);
	b2Vec2 normalA = {tangentA.y, -tangentA.x};

	float lengthB;
	b2Vec2 tangentB = b2GetLengthAndNormalize(&lengthB, dB);
	b2Vec2 normalB = {tangentB.y, -tangentB.x};

	// const float cosTol = 0.7071067811865475f;
	// float cosAngle = B2_ABS(b2Dot(tangentA, tangentB));
	// bool canClip = (cosAngle < -cosTol || cosTol < cosAngle);

	// compute closest points
	b2SegmentDistanceResult result = b2SegmentDistance(pA, qA, pB, qB);
	if (result.distanceSquared > b2_linearSlop * b2_linearSlop)
	{
		b2Vec2 normal = b2Normalize(b2Sub(result.closest2, result.closest1));

		float cosA = B2_ABS(b2Dot(normalA, normal));
		float cosB = B2_ABS(b2Dot(normalB, normal));

		b2Vec2 localA = b2Lerp(capsuleA->point1, capsuleA->point2, result.fraction1);
		b2Vec2 localB = b2Lerp(capsuleB->point1, capsuleB->point2, result.fraction2);
		b2Manifold manifold = b2EmptyManifold();
		manifold.pointCount = 1;

		if (result.fraction1 == 0.0f || result.fraction1 == 1.0f)
		{
			if (result.fraction2 == 0.0f || result.fraction2 == 1.0f)
			{
				manifold.type == b2_manifoldCircles;
				manifold.localPoint = localA;
				manifold.points[0].localPoint = localB;
				manifold.points[0].id.key = result.fraction1 < 0.5f ? 0 : 1;
			}
			else
			{
				b2Vec2 localNormal = b2InvRotateVector(xfB.q, normalB);

				if (b2Dot(normalB, b2Sub(result.closest2, result.closest1)) < 0.0f)
				{
					localNormal = b2Neg(localNormal);
				}

				manifold.type == b2_manifoldFaceB;
				manifold.localPoint = localB;
				manifold.localNormal = manifold.points[0].localPoint = localA;
				manifold.points[0].id.key = result.fraction1 < 0.5f ? 0 : 1;
			}
		}
	}

	// face A separation
	float sepA;
	{
		float sepP = b2Dot(b2Sub(pB, pA), normalA);
		float sepQ = b2Dot(b2Sub(qB, pA), normalA);
		float sepPos = B2_MIN(sepP, sepQ);
		float sepNeg = B2_MIN(-sepP, -sepQ);
		if (sepNeg > sepPos)
		{
			sepA = sepNeg;
			normalA = b2Neg(normalA);
		}
		else
		{
			sepA = sepPos;
		}
	}

	// face B separation
	float sepB;
	{
		float sepP = b2Dot(b2Sub(pA, pB), normalB);
		float sepQ = b2Dot(b2Sub(qA, pB), normalB);
		float sepPos = B2_MIN(sepP, sepQ);
		float sepNeg = B2_MIN(-sepP, -sepQ);
		if (sepNeg > sepPos)
		{
			sepB = sepNeg;
			normalB = b2Neg(normalB);
		}
		else
		{
			sepB = sepPos;
		}
	}

	b2Manifold manifold = b2EmptyManifold();

	float bound_pA, bound_qA;
	float bound_pB, bound_qB;
	float lowerA, upperA;
	float lowerB, upperB;

	// Find reference face (edge)
	// Tolerance to stabilize contact points during stacking scenarios
	const float k_faceTol = 0.5f * b2_linearSlop;
	if (sepB > sepA + k_faceTol)
	{
		// segment B is the reference face
		manifold.type = b2_manifoldFaceB;
		manifold.localPoint = capsuleB->point1;
		manifold.localNormal = b2InvRotateVector(xfB.q, normalB);

		bound_pA = b2Dot(b2Sub(pA, pB), tangentB);
		bound_qA = b2Dot(b2Sub(qA, pB), tangentB);

		if (bound_pA < bound_qA)
		{
			lowerA = bound_pA;
			upperA = bound_qA;
		}
		else
		{
			lowerA = bound_qA;
			upperA = bound_pA;
		}

		lowerB = bound_pB = 0.0f;
		upperB = bound_qB = lengthB;
	}
	else
	{
		// segment A is the reference face
		manifold.type = b2_manifoldFaceA;
		manifold.localPoint = capsuleA->point1;
		manifold.localNormal = b2InvRotateVector(xfA.q, normalA);

		lowerA = bound_pA = 0.0f;
		upperA = bound_qA = lengthA;

		bound_pB = b2Dot(b2Sub(pB, pA), tangentA);
		bound_qB = b2Dot(b2Sub(qB, pA), tangentA);

		if (bound_pB < bound_qB)
		{
			lowerB = bound_pB;
			upperB = bound_qB;
		}
		else
		{
			lowerB = bound_qB;
			upperB = bound_pB;
		}
	}

	// Do intervals overlap?
	if (upperB >= lowerA && upperA >= lowerB)
	{
		float lower = B2_MAX(lowerA, lowerB);
		float upper = B2_MIN(upperA, upperB);

		if (manifold.type == b2_manifoldFaceB)
		{
			float fraction1, fraction2;
			float denA = bound_qA - bound_pA;
			if (-FLT_EPSILON < denA && denA < FLT_EPSILON)
			{
				fraction1 = 0.0f;
				fraction2 = 1.0f;
			}
			else
			{
				denA = 1.0f / denA;
				fraction1 = (lower - bound_pA) * denA;
				fraction2 = (upper - bound_pA) * denA;
			}

			if (tryClip == false)
			{
				b2Vec2 a1 = b2MulAdd(pA, fraction1, tangentA);
				b2Vec2 a2 = b2MulAdd(pA, fraction2, tangentA);

				float sep1 = b2Dot(b2Sub(a1, pB), normalB);
				float sep2 = b2Dot(b2Sub(a2, pB), normalB);

				if (sep1 < sep2)
				{
					manifold.points[0].localPoint = b2Lerp(capsuleA->point1, capsuleA->point2, fraction1);

					if (fraction1 < fraction2)
					{
						manifold.points[0].id.key = 0;
					}
					else
					{
						manifold.points[0].id.key = 1;
					}
				}
				else
				{
					manifold.points[0].localPoint = b2Lerp(capsuleA->point1, capsuleA->point2, fraction2);

					if (fraction2 < fraction1)
					{
						manifold.points[0].id.key = 0;
					}
					else
					{
						manifold.points[0].id.key = 1;
					}
				}
			}
			else
			{
				manifold.points[0].localPoint = b2Lerp(capsuleA->point1, capsuleA->point2, fraction1);
				manifold.points[1].localPoint = b2Lerp(capsuleA->point1, capsuleA->point2, fraction2);

				if (fraction1 < fraction2)
				{
					manifold.points[0].id.key = 0;
					manifold.points[1].id.key = 1;
				}
				else
				{
					manifold.points[0].id.key = 1;
					manifold.points[1].id.key = 0;
				}
			}
		}
		else
		{
			float fraction1, fraction2;
			float denB = bound_qB - bound_pB;
			if (-FLT_EPSILON < denB && denB < FLT_EPSILON)
			{
				fraction1 = 0.0f;
				fraction2 = 1.0f;
			}
			else
			{
				denB = 1.0f / denB;
				fraction1 = (lower - bound_pB) * denB;
				fraction2 = (upper - bound_pB) * denB;
			}

			b2Vec2 b1 = b2Lerp(pB, qB, fraction1);
			b2Vec2 b2 = b2Lerp(pB, qB, fraction2);

			float dist1 = b2DistanceSquared(b1, pA);
			float dist2 = b2DistanceSquared(b2, pA);

			if (tryClip == false)
			{
				float sep1 = b2Dot(b2Sub(b1, pA), normalA);
				float sep2 = b2Dot(b2Sub(b2, pA), normalA);

				if (sep1 < sep2)
				{
					manifold.points[0].localPoint = b2Lerp(capsuleB->point1, capsuleB->point2, fraction1);

					if (dist1 < dist2)
					{
						manifold.points[0].id.key = 0;
					}
					else
					{
						manifold.points[0].id.key = 1;
					}
				}
				else
				{
					manifold.points[0].localPoint = b2Lerp(capsuleB->point1, capsuleB->point2, fraction2);

					if (dist2 < dist1)
					{
						manifold.points[0].id.key = 0;
					}
					else
					{
						manifold.points[0].id.key = 1;
					}
				}
			}
			else
			{
				manifold.points[0].localPoint = b2Lerp(capsuleB->point1, capsuleB->point2, fraction1);
				manifold.points[1].localPoint = b2Lerp(capsuleB->point1, capsuleB->point2, fraction2);

				if (dist1 < dist2)
				{
					manifold.points[0].id.key = 0;
					manifold.points[1].id.key = 1;
				}
				else
				{
					manifold.points[0].id.key = 1;
					manifold.points[1].id.key = 0;
				}
			}
		}

		manifold.pointCount = tryClip ? 2 : 1;
		return manifold;
	}

	// All points clipped away, so use vertex-vertex contact

	manifold = b2EmptyManifold();
	manifold.type = b2_manifoldCircles;
	manifold.localNormal = b2Vec2_zero;
	manifold.pointCount = 1;

	// Find closest vertex pair

	float distPP = b2DistanceSquared(pB, pA);
	float minDist = distPP;
	manifold.localPoint = capsuleA->point1;
	manifold.points[0].localPoint = capsuleB->point1;
	manifold.points[0].id.key = 0;

	float distPQ = b2DistanceSquared(qB, pA);
	if (distPQ < minDist)
	{
		minDist = distPQ;
		manifold.localPoint = capsuleA->point1;
		manifold.points[0].localPoint = capsuleB->point2;
		manifold.points[0].id.key = 0;
	}

	float distQP = b2DistanceSquared(pB, qA);
	if (distQP < minDist)
	{
		minDist = distQP;
		manifold.localPoint = capsuleA->point2;
		manifold.points[0].localPoint = capsuleB->point1;
		manifold.points[0].id.key = 1;
	}

	float distQQ = b2DistanceSquared(qB, qA);
	if (distQQ < minDist)
	{
		minDist = distQQ;
		manifold.localPoint = capsuleA->point2;
		manifold.points[0].localPoint = capsuleB->point2;
		manifold.points[0].id.key = 1;
	}

	return manifold;
}
#endif

#if 0
// This doesn't work well when capsules are perpendicular.
// But it does have some simple clipping
b2Manifold b2CollideCapsules3(const b2Capsule* capsuleA, b2Transform xfA, const b2Capsule* capsuleB,
							 b2Transform xfB)
{
	b2Vec2 pA = b2TransformPoint(xfA, capsuleA->point1);
	b2Vec2 qA = b2TransformPoint(xfA, capsuleA->point2);

	b2Vec2 pB = b2TransformPoint(xfB, capsuleB->point1);
	b2Vec2 qB = b2TransformPoint(xfB, capsuleB->point2);

	b2Vec2 dA = b2Sub(qA, pA);
	b2Vec2 dB = b2Sub(qB, pB);

	b2Vec2 midAxis;
	if (b2Dot(dA, dB) < 0.0f)
	{
		midAxis = b2Sub(dA, dB);
	}
	else
	{
		midAxis = b2Add(dA, dB);
	}

	assert(b2Dot(midAxis, midAxis) > FLT_EPSILON);

	midAxis = b2Normalize(midAxis);

	float bound_pA = 0.0f;
	float bound_qA = b2Dot(dA, midAxis);
	float bound_pB = b2Dot(b2Sub(pB, pA), midAxis);
	float bound_qB = b2Dot(b2Sub(qB, pA), midAxis);

	float lowerA, upperA;
	if (bound_pA < bound_qA)
	{
		lowerA = bound_pA;
		upperA = bound_qA;
	}
	else
	{
		lowerA = bound_qA;
		upperA = bound_pA;
	}
	assert(upperA > lowerA);

	float lowerB, upperB;
	if (bound_pB < bound_qB)
	{
		lowerB = bound_pB;
		upperB = bound_qB;
	}
	else
	{
		lowerB = bound_qB;
		upperB = bound_pB;
	}
	assert(upperB > lowerB);

	if (upperB < lowerA || upperA < lowerB)
	{
		// All points clipped away, so use vertex-vertex contact

		b2Manifold manifold = b2EmptyManifold();
		manifold.type = b2_manifoldCircles;
		manifold.localNormal = b2Vec2_zero;
		manifold.pointCount = 1;

		// Find closest vertex pair

		float distPP = b2DistanceSquared(pB, pA);
		float minDist = distPP;
		manifold.localPoint = capsuleA->point1;
		manifold.points[0].localPoint = capsuleB->point1;
		manifold.points[0].id.key = 0;

		float distPQ = b2DistanceSquared(qB, pA);
		if (distPQ < minDist)
		{
			minDist = distPQ;
			manifold.localPoint = capsuleA->point1;
			manifold.points[0].localPoint = capsuleB->point2;
			manifold.points[0].id.key = 0;
		}

		float distQP = b2DistanceSquared(pB, qA);
		if (distQP < minDist)
		{
			minDist = distQP;
			manifold.localPoint = capsuleA->point2;
			manifold.points[0].localPoint = capsuleB->point1;
			manifold.points[0].id.key = 1;
		}

		float distQQ = b2DistanceSquared(qB, qA);
		if (distQQ < minDist)
		{
			minDist = distQQ;
			manifold.localPoint = capsuleA->point2;
			manifold.points[0].localPoint = capsuleB->point2;
			manifold.points[0].id.key = 1;
		}

		return manifold;
	}

	float lower = B2_MAX(lowerA, lowerB);
	float upper = B2_MIN(upperA, upperB);

	float denA = 1.0f / (bound_qA - bound_pA);
	b2Vec2 a1 = b2Lerp(pA, qA, (lower - bound_pA) * denA);
	b2Vec2 a2 = b2Lerp(pA, qA, (upper - bound_pA) * denA);

	float denB = 1.0f / (bound_qB - bound_pB);
	b2Vec2 b1 = b2Lerp(pB, qB, (lower - bound_pB) * denB);
	b2Vec2 b2 = b2Lerp(pB, qB, (upper - bound_pB) * denB);

	b2Vec2 normal = {midAxis.y, -midAxis.x};
	float sep1 = b2Dot(b2Sub(b1, a1), normal);
	float sep2 = b2Dot(b2Sub(b2, a2), normal);

	float sepPos = B2_MIN(sep1, sep2);
	float sepNeg = B2_MIN(-sep1, -sep2);
	b2Vec2 a;
	if (sepNeg > sepPos)
	{
		if (sep1 > sep2)
		{
			a = a1;
		}
		else
		{
			a = a2;
		}

		normal = b2Neg(normal);
	}
	else
	{
		if (sep1 < sep2)
		{
			a = a1;
		}
		else
		{
			a = a2;
		}
	}

	b2Manifold manifold = b2EmptyManifold();
	manifold.type = b2_manifoldFaceA;
	manifold.localPoint = b2InvTransformPoint(xfA, a);
	manifold.localNormal = b2InvRotateVector(xfA.q, normal);
	manifold.points[0].localPoint = b2InvTransformPoint(xfB, b1);
	manifold.points[1].localPoint = b2InvTransformPoint(xfB, b2);

	float dist1 = b2DistanceSquared(b1, pA);
	float dist2 = b2DistanceSquared(b2, pA);

	if (dist1 < dist2)
	{
		manifold.points[0].id.key = 0;
		manifold.points[1].id.key = 1;
	}
	else
	{
		manifold.points[0].id.key = 1;
		manifold.points[1].id.key = 0;
	}

	manifold.pointCount = 2;
	return manifold;
}
#endif

// TODO try O(n) algorithm in de Berg p. 279
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

// 1. Find edge normal of max separation on A
// 2. Find edge normal of max separation on B
// 3. Choose reference edge as min(minA, minB)
// 4. Find incident edge
// 5. Clip
// The normal points from A to B
b2Manifold b2CollidePolygons(const b2Polygon* polyA, b2Transform xfA, const b2Polygon* polyB, b2Transform xfB)
{
	b2Manifold manifold = b2EmptyManifold();

	int32_t edgeA = 0;
	float separationA = b2FindMaxSeparation(&edgeA, polyA, xfA, polyB, xfB);

	if (separationA > b2_speculativeDistance)
	{
		return manifold;
	}

	int32_t edgeB = 0;
	float separationB = b2FindMaxSeparation(&edgeB, polyB, xfB, polyA, xfA);

	if (separationB > b2_speculativeDistance)
	{
		return manifold;
	}

	const b2Polygon* poly1; // reference polygon
	const b2Polygon* poly2; // incident polygon
	b2Transform xf;
	int32_t edge1; // reference edge

	// TODO_ERIN feature flipping seems fine now
	const float k_tol = 0.0f; //0.1f * b2_linearSlop;
	bool flip;

	if (separationB > separationA + k_tol)
	{
		poly1 = polyB;
		poly2 = polyA;
		// take points in frame A into frame B
		xf = b2InvMulTransforms(xfB, xfA);
		edge1 = edgeB;
		flip = true;
	}
	else
	{
		poly1 = polyA;
		poly2 = polyB;
		// take points in frame B into frame A
		xf = b2InvMulTransforms(xfA, xfB);
		edge1 = edgeA;
		flip = false;
	}

	int32_t count1 = poly1->count;
	int32_t count2 = poly2->count;

	b2Vec2 normal = poly1->normals[edge1];

	// Get the normal of the reference edge in poly2's frame.
	b2Vec2 searchDirection = b2InvRotateVector(xf.q, normal);

	// Find the incident edge on poly2.
	const b2Vec2* normals2 = poly2->normals;
	int32_t edge2 = 0;
	float minDot = FLT_MAX;
	for (int32_t i = 0; i < count2; ++i)
	{
		float dot = b2Dot(searchDirection, normals2[i]);
		if (dot < minDot)
		{
			minDot = dot;
			edge2 = i;
		}
	}

	// Build the clip vertices for the incident edge.
	int32_t i21 = edge2;
	int32_t i22 = edge2 + 1 < count2 ? edge2 + 1 : 0;
	b2Vec2 v21 = b2TransformPoint(xf, poly2->vertices[i21]);
	b2Vec2 v22 = b2TransformPoint(xf, poly2->vertices[i22]);

	int32_t i11 = edge1;
	int32_t i12 = edge1 + 1 < count1 ? edge1 + 1 : 0;
	b2Vec2 v11 = poly1->vertices[i11];
	b2Vec2 v12 = poly1->vertices[i12];

	b2Vec2 tangent = b2CrossSV(1.0f, normal);

	float lower1 = 0.0f;
	float upper1 = b2Dot(b2Sub(v12, v11), tangent);

	// Incident edge should point opposite of tangent due to polygon CCW winding
	float upper2 = b2Dot(b2Sub(v21, v11), tangent);
	float lower2 = b2Dot(b2Sub(v22, v11), tangent);

	if (upper2 < lower1 || upper1 < lower2 || upper2 - lower2 < FLT_EPSILON)
	{
		return manifold;
	}

	b2Vec2 vLower;
	if (lower2 < lower1)
	{
		vLower = b2Lerp(v22, v21, (lower1 - lower2) / (upper2 - lower2));
	}
	else
	{
		vLower = v22;
	}

	b2Vec2 vUpper;
	if (upper2 > upper1)
	{
		vUpper = b2Lerp(v22, v21, (upper1 - lower2) / (upper2 - lower2));
	}
	else
	{
		vUpper = v21;
	}

	float separationLower = b2Dot(b2Sub(vLower, v11), normal);
	float separationUpper = b2Dot(b2Sub(vUpper, v11), normal);

	// Put contact points at midpoint
	vLower = b2MulSub(vLower, 0.5f * separationLower, normal);
	vUpper = b2MulSub(vUpper, 0.5f * separationUpper, normal);

	if (flip == false)
	{
		manifold.normal = b2RotateVector(xfA.q, normal);
		b2ManifoldPoint* cp = manifold.points + 0;

		if (separationLower <= b2_speculativeDistance)
		{
			cp->point = b2TransformPoint(xfA, vLower);
			cp->separation = separationLower;
			cp->id = B2_MAKE_ID(i11, i22);
			manifold.pointCount += 1;
			cp += 1;
		}
		
		if (separationUpper <= b2_speculativeDistance)
		{
			cp->point = b2TransformPoint(xfA, vUpper);
			cp->separation = separationUpper;
			cp->id = B2_MAKE_ID(i12, i21);
			manifold.pointCount += 1;
		}
	}
	else
	{
		manifold.normal = b2RotateVector(xfB.q, b2Neg(normal));
		b2ManifoldPoint* cp = manifold.points + 0;

		if (separationUpper <= b2_speculativeDistance)
		{
			cp->point = b2TransformPoint(xfB, vUpper);
			cp->separation = separationUpper;
			cp->id = B2_MAKE_ID(i21, i12);
			manifold.pointCount += 1;
			cp += 1;
		}

		if (separationLower <= b2_speculativeDistance)
		{
			cp->point = b2TransformPoint(xfB, vLower);
			cp->separation = separationLower;
			cp->id = B2_MAKE_ID(i22, i11);
			manifold.pointCount += 1;
		}
	}

	return manifold;
}

#if 0
b2Manifold b2CollideSegmentAndCircle(const b2Segment* segmentA, b2Transform xfA, const b2Circle* circleB,
									 b2Transform xfB)
{
	b2Manifold manifold = b2EmptyManifold();

	// Compute circle in frame of edge
	b2Vec2 Q = b2InvTransformPoint(xfA, b2TransformPoint(xfB, circleB->point));

	b2Vec2 A = segmentA->point1, B = segmentA->point2;
	b2Vec2 e = b2Sub(B, A);

	// Normal points to the right for a CCW winding
	b2Vec2 n = {e.y, -e.x};
	float offset = b2Dot(n, b2Sub(Q, A));

	// Barycentric coordinates
	float u = b2Dot(e, b2Sub(B, Q));
	float v = b2Dot(e, b2Sub(Q, A));

	b2ContactFeature cf;
	cf.indexB = 0;
	cf.typeB = b2_vertexFeature;

	// Region A
	if (v <= 0.0f)
	{
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
	if (offset < 0.0f)
	{
		n = b2Neg(n);
	}

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

b2Manifold b2CollideSegmentAndPolygon(const b2Segment* segmentA, b2Transform xfA, const b2Polygon* polygonB,
									  b2Transform xfB)
{
	b2Manifold manifold = b2EmptyManifold();

	b2Transform xf = b2InvMulTransforms(xfA, xfB);

	b2Vec2 v1 = segmentA->point1;
	b2Vec2 v2 = segmentA->point2;

	b2Vec2 edge1 = b2Normalize(b2Sub(v2, v1));

	// Normal points to the right for a CCW winding
	b2Vec2 normal1 = {edge1.y, -edge1.x};

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
