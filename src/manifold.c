// SPDX-FileCopyrightText: 2022 Erin Catto
// SPDX-License-Identifier: MIT

#include "box2d/manifold.h"
#include "box2d/math.h"
#include "box2d/shapes.h"

#include <assert.h>
#include <float.h>
#include <string.h>

static inline b2Manifold b2EmptyManifold()
{
	b2Manifold m;
	memset(&m, 0, sizeof(b2Manifold));
	return m;
}

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

void b2GetPointStates(b2PointState state1[b2_maxManifoldPoints], b2PointState state2[b2_maxManifoldPoints],
					  const b2Manifold* manifold1, const b2Manifold* manifold2)
{
	for (int32_t i = 0; i < b2_maxManifoldPoints; ++i)
	{
		state1[i] = b2_nullState;
		state2[i] = b2_nullState;
	}

	// Detect persists and removes.
	for (int32_t i = 0; i < manifold1->pointCount; ++i)
	{
		b2ContactID id = manifold1->points[i].id;

		state1[i] = b2_removeState;

		for (int32_t j = 0; j < manifold2->pointCount; ++j)
		{
			if (manifold2->points[j].id.key == id.key)
			{
				state1[i] = b2_persistState;
				break;
			}
		}
	}

	// Detect persists and adds.
	for (int32_t i = 0; i < manifold2->pointCount; ++i)
	{
		b2ContactID id = manifold2->points[i].id;

		state2[i] = b2_addState;

		for (int32_t j = 0; j < manifold1->pointCount; ++j)
		{
			if (manifold1->points[j].id.key == id.key)
			{
				state2[i] = b2_persistState;
				break;
			}
		}
	}
}

/// Used for computing contact manifolds.
typedef struct b2ClipVertex
{
	b2Vec2 v;
	b2ContactID id;
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
		vOut[count].id.cf.indexA = (uint8_t)vertexIndexA;
		vOut[count].id.cf.indexB = vIn[0].id.cf.indexB;
		vOut[count].id.cf.typeA = b2_vertexFeature;
		vOut[count].id.cf.typeB = b2_faceFeature;
		++count;

		assert(count == 2);
	}

	return count;
}

#if 0
bool b2TestOverlap(	const b2Shape* shapeA, int32_t indexA,
					const b2Shape* shapeB, int32_t indexB,
					b2Transform xfA, b2Transform xfB)
{
	b2DistanceInput input;
	input->proxyA.Set(shapeA, indexA);
	input->proxyB.Set(shapeB, indexB);
	input->transformA = xfA;
	input->transformB = xfB;
	input->useRadii = true;

	b2DistanceCache cache;
	cache.count = 0;

	b2DistanceOutput output;

	b2Distance(&output, &cache, &input);

	return output.distance < 10.0f * b2_epsilon;
}
#endif

b2Manifold b2CollideCircles(const b2CircleShape* circleA, const b2CircleShape* circleB)
{
	b2Manifold manifold = b2EmptyManifold();
	manifold.type = b2_manifoldCircles;
	manifold.localPoint = circleA->point;
	manifold.localNormal = (b2Vec2){0.0f, 0.0f};
	manifold.pointCount = 1;

	manifold.points[0].localPoint = circleB->point;
	manifold.points[0].id.key = 0;
	return manifold;
}

/// Compute the collision manifold between a capulse and circle
b2Manifold b2CollideCapsuleAndCircle(const b2CapsuleShape* capsuleA, b2Transform xfA, const b2CircleShape* circleB,
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

b2Manifold b2CollidePolygonAndCircle(const b2PolygonShape* polygonA, b2Transform xfA, const b2CircleShape* circleB,
									 b2Transform xfB)
{
	b2Manifold manifold = b2EmptyManifold();

	// Compute circle position in the frame of the polygon.
	b2Vec2 c = b2TransformPoint(xfB, circleB->point);
	b2Vec2 cLocal = b2InvTransformPoint(xfA, c);

	// Find the min separating edge.
	int32_t normalIndex = 0;
	float separation = -FLT_MAX;
	int32_t vertexCount = polygonA->count;
	const b2Vec2* vertices = polygonA->vertices;
	const b2Vec2* normals = polygonA->normals;

	for (int32_t i = 0; i < vertexCount; ++i)
	{
		float s = b2Dot(normals[i], b2Sub(cLocal, vertices[i]));
		if (s > separation)
		{
			separation = s;
			normalIndex = i;
		}
	}

	// Vertices that subtend the incident face.
	int32_t vertIndex1 = normalIndex;
	int32_t vertIndex2 = vertIndex1 + 1 < vertexCount ? vertIndex1 + 1 : 0;
	b2Vec2 v1 = vertices[vertIndex1];
	b2Vec2 v2 = vertices[vertIndex2];

	// If the center is inside the polygon ...
	if (separation < FLT_EPSILON)
	{
		manifold.pointCount = 1;
		manifold.type = b2_manifoldFaceA;
		manifold.localNormal = normals[normalIndex];
		manifold.localPoint = (b2Vec2){0.5f * v1.x + 0.5f * v2.x, 0.5f * v1.y + 0.5f * v2.y};
		manifold.points[0].localPoint = circleB->point;
		manifold.points[0].id.key = 0;
		return manifold;
	}

	// Compute barycentric coordinates
	float u1 = b2Dot(b2Sub(cLocal, v1), b2Sub(v2, v1));
	float u2 = b2Dot(b2Sub(cLocal, v2), b2Sub(v1, v2));
	if (u1 <= 0.0f)
	{
		manifold.pointCount = 1;
		manifold.type = b2_manifoldFaceA;
		manifold.localNormal = b2Normalize(b2Sub(cLocal, v1));
		manifold.localPoint = v1;
		manifold.points[0].localPoint = circleB->point;
		manifold.points[0].id.key = 0;
	}
	else if (u2 <= 0.0f)
	{
		manifold.pointCount = 1;
		manifold.type = b2_manifoldFaceA;
		manifold.localNormal = b2Normalize(b2Sub(cLocal, v2));
		manifold.localPoint = v2;
		manifold.points[0].localPoint = circleB->point;
		manifold.points[0].id.key = 0;
	}
	else
	{
		manifold.pointCount = 1;
		manifold.type = b2_manifoldFaceA;
		manifold.localNormal = normals[vertIndex1];
		manifold.localPoint = v1;
		manifold.points[0].localPoint = circleB->point;
		manifold.points[0].id.key = 0;
	}

	return manifold;
}

/// Compute the collision manifold between a capulse and circle
/// Follows Ericson 5.1.9 Closest Points of Two Line Segments
/// Algorithm:
/// - find closest points between segments
/// 
#if 0
b2Manifold b2CollideCapsules(const b2CapsuleShape* capsuleA, b2Transform xfA, const b2CapsuleShape* capsuleB,
							 b2Transform xfB)
{
	b2Vec2 pA = b2TransformPoint(xfA, capsuleA->point1);
	b2Vec2 qA = b2TransformPoint(xfA, capsuleA->point2);
	b2Vec2 dA = b2Sub(qA, pA);
	float a = b2Dot(dA, dA);

	b2Vec2 pB = b2TransformPoint(xfB, capsuleB->point1);
	b2Vec2 qB = b2TransformPoint(xfB, capsuleB->point2);
	b2Vec2 dB = b2Sub(qB, pB);
	float e = b2Dot(dB, dB);

	const float epsSqr = FLT_EPSILON * FLT_EPSILON;
	assert(a > epsSqr && e > epsSqr);

	b2Manifold manifold = b2EmptyManifold();
	if (a < epsSqr || e < epsSqr)
	{
		// degenerate capsule
		// todo handle this
		return manifold;
	}

	// Compute closest point between segments
	b2Vec2 r = b2Sub(pA, pB);
	float f = b2Dot(dB, r);
	float b = b2Dot(dA, dB);
	float c = b2Dot(dA, r);

	float denom = a * e - b * b;
	float s = 0.5f;
	if (denom > epsSqr)
	{
		// Fraction on segment A
		s = B2_CLAMP((b * f - c * e) / denom, 0.0f, 1.0f);
	}
	
	// Compute point on segment B closest to pA + s * dA
	float t = (b * s + f) / e;

	if (t < 0.0f)
	{
		t = 0.0f;
		s = B2_CLAMP(-c / a, 0.0f, 1.0f);
	}
	else if (t > 1.0f)
	{
		t = 1.0f;
		s = B2_CLAMP((b - c) / a, 0.0f, 1.0f);
	}

	// reference edge
	b2Vec2 pR, qR;

	// incident edge
	b2Vec2 pI, qI;

	int32_t key1, key2;

	b2Transform xfR, xfI;

	const b2CapsuleShape* capsuleR;
	const b2CapsuleShape* capsuleI;

	if (0.0f < s && s < 1.0f)
	{
		capsuleR = capsuleA;
		capsuleI = capsuleB;

		// segment A is the reference face
		manifold.type = b2_manifoldFaceA;

		key1 = 0;
		key2 = 1;

		pR = pA;
		qR = qA;
		pI = pB;
		qI = qB;
		xfR = xfA;
		xfI = xfB;
	}
	else if (0.0f < t && t < 1.0f)
	{
		capsuleR = capsuleB;
		capsuleI = capsuleA;

		// segment B is the reference face
		manifold.type = b2_manifoldFaceB;

		pR = pB;
		qR = qB;
		pI = pA;
		qI = qA;

		key1 = 2;
		key2 = 3;

		xfR = xfB;
		xfI = xfA;
	}
	else
	{
		manifold.type = b2_manifoldCircles;
		manifold.localNormal = b2Vec2_zero;
		manifold.pointCount = 1;

		if (s == 0.0f)
		{
			manifold.localPoint = capsuleA->point1;
			manifold.points[0].id.key = 0;
			if (t == 0.0f)
			{
				manifold.points[0].localPoint = capsuleB->point1;
			}
			else
			{
				manifold.points[0].localPoint = capsuleB->point2;
			}
		}
		else
		{
			manifold.localPoint = capsuleA->point2;
			manifold.points[0].id.key = 1;
			if (t == 0.0f)
			{
				manifold.points[0].localPoint = capsuleB->point1;
			}
			else
			{
				manifold.points[0].localPoint = capsuleB->point2;
			}
		}

		return manifold;
	}

	b2ClipVertex incidentEdge[2];
	incidentEdge[0].id.key = key1;
	incidentEdge[0].v = pI;
	incidentEdge[1].id.key = key2;
	incidentEdge[1].v = qI;

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
	b2ClipVertex clipPoints1[2];
	b2ClipVertex clipPoints2[2];
	int32_t np;

	// First side edge
	np = b2ClipSegmentToLine(clipPoints1, incidentEdge, b2Neg(axis), sideOffset1, 0);

	if (np < 2)
	{
		return manifold;
	}

	// Second side edge
	np = b2ClipSegmentToLine(clipPoints2, clipPoints1, axis, sideOffset2, 1);

	if (np < 2)
	{
		return manifold;
	}

	// Now clipPoints2 contains the clipped points.
	manifold.localNormal = localNormal;
	manifold.localPoint = capsuleR->point1;

	for (int32_t i = 0; i < b2_maxManifoldPoints; ++i)
	{
		b2ManifoldPoint* cp = manifold.points + i;
		cp->localPoint = b2InvTransformPoint(xfI, clipPoints2[i].v);
		cp->id = clipPoints2[i].id;
	}

	manifold.pointCount = b2_maxManifoldPoints;

	return manifold;

	#if 0
		float t1 = 0.0f, t2 = 1.0f;

		float distanceP = b2Dot(b2Sub(pB, pA), axis);
		float distanceQ = b2Dot(b2Sub(qB, pA), axis);


		if (distanceP < 0.0f)
		{
			if (0.0f < distanceQ && distanceQ < length)
			{
				// Find intersection point of edge and plane
				float interp = distanceP / (distanceP - distanceQ);

				manifold.points[0].localPoint = b2Lerp(capsuleB->point1, capsuleB->point2, interp);
				manifold.points[0].id.key = 0;

				manifold.points[1].localPoint = capsuleB->point2;
				manifold.points[1].id.key = 1;

				manifold.pointCount = 2;
			}
			else if (length < distanceQ)
			{
				float interp1 = -distanceQ / (distanceP - distanceQ);
				float interp2 = (length - distanceQ) / (distanceP - distanceQ);

				manifold.points[0].localPoint = b2Lerp(capsuleB->point2, capsuleB->point1, interp1);
				manifold.points[0].id.key = 0;

				manifold.points[1].localPoint = b2Lerp(capsuleB->point1, capsuleB->point2, interp2);
				manifold.points[1].id.key = 1;

				manifold.pointCount = 2;
			}
		}
		else if (0.0f <= distanceP && distanceP <= length)
		{
			if (distanceQ < 0.0f)
			{
				// Find intersection point of edge and plane
				float interp = distanceQ / (distanceQ - distanceP);

				manifold.points[0].localPoint = b2Lerp(capsuleB->point2, capsuleB->point1, interp);
				manifold.points[0].id.key = 0;

				manifold.points[1].localPoint = capsuleB->point1;
				manifold.points[1].id.key = 1;

				manifold.pointCount = 2;
			}
			else if (0.0f <= distanceQ && distanceQ <= length)
			{
				if (distanceQ > distanceP)
				{

					manifold.points[1].localPoint = capsuleB->point2;
					manifold.points[1].id.key = 1;

					manifold.pointCount = 2;
				}
				else
				{
					manifold.points[0].localPoint = capsuleB->point2;
					manifold.points[0].id.key = 0;

					manifold.points[1].localPoint = capsuleB->point1;
					manifold.points[1].id.key = 1;

					manifold.pointCount = 2;
				}			
			}
			else
			{
				// distanceQ > length
				float interp = (length - distanceP) / (distanceQ - distanceP);

				manifold.points[0].localPoint = capsuleB->point1;
				manifold.points[0].id.key = 0;

				manifold.points[1].localPoint = b2Lerp(capsuleB->point1, capsuleB->point2, interp);
				manifold.points[1].id.key = 1;

				manifold.pointCount = 2;
			}
		}
		else
		{
			// distanceP > length
			if (distanceQ < 0.0f)
			{
				float interp1 = -distanceQ / (distanceP - distanceQ);
				float interp2 = (length - distanceQ) / (distanceP - distanceQ);

				manifold.points[0].localPoint = b2Lerp(capsuleB->point2, capsuleB->point1, interp1);
				manifold.points[0].id.key = 0;

				manifold.points[1].localPoint = b2Lerp(capsuleB->point1, capsuleB->point2, interp2);
				manifold.points[1].id.key = 1;

				manifold.pointCount = 2;
			}
		}

		manifold.localNormal = normal;
	}
	else if (0.0f < t && t < 1.0f)
	{
		manifold.type = b2_manifoldFaceB;
	}

	{
		manifold.type = b2_manifoldCircles;
	}

	if (-FLT_EPSILON < den && den < FLT_EPSILON)
	{
		// parallel, check bounds
		float lowerA = 0.0f;
		float upperA = lengthA;

		float dot_pB = b2Dot(b2Sub(pB, pA), dA);
		float dot_qB = b2Dot(b2Sub(qB, pA), dA);

		if (dot_qB > dot_pB)
		{
			float lowerB = dot_pB;
			float upperB = dot_qB;

			if (upperB < lowerA)
			{
				manifold.type = b2_manifoldCircles;
				manifold.localPoint = capsuleA->point1;
				manifold.localNormal = b2Vec2_zero;
				manifold.pointCount = 1;
				manifold.points[0].localPoint = capsuleB->point2;
				manifold.points[0].id.key = 0;
				return manifold;
			}

			if (upperA < lowerB)
			{
				manifold.type = b2_manifoldCircles;
				manifold.localPoint = capsuleA->point2;
				manifold.localNormal = b2Vec2_zero;
				manifold.pointCount = 1;
				manifold.points[0].localPoint = capsuleB->point1;
				manifold.points[0].id.key = 1;
				return manifold;
			}

			// intervals overlap
			float lower = B2_MAX(lowerA, lowerB);
			float upper = B2_MIN(upperA, upperB);

			b2Vec2 normal = {dA.y, -dA.x};
			b2Vec2 centerB = b2MulSV(0.5f, b2Add(pB, qB));
			float side = b2Dot(b2Sub(centerB, pA), normal);
			if (side < 0.0f)
			{
				normal = b2Neg(normal);
			}

			b2Vec2 dBLocal = b2Normalize(b2Sub(capsuleB->point2, capsuleB->point1));
			manifold.type = b2_manifoldFaceA;
			manifold.localPoint = capsuleA->point1;
			manifold.localNormal = normal;
			manifold.pointCount = 2;
			manifold.points[0].localPoint = b2MulAdd(capsuleB->point1, lower - lowerB, dBLocal);
			manifold.points[0].id.key = 0;
			manifold.points[1].localPoint = b2MulAdd(capsuleB->point1, upper - lowerB, dBLocal);
			manifold.points[1].id.key = 1;

			return manifold;
		}
		else
		{
			float lowerB = dot_qB;
			float upperB = dot_pB;

			if (upperB < lowerA)
			{
				manifold.type = b2_manifoldCircles;
				manifold.localPoint = capsuleA->point1;
				manifold.localNormal = b2Vec2_zero;
				manifold.pointCount = 1;
				manifold.points[0].localPoint = capsuleB->point1;
				manifold.points[0].id.key = 0;
				return manifold;
			}

			if (upperA < lowerB)
			{
				manifold.type = b2_manifoldCircles;
				manifold.localPoint = capsuleA->point2;
				manifold.localNormal = b2Vec2_zero;
				manifold.pointCount = 1;
				manifold.points[0].localPoint = capsuleB->point2;
				manifold.points[0].id.key = 0;
				return manifold;
			}

			// intervals overlap
			float lower = B2_MAX(lowerA, lowerB);
			float upper = B2_MIN(upperA, upperB);

			b2Vec2 normal = {dA.y, -dA.x};
			b2Vec2 centerB = b2MulSV(0.5f, b2Add(pB, qB));
			float side = b2Dot(b2Sub(centerB, pA), normal);
			if (side < 0.0f)
			{
				normal = b2Neg(normal);
			}

			b2Vec2 dBLocal = b2Normalize(b2Sub(capsuleB->point1, capsuleB->point2));
			manifold.type = b2_manifoldFaceA;
			manifold.localPoint = capsuleA->point1;
			manifold.localNormal = normal;
			manifold.pointCount = 1;
			manifold.points[0].localPoint = b2MulAdd(capsuleB->point2, lower - lowerB, dBLocal);
			manifold.points[0].id.key = 0;
			manifold.points[1].localPoint = b2MulAdd(capsuleB->point2, upper - lowerB, dBLocal);
			manifold.points[1].id.key = 1;
		}
	}

	b2Vec2 rhs = b2Sub(pB, pA);

	// Cramer's rule [rhs -dB]
	float a = (-rhs.x * dB.y + dB.x * rhs.y) / den;

	// Cramer's rule [dA rhs]
	float b = (dA.x * rhs.y - rhs.x * dA.y) / den;

	if (0.0f < a && a < lengthA)
	{
		// face A contact?
	}
	else if (0.0f < b && b < lengthB)
	{
		// face B contact?
	}
	else
	{
		// vertex contact
	}
	#endif
}
#endif

// Algorithm
// - find best edge separating axis
// - clip incident edge
// - if there are no points, then find best vertex-vertex
b2Manifold b2CollideCapsules(const b2CapsuleShape* capsuleA, b2Transform xfA, const b2CapsuleShape* capsuleB,
							 b2Transform xfB)
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
	int32_t signA;
	{
		float sepP = b2Dot(b2Sub(pB, pA), normalA);
		float sepQ = b2Dot(b2Sub(qB, pA), normalA);
		float sepPos = B2_MIN(sepP, sepQ);
		float sepNeg = B2_MIN(-sepP, -sepQ);
		if (sepNeg > sepPos)
		{
			sepA = sepNeg;
			normalA = b2Neg(normalA);
			signA = -1;
		}
		else
		{
			sepA = sepPos;
			signA = 1;
		}
	}

	// face B
	float sepB;
	int32_t signB;
	{
		float sepP = b2Dot(b2Sub(pA, pB), normalB);
		float sepQ = b2Dot(b2Sub(qA, pB), normalB);
		float sepPos = B2_MIN(sepP, sepQ);
		float sepNeg = B2_MIN(-sepP, -sepQ);
		if (sepNeg > sepPos)
		{
			sepB = sepNeg;
			normalB = b2Neg(normalB);
			signB = -1;
		}
		else
		{
			sepB = sepPos;
			signB = 1;
		}
	}

	const float k_faceTol = 0.1f * b2_linearSlop;

	b2Manifold manifold = b2EmptyManifold();

	// reference edge
	b2Vec2 pR, qR;

	// incident edge
	b2Vec2 pI, qI;

	int32_t key1, key2;

	b2Transform xfR, xfI;

	const b2CapsuleShape* capsuleR;
	const b2CapsuleShape* capsuleI;

	if (sepB > sepA + k_faceTol)
	{
		capsuleR = capsuleB;
		capsuleI = capsuleA;

		// segment B is the reference face
		manifold.type = b2_manifoldFaceB;

		pR = pB;
		qR = qB;
		pI = pA;
		qI = qA;

		key1 = 2;
		key2 = 3;

		xfR = xfB;
		xfI = xfA;
	}
	else
	{
		capsuleR = capsuleA;
		capsuleI = capsuleB;

		// segment A is the reference face
		manifold.type = b2_manifoldFaceA;

		key1 = 0;
		key2 = 1;

		pR = pA;
		qR = qA;
		pI = pB;
		qI = qB;
		xfR = xfA;
		xfI = xfB;
	}

	b2ClipVertex incidentEdge[2];
	incidentEdge[0].id.key = key1;
	incidentEdge[0].v = pI;
	incidentEdge[1].id.key = key2;
	incidentEdge[1].v = qI;

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
	b2ClipVertex clipPoints1[2];
	b2ClipVertex clipPoints2[2];
	int32_t np;

	static_assert(b2_maxManifoldPoints == 2, "b2_maxManifoldPoints != 2");

	// First side edge
	np = b2ClipSegmentToLine(clipPoints1, incidentEdge, b2Neg(axis), sideOffset1, 0);

	if (np == 2)
	{
		// Second side edge
		np = b2ClipSegmentToLine(clipPoints2, clipPoints1, axis, sideOffset2, 1);
	}

	if (np == 2)
	{
		// Now clipPoints2 contains the clipped points.
		manifold.localNormal = localNormal;
		manifold.localPoint = capsuleR->point1;

		for (int32_t i = 0; i < 2; ++i)
		{
			b2ManifoldPoint* cp = manifold.points + i;
			cp->localPoint = b2InvTransformPoint(xfI, clipPoints2[i].v);
			cp->id = clipPoints2[i].id;
		}

		manifold.pointCount = 2;
		return manifold;
	}

	manifold.type = b2_manifoldCircles;
	manifold.localNormal = b2Vec2_zero;
	manifold.pointCount = 1;

	float distPP = b2DistanceSquared(pB, pA);
	float minDist = distPP;
	manifold.localPoint = capsuleA->point1;
	manifold.points[0].localPoint = capsuleB->point1;

	float distPQ = b2DistanceSquared(qB, pA);
	if (distPQ < minDist)
	{
		minDist = distPQ;
		manifold.localPoint = capsuleA->point1;
		manifold.points[0].localPoint = capsuleB->point2;
	}

	float distQP = b2DistanceSquared(pB, qA);
	if (distQP < minDist)
	{
		minDist = distQP;
		manifold.localPoint = capsuleA->point2;
		manifold.points[0].localPoint = capsuleB->point1;
	}

	float distQQ = b2DistanceSquared(qB, qA);
	if (distQQ < minDist)
	{
		minDist = distQQ;
		manifold.localPoint = capsuleA->point2;
		manifold.points[0].localPoint = capsuleB->point2;
	}

	return manifold;

	#if 0
	int32_t typeVertex = -1;

	// vertex P-P
	float lengthPP;
	b2Vec2 normalPP = b2GetLengthAndNormalize(&lengthPP, b2Sub(pB, pA));
	if (lengthPP > FLT_EPSILON)
	{
		// The other endpoints must point away
		float checkA = b2Dot(b2Sub(qA, pA), normalPP);
		float checkB = b2Dot(b2Sub(qB, pB), normalPP);
		if (checkA < 0.0f && checkB > 0.0f)
		{
			sepVertex = lengthPP;
			typeVertex = 0;
		}
	}

	// vertex P-Q
	float lengthPQ;
	b2Vec2 normalPQ = b2GetLengthAndNormalize(&lengthPQ, b2Sub(qB, pA));
	if (lengthPQ > FLT_EPSILON && lengthPQ < sepVertex)
	{
		// The other endpoints must point away
		float checkA = b2Dot(b2Sub(qA, pA), normalPQ);
		float checkB = b2Dot(b2Sub(pB, qB), normalPQ);
		if (checkA < 0.0f && checkB > 0.0f)
		{
			sepVertex = lengthPQ;
			typeVertex = 1;
		}
	}

	// vertex Q-P
	float lengthQP;
	b2Vec2 normalQP = b2GetLengthAndNormalize(&lengthQP, b2Sub(pB, qA));
	if (lengthQP > FLT_EPSILON && lengthQP < sepVertex)
	{
		// The other endpoints must point away
		float checkA = b2Dot(b2Sub(pA, qA), normalQP);
		float checkB = b2Dot(b2Sub(qB, pB), normalQP);
		if (checkA < 0.0f && checkB > 0.0f)
		{
			sepVertex = lengthQP;
			typeVertex = 2;
		}
	}

	float lengthQQ;
	b2Vec2 normalQQ = b2GetLengthAndNormalize(&lengthQQ, b2Sub(qB, qA));
	if (lengthQQ > FLT_EPSILON && lengthQQ < sepVertex)
	{
		// The other endpoints must point away
		float checkA = b2Dot(b2Sub(qA, qA), normalQQ);
		float checkB = b2Dot(b2Sub(qB, qB), normalQQ);
		if (checkA < 0.0f && checkB > 0.0f)
		{
			sepVertex = lengthQQ;
			typeVertex = 3;
		}
	}

	return b2EmptyManifold();
	#endif
}


	// TODO try O(n) algorithm in de Berg p. 279

// Find the max separation between poly1 and poly2 using edge normals from poly1.
static float b2FindMaxSeparation(int32_t* edgeIndex, const b2PolygonShape* poly1, b2Transform xf1,
								 const b2PolygonShape* poly2, b2Transform xf2)
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

static void b2FindIncidentEdge(b2ClipVertex c[2], const b2PolygonShape* poly1, b2Transform xf1, int32_t edge1,
							   const b2PolygonShape* poly2, b2Transform xf2)
{
	const b2Vec2* normals1 = poly1->normals;

	int32_t count2 = poly2->count;
	const b2Vec2* vertices2 = poly2->vertices;
	const b2Vec2* normals2 = poly2->normals;

	assert(0 <= edge1 && edge1 < poly1->count);

	// Get the normal of the reference edge in poly2's frame.
	b2Vec2 normal1 = b2InvRotateVector(xf2.q, b2RotateVector(xf1.q, normals1[edge1]));

	// Find the incident edge on poly2.
	int32_t index = 0;
	float minDot = FLT_MAX;
	for (int32_t i = 0; i < count2; ++i)
	{
		float dot = b2Dot(normal1, normals2[i]);
		if (dot < minDot)
		{
			minDot = dot;
			index = i;
		}
	}

	// Build the clip vertices for the incident edge.
	int32_t i1 = index;
	int32_t i2 = i1 + 1 < count2 ? i1 + 1 : 0;

	c[0].v = b2TransformPoint(xf2, vertices2[i1]);
	c[0].id.cf.indexA = (uint8_t)edge1;
	c[0].id.cf.indexB = (uint8_t)i1;
	c[0].id.cf.typeA = b2_faceFeature;
	c[0].id.cf.typeB = b2_vertexFeature;

	c[1].v = b2TransformPoint(xf2, vertices2[i2]);
	c[1].id.cf.indexA = (uint8_t)edge1;
	c[1].id.cf.indexB = (uint8_t)i2;
	c[1].id.cf.typeA = b2_faceFeature;
	c[1].id.cf.typeB = b2_vertexFeature;
}

// Find edge normal of max separation on A - return if separating axis is found
// Find edge normal of max separation on B - return if separation axis is found
// Choose reference edge as min(minA, minB)
// Find incident edge
// Clip

// The normal points from A to B
b2Manifold b2CollidePolygons(const b2PolygonShape* polyA, b2Transform xfA, const b2PolygonShape* polyB, b2Transform xfB)
{
	b2Manifold manifold = b2EmptyManifold();

	int32_t edgeA = 0;
	float separationA = b2FindMaxSeparation(&edgeA, polyA, xfA, polyB, xfB);

	int32_t edgeB = 0;
	float separationB = b2FindMaxSeparation(&edgeB, polyB, xfB, polyA, xfA);

	const b2PolygonShape* poly1; // reference polygon
	const b2PolygonShape* poly2; // incident polygon
	b2Transform xf1, xf2;
	int32_t edge1; // reference edge
	uint8_t flip;
	const float k_tol = 0.1f * b2_linearSlop;

	if (separationB > separationA + k_tol)
	{
		poly1 = polyB;
		poly2 = polyA;
		xf1 = xfB;
		xf2 = xfA;
		edge1 = edgeB;
		manifold.type = b2_manifoldFaceB;
		flip = 1;
	}
	else
	{
		poly1 = polyA;
		poly2 = polyB;
		xf1 = xfA;
		xf2 = xfB;
		edge1 = edgeA;
		manifold.type = b2_manifoldFaceA;
		flip = 0;
	}

	b2ClipVertex incidentEdge[2];
	b2FindIncidentEdge(incidentEdge, poly1, xf1, edge1, poly2, xf2);

	int32_t count1 = poly1->count;
	const b2Vec2* vertices1 = poly1->vertices;

	int32_t iv1 = edge1;
	int32_t iv2 = edge1 + 1 < count1 ? edge1 + 1 : 0;

	b2Vec2 v11 = vertices1[iv1];
	b2Vec2 v12 = vertices1[iv2];

	b2Vec2 localTangent = b2Normalize(b2Sub(v12, v11));

	b2Vec2 localNormal = b2CrossVS(localTangent, 1.0f);
	b2Vec2 planePoint = b2Lerp(v11, v12, 0.5f);

	b2Vec2 tangent = b2RotateVector(xf1.q, localTangent);

	v11 = b2TransformPoint(xf1, v11);
	v12 = b2TransformPoint(xf1, v12);

	// Side offsets for clipping
	float sideOffset1 = -b2Dot(tangent, v11);
	float sideOffset2 = b2Dot(tangent, v12);

	// Clip incident edge against extruded edge1 side edges.
	b2ClipVertex clipPoints1[2];
	b2ClipVertex clipPoints2[2];
	int np;

	// TODO stabilize ids by using only vertex indices?

	// First side edge
	np = b2ClipSegmentToLine(clipPoints1, incidentEdge, b2Neg(tangent), sideOffset1, iv1);

	if (np < 2)
	{
		return manifold;
	}

	// Second side edge
	np = b2ClipSegmentToLine(clipPoints2, clipPoints1, tangent, sideOffset2, iv2);

	if (np < 2)
	{
		return manifold;
	}

	// Now clipPoints2 contains the clipped points.
	manifold.localNormal = localNormal;
	manifold.localPoint = planePoint;

	for (int32_t i = 0; i < b2_maxManifoldPoints; ++i)
	{
		b2ManifoldPoint* cp = manifold.points + i;
		cp->localPoint = b2InvTransformPoint(xf2, clipPoints2[i].v);
		cp->id = clipPoints2[i].id;
		if (flip)
		{
			// Swap features
			b2ContactFeature cf = cp->id.cf;
			cp->id.cf.indexA = cf.indexB;
			cp->id.cf.indexB = cf.indexA;
			cp->id.cf.typeA = cf.typeB;
			cp->id.cf.typeB = cf.typeA;
		}
	}

	manifold.pointCount = b2_maxManifoldPoints;

	return manifold;
}

b2Manifold b2CollideSegmentAndCircle(const b2SegmentShape* segmentA, b2Transform xfA, const b2CircleShape* circleB,
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

b2Manifold b2CollideSmoothSegmentAndCircle(const b2SmoothSegmentShape* segmentA, b2Transform xfA,
										   const b2CircleShape* circleB, b2Transform xfB)
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

b2Manifold b2CollideSegmentAndPolygon(const b2SegmentShape* segmentA, b2Transform xfA, const b2PolygonShape* polygonB,
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

b2Manifold b2CollideSmoothSegmentAndPolygon(const b2SmoothSegmentShape* segmentA, b2Transform xfA,
											const b2PolygonShape* polygonB, b2Transform xfB)
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
