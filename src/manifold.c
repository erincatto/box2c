// SPDX-FileCopyrightText: 2022 Erin Catto
// SPDX-License-Identifier: MIT

#include "manifold.h"

#include "box2d/vec_math.h"

#include <assert.h>
#include <float.h>

void b2WorldManifold_Initialize(b2WorldManifold* wm, const b2Manifold* manifold, b2Transform xfA, float radiusA,
                                b2Transform xfB, float radiusB)
{
	if (manifold->pointCount == 0)
	{
		return;
	}

	switch (manifold->type)
	{
		case b2_manifoldCircles:
		{
			wm->normal = (b2Vec2){1.0f, 0.0f};
			b2Vec2 pointA = b2TransformPoint(xfA, manifold->localPoint);
			b2Vec2 pointB = b2TransformPoint(xfB, manifold->points[0].localPoint);
			if (b2DistanceSquared(pointA, pointB) > FLT_EPSILON * FLT_EPSILON)
			{
				wm->normal = b2Normalize(b2Sub(pointB, pointA));
			}

			b2Vec2 cA = b2MulAdd(pointA, radiusA, wm->normal);
			b2Vec2 cB = b2MulAdd(pointB, -radiusB, wm->normal);
			wm->points[0] = b2MulSV(0.5f, b2Add(cA, cB));
			wm->separations[0] = b2Dot(b2Sub(cB, cA), wm->normal);
		}
		break;

		case b2_manifoldFaceA:
		{
			wm->normal = b2RotateVector(xfA.q, manifold->localNormal);
			b2Vec2 planePoint = b2TransformPoint(xfA, manifold->localPoint);

			for (int32_t i = 0; i < manifold->pointCount; ++i)
			{
				b2Vec2 clipPoint = b2TransformPoint(xfB, manifold->points[i].localPoint);
				b2Vec2 cA =
					b2MulAdd(clipPoint, (radiusA - b2Dot(b2Sub(clipPoint, planePoint), wm->normal)), wm->normal);
				b2Vec2 cB = b2MulAdd(clipPoint, -radiusB, wm->normal);
				wm->points[i] = b2MulSV(0.5f, b2Add(cA, cB));
				wm->separations[i] = b2Dot(b2Sub(cB, cA), wm->normal);
			}
		}
		break;

		case b2_manifoldFaceB:
		{
			wm->normal = b2RotateVector(xfB.q, manifold->localNormal);
			b2Vec2 planePoint = b2TransformPoint(xfB, manifold->localPoint);

			for (int32_t i = 0; i < manifold->pointCount; ++i)
			{
				b2Vec2 clipPoint = b2TransformPoint(xfA, manifold->points[i].localPoint);
				b2Vec2 cB =
					b2MulAdd(clipPoint, (radiusB - b2Dot(b2Sub(clipPoint, planePoint), wm->normal)), wm->normal);
				b2Vec2 cA = b2MulAdd(clipPoint, -radiusA, wm->normal);
				wm->points[i] = b2MulSV(0.5f, b2Add(cA, cB));
				wm->separations[i] = b2Dot(b2Sub(cA, cB), wm->normal);
			}

			// Ensure normal points from A to B.
			wm->normal = b2Neg(wm->normal);
		}
		break;
	}
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

// Sutherland-Hodgman clipping.
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
