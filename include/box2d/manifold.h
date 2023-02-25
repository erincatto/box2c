// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "box2d/types.h"

#define b2_nullFeature UCHAR_MAX
#define b2_maxManifoldPoints 2

typedef struct b2Circle b2Circle;
typedef struct b2Capsule b2Capsule;
typedef struct b2Polygon b2Polygon;
typedef struct b2Segment b2Segment;
typedef struct b2SmoothSegment b2SmoothSegment;

/// A manifold point is a contact point belonging to a contact
/// manifold. It holds details related to the geometry and dynamics
/// of the contact points.
typedef struct b2ManifoldPoint
{
	/// world coordinates of contact point
	b2Vec2 point;

	/// the separation of the contact point, negative if penetrating
	float separation;

	/// the non-penetration impulse
	float normalImpulse;

	/// the friction impulse
	float tangentImpulse;

	/// uniquely identifies a contact point between two shapes
	uint16_t id;

	/// did this contact point exist the previous step?
	bool persisted;
} b2ManifoldPoint;

/// Conact manifold convex shapes.
typedef struct b2Manifold
{
	b2ManifoldPoint points[b2_maxManifoldPoints];
	b2Vec2 normal;
	int32_t pointCount;
} b2Manifold;

#ifdef __cplusplus
extern "C"
{
#endif

static inline b2Manifold b2EmptyManifold(void)
{
	b2Manifold m = {0};
	return m;
}

/// Compute the collision manifold between two circles.
b2Manifold b2CollideCircles(const b2Circle* circleA, const b2Circle* circleB);

/// Compute the collision manifold between a capulse and circle
b2Manifold b2CollideCapsuleAndCircle(const b2Capsule* capsuleA, b2Transform xfA, const b2Circle* circleB,
									 b2Transform xfB);

/// Compute the collision manifold between an segment and a circle.
b2Manifold b2CollideSegmentAndCircle(const b2Segment* segmentA, b2Transform xfA, const b2Circle* circleB,
									 b2Transform xfB);

/// Compute the collision manifold between a smooth segment and a circle.
b2Manifold b2CollideSmoothSegmentAndCircle(const b2SmoothSegment* segmentA, b2Transform xfA,
										   const b2Circle* circleB, b2Transform xfB);

/// Compute the collision manifold between a polygon and a circle.
b2Manifold b2CollidePolygonAndCircle(const b2Polygon* polygonA, b2Transform xfA, const b2Circle* circleB,
									 b2Transform xfB);

/// Compute the collision manifold between a capulse and circle
b2Manifold b2CollideCapsules(const b2Capsule* capsuleA, b2Transform xfA, const b2Capsule* capsuleB,
									 b2Transform xfB);

// TODO temp
b2Manifold b2CollideCapsules2(const b2Capsule* capsuleA, b2Transform xfA, const b2Capsule* capsuleB,
							  b2Transform xfB);

	/// Compute the collision manifold between an segment and a capsule.
b2Manifold b2CollideSegmentAndCapsule(const b2Segment* segmentA, b2Transform xfA, const b2Capsule* capsuleB,
									 b2Transform xfB);

/// Compute the collision manifold between a smooth segment and a capsule.
b2Manifold b2CollideSmoothSegmentAndCapsule(const b2SmoothSegment* segmentA, b2Transform xfA,
										   const b2Circle* capsuleB, b2Transform xfB);

/// Compute the collision manifold between a polygon and capsule
b2Manifold b2CollidePolygonAndCapsule(const b2Polygon* polygonA, b2Transform xfA, const b2Capsule* capsuleB,
									 b2Transform xfB);

/// Compute the collision manifold between two polygons.
b2Manifold b2CollidePolygons(const b2Polygon* polygonA, b2Transform xfA, const b2Polygon* polygonB,
							 b2Transform xfB);

/// Compute the collision manifold between an segment and a polygon.
b2Manifold b2CollideSegmentAndPolygon(const b2Segment* segmentA, b2Transform xfA, const b2Polygon* polygonB,
									  b2Transform xfB);

/// Compute the collision manifold between a smooth segment and a polygon.
b2Manifold b2CollideSmoothSegmentAndPolygon(const b2SmoothSegment* segmentA, b2Transform xfA,
											const b2Polygon* polygonB, b2Transform xfB);

#ifdef __cplusplus
}
#endif
