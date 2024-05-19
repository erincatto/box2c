// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "api.h"
#include "types.h"

typedef struct b2Circle b2Circle;
typedef struct b2Capsule b2Capsule;
typedef struct b2DistanceCache b2DistanceCache;
typedef struct b2Polygon b2Polygon;
typedef struct b2Segment b2Segment;
typedef struct b2SmoothSegment b2SmoothSegment;

/**
 * @defgroup collision Collision
 * @brief Functions for colliding pairs of shapes
 * @{
 */

/// A manifold point is a contact point belonging to a contact
/// manifold. It holds details related to the geometry and dynamics
/// of the contact points.
typedef struct b2ManifoldPoint
{
	/// Location of the contact point in world space. Subject to precision loss at large coordinates.
	///	@note Should only be used for debugging.
	b2Vec2 point;

	/// Location of contact point relative to body origin in world space.
	///	@note When used internally to the Box2D solver, these are relative to the center of mass.
	b2Vec2 anchorA, anchorB;

	/// The separation of the contact point, negative if penetrating
	float separation;

	/// The impulse along the manifold normal vector.
	float normalImpulse;

	/// The friction impulse
	float tangentImpulse;

	/// The maximum normal impulse applied during sub-stepping
	///	todo not sure this is needed
	float maxNormalImpulse;

	/// Relative normal velocity pre-solve. Used for hit events. If the normal impulse is
	/// zero then there was no hit. Negative means shapes are approaching.
	float normalVelocity;

	/// Uniquely identifies a contact point between two shapes
	uint16_t id;

	/// Did this contact point exist the previous step?
	bool persisted;
} b2ManifoldPoint;

/// A contact manifold describes the contact points between colliding shapes
typedef struct b2Manifold
{
	/// The manifold points, up to two are possible in 2D
	b2ManifoldPoint points[2];

	/// The unit normal vector in world space, points from shape A to bodyB
	b2Vec2 normal;

	/// The number of contacts points, will be 0, 1, or 2
	int32_t pointCount;
} b2Manifold;

/// Compute the contact manifold between two circles
B2_API b2Manifold b2CollideCircles(const b2Circle* circleA, b2Transform xfA, const b2Circle* circleB, b2Transform xfB);

/// Compute the contact manifold between a capsule and circle
B2_API b2Manifold b2CollideCapsuleAndCircle(const b2Capsule* capsuleA, b2Transform xfA, const b2Circle* circleB, b2Transform xfB);

/// Compute the contact manifold between an segment and a circle
B2_API b2Manifold b2CollideSegmentAndCircle(const b2Segment* segmentA, b2Transform xfA, const b2Circle* circleB, b2Transform xfB);

/// Compute the contact manifold between a polygon and a circle
B2_API b2Manifold b2CollidePolygonAndCircle(const b2Polygon* polygonA, b2Transform xfA, const b2Circle* circleB, b2Transform xfB);

/// Compute the contact manifold between a capsule and circle
B2_API b2Manifold b2CollideCapsules(const b2Capsule* capsuleA, b2Transform xfA, const b2Capsule* capsuleB, b2Transform xfB,
									b2DistanceCache* cache);

/// Compute the contact manifold between an segment and a capsule
B2_API b2Manifold b2CollideSegmentAndCapsule(const b2Segment* segmentA, b2Transform xfA, const b2Capsule* capsuleB,
											 b2Transform xfB, b2DistanceCache* cache);

/// Compute the contact manifold between a polygon and capsule
B2_API b2Manifold b2CollidePolygonAndCapsule(const b2Polygon* polygonA, b2Transform xfA, const b2Capsule* capsuleB,
											 b2Transform xfB, b2DistanceCache* cache);

/// Compute the contact manifold between two polygons
B2_API b2Manifold b2CollidePolygons(const b2Polygon* polyA, b2Transform xfA, const b2Polygon* polyB, b2Transform xfB,
									b2DistanceCache* cache);

/// Compute the contact manifold between an segment and a polygon
B2_API b2Manifold b2CollideSegmentAndPolygon(const b2Segment* segmentA, b2Transform xfA, const b2Polygon* polygonB,
											 b2Transform xfB, b2DistanceCache* cache);

/// Compute the contact manifold between a smooth segment and a circle
B2_API b2Manifold b2CollideSmoothSegmentAndCircle(const b2SmoothSegment* smoothSegmentA, b2Transform xfA, const b2Circle* circleB,
												  b2Transform xfB);

/// Compute the contact manifold between an segment and a capsule
B2_API b2Manifold b2CollideSmoothSegmentAndCapsule(const b2SmoothSegment* smoothSegmentA, b2Transform xfA,
												   const b2Capsule* capsuleB, b2Transform xfB, b2DistanceCache* cache);

/// Compute the contact manifold between a smooth segment and a rounded polygon
B2_API b2Manifold b2CollideSmoothSegmentAndPolygon(const b2SmoothSegment* smoothSegmentA, b2Transform xfA,
												   const b2Polygon* polygonB, b2Transform xfB, b2DistanceCache* cache);

/**@}*/
