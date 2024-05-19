// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "base.h"
#include "math_functions.h"

#include <stdbool.h>

typedef struct b2Circle b2Circle;
typedef struct b2Capsule b2Capsule;
typedef struct b2DistanceCache b2DistanceCache;
typedef struct b2Polygon b2Polygon;
typedef struct b2Segment b2Segment;
typedef struct b2SmoothSegment b2SmoothSegment;

typedef struct b2Hull b2Hull;

/**
 * @defgroup geometry Geometry
 * @brief Geometry types and algorithms
 *
 * Definitions of circles, capsules, segments, and polygons. Various algorithms to compute hulls, mass properties, and so on.
 * @{
 */

/// The maximum number of vertices on a convex polygon. Changing this affects performance even if you
///	don't use more vertices.
#define b2_maxPolygonVertices 8

/// Low level ray-cast input data
typedef struct b2RayCastInput
{
	b2Vec2 origin, translation;
	float maxFraction;
} b2RayCastInput;

/// Low level shape cast input in generic form
typedef struct b2ShapeCastInput
{
	b2Vec2 points[b2_maxPolygonVertices];
	int32_t count;
	float radius;
	b2Vec2 translation;
	float maxFraction;
} b2ShapeCastInput;

/// Low level ray-cast or shape-cast output data
typedef struct b2CastOutput
{
	b2Vec2 normal;
	b2Vec2 point;
	float fraction;
	int32_t iterations;
	bool hit;
} b2CastOutput;

/// This holds the mass data computed for a shape.
typedef struct b2MassData
{
	/// The mass of the shape, usually in kilograms.
	float mass;

	/// The position of the shape's centroid relative to the shape's origin.
	b2Vec2 center;

	/// The rotational inertia of the shape about the local origin.
	float I;
} b2MassData;

/// A solid circle
typedef struct b2Circle
{
	b2Vec2 center;
	float radius;
} b2Circle;

/// A solid capsule
typedef struct b2Capsule
{
	b2Vec2 center1, center2;
	float radius;
} b2Capsule;

/// A solid convex polygon. It is assumed that the interior of the polygon is to
/// the left of each edge.
/// Polygons have a maximum number of vertices equal to b2_maxPolygonVertices.
/// In most cases you should not need many vertices for a convex polygon.
///	@warning DO NOT fill this out manually, instead use a helper function like
///	b2MakePolygon or b2MakeBox.
typedef struct b2Polygon
{
	b2Vec2 vertices[b2_maxPolygonVertices];
	b2Vec2 normals[b2_maxPolygonVertices];
	b2Vec2 centroid;
	float radius;
	int32_t count;
} b2Polygon;

/// A line segment with two-sided collision.
typedef struct b2Segment
{
	b2Vec2 point1, point2;
} b2Segment;

/// A smooth line segment with one-sided collision. Only collides on the right side.
/// Several of these are generated for a chain shape.
/// ghost1 -> point1 -> point2 -> ghost2
typedef struct b2SmoothSegment
{
	/// The tail ghost vertex
	b2Vec2 ghost1;

	/// The line segment
	b2Segment segment;

	/// The head ghost vertex
	b2Vec2 ghost2;

	/// The owning chain shape index (internal usage only)
	int32_t chainId;
} b2SmoothSegment;

/// Validate ray cast input data (NaN, etc)
B2_API bool b2IsValidRay(const b2RayCastInput* input);

/// Make a convex polygon from a convex hull. This will assert if the hull is not valid.
B2_API b2Polygon b2MakePolygon(const b2Hull* hull, float radius);

/// Make an offset convex polygon from a convex hull. This will assert if the hull is not valid.
B2_API b2Polygon b2MakeOffsetPolygon(const b2Hull* hull, float radius, b2Transform transform);

/// Make a square polygon, bypassing the need for a convex hull.
B2_API b2Polygon b2MakeSquare(float h);

/// Make a box (rectangle) polygon, bypassing the need for a convex hull.
B2_API b2Polygon b2MakeBox(float hx, float hy);

/// Make a rounded box, bypassing the need for a convex hull.
B2_API b2Polygon b2MakeRoundedBox(float hx, float hy, float radius);

/// Make an offset box, bypassing the need for a convex hull.
B2_API b2Polygon b2MakeOffsetBox(float hx, float hy, b2Vec2 center, float angle);

/// Transform a polygon. This is useful for transferring a shape from one body to another.
B2_API b2Polygon b2TransformPolygon(b2Transform transform, const b2Polygon* polygon);

/// Compute mass properties of a circle
B2_API b2MassData b2ComputeCircleMass(const b2Circle* shape, float density);

/// Compute mass properties of a capsule
B2_API b2MassData b2ComputeCapsuleMass(const b2Capsule* shape, float density);

/// Compute mass properties of a polygon
B2_API b2MassData b2ComputePolygonMass(const b2Polygon* shape, float density);

/// Compute the bounding box of a transformed circle
B2_API b2AABB b2ComputeCircleAABB(const b2Circle* shape, b2Transform transform);

/// Compute the bounding box of a transformed capsule
B2_API b2AABB b2ComputeCapsuleAABB(const b2Capsule* shape, b2Transform transform);

/// Compute the bounding box of a transformed polygon
B2_API b2AABB b2ComputePolygonAABB(const b2Polygon* shape, b2Transform transform);

/// Compute the bounding box of a transformed line segment
B2_API b2AABB b2ComputeSegmentAABB(const b2Segment* shape, b2Transform transform);

/// Test a point for overlap with a circle in local space
B2_API bool b2PointInCircle(b2Vec2 point, const b2Circle* shape);

/// Test a point for overlap with a capsule in local space
B2_API bool b2PointInCapsule(b2Vec2 point, const b2Capsule* shape);

/// Test a point for overlap with a convex polygon in local space
B2_API bool b2PointInPolygon(b2Vec2 point, const b2Polygon* shape);

/// Ray cast versus circle in shape local space. Initial overlap is treated as a miss.
B2_API b2CastOutput b2RayCastCircle(const b2RayCastInput* input, const b2Circle* shape);

/// Ray cast versus capsule in shape local space. Initial overlap is treated as a miss.
B2_API b2CastOutput b2RayCastCapsule(const b2RayCastInput* input, const b2Capsule* shape);

/// Ray cast versus segment in shape local space. Optionally treat the segment as one-sided with hits from
/// the left side being treated as a miss.
B2_API b2CastOutput b2RayCastSegment(const b2RayCastInput* input, const b2Segment* shape, bool oneSided);

/// Ray cast versus polygon in shape local space. Initial overlap is treated as a miss.
B2_API b2CastOutput b2RayCastPolygon(const b2RayCastInput* input, const b2Polygon* shape);

/// Shape cast versus a circle. Initial overlap is treated as a miss.
B2_API b2CastOutput b2ShapeCastCircle(const b2ShapeCastInput* input, const b2Circle* shape);

/// Shape cast versus a capsule. Initial overlap is treated as a miss.
B2_API b2CastOutput b2ShapeCastCapsule(const b2ShapeCastInput* input, const b2Capsule* shape);

/// Shape cast versus a line segment. Initial overlap is treated as a miss.
B2_API b2CastOutput b2ShapeCastSegment(const b2ShapeCastInput* input, const b2Segment* shape);

/// Shape cast versus a convex polygon. Initial overlap is treated as a miss.
B2_API b2CastOutput b2ShapeCastPolygon(const b2ShapeCastInput* input, const b2Polygon* shape);

/// A convex hull. Used to create convex polygons.
typedef struct b2Hull
{
	b2Vec2 points[b2_maxPolygonVertices];
	int32_t count;
} b2Hull;

/// Compute the convex hull of a set of points. Returns an empty hull if it fails.
/// Some failure cases:
/// - all points very close together
/// - all points on a line
/// - less than 3 points
/// - more than b2_maxPolygonVertices points
/// This welds close points and removes collinear points.
B2_API b2Hull b2ComputeHull(const b2Vec2* points, int32_t count);

/// This determines if a hull is valid. Checks for:
/// - convexity
/// - collinear points
/// This is expensive and should not be called at runtime.
B2_API bool b2ValidateHull(const b2Hull* hull);

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

/**
 * @defgroup distance Distance
 * Functions for computing the distance between shapes.
 *
 * These are advanced functions you can use to perform distance calculations. There
 * are functions for computing the closest points between shapes, doing linear shape casts,
 * and doing rotational shape casts. The latter is called time of impact (TOI).
 * @{
 */

/// Result of computing the distance between two line segments
typedef struct b2SegmentDistanceResult
{
	b2Vec2 closest1;
	b2Vec2 closest2;
	float fraction1;
	float fraction2;
	float distanceSquared;
} b2SegmentDistanceResult;

/// Compute the distance between two line segments, clamping at the end points if needed.
B2_API b2SegmentDistanceResult b2SegmentDistance(b2Vec2 p1, b2Vec2 q1, b2Vec2 p2, b2Vec2 q2);

/// A distance proxy is used by the GJK algorithm. It encapsulates any shape.
typedef struct b2DistanceProxy
{
	b2Vec2 vertices[b2_maxPolygonVertices];
	int32_t count;
	float radius;
} b2DistanceProxy;

/// Used to warm start b2Distance.
/// Set count to zero on first call.
typedef struct b2DistanceCache
{
	float metric; ///< length or area
	uint16_t count;
	uint8_t indexA[3]; ///< vertices on shape A
	uint8_t indexB[3]; ///< vertices on shape B
} b2DistanceCache;

static const b2DistanceCache b2_emptyDistanceCache = B2_ZERO_INIT;

/// Input for b2Distance.
/// You have to option to use the shape radii
/// in the computation. Even
typedef struct b2DistanceInput
{
	b2DistanceProxy proxyA;
	b2DistanceProxy proxyB;
	b2Transform transformA;
	b2Transform transformB;
	bool useRadii;
} b2DistanceInput;

/// Output for b2Distance.
typedef struct b2DistanceOutput
{
	b2Vec2 pointA; ///< closest point on shapeA
	b2Vec2 pointB; ///< closest point on shapeB
	float distance;
	int32_t iterations; ///< number of GJK iterations used
} b2DistanceOutput;

/// Compute the closest points between two shapes. Supports any combination of:
/// b2Circle, b2Polygon, b2EdgeShape. The simplex cache is input/output.
/// On the first call set b2SimplexCache.count to zero.
B2_API b2DistanceOutput b2ShapeDistance(b2DistanceCache* cache, const b2DistanceInput* input);

/// Input parameters for b2ShapeCast
typedef struct b2ShapeCastPairInput
{
	b2DistanceProxy proxyA;
	b2DistanceProxy proxyB;
	b2Transform transformA;
	b2Transform transformB;
	b2Vec2 translationB;
	float maxFraction;
} b2ShapeCastPairInput;

/// Perform a linear shape cast of shape B moving and shape A fixed. Determines the hit point, normal, and translation fraction.
B2_API b2CastOutput b2ShapeCast(const b2ShapeCastPairInput* input);

/// Make a proxy for use in GJK and related functions.
B2_API b2DistanceProxy b2MakeProxy(const b2Vec2* vertices, int32_t count, float radius);

/// This describes the motion of a body/shape for TOI computation. Shapes are defined with respect to the body origin,
/// which may not coincide with the center of mass. However, to support dynamics we must interpolate the center of mass
/// position.
typedef struct b2Sweep
{
	/// local center of mass position
	b2Vec2 localCenter;

	/// center world positions
	b2Vec2 c1, c2;

	/// world rotations
	b2Rot q1, q2;
} b2Sweep;

/// Evaluate the transform sweep at a specific time.
B2_API b2Transform b2GetSweepTransform(const b2Sweep* sweep, float time);

/// Input parameters for b2TimeOfImpact
typedef struct b2TOIInput
{
	b2DistanceProxy proxyA;
	b2DistanceProxy proxyB;
	b2Sweep sweepA;
	b2Sweep sweepB;

	// defines sweep interval [0, tMax]
	float tMax;
} b2TOIInput;

/// Describes the TOI output
typedef enum b2TOIState
{
	b2_toiStateUnknown,
	b2_toiStateFailed,
	b2_toiStateOverlapped,
	b2_toiStateHit,
	b2_toiStateSeparated
} b2TOIState;

/// Output parameters for b2TimeOfImpact.
typedef struct b2TOIOutput
{
	b2TOIState state;
	float t;
} b2TOIOutput;

/// Compute the upper bound on time before two shapes penetrate. Time is represented as
/// a fraction between [0,tMax]. This uses a swept separating axis and may miss some intermediate,
/// non-tunneling collisions. If you change the time interval, you should call this function
/// again.
B2_API b2TOIOutput b2TimeOfImpact(const b2TOIInput* input);

/**@}*/
