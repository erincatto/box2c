// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "constants.h"
#include "types.h"

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
/// @returns true if hit, false if there is no hit or an initial overlap
B2_API b2RayCastOutput b2ShapeCast(const b2ShapeCastPairInput* input);

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

	/// world angles
	float a1, a2;
} b2Sweep;

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
