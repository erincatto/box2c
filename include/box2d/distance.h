// SPDX-FileCopyrightText: 2022 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "constants.h"
#include "types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct b2SegmentDistanceResult
{
	b2Vec2 closest1;
	b2Vec2 closest2;
	float fraction1;
	float fraction2;
	float distanceSquared;
} b2SegmentDistanceResult;

/// Compute the distance between two line segments, clamping at the end points if needed.
b2SegmentDistanceResult b2SegmentDistance(b2Vec2 p1, b2Vec2 q1, b2Vec2 p2, b2Vec2 q2);

/// A distance proxy is used by the GJK algorithm.
/// It encapsulates any shape.
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
	float metric;		///< length or area
	uint16_t count;
	uint8_t indexA[3];	///< vertices on shape A
	uint8_t indexB[3];	///< vertices on shape B
} b2DistanceCache;

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
	b2Vec2 pointA;		///< closest point on shapeA
	b2Vec2 pointB;		///< closest point on shapeB
	float distance;
	int32_t iterations;	///< number of GJK iterations used
} b2DistanceOutput;

/// Compute the closest points between two shapes. Supports any combination of:
/// b2Circle, b2Polygon, b2EdgeShape. The simplex cache is input/output.
/// On the first call set b2SimplexCache.count to zero.
void b2ShapeDistance(b2DistanceOutput* output, b2DistanceCache* cache, const b2DistanceInput* input);

/// Input parameters for b2ShapeCast
typedef struct b2ShapeCastInput
{
	b2DistanceProxy proxyA;
	b2DistanceProxy proxyB;
	b2Transform transformA;
	b2Transform transformB;
	b2Vec2 translationB;
} b2ShapeCastInput;

/// Output results for b2ShapeCast
typedef struct b2ShapeCastOutput
{
	b2Vec2 point;
	b2Vec2 normal;
	float lambda;
	int32_t iterations;
} b2ShapeCastOutput;

/// Perform a linear shape cast of shape B moving and shape A fixed. Determines the hit point, normal, and translation fraction.
/// @returns true if hit, false if there is no hit or an initial overlap
bool b2ShapeCast(b2ShapeCastOutput* output, const b2ShapeCastInput* input);

b2DistanceProxy b2MakeProxy(const b2Vec2* vertices, int32_t count, float radius);

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
void b2TimeOfImpact(b2TOIOutput* output, const b2TOIInput* input);

#ifdef __cplusplus
}
#endif
