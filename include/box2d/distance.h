// SPDX-FileCopyrightText: 2022 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "types.h"

#ifdef __cplusplus
extern "C" {
#endif

/// A distance proxy is used by the GJK algorithm.
/// It encapsulates any shape.
typedef struct b2DistanceProxy
{
	const b2Vec2* vertices;
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
/// b2CircleShape, b2PolygonShape, b2EdgeShape. The simplex cache is input/output.
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

static inline b2DistanceProxy b2MakeProxy(const b2Vec2* vertices, int32_t count, float radius)
{
	return B2_LITERAL(b2DistanceProxy) { vertices, count, radius };
}

#ifdef __cplusplus
}
#endif
