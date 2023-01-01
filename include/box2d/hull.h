// SPDX-FileCopyrightText: 2022 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "constants.h"
#include "types.h"

#ifdef __cplusplus
extern "C"
{
#endif

/// Convex hull used for polygon collision
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
b2Hull b2ComputeHull(const b2Vec2* points, int32_t count);

/// This determines if a hull is valid. Checks for:
/// - convexity
/// - collinear points
/// This is expensive and should not be called at runtime.
bool b2ValidateHull(const b2Hull* hull);

#ifdef __cplusplus
}
#endif
