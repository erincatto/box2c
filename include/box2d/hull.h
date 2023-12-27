// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "api.h"
#include "constants.h"
#include "types.h"

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
BOX2D_API b2Hull b2ComputeHull(const b2Vec2* points, int32_t count);

/// This determines if a hull is valid. Checks for:
/// - convexity
/// - collinear points
/// This is expensive and should not be called at runtime.
BOX2D_API bool b2ValidateHull(const b2Hull* hull);
