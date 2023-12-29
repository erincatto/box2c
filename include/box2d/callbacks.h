// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "box2d/api.h"
#include "box2d/id.h"
#include "box2d/types.h"

typedef struct b2Manifold b2Manifold;

/// Prototype for a pre-solve callback.
/// This is called after a contact is updated. This allows you to inspect a
/// contact before it goes to the solver. If you are careful, you can modify the
/// contact manifold (e.g. disable contact).
/// Notes:
///	- this function must be thread-safe
///	- this is only called if the shape has enabled presolve events
/// - this is called only for awake dynamic bodies
/// - this is not called for sensors
/// - the supplied manifold has impulse values from the previous step
///	Return false if you want to disable the contact this step
typedef bool b2PreSolveFcn(b2ShapeId shapeIdA, b2ShapeId shapeIdB, b2Manifold* manifold, void* context);

/// Register the pre-solve callback. This is optional.
BOX2D_API void b2World_SetPreSolveCallback(b2WorldId worldId, b2PreSolveFcn* fcn, void* context);

/// Prototype callback for AABB queries.
/// See b2World_Query
/// Called for each shape found in the query AABB.
/// @return false to terminate the query.
typedef bool b2QueryResultFcn(b2ShapeId shapeId, void* context);

/// Prototype callback for ray casts.
/// See b2World::RayCast
/// Called for each shape found in the query. You control how the ray cast
/// proceeds by returning a float:
/// return -1: ignore this shape and continue
/// return 0: terminate the ray cast
/// return fraction: clip the ray to this point
/// return 1: don't clip the ray and continue
/// @param shape the shape hit by the ray
/// @param point the point of initial intersection
/// @param normal the normal vector at the point of intersection
/// @param fraction the fraction along the ray at the point of intersection
/// @return -1 to filter, 0 to terminate, fraction to clip the ray for
/// closest hit, 1 to continue
typedef float b2RayResultFcn(b2ShapeId shapeId, b2Vec2 point, b2Vec2 normal, float fraction, void* context);

/// Use an instance of this structure and the callback below to get the closest hit.
typedef struct b2RayResult
{
	b2ShapeId shapeId;
	b2Vec2 point;
	b2Vec2 normal;
	float fraction;
	bool hit;
} b2RayResult;

static const b2RayResult b2_emptyRayResult = {{-1, -1, 0}, {0.0f, 0.0f}, {0.0f, 0.0f}, 0.0f, false};
