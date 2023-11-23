// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "box2d/api.h"
#include "box2d/id.h"
#include "box2d/types.h"

//typedef struct b2ContactImpulse b2ContactImpulse;
typedef struct b2Manifold b2Manifold;

/// Joints and shapes are destroyed when their associated
/// body is destroyed. Implement this listener so that you
/// may nullify references to these joints and shapes.
typedef void b2JointDestroyedFcn(b2JointId jointId, void* context);
typedef void b2ShapeDestroyedFcn(b2ShapeId shapeId, void* context);

/// Implement this class to provide collision filtering. In other words, you can implement
/// this class if you want finer control over contact creation.
/// Return true if contact calculations should be performed between these two shapes.
/// @warning for performance reasons this is only called when the AABBs begin to overlap.
typedef bool b2ShouldCollideFcn(b2ShapeId shapeIdA, b2ShapeId shapeIdB, void* context);

/// Implement these callbacks to get contact information. You can use these results for
/// things like sounds and game logic. You can also get contact results by
/// traversing the contact lists after the time step. However, you might miss
/// some contacts because continuous physics leads to sub-stepping.
/// Additionally you may receive multiple callbacks for the same contact in a
/// single time step.
/// You should strive to make your callbacks efficient because there may be
/// many callbacks per time step.
/// @warning You cannot create/destroy Box2D entities inside these callbacks.
/// Called when two shapes begin to touch.
typedef void b2BeginContactFcn(b2ShapeId shapeIdA, b2ShapeId shapeIdB, void* context);

/// Called when two shapes cease to touch.
typedef void b2EndContactFcn(b2ShapeId shapeIdA, b2ShapeId shapeIdB, void* context);

/// This is called after a contact is updated. This allows you to inspect a
/// contact before it goes to the solver. If you are careful, you can modify the
/// contact manifold (e.g. disable contact).
/// Notes:
/// - this is called only for awake bodies.
/// - this is called even when the number of contact points is zero.
/// - this is not called for sensors.
/// - if you set the number of contact points to zero, you will not
/// get an EndContact callback. However, you may get a BeginContact callback
/// the next step.
/// - the supplied manifold has impulse values from the previous frame
typedef bool b2PreSolveFcn(b2ShapeId shapeIdA, b2ShapeId shapeIdB, b2Manifold* manifold, int32_t color, void* context);
BOX2D_API void b2World_SetPreSolveCallback(b2WorldId worldId, b2PreSolveFcn* fcn, void* context);

/// This lets you inspect a contact after the solver is finished. This is useful
/// for inspecting impulses.
/// Note: the contact manifold does not include time of impact impulses, which can be
/// arbitrarily large if the sub-step is small. Hence the impulse is provided explicitly
/// in a separate data structure.
/// Note: this is only called for contacts that are touching, solid, and awake.
typedef void b2PostSolveFcn(b2ShapeId shapeIdA, b2ShapeId shapeIdB, const b2Manifold* manifold, void* context);
BOX2D_API void b2World_SetPostSolveCallback(b2WorldId worldId, b2PostSolveFcn* fcn, void* context);

typedef struct b2WorldCallbacks
{
	b2JointDestroyedFcn* jointDestroyedFcn;
	b2ShapeDestroyedFcn* shapeDestroyedFcn;
	b2ShouldCollideFcn* shouldCollideFcn;
	b2BeginContactFcn* beginContactFcn;
	b2EndContactFcn* endContactFcn;
	b2PreSolveFcn* preSolveFcn;
	b2PostSolveFcn* postSolveFcn;
} b2WorldCallbacks;

/// Callback class for AABB queries.
/// See b2World_Query
/// Called for each shape found in the query AABB.
/// @return false to terminate the query.
typedef bool b2QueryResultFcn(b2ShapeId shapeId, void* context);

/// Callback class for ray casts.
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
