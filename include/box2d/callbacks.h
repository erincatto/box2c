// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "box2d/id.h"
#include "box2d/types.h"

typedef struct b2Manifold b2Manifold;
typedef struct b2ContactImpulse b2ContactImpulse;

/// Joints and shapes are destroyed when their associated
/// body is destroyed. Implement this listener so that you
/// may nullify references to these joints and shapes.
typedef void b2JointDestroyedFcn(b2JointId jointId);
typedef void b2ShapeDestroyedFcn(b2JointId jointId);

/// Implement this class to provide collision filtering. In other words, you can implement
/// this class if you want finer control over contact creation.
/// Return true if contact calculations should be performed between these two shapes.
/// @warning for performance reasons this is only called when the AABBs begin to overlap.
typedef bool b2ShouldCollideFcn(b2ShapeId shapeIdA, b2ShapeId shapeIdB);

/// Contact impulses for reporting. Impulses are used instead of forces because
/// sub-step forces may approach infinity for rigid body collisions. These
/// match up one-to-one with the contact points in b2Manifold.
struct b2ContactImpulse
{
	float normalImpulses[2];
	float tangentImpulses[2];
	int32_t count;
};

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
typedef void b2BeginContactFcn(b2ShapeId shapeIdA, b2ShapeId shapeIdB);

/// Called when two shapes cease to touch.
typedef void b2EndContactFcn(b2ShapeId shapeIdA, b2ShapeId shapeIdB);

/// This is called after a contact is updated. This allows you to inspect a
/// contact before it goes to the solver. If you are careful, you can modify the
/// contact manifold (e.g. disable contact).
/// A copy of the old manifold is provided so that you can detect changes.
/// Note: this is called only for awake bodies.
/// Note: this is called even when the number of contact points is zero.
/// Note: this is not called for sensors.
/// Note: if you set the number of contact points to zero, you will not
/// get an EndContact callback. However, you may get a BeginContact callback
/// the next step.
typedef void b2PreSolveFcn(b2ShapeId shapeIdA, b2ShapeId shapeIdB, b2Manifold* manifold, const b2Manifold* oldManifold);

/// This lets you inspect a contact after the solver is finished. This is useful
/// for inspecting impulses.
/// Note: the contact manifold does not include time of impact impulses, which can be
/// arbitrarily large if the sub-step is small. Hence the impulse is provided explicitly
/// in a separate data structure.
/// Note: this is only called for contacts that are touching, solid, and awake.
typedef void b2PostSolveFcn(b2ShapeId shapeIdA, b2ShapeId shapeIdB, const b2ContactImpulse* impulse);

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
typedef bool b2QueryResultFcn(b2ShapeId shapeId);

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
typedef float b2RayResultFcn(b2ShapeId shape, b2Vec2 point, b2Vec2 normal, float fraction);

#ifdef __cplusplus
extern "C" {
#endif

static inline b2WorldCallbacks b2DefaultWorldCallbacks()
{
	b2WorldCallbacks callbacks = {0};
	return callbacks;
}

#ifdef __cplusplus
}
#endif
