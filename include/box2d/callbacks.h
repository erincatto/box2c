// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "id.h"
#include "math_types.h"

#include <stdbool.h>

typedef struct b2Manifold b2Manifold;

/// Task interface
/// This is prototype for a Box2D task. Your task system is expected to invoke the Box2D task with these arguments.
/// The task spans a range of the parallel-for: [startIndex, endIndex)
/// The worker index must correctly identify each worker in the user thread pool, expected in [0, workerCount).
///	A worker must only exist on only one thread at a time and is analogous to the thread index.
/// The task context is the context pointer sent from Box2D when it is enqueued.
///	The startIndex and endIndex are expected in the range [0, itemCount) where itemCount is the argument to b2EnqueueTaskCallback
/// below. Box2D expects startIndex < endIndex and will execute a loop like this:
///
/// for (int i = startIndex; i < endIndex; ++i)
///	{
///		DoWork();
///	}
typedef void b2TaskCallback(int32_t startIndex, int32_t endIndex, uint32_t workerIndex, void* taskContext);

/// These functions can be provided to Box2D to invoke a task system. These are designed to work well with enkiTS.
/// Returns a pointer to the user's task object. May be nullptr. A nullptr indicates to Box2D that the work was executed
///	serially within the callback and there is no need to call b2FinishTaskCallback.
///	The itemCount is the number of Box2D work items that are to be partitioned among workers by the user's task system.
///	This is essentially a parallel-for. The minRange parameter is a suggestion of the minimum number of items to assign
///	per worker to reduce overhead. For example, suppose the task is small and that itemCount is 16. A minRange of 8 suggests
///	that your task system should split the work items among just two workers, even if you have more available.
///	In general the range [startIndex, endIndex) send to b2TaskCallback should obey:
///	endIndex - startIndex >= minRange
///	The exception of course is when itemCount < minRange.
typedef void* b2EnqueueTaskCallback(b2TaskCallback* task, int32_t itemCount, int32_t minRange, void* taskContext,
									void* userContext);

/// Finishes a user task object that wraps a Box2D task.
typedef void b2FinishTaskCallback(void* userTask, void* userContext);

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

/// Prototype callback for overlap queries.
/// See b2World_QueryAABB
/// Called for each shape found in the query AABB.
/// @return false to terminate the query.
typedef bool b2OverlapResultFcn(b2ShapeId shapeId, void* context);

/// Prototype callback for ray casts.
/// See b2World::RayCast
/// Called for each shape found in the query. You control how the ray cast
/// proceeds by returning a float:
/// #todo rework this to return penetration
/// return -1: ignore this shape and continue
/// return 0: terminate the ray cast
/// return fraction: clip the ray to this point
/// return 1: don't clip the ray and continue
/// @param shapeId the shape hit by the ray
/// @param point the point of initial intersection
/// @param normal the normal vector at the point of intersection
/// @param fraction the fraction along the ray at the point of intersection
/// @return -1 to filter, 0 to terminate, fraction to clip the ray for
/// closest hit, 1 to continue
typedef float b2CastResultFcn(b2ShapeId shapeId, b2Vec2 point, b2Vec2 normal, float fraction, void* context);
