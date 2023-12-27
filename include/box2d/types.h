// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

/**
 * @file types.h
 * @brief types used by the Box2D API
 *
 * Mostly definition structs
 * @see http://www.box2d.org
 */

#pragma once

#include "box2d/color.h"
#include "box2d/constants.h"
#include "box2d/id.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
#define B2_LITERAL(T) T
#define B2_ZERO_INIT                                                                                                             \
	{                                                                                                                            \
	}
#else
/// Used for C literals like (b2Vec2){1.0f, 2.0f} where C++ requires b2Vec2{1.0f, 2.0f}
#define B2_LITERAL(T) (T)

/// Used for C zero initialization, such as b2Vec2 v = {0} where C++ requires b2Vec2 v = {}
#define B2_ZERO_INIT                                                                                                             \
	{                                                                                                                            \
		0                                                                                                                        \
	}
#endif

/// Returns the number of elements of an array
#define B2_ARRAY_COUNT(A) (int)(sizeof(A) / sizeof(A[0]))

/// Used to prevent the compiler from warning about unused variables
#define B2_MAYBE_UNUSED(x) ((void)(x))

/// A 2D vector
/// This can be used to represent a point or free vector.
typedef struct b2Vec2
{
	/// coordinates
	float x, y;
} b2Vec2;

/// 2D rotation
typedef struct b2Rot
{
	/// Sine and cosine
	float s, c;
} b2Rot;

/// A 2D rigid transform
typedef struct b2Transform
{
	b2Vec2 p;
	b2Rot q;
} b2Transform;

/// A 2-by-2 Matrix
typedef struct b2Mat22
{
	/// columns
	b2Vec2 cx, cy;
} b2Mat22;

/// Axis-aligned bounding box
typedef struct b2AABB
{
	b2Vec2 lowerBound;
	b2Vec2 upperBound;
} b2AABB;

/// Low level ray-cast input data
typedef struct b2RayCastInput
{
	b2Vec2 origin, translation;
	float maxFraction;
} b2RayCastInput;

/// Low level hape cast input in generic form
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
} b2RayCastOutput;

/// Task interface
/// This is prototype for a Box2D task. Your task system is expected to invoke the Box2D task with these arguments.
/// The task spans a range of the parallel-for: [startIndex, endIndex)
/// The thread index must correctly identify each thread in the user thread pool, expected in [0, workerCount)
/// The task context is the context pointer sent from Box2D when it is enqueued.
typedef void b2TaskCallback(int32_t startIndex, int32_t endIndex, uint32_t threadIndex, void* taskContext);

/// These functions can be provided to Box2D to invoke a task system. These are designed to work well with enkiTS.
/// Returns a pointer to the user's task object. May be nullptr.
typedef void* b2EnqueueTaskCallback(b2TaskCallback* task, int32_t itemCount, int32_t minRange, void* taskContext,
									void* userContext);

/// Finishes a user task object that wraps a Box2D task.
typedef void b2FinishTaskCallback(void* userTask, void* userContext);

/// World definition used to create a simulation world. Must be initialized using b2DefaultWorldDef.
typedef struct b2WorldDef
{
	/// Gravity vector. Box2D has no up-vector defined.
	b2Vec2 gravity;

	/// Restitution velocity threshold, usually in m/s. Collisions above this
	/// speed have restitution applied (will bounce).
	float restitutionThreshold;

	/// This parameter controls how fast overlap is resolved and has units of meters per second
	float contactPushoutVelocity;

	/// Contact stiffness. Cycles per second.
	float contactHertz;

	/// Contact bounciness. Non-dimensional.
	float contactDampingRatio;

	/// Can bodies go to sleep to improve performance
	bool enableSleep;

	/// Capacity for bodies. This may not be exceeded.
	int32_t bodyCapacity;

	/// initial capacity for shapes
	int32_t shapeCapacity;

	/// Capacity for contacts. This may not be exceeded.
	int32_t contactCapacity;

	/// Capacity for joints
	int32_t jointCapacity;

	/// Stack allocator capacity. This controls how much space box2d reserves for per-frame calculations.
	/// Larger worlds require more space. b2Statistics can be used to determine a good capacity for your
	/// application.
	int32_t stackAllocatorCapacity;

	/// task system hookup
	uint32_t workerCount;
	b2EnqueueTaskCallback* enqueueTask;
	b2FinishTaskCallback* finishTask;
	void* userTaskContext;
} b2WorldDef;

/// Use this to initialize your world definition
static inline b2WorldDef b2DefaultWorldDef(void)
{
	b2WorldDef def = B2_ZERO_INIT;
	def.gravity = B2_LITERAL(b2Vec2){0.0f, -10.0f};
	def.restitutionThreshold = 1.0f * b2_lengthUnitsPerMeter;
	def.contactPushoutVelocity = 3.0f * b2_lengthUnitsPerMeter;
	def.contactHertz = 30.0f;
	def.contactDampingRatio = 1.0f;
	def.enableSleep = true;
	def.bodyCapacity = 8;
	def.shapeCapacity = 8;
	def.contactCapacity = 8;
	def.jointCapacity = 8;
	def.stackAllocatorCapacity = 1024 * 1024;
	return def;
}

/// The body type.
/// static: zero mass, zero velocity, may be manually moved
/// kinematic: zero mass, non-zero velocity set by user, moved by solver
/// dynamic: positive mass, non-zero velocity determined by forces, moved by solver
typedef enum b2BodyType
{
	b2_staticBody = 0,
	b2_kinematicBody = 1,
	b2_dynamicBody = 2,
	b2_bodyTypeCount
} b2BodyType;

/// A body definition holds all the data needed to construct a rigid body.
/// You can safely re-use body definitions. Shapes are added to a body after construction.
typedef struct b2BodyDef
{
	/// The body type: static, kinematic, or dynamic.
	/// Note: if a dynamic body would have zero mass, the mass is set to one.
	b2BodyType type;

	/// The world position of the body. Avoid creating bodies at the origin
	/// since this can lead to many overlapping shapes.
	b2Vec2 position;

	/// The world angle of the body in radians.
	float angle;

	/// The linear velocity of the body's origin in world co-ordinates.
	b2Vec2 linearVelocity;

	/// The angular velocity of the body.
	float angularVelocity;

	/// Linear damping is use to reduce the linear velocity. The damping parameter
	/// can be larger than 1.0f but the damping effect becomes sensitive to the
	/// time step when the damping parameter is large.
	float linearDamping;

	/// Angular damping is use to reduce the angular velocity. The damping parameter
	/// can be larger than 1.0f but the damping effect becomes sensitive to the
	/// time step when the damping parameter is large.
	float angularDamping;

	/// Scale the gravity applied to this body.
	float gravityScale;

	/// Use this to store application specific body data.
	void* userData;

	/// Set this flag to false if this body should never fall asleep. Note that
	/// this increases CPU usage.
	bool enableSleep;

	/// Is this body initially awake or sleeping?
	bool isAwake;

	/// Should this body be prevented from rotating? Useful for characters.
	bool fixedRotation;

	/// Does this body start out enabled?
	bool isEnabled;
} b2BodyDef;

/// Use this to initialize your body definition
static const b2BodyDef b2_defaultBodyDef = {
	b2_staticBody, {0.0f, 0.0f}, 0.0f, {0.0f, 0.0f}, 0.0f, 0.0f, 0.0f, 1.0f, NULL, true, true, false, true,
};

/// This holds contact filtering data.
typedef struct b2Filter
{
	/// The collision category bits. Normally you would just set one bit.
	uint32_t categoryBits;

	/// The collision mask bits. This states the categories that this
	/// shape would accept for collision.
	uint32_t maskBits;

	/// Collision groups allow a certain group of objects to never collide (negative)
	/// or always collide (positive). Zero means no collision group. Non-zero group
	/// filtering always wins against the mask bits.
	int32_t groupIndex;
} b2Filter;

/// Use this to initialize your filter
static const b2Filter b2_defaultFilter = {0x00000001, 0xFFFFFFFF, 0};

/// This holds contact filtering data.
typedef struct b2QueryFilter
{
	/// The collision category bits. Normally you would just set one bit.
	uint32_t categoryBits;

	/// The collision mask bits. This states the categories that this
	/// shape would accept for collision.
	uint32_t maskBits;
} b2QueryFilter;

/// Use this to initialize your query filter
static const b2QueryFilter b2_defaultQueryFilter = {0x00000001, 0xFFFFFFFF};

/// Shape type
typedef enum b2ShapeType
{
	b2_capsuleShape,
	b2_circleShape,
	b2_polygonShape,
	b2_segmentShape,
	b2_smoothSegmentShape,
	b2_shapeTypeCount
} b2ShapeType;

/// Used to create a shape
typedef struct b2ShapeDef
{
	/// Use this to store application specific shape data.
	void* userData;

	/// The friction coefficient, usually in the range [0,1].
	float friction;

	/// The restitution (bounce) usually in the range [0,1].
	float restitution;

	/// The density, usually in kg/m^2.
	float density;

	/// Contact filtering data.
	b2Filter filter;

	/// A sensor shape collects contact information but never generates a collision response.
	bool isSensor;

	/// Enable sensor events for this shape. Only applies to kinematic and dynamic bodies. Ignored for sensors.
	bool enableSensorEvents;

	/// Enable contact events for this shape. Only applies to kinematic and dynamic bodies. Ignored for sensors.
	bool enableContactEvents;

	/// Enable pre-solve contact events for this shape. Only applies to dynamic bodies. These are expensive
	///	and must be carefully handled due to multi-threading. Ignored for sensors.
	bool enablePreSolveEvents;

} b2ShapeDef;

/// Use this to initialize your shape definition
static const b2ShapeDef b2_defaultShapeDef = {
	NULL, 0.6f, 0.0f, 1.0f, {0x00000001, 0xFFFFFFFF, 0}, false, true, true, false,
};

/// Used to create a chain of edges. This is designed to eliminate ghost collisions with some limitations.
///	- DO NOT use chain shapes unless you understand the limitations. This is an advanced feature!
///	- chains are one-sided
///	- chains have no mass and should be used on static bodies
///	- the front side of the chain points the right of the point sequence
///	- chains are either a loop or open
/// - a chain must have at least 4 points
///	- the distance between any two points must be greater than b2_linearSlop
///	- a chain shape should not self intersect (this is not validated)
///	- an open chain shape has NO COLLISION on the first and final edge
///	- you may overlap two open chains on their first three and/or last three points to get smooth collision
///	- a chain shape creates multiple hidden shapes on the body
typedef struct b2ChainDef
{
	/// An array of at least 4 points. These are cloned and may be temporary.
	const b2Vec2* points;

	/// The point count, must be 4 or more.
	int32_t count;

	/// Indicates a closed chain formed by connecting the first and last points
	bool loop;

	/// Use this to store application specific shape data.
	void* userData;

	/// The friction coefficient, usually in the range [0,1].
	float friction;

	/// The restitution (elasticity) usually in the range [0,1].
	float restitution;

	/// Contact filtering data.
	b2Filter filter;
} b2ChainDef;

/// Use this to initialize your chain definition
static const b2ChainDef b2_defaultChainDef = {NULL, 0, false, NULL, 0.6f, 0.0f, {0x00000001, 0xFFFFFFFF, 0}};
