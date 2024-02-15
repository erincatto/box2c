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

#include "color.h"
#include "constants.h"
#include "id.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// clang-format off
#ifdef __cplusplus
	#define B2_LITERAL(T) T
	#define B2_ZERO_INIT {}
#else
	/// Used for C literals like (b2Vec2){1.0f, 2.0f} where C++ requires b2Vec2{1.0f, 2.0f}
	#define B2_LITERAL(T) (T)

	/// Used for C zero initialization, such as b2Vec2 v = {0} where C++ requires b2Vec2 v = {}
	#define B2_ZERO_INIT {0}
#endif
// clang-format on

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
	/// sine and cosine
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
/// The worker index must correctly identify each worker in the user thread pool, expected in [0, workerCount).
///	A worker must only exist on only one thread at a time and is analogous to the thread index.
/// The task context is the context pointer sent from Box2D when it is enqueued.
///	The startIndex and endIndex are expected in the range [0, itemCount) where itemCount is the argument to b2EnqueueTaskCallback below.
///	Box2D expects startIndex < endIndex and will execute a loop like this:
///	for (int i = startIndex; i < endIndex; ++i)
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
///	that your task system should split the work items amoung just two workers, even if you have more available.
///	In general the range [startIndex, endIndex) send to b2TaskCallback should obey:
///	endIndex - startIndex >= minRange
///	The exception of course is when itemCount < minRange.
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

	/// Enable continuous collision
	bool enableContinous;

	/// Capacity for bodies. This may not be exceeded.
	int32_t bodyCapacity;

	/// initial capacity for shapes
	int32_t shapeCapacity;

	/// Capacity for contacts. This may not be exceeded.
	int32_t contactCapacity;

	/// Capacity for joints
	int32_t jointCapacity;

	/// Stack allocator capacity. This controls how much space box2d reserves for per-frame calculations.
	/// Larger worlds require more space. b2Counters can be used to determine a good capacity for your
	/// application.
	int32_t stackAllocatorCapacity;

	/// task system hookup
	uint32_t workerCount;

	/// function to spawn task
	b2EnqueueTaskCallback* enqueueTask;

	/// function to finish a task
	b2FinishTaskCallback* finishTask;

	/// User context that is provided to enqueueTask and finishTask
	void* userTaskContext;
} b2WorldDef;

/// Use this to initialize your world definition
static inline b2WorldDef b2DefaultWorldDef()
{
	b2WorldDef def = B2_ZERO_INIT;
	def.gravity.x = 0.0f;
	def.gravity.y = -10.0f;
	def.restitutionThreshold = 1.0f * b2_lengthUnitsPerMeter;
	def.contactPushoutVelocity = 3.0f * b2_lengthUnitsPerMeter;
	def.contactHertz = 30.0;
	def.contactDampingRatio = 1.0f;
	def.enableSleep = true;
	def.enableContinous = true;
	def.bodyCapacity = 0;
	def.shapeCapacity = 0;
	def.contactCapacity = 0;
	def.jointCapacity = 0;
	def.stackAllocatorCapacity = 1024 * 1024;
	def.workerCount = 0;
	def.enqueueTask = NULL;
	def.finishTask = NULL;
	def.userTaskContext = NULL;
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
	b2_staticBody, // bodyType
	{0.0f, 0.0f},  // position
	0.0f,		   // angle
	{0.0f, 0.0f},  // linearVelocity
	0.0f,		   // angularVelocity
	0.0f,		   // linearDamping
	0.0f,		   // angularDamping
	1.0f,		   // gravityScale
	NULL,		   // userData
	true,		   // enableSleep
	true,		   // isAwake
	false,		   // fixedRotation
	true,		   // isEnabled
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
	NULL,						 // userData
	0.6f,						 // friction
	0.0f,						 // restitution
	1.0f,						 // density
	{0x00000001, 0xFFFFFFFF, 0}, // filter
	false,						 // isSensor
	true,						 // enableSensorEvents
	true,						 // enableContactEvents
	false,						 // enablePreSolveEvents
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
static const b2ChainDef b2_defaultChainDef = {
	NULL,						// points
	0,							// count
	false,						// loop
	NULL,						// userData
	0.6f,						// friction
	0.0f,						// restitution
	{0x00000001, 0xFFFFFFFF, 0} // filter
};

/// Profiling data. Times are in milliseconds.
typedef struct b2Profile
{
	float step;
	float pairs;
	float collide;
	float solve;
	float buildIslands;
	float solveConstraints;
	float broadphase;
	float continuous;
} b2Profile;

/// Use this to initialize your profile
static const b2Profile b2_emptyProfile = B2_ZERO_INIT;

/// Counters that give details of the simulation size
typedef struct b2Counters
{
	int32_t islandCount;
	int32_t bodyCount;
	int32_t contactCount;
	int32_t jointCount;
	int32_t proxyCount;
	int32_t pairCount;
	int32_t treeHeight;
	int32_t stackCapacity;
	int32_t stackUsed;
	int32_t byteCount;
	int32_t taskCount;
	int32_t colorCounts[b2_graphColorCount + 1];
} b2Counters;

/// Use this to initialize your counters
static const b2Counters b2_emptyCounters = B2_ZERO_INIT;
