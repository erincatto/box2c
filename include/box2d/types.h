// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

/**
 * @file types.h
 * @brief types used by the Box2D API
 *
 * Mostly definition structures
 * @see http://www.box2d.org
 */

#pragma once

#include "api.h"
#include "callbacks.h"
#include "constants.h"
#include "id.h"
#include "math_types.h"

#include <stdbool.h>
#include <stdint.h>

/// Result from b2World_RayCastClosest
typedef struct b2RayResult
{
	b2ShapeId shapeId;
	b2Vec2 point;
	b2Vec2 normal;
	float fraction;
	bool hit;
} b2RayResult;

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

	/// Joint stiffness. Cycles per second.
	float jointHertz;

	/// Joint bounciness. Non-dimensional.
	float jointDampingRatio;

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

/// The body type.
/// static: zero mass, zero velocity, may not be moved
/// kinematic: zero mass, non-zero velocity set by user, moved by solver
/// dynamic: positive mass, non-zero velocity determined by forces, moved by solver
typedef enum b2BodyType
{
	b2_staticBody = 0,
	b2_kinematicBody = 1,
	b2_dynamicBody = 2,
	b2_bodyTypeCount
} b2BodyType;

typedef struct b2StaticBodyDef
{
	/// The world position of the body. Avoid creating bodies at the origin
	/// since this can lead to many overlapping shapes.
	b2Vec2 position;

	/// The world angle of the body in radians.
	float angle;

	/// Use this to store application specific body data.
	void* userData;
} b2StaticBodyDef;

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

	/// Treat this body as high speed object that performs continuous collision detection
	/// against dynamic and kinematic bodies, but not other bullet bodies.
	bool isBullet;

	/// Does this body start out enabled?
	bool isEnabled;
} b2BodyDef;

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

/// This holds contact filtering data.
typedef struct b2QueryFilter
{
	/// The collision category bits. Normally you would just set one bit.
	uint32_t categoryBits;

	/// The collision mask bits. This states the categories that this
	/// shape would accept for collision.
	uint32_t maskBits;
} b2QueryFilter;

/// Shape type
typedef enum b2ShapeType
{
	b2_circleShape,
	b2_capsuleShape,
	b2_segmentShape,
	b2_polygonShape,
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
/// https://en.wikipedia.org/wiki/Polygonal_chain
typedef struct b2ChainDef
{
	/// An array of at least 4 points. These are cloned and may be temporary.
	const b2Vec2* points;

	/// The point count, must be 4 or more.
	int32_t count;

	/// Indicates a closed chain formed by connecting the first and last points
	bool isLoop;

	/// Use this to store application specific shape data.
	void* userData;

	/// The friction coefficient, usually in the range [0,1].
	float friction;

	/// The restitution (elasticity) usually in the range [0,1].
	float restitution;

	/// Contact filtering data.
	b2Filter filter;
} b2ChainDef;

/// Profiling data. Times are in milliseconds.
typedef struct b2Profile
{
	float step;
	float pairs;
	float collide;
	float solve;
	float buildIslands;
	float solveConstraints;
	float prepareTasks;
	float solverTasks;
	float prepareConstraints;
	float integrateVelocities;
	float warmStart;
	float solveVelocities;
	float integratePositions;
	float relaxVelocities;
	float applyRestitution;
	float storeImpulses;
	float finalizeBodies;
	float awakeUpdate;
	float broadphase;
	float continuous;
} b2Profile;

/// Counters that give details of the simulation size
typedef struct b2Counters
{
	int staticBodyCount;
	int bodyCount;
	int shapeCount;
	int contactCount;
	int jointCount;
	int islandCount;
	int stackUsed;
	int treeHeight;
	int byteCount;
	int taskCount;
	int colorCounts[b2_graphColorCount];
} b2Counters;

/// Use this to initialize your world definition
B2_API b2WorldDef b2DefaultWorldDef();

/// Use this to initialize your static body definition
B2_API b2StaticBodyDef b2DefaultStaticBodyDef();

/// Use this to initialize your body definition
B2_API b2BodyDef b2DefaultBodyDef();

/// Use this to initialize your filter
B2_API b2Filter b2DefaultFilter();

/// Use this to initialize your query filter
B2_API b2QueryFilter b2DefaultQueryFilter();

/// Use this to initialize your shape definition
B2_API b2ShapeDef b2DefaultShapeDef();

/// Use this to initialize your chain definition
B2_API b2ChainDef b2DefaultChainDef();
