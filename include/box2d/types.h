// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "api.h"
#include "callbacks.h"
#include "constants.h"
#include "id.h"
#include "math_types.h"

#include <stdbool.h>
#include <stdint.h>

/// Result from b2World_RayCastClosest
/// @ingroup world
typedef struct b2RayResult
{
	b2ShapeId shapeId;
	b2Vec2 point;
	b2Vec2 normal;
	float fraction;
	bool hit;
} b2RayResult;

/// World definition used to create a simulation world.
/// Must be initialized using b2DefaultWorldDef.
/// @ingroup world
typedef struct b2WorldDef
{
	/// Gravity vector. Box2D has no up-vector defined.
	b2Vec2 gravity;

	/// Restitution velocity threshold, usually in m/s. Collisions above this
	/// speed have restitution applied (will bounce).
	float restitutionThreshold;

	/// This parameter controls how fast overlap is resolved and has units of meters per second
	float contactPushoutVelocity;

	/// Threshold velocity for hit events. Usually meters per second.
	float hitEventThreshold;

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

	/// Number of workers to use with the provided task system. Box2D performs best when using only
	///	performance cores and accessing a single L2 cache. Efficiency cores and hyper-threading provide
	///	little benefit and may even harm performance.
	int32_t workerCount;

	/// Function to spawn tasks
	b2EnqueueTaskCallback* enqueueTask;

	/// Function to finish a task
	b2FinishTaskCallback* finishTask;

	/// User context that is provided to enqueueTask and finishTask
	void* userTaskContext;
} b2WorldDef;

/// The body simulation type.
/// Each body is one of these three types. The type determines how the body behaves in the simulation.
/// @ingroup body
typedef enum b2BodyType
{
	/// zero mass, zero velocity, may be manually moved
	b2_staticBody = 0,

	/// zero mass, velocity set by user, moved by solver
	b2_kinematicBody = 1,

	/// positive mass, velocity determined by forces, moved by solver
	b2_dynamicBody = 2,
} b2BodyType;

/// A body definition holds all the data needed to construct a rigid body.
/// You can safely re-use body definitions. Shapes are added to a body after construction.
///	Body definitions are temporary objects used to bundle creation parameters.
/// @ingroup body
typedef struct b2BodyDef
{
	/// The body type: static, kinematic, or dynamic.
	b2BodyType type;

	/// The initial world position of the body. Bodies should be created with the desired position.
	/// @warning Creating bodies at the origin and then moving them nearly doubles the cost of body creation, especially
	///	if the body is moved after shapes have been added.
	b2Vec2 position;

	/// The initial world angle of the body in radians.
	float angle;

	/// The initial linear velocity of the body's origin. Typically in meters per second.
	b2Vec2 linearVelocity;

	/// The initial angular velocity of the body. Typically in meters per second.
	float angularVelocity;

	/// Linear damping is use to reduce the linear velocity. The damping parameter
	/// can be larger than 1.0f but the damping effect becomes sensitive to the
	/// time step when the damping parameter is large.
	///	Generally linear damping is undesirable because it makes objects move slowly
	///	as if they are floating.
	float linearDamping;

	/// Angular damping is use to reduce the angular velocity. The damping parameter
	/// can be larger than 1.0f but the damping effect becomes sensitive to the
	/// time step when the damping parameter is large.
	///	Angular damping can be use slow down rotating bodies.
	float angularDamping;

	/// Scale the gravity applied to this body. Non-dimensional.
	float gravityScale;

	/// Sleep velocity threshold, default is 0.05 meter per second
	float sleepThreshold;

	/// Use this to store application specific body data.
	void* userData;

	/// Set this flag to false if this body should never fall asleep.
	bool enableSleep;

	/// Is this body initially awake or sleeping?
	bool isAwake;

	/// Should this body be prevented from rotating? Useful for characters.
	bool fixedRotation;

	/// Treat this body as high speed object that performs continuous collision detection
	/// against dynamic and kinematic bodies, but not other bullet bodies.
	///	@warning Bullets should be used sparingly. They are not a solution for general dynamic-versus-dynamic
	///	continuous collision. They may interfere with joint constraints.
	bool isBullet;

	/// Used to disable a body. A disabled body does not move or collide.
	bool isEnabled;

	/// Automatically compute mass and related properties on this body from shapes.
	/// Triggers whenever a shape is add/removed/changed. Default is true.
	bool automaticMass;
} b2BodyDef;

/// This is used to filter collision on shapes. It affects shape-vs-shape collision
///	and shape-versus-query collision (such as b2World_CastRay).
/// @ingroup shape
typedef struct b2Filter
{
	/// The collision category bits. Normally you would just set one bit. The category bits should
	///	represent your application object types. For example:
	///	@code{.cpp}
	///	enum MyCategories
	///	{
	///	   Static  = 0x00000001,
	///	   Dynamic = 0x00000002,
	///	   Debris  = 0x00000004,
	///	   Player  = 0x00000008,
	///	   // etc
	/// };
	///	@endcode
	uint32_t categoryBits;

	/// The collision mask bits. This states the categories that this
	/// shape would accept for collision.
	///	For example, you may want your player to only collide with static objects
	///	and other players.
	///	@code{.c}
	///	maskBits = Static | Player;
	///	@endcode
	uint32_t maskBits;

	/// Collision groups allow a certain group of objects to never collide (negative)
	/// or always collide (positive). A group index of zero has no effect. Non-zero group filtering
	/// always wins against the mask bits.
	///	For example, you may want ragdolls to collide with other ragdolls but you don't want
	///	ragdoll self-collision. In this case you would give each ragdoll a unique negative group index
	///	and apply that group index to all shapes on the ragdoll.
	int32_t groupIndex;
} b2Filter;

/// The query filter is used to filter collisions between queries and shapes. For example,
///	you may want a ray-cast representing a projectile to hit players and the static environment
///	but not debris.
/// @ingroup shape
typedef struct b2QueryFilter
{
	/// The collision category bits of this query. Normally you would just set one bit.
	uint32_t categoryBits;

	/// The collision mask bits. This states the shape categories that this
	/// query would accept for collision.
	uint32_t maskBits;
} b2QueryFilter;

/// Shape type
/// @ingroup shape
typedef enum b2ShapeType
{
	/// A circle with an offset
	b2_circleShape,

	/// A capsule is an extruded circle
	b2_capsuleShape,

	/// A line segment
	b2_segmentShape,

	/// A convex polygon
	b2_polygonShape,

	/// A smooth segment owned by a chain shape
	b2_smoothSegmentShape,

	b2_shapeTypeCount
} b2ShapeType;

/// Used to create a shape.
/// This is a temporary object used to bundle shape creation parameters. You may use
///	the same shape definition to create multiple shapes.
/// @ingroup shape
typedef struct b2ShapeDef
{
	/// Use this to store application specific shape data.
	void* userData;

	/// The Coulomb (dry) friction coefficient, usually in the range [0,1].
	float friction;

	/// The restitution (bounce) usually in the range [0,1].
	float restitution;

	/// The density, usually in kg/m^2.
	float density;

	/// Collision filtering data.
	b2Filter filter;

	/// A sensor shape generates overlap events but never generates a collision response.
	bool isSensor;

	/// Enable sensor events for this shape. Only applies to kinematic and dynamic bodies. Ignored for sensors.
	bool enableSensorEvents;

	/// Enable contact events for this shape. Only applies to kinematic and dynamic bodies. Ignored for sensors.
	bool enableContactEvents;

	/// Enable hit events for this shape. Only applies to kinematic and dynamic bodies. Ignored for sensors.
	bool enableHitEvents;

	/// Enable pre-solve contact events for this shape. Only applies to dynamic bodies. These are expensive
	///	and must be carefully handled due to multi-threading. Ignored for sensors.
	bool enablePreSolveEvents;

	/// Normally shapes on static bodies don't invoke contact creation when they are added to the world. This overrides
	///	that behavior and causes contact creation. This significantly slows down static body creation which can be important
	///	when there are many static shapes.
	bool forceContactCreation;

} b2ShapeDef;

/// Used to create a chain of edges. This is designed to eliminate ghost collisions with some limitations.
///	- chains are one-sided
///	- chains have no mass and should be used on static bodies
///	- chains have a counter-clockwise winding order
///	- chains are either a loop or open
/// - a chain must have at least 4 points
///	- the distance between any two points must be greater than b2_linearSlop
///	- a chain shape should not self intersect (this is not validated)
///	- an open chain shape has NO COLLISION on the first and final edge
///	- you may overlap two open chains on their first three and/or last three points to get smooth collision
///	- a chain shape creates multiple smooth edges shapes on the body
/// https://en.wikipedia.org/wiki/Polygonal_chain
///	@warning DO NOT use chain shapes unless you understand the limitations. This is an advanced feature!
/// @ingroup shape
typedef struct b2ChainDef
{
	/// Use this to store application specific shape data.
	void* userData;

	/// An array of at least 4 points. These are cloned and may be temporary.
	const b2Vec2* points;

	/// The point count, must be 4 or more.
	int32_t count;

	/// The friction coefficient, usually in the range [0,1].
	float friction;

	/// The restitution (elasticity) usually in the range [0,1].
	float restitution;

	/// Contact filtering data.
	b2Filter filter;

	/// Indicates a closed chain formed by connecting the first and last points
	bool isLoop;
} b2ChainDef;

//! @cond
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
	float splitIslands;
	float sleepIslands;
	float hitEvents;
	float broadphase;
	float continuous;
} b2Profile;

/// Counters that give details of the simulation size.
typedef struct b2Counters
{
	int32_t staticBodyCount;
	int32_t bodyCount;
	int32_t shapeCount;
	int32_t contactCount;
	int32_t jointCount;
	int32_t islandCount;
	int32_t stackUsed;
	int32_t staticTreeHeight;
	int32_t treeHeight;
	int32_t byteCount;
	int32_t taskCount;
	int32_t colorCounts[b2_graphColorCount];
} b2Counters;
//! @endcond

/// Use this to initialize your world definition
/// @ingroup world
B2_API b2WorldDef b2DefaultWorldDef();

/// Use this to initialize your body definition
/// @ingroup body
B2_API b2BodyDef b2DefaultBodyDef();

/// Use this to initialize your filter
/// @ingroup shape
B2_API b2Filter b2DefaultFilter();

/// Use this to initialize your query filter
/// @ingroup shape
B2_API b2QueryFilter b2DefaultQueryFilter();

/// Use this to initialize your shape definition
/// @ingroup shape
B2_API b2ShapeDef b2DefaultShapeDef();

/// Use this to initialize your chain definition
/// @ingroup shape
B2_API b2ChainDef b2DefaultChainDef();
