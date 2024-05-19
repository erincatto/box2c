// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "id.h"
#include "collision.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/**
 * @defgroup events Events
 * World event types.
 *
 * Events are used to collect events that occur during the world time step. These events
 * are then available to query after the time step is complete. This is preferable to callbacks
 * because Box2D uses multi-threaded simulation.
 *
 * Also when events occur in the simulation step it may be problematic to modify the world, which is
 * often what applications want to do when events occur.
 *
 * With event arrays, you can scan the events in a loop and modify the world. However, you need to be careful
 * that some event data may become invalid. There are several samples that show how to do this safely.
 *
 * @{
 */

/// A begin touch event is generated when a shape starts to overlap a sensor shape.
typedef struct b2SensorBeginTouchEvent
{
	/// The id of the sensor shape
	b2ShapeId sensorShapeId;

	/// The id of the dynamic shape that began touching the sensor shape
	b2ShapeId visitorShapeId;
} b2SensorBeginTouchEvent;

/// An end touch event is generated when a shape stops overlapping a sensor shape.
typedef struct b2SensorEndTouchEvent
{
	/// The id of the sensor shape
	b2ShapeId sensorShapeId;

	/// The id of the dynamic shape that stopped touching the sensor shape
	b2ShapeId visitorShapeId;
} b2SensorEndTouchEvent;

/// Sensor events are buffered in the Box2D world and are available
///	as begin/end overlap event arrays after the time step is complete.
///	Note: these may become invalid if bodies and/or shapes are destroyed
typedef struct b2SensorEvents
{
	/// Array of sensor begin touch events
	b2SensorBeginTouchEvent* beginEvents;

	/// Array of sensor end touch events
	b2SensorEndTouchEvent* endEvents;

	/// The number of begin touch events
	int32_t beginCount;

	/// The number of end touch events
	int32_t endCount;
} b2SensorEvents;

/// A begin touch event is generated when two shapes begin touching.
typedef struct b2ContactBeginTouchEvent
{
	/// Id of the first shape
	b2ShapeId shapeIdA;

	/// Id of the second shape
	b2ShapeId shapeIdB;
} b2ContactBeginTouchEvent;

/// An end touch event is generated when two shapes stop touching.
typedef struct b2ContactEndTouchEvent
{
	/// Id of the first shape
	b2ShapeId shapeIdA;

	/// Id of the second shape
	b2ShapeId shapeIdB;
} b2ContactEndTouchEvent;

/// A hit touch event is generated when two shapes collide with a speed faster than the hit speed threshold.
typedef struct b2ContactHitEvent
{
	/// Id of the first shape
	b2ShapeId shapeIdA;

	/// Id of the second shape
	b2ShapeId shapeIdB;

	/// Point where the shapes hit
	b2Vec2 point;

	/// Normal vector pointing from shape A to shape B
	b2Vec2 normal;

	/// The speed the shapes are approaching. Always positive. Typically in meters per second.
	float approachSpeed;
} b2ContactHitEvent;

/// Contact events are buffered in the Box2D world and are available
///	as event arrays after the time step is complete.
///	Note: these may become invalid if bodies and/or shapes are destroyed
typedef struct b2ContactEvents
{
	/// Array of begin touch events
	b2ContactBeginTouchEvent* beginEvents;

	/// Array of end touch events
	b2ContactEndTouchEvent* endEvents;

	/// Array of hit events
	b2ContactHitEvent* hitEvents;

	/// Number of begin touch events
	int32_t beginCount;

	/// Number of end touch events
	int32_t endCount;

	/// Number of hit events
	int32_t hitCount;
} b2ContactEvents;

/// Body move events triggered when a body moves.
/// Triggered when a body moves due to simulation. Not reported for bodies moved by the user.
/// This also has a flag to indicate that the body went to sleep so the application can also
/// sleep that actor/entity/object associated with the body.
/// On the other hand if the flag does not indicate the body went to sleep then the application
/// can treat the actor/entity/object associated with the body as awake.
///	This is an efficient way for an application to update game object transforms rather than
///	calling functions such as b2Body_GetTransform() because this data is delivered as a contiguous array
///	and it is only populated with bodies that have moved.
///	@note If sleeping is disabled all dynamic and kinematic bodies will trigger move events.
typedef struct b2BodyMoveEvent
{
	b2Transform transform;
	b2BodyId bodyId;
	void* userData;
	bool fellAsleep;
} b2BodyMoveEvent;

/// Body events are buffered in the Box2D world and are available
///	as event arrays after the time step is complete.
///	Note: this date becomes invalid if bodies are destroyed
typedef struct b2BodyEvents
{
	/// Array of move events
	b2BodyMoveEvent* moveEvents;

	/// Number of move events
	int32_t moveCount;
} b2BodyEvents;

/// The contact data for two shapes. By convention the manifold normal points
///	from shape A to shape B.
///	@see b2Shape_GetContactData() and b2Body_GetContactData()
typedef struct b2ContactData
{
	b2ShapeId shapeIdA;
	b2ShapeId shapeIdB;
	b2Manifold manifold;
} b2ContactData;

/**@}*/
