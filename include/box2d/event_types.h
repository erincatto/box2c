// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "id.h"
#include "manifold.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/// A begin touch event is generated when a shape starts to overlap a sensor shape.
typedef struct b2SensorBeginTouchEvent
{
	b2ShapeId sensorShapeId;
	b2ShapeId visitorShapeId;
} b2SensorBeginTouchEvent;

/// An end touch event is generated when a shape stops overlapping a sensor shape.
typedef struct b2SensorEndTouchEvent
{
	b2ShapeId sensorShapeId;
	b2ShapeId visitorShapeId;
} b2SensorEndTouchEvent;

/// Sensor events are buffered in the Box2D world and are available
///	as begin/end overlap event arrays after the time step is complete.
///	Note: these may become invalid if bodies and/or shapes are destroyed
typedef struct b2SensorEvents
{
	b2SensorBeginTouchEvent* beginEvents;
	b2SensorEndTouchEvent* endEvents;
	int32_t beginCount;
	int32_t endCount;
} b2SensorEvents;

/// A begin touch event is generated when two shapes begin touching. By convention the manifold
/// normal points from shape A to shape B.
typedef struct b2ContactBeginTouchEvent
{
	b2ShapeId shapeIdA;
	b2ShapeId shapeIdB;
} b2ContactBeginTouchEvent;

/// An end touch event is generated when two shapes stop touching.
typedef struct b2ContactEndTouchEvent
{
	b2ShapeId shapeIdA;
	b2ShapeId shapeIdB;
} b2ContactEndTouchEvent;

/// A hit touch event is generated when two shapes collide with a speed faster than the hit speed threshold.
typedef struct b2ContactHitEvent
{
	b2ShapeId shapeIdA;
	b2ShapeId shapeIdB;

	// point where the shapes hit
	b2Vec2 point;

	// normal vector pointing from shape A to shape B
	b2Vec2 normal;

	float approachSpeed;
} b2ContactHitEvent;

/// Contact events are buffered in the Box2D world and are available
///	as event arrays after the time step is complete.
///	Note: these may become invalid if bodies and/or shapes are destroyed
typedef struct b2ContactEvents
{
	b2ContactBeginTouchEvent* beginEvents;
	b2ContactEndTouchEvent* endEvents;
	int32_t beginCount;
	int32_t endCount;
} b2ContactEvents;

/// The contact data for two shapes. By convention the manifold normal points
///	from shape A to shape B.
typedef struct b2ContactData
{
	b2ShapeId shapeIdA;
	b2ShapeId shapeIdB;
	b2Manifold manifold;
} b2ContactData;

/// Triggered when a body moves from simulation. Not reported for bodies moved by the user.
/// This also has a flag to indicate that the body went to sleep so the application can also
/// sleep that actor/entity/object associated with the body.
/// On the other hand if the flag does not indicate the body went to sleep then the application
/// can treat the actor/entity/object associated with the body as awake.
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
	b2BodyMoveEvent* moveEvents;
	int32_t moveCount;
} b2BodyEvents;
