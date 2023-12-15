// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "box2d/id.h"
#include "box2d/manifold.h"

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
	int beginCount;
	int endCount;
} b2SensorEvents;

/// A begin touch event is generated when two shapes begin touching.
typedef struct b2ContactBeginTouchEvent
{
	b2ShapeId shapeIdA;
	b2ShapeId shapeIdB;
	b2Manifold manifold;
} b2ContactBeginTouchEvent;

/// An end touch event is generated when two shapes stop touching.
typedef struct b2ContactEndTouchEvent
{
	b2ShapeId shapeIdA;
	b2ShapeId shapeIdB;
} b2ContactEndTouchEvent;

/// Contact events are buffered in the Box2D world and are available
///	as event arrays after the time step is complete.
///	Note: these may become invalid if bodies and/or shapes are destroyed
typedef struct b2ContactEvents
{
	b2ContactBeginTouchEvent* beginEvents;
	b2ContactEndTouchEvent* endEvents;
	int beginCount;
	int endCount;
} b2ContactEvents;

/// This is the data you can access using a b2ContactId
typedef struct b2ContactData
{
	b2ShapeId shapeIdA;
	b2ShapeId shapeIdB;
	b2Manifold manifold;
} b2ContactData;
