// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "box2d/id.h"

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
typedef struct b2SensorEvents
{
	b2SensorBeginTouchEvent* beginEvents;
	b2SensorEndTouchEvent* endEvents;
	int beginCount;
	int endCount;
} b2SensorEvents;
