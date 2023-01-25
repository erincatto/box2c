// SPDX-FileCopyrightText: 2022 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include <stdint.h>

/// These ids serve as handles to internal Box2D objects. These should be considered opaque data and passed by value.

/// References a world instance
typedef struct b2WorldId
{
	int16_t index;
	uint16_t revision;
} b2WorldId;

/// References a rigid body instance
typedef struct b2BodyId
{
	int32_t index;
	int16_t world;
	uint16_t revision;
} b2BodyId;

/// References a shape instance
typedef struct b2ShapeId
{
	int32_t index;
	int16_t world;
	uint16_t revision;
} b2ShapeId;

/// References a joint instance
typedef struct b2JointId
{
	int32_t index;
	int16_t world;
	uint16_t revision;
} b2JointId;

static const b2WorldId b2_nullWorldId = {-1, 0};
static const b2BodyId b2_nullBodyId = {-1, -1, 0};
static const b2ShapeId b2_nullShapeId = {-1, -1, 0};
static const b2JointId b2_nullJointId = {-1, -1, 0};
