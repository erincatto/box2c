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

static inline b2WorldId b2NullWorldId()
{
	b2WorldId nullId = {-1, 0};
	return nullId
}

static inline b2BodyId b2NullBodyId()
{
	b2BodyId nullId = {-1, -1, 0};
	return nullId
}

static inline b2ShapeId b2NullShapeId()
{
	b2ShapeId nullId = {-1, -1, 0};
	return nullId
}

static inline b2JointId b2NullJointId()
{
	b2JointId nullId = {-1, -1, 0};
	return nullId
}
