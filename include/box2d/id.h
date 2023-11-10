// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include <stdint.h>

/// These ids serve as handles to internal Box2D objects. These should be considered opaque data and passed by value.
/// Include this header if you need the id definitions and not the whole Box2D API.

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

typedef struct b2ChainId
{
	int16_t index;
	uint16_t revision;
} b2ChainId;

static const b2WorldId b2_nullWorldId = {-1, 0};
static const b2BodyId b2_nullBodyId = {-1, -1, 0};
static const b2ShapeId b2_nullShapeId = {-1, -1, 0};
static const b2JointId b2_nullJointId = {-1, -1, 0};
static const b2ChainId b2_nullChainId = {-1, 0};

#define B2_IS_NULL(ID) (ID.index == -1)
#define B2_NON_NULL(ID) (ID.index != -1)
