// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include <stdint.h>

/// These ids serve as handles to internal Box2D objects. These should be considered opaque data and passed by value.
/// Include this header if you need the id definitions and not the whole Box2D API.

/// World identifier
typedef struct b2WorldId
{
	int16_t index;
	uint16_t revision;
} b2WorldId;

/// Body identitifier
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

/// References a chain instances
typedef struct b2ChainId
{
	int32_t index;
	int16_t world;
	uint16_t revision;
} b2ChainId;

/// Macros is needed for constant definitions
#define B2_NULL_BODY_ID {-1, -1, 0}

/// Use these to make your identifiers null
static const b2WorldId b2_nullWorldId = {-1, 0};
static const b2BodyId b2_nullBodyId = B2_NULL_BODY_ID;
static const b2ShapeId b2_nullShapeId = {-1, -1, 0};
static const b2JointId b2_nullJointId = {-1, -1, 0};
static const b2ChainId b2_nullChainId = {-1, -1, 0};

/// Macro to determine if any id is null
#define B2_IS_NULL(id) (id.index == -1)

/// Macro to determine if any id is non-null
#define B2_NON_NULL(id) (id.index != -1)

// Compare two ids for equality. Doesn't work for b2WorldId.
#define B2_ID_EQUALS(id1, id2) (id1.index == id2.index && id1.world == id2.world && id1.revision == id2.revision)
