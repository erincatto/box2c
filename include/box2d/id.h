// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include <stdint.h>

// clang-format off
#ifdef __cplusplus
	#define B2_ZERO_INIT {}
#else
	/// Used for C zero initialization, such as b2BodyId id = {0} where C++ requires b2BodyId id = {}
	#define B2_ZERO_INIT {0}
#endif
// clang-format on

/// These ids serve as handles to internal Box2D objects. These should be considered opaque data and passed by value.
/// Include this header if you need the id definitions and not the whole Box2D API.

/// World identifier
typedef struct b2WorldId
{
	uint16_t index1;
	uint16_t revision;
} b2WorldId;

/// Static body identifier
typedef struct b2StaticBodyId
{
	int32_t index1;
	uint16_t world0;
	uint16_t revision;
} b2StaticBodyId;

/// Dynamic or kinematic body identifier
typedef struct b2BodyId
{
	int32_t index1;
	uint16_t world0;
	uint16_t revision;
} b2BodyId;

/// References a shape instance
typedef struct b2ShapeId
{
	int32_t index1;
	uint16_t world0;
	uint16_t revision;
} b2ShapeId;

/// References a joint instance
typedef struct b2JointId
{
	int32_t index1;
	uint16_t world0;
	uint16_t revision;
} b2JointId;

/// References a chain instances
typedef struct b2ChainId
{
	int32_t index1;
	int16_t world0;
	uint16_t revision;
} b2ChainId;

/// Use these to make your identifiers null.
/// You may also use zero initialization to get null.
static const b2WorldId b2_nullWorldId = B2_ZERO_INIT;
static const b2StaticBodyId b2_nullStaticBodyId = B2_ZERO_INIT;
static const b2BodyId b2_nullBodyId = B2_ZERO_INIT;
static const b2ShapeId b2_nullShapeId = B2_ZERO_INIT;
static const b2JointId b2_nullJointId = B2_ZERO_INIT;
static const b2ChainId b2_nullChainId = B2_ZERO_INIT;

/// Macro to determine if any id is null
#define B2_IS_NULL(id) (id.index1 == 0)

/// Macro to determine if any id is non-null
#define B2_IS_NON_NULL(id) (id.index1 != 0)

// Compare two ids for equality. Doesn't work for b2WorldId.
#define B2_ID_EQUALS(id1, id2) (id1.index1 == id2.index1 && id1.world0 == id2.world0 && id1.revision == id2.revision)
