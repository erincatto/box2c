// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "box2d/id.h"
#include "box2d/types.h"

#include <stdbool.h>
#include <stdint.h>

#define B2_DYNAMIC_BIT 0x80000000

typedef struct b2World b2World;

// Body organizational details that are not used in the solver.
typedef struct b2BodyLookup
{
	void* userData;

	// index of solver set stored in b2World
	// may be B2_NULL_INDEX
	int setIndex;

	// body sim and state index within set
	// may be B2_NULL_INDEX
	int localIndex;

	// [31 : contactId | 1 : edgeIndex]
	int headContactKey;
	int contactCount;

	int headShapeId;
	int shapeCount;

	int headChainId;

	// [31 : contactId | 1 : edgeIndex]
	int headJointKey;
	int jointCount;

	// All enabled dynamic and kinematic bodies are in an island.
	int islandId;

	// doubly-linked island list
	int islandPrev;
	int islandNext;

	// todo maybe remove this
	int bodyId;

	// This is monotonically advanced when a body is allocated in this slot
	// Used to check for invalid b2BodyId
	uint16_t revision;

	int16_t worldId;

} b2BodyLookup;

b2BodyLookup* b2GetBodyFullId(b2World* world, b2BodyId bodyId);

b2BodyLookup* b2GetBody(b2World* world, int bodyId);

// Get a validated body from a world using an id.
// todo remove this function and instead use B2_ASSERT(b2Body_IsValid(bodyId))
b2BodyLookup* b2GetBodyFullId(b2World* world, b2BodyId bodyId);

b2Transform b2GetBodyTransformQuick(b2World* world, b2BodyLookup* body);
b2Transform b2GetBodyTransform(b2World* world, int bodyId);

// Create a b2BodyId from a key.
b2BodyId b2MakeBodyId(b2World* world, int bodyKey);

bool b2ShouldBodiesCollide(b2World* world, b2BodyLookup* bodyA, b2BodyLookup* bodyB);
bool b2IsBodyAwake(b2World* world, int bodyId);
