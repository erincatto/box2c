// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "box2d/distance.h"
#include "box2d/math.h"

// A static rigid body
typedef struct b2StaticBody
{
	void* userData;

	// rotation in world space
	b2Rot rotation;

	// origin in world space
	b2Vec2 origin;

	// shapeId
	int shapeList;
	int shapeCount;

	// chainId
	int chainList;
	int chainCount;

	// This is a key: [jointId:31, edgeIndex:1]
	int jointList;
	int jointCount;

	// This is a key: [contactId:31, edgeIndex:1]
	int contactList;
	int contactCount;

	// body data can be moved around, the id is stable (used in b2StaticBodyId)
	int staticBodyId;
	uint16_t revision;
	int16_t world;
} b2StaticBody;
