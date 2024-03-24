// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "contact_list.h"
#include "pool.h"

#include "box2d/math.h"

#include <stdint.h>

typedef struct b2World b2World;

// A static rigid body
typedef struct b2StaticBody
{
	void* userData;

	// rotation in world space
	b2Rot rotation;

	// origin in world space
	b2Vec2 origin;

	b2ShapeList shapeList;

	// chainId
	int chainList;
	int chainCount;

	b2ContactList contactList;

	int bodyId;
	uint16_t revision;
	int16_t worldId;
} b2StaticBody;


inline b2Transform b2MakeStaticTransform(b2StaticBody* staticBody)
{
	return (b2Transform){staticBody->origin, staticBody->rotation};
}

b2StaticBody* b2GetStaticBody(b2World* world, int bodyId);
b2Transform b2GetStaticBodyTransform(b2World* world, int bodyId);

