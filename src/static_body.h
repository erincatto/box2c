// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "contact_list.h"

#include "box2d/math_functions.h"

#include <stdint.h>

typedef struct b2World b2World;

// A static rigid body
typedef struct b2StaticBody
{
	void* userData;

	b2Transform transform;
	b2ShapeList shapeList;
	b2ChainList chainList;
	b2ContactList contactList;

	int bodyId;
	uint16_t revision;
	int16_t worldId;
} b2StaticBody;


b2StaticBody* b2GetStaticBodyFullId(b2World* world, b2StaticBodyId bodyId);
b2StaticBody* b2GetStaticBody(b2World* world, int bodyId);
b2Transform b2GetStaticBodyTransform(b2World* world, int bodyId);

void b2DestroyStaticShape(b2World* world, b2Shape* shape);
