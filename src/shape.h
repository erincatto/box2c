// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "box2d/distance.h"
#include "box2d/geometry.h"
#include "box2d/types.h"

#include "pool.h"

typedef struct b2BroadPhase b2BroadPhase;

typedef enum b2ShapeType
{
	b2_circleShape,
	b2_polygonShape,
	b2_shapeTypeCount
} b2ShapeType;

typedef struct b2Shape
{
	b2Object object;
	int32_t bodyIndex;
	int32_t nextShapeIndex;
	enum b2ShapeType type;
	float density;
	float friction;
	float restitution;

	b2AABB aabb;
	b2AABB fatAABB;
	int32_t proxyKey;

	b2Filter filter;
	void* userData;

	bool isSensor;
	bool reportContacts;

	// TODO_ERIN maybe not anonymous, check asm
	union
	{
		b2Circle circle;
		b2Polygon polygon;
	};
} b2Shape;

b2MassData b2Shape_ComputeMass(const b2Shape* shape);
b2AABB b2Shape_ComputeAABB(const b2Shape* shape, b2Transform xf);

void b2Shape_CreateProxy(b2Shape* shape, b2BroadPhase* bp, b2BodyType type, b2Transform xf);
void b2Shape_DestroyProxy(b2Shape* shape, b2BroadPhase* bp);

b2DistanceProxy b2Shape_MakeDistanceProxy(const b2Shape* shape);

float b2Shape_GetRadius(const b2Shape* shape);
