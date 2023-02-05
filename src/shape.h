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

// This proxy is used to connect shapes to the broad-phase.
typedef struct b2ShapeProxy
{
	b2AABB aabb;
	int32_t shapeIndex;
	int32_t childIndex;
	int32_t proxyKey;
} b2ShapeProxy;

typedef struct b2Shape
{
	b2Object object;
	int32_t bodyIndex;
	int32_t nextShapeIndex;
	enum b2ShapeType type;
	float density;
	float friction;
	float restitution;

	// Chain shapes need to have multiple proxies
	b2ShapeProxy* proxies;
	int32_t proxyCount;

	b2Filter filter;
	void* userData;

	struct b2ContactEdge* contacts;
	int32_t contactCount;

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
b2AABB b2Shape_ComputeAABB(const b2Shape* shape, b2Transform xf, int32_t childIndex);

void b2Shape_CreateProxies(b2Shape* shape, b2BroadPhase* bp, b2BodyType type, b2Transform xf);
void b2Shape_DestroyProxies(b2Shape* shape, b2BroadPhase* bp);

b2DistanceProxy b2Shape_MakeDistanceProxy(const b2Shape* shape, int32_t child);

float b2Shape_GetRadius(const b2Shape* shape);