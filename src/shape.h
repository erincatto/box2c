// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "pool.h"

#include "box2d/distance.h"
#include "box2d/geometry.h"
#include "box2d/id.h"
#include "box2d/types.h"

typedef struct b2BroadPhase b2BroadPhase;
typedef struct b2World b2World;

typedef struct b2Shape
{
	b2Object object;
	int32_t bodyIndex;
	int32_t nextShapeIndex;
	b2ShapeType type;
	float density;
	float friction;
	float restitution;

	b2AABB aabb;
	b2AABB fatAABB;
	b2Vec2 localCentroid;
	int32_t proxyKey;

	b2Filter filter;
	void* userData;

	bool isSensor;
	bool enableSensorEvents;
	bool enableContactEvents;
	bool enablePreSolveEvents;
	bool enlargedAABB;
	bool isFast;

	// TODO_ERIN maybe not anonymous, check asm
	union
	{
		b2Capsule capsule;
		b2Circle circle;
		b2Polygon polygon;
		b2Segment segment;
		b2SmoothSegment smoothSegment;
	};
} b2Shape;

typedef struct b2ChainShape
{
	b2Object object;
	int32_t bodyIndex;
	int32_t nextIndex;
	int32_t* shapeIndices;
	int32_t count;
} b2ChainShape;

void b2CreateShapeProxy(b2Shape* shape, b2BroadPhase* bp, b2BodyType type, b2Transform xf);
void b2DestroyShapeProxy(b2Shape* shape, b2BroadPhase* bp);

b2MassData b2ComputeShapeMass(const b2Shape* shape);
b2AABB b2ComputeShapeAABB(const b2Shape* shape, b2Transform xf);
b2Vec2 b2GetShapeCentroid(const b2Shape* shape);

b2DistanceProxy b2MakeShapeDistanceProxy(const b2Shape* shape);

b2RayCastOutput b2RayCastShape(const b2RayCastInput* input, const b2Shape* shape, b2Transform xf);
b2RayCastOutput b2ShapeCastShape(const b2ShapeCastInput* input, const b2Shape* shape, b2Transform xf);

b2Shape* b2GetShape(b2World* world, b2ShapeId shapeId);
