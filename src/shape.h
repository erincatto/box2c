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

	// body key is split into bits
	// (31 | 1)
	// (bodyId | type)
	// where type is 0 for a static body or 1 for a dynamic/kinematic body
	int bodyKey;
	int nextShapeIndex;
	b2ShapeType type;
	float density;
	float friction;
	float restitution;

	b2AABB aabb;
	b2AABB fatAABB;
	b2Vec2 localCentroid;
	int proxyKey;

	b2Filter filter;
	void* userData;

	union
	{
		b2Capsule capsule;
		b2Circle circle;
		b2Polygon polygon;
		b2Segment segment;
		b2SmoothSegment smoothSegment;
	};

	bool isSensor;
	bool enableSensorEvents;
	bool enableContactEvents;
	bool enablePreSolveEvents;
	bool enlargedAABB;
	bool isFast;
} b2Shape;

typedef struct b2ChainShape
{
	b2Object object;
	int bodyId;
	int nextIndex;
	int* shapeIndices;
	int count;
} b2ChainShape;

typedef struct b2ShapeExtent
{
	float minExtent;
	float maxExtent;
} b2ShapeExtent;

void b2CreateShapeProxy(b2Shape* shape, b2BroadPhase* bp, b2BodyType type, b2Transform xf);
void b2DestroyShapeProxy(b2Shape* shape, b2BroadPhase* bp);

b2MassData b2ComputeShapeMass(const b2Shape* shape);
b2ShapeExtent b2ComputeShapeExtent(const b2Shape* shape);
b2AABB b2ComputeShapeAABB(const b2Shape* shape, b2Transform xf);
b2Vec2 b2GetShapeCentroid(const b2Shape* shape);

b2DistanceProxy b2MakeShapeDistanceProxy(const b2Shape* shape);

b2CastOutput b2RayCastShape(const b2RayCastInput* input, const b2Shape* shape, b2Transform xf);
b2CastOutput b2ShapeCastShape(const b2ShapeCastInput* input, const b2Shape* shape, b2Transform xf);

b2Shape* b2GetShape(b2World* world, b2ShapeId shapeId);
