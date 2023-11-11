// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "pool.h"

#include "box2d/distance.h"
#include "box2d/geometry.h"
#include "box2d/types.h"

typedef struct b2BroadPhase b2BroadPhase;

typedef enum b2ShapeType
{
	b2_capsuleShape,
	b2_circleShape,
	b2_polygonShape,
	b2_segmentShape,
	b2_chainShape,
	b2_shapeTypeCount
} b2ShapeType;

/// A smooth line segment with one-sided collision. Only collides on the right side.
/// Several of these are generated for a chain shape.
/// ghost1 -> point1 -> point2 -> ghost2
typedef struct b2Chain
{
	/// The tail ghost vertex
	b2Vec2 ghost1;

	/// The line segment
	b2Vec2 point1, point2;

	/// The head ghost vertex
	b2Vec2 ghost2;
} b2Chain;

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
	bool enlargedAABB;
	bool isFast;

	// TODO_ERIN maybe not anonymous, check asm
	union
	{
		b2Capsule capsule;
		b2Circle circle;
		b2Polygon polygon;
		b2Segment segment;
		b2Chain chain;
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

b2MassData b2ComputeShapeMass(const b2Shape* shape);
b2AABB b2ComputeShapeAABB(const b2Shape* shape, b2Transform xf);

void b2CreateShapeProxy(b2Shape* shape, b2BroadPhase* bp, b2BodyType type, b2Transform xf);
void b2DestroyShapeProxy(b2Shape* shape, b2BroadPhase* bp);

b2DistanceProxy b2MakeShapeDistanceProxy(const b2Shape* shape);
