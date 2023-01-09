// SPDX-FileCopyrightText: 2022 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "types.h"
#include "constants.h"

typedef struct b2Hull b2Hull;
typedef struct b2RayCastOutput b2RayCastOutput;
typedef struct b2RayCastInput b2RayCastInput;

/// This holds the mass data computed for a shape.
typedef struct b2MassData
{
	/// The mass of the shape, usually in kilograms.
	float mass;

	/// The position of the shape's centroid relative to the shape's origin.
	b2Vec2 center;

	/// The rotational inertia of the shape about the local origin.
	float I;
} b2MassData;

/// A solid circle
typedef struct b2CircleShape
{
	b2Vec2 point;
	float radius;
} b2CircleShape;

/// A solid capsule shape
typedef struct b2CapsuleShape
{
	b2Vec2 point1, point2;
	float radius;
} b2CapsuleShape;

/// A solid convex polygon. It is assumed that the interior of the polygon is to
/// the left of each edge.
/// Polygons have a maximum number of vertices equal to b2_maxPolygonVertices.
/// In most cases you should not need many vertices for a convex polygon.
typedef struct b2PolygonShape
{
	b2Vec2 vertices[b2_maxPolygonVertices];
	b2Vec2 normals[b2_maxPolygonVertices];
	b2Vec2 centroid;
	int32_t count;
} b2PolygonShape;

/// A line segment with two-sided collision.
typedef struct b2SegmentShape
{
	b2Vec2 point1, point2;
} b2SegmentShape;

/// A smooth line segment with one-sided collision. Only collides on the right side.
/// Normally these are generated from a chain shape.
/// ghost1 -> point1 -> point2 -> ghost2
/// This is only relevant for contact manifolds, otherwise use a regular segment.
typedef struct b2SmoothSegmentShape
{
	/// The tail ghost vertex
	b2Vec2 ghost1;

	/// The line segment
	b2Vec2 point1, point2;
	
	/// The head ghost vertex
	b2Vec2 ghost2;
} b2SmoothSegmentShape;


#ifdef __cplusplus
extern "C"
{
#endif

b2PolygonShape b2MakePolygon(const b2Hull* hull);
b2PolygonShape b2MakeBox(float hx, float hy, b2Vec2 center, float angle);

b2MassData b2ComputeCircleMass(const b2CircleShape* shape, float density);
b2MassData b2ComputeCapsuleMass(const b2CapsuleShape* shape, float density);
b2MassData b2ComputePolygonMass(const b2PolygonShape* shape, float density);

b2AABB b2ComputeCircleAABB(const b2CircleShape* shape, b2Transform xf);
b2AABB b2ComputeCapsuleAABB(const b2CapsuleShape* shape, b2Transform xf);
b2AABB b2ComputePolygonAABB(const b2PolygonShape* shape, b2Transform xf);
b2AABB b2ComputeSegmentAABB(const b2SegmentShape* shape, b2Transform xf);

bool b2PointInCircle(b2Vec2 point, const b2CircleShape* shape, b2Transform xf);
bool b2PointInCapsule(b2Vec2 point, const b2CapsuleShape* shape, b2Transform xf);
bool b2PointInPolygon(b2Vec2 point, const b2PolygonShape* shape, b2Transform xf);

// Ray cast versus shape. Initial overlap is treated as a miss.
b2RayCastOutput b2RayCastCircle(const b2RayCastInput* input, const b2CircleShape* shape, b2Transform xf);
b2RayCastOutput b2RayCastCapsule(const b2RayCastInput* input, const b2CapsuleShape* shape, b2Transform xf);
b2RayCastOutput b2RayCastSegment(const b2RayCastInput* input, const b2SegmentShape* shape, b2Transform xf);
b2RayCastOutput b2RayCastPolygon(const b2RayCastInput* input, const b2PolygonShape* shape, b2Transform xf);

#ifdef __cplusplus
}
#endif
