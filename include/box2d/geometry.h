// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "box2d/api.h"
#include "box2d/constants.h"
#include "box2d/types.h"

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

	/// Distance from shape centroid to closest point on perimeter.
	float minExtent;

	/// Distance from shape origin to furthest point on perimeter.
	float maxExtent;
} b2MassData;

/// A solid circle
typedef struct b2Circle
{
	b2Vec2 point;
	float radius;
} b2Circle;

/// A solid capsule
typedef struct b2Capsule
{
	b2Vec2 point1, point2;
	float radius;
} b2Capsule;

/// A solid convex polygon. It is assumed that the interior of the polygon is to
/// the left of each edge.
/// Polygons have a maximum number of vertices equal to b2_maxPolygonVertices.
/// In most cases you should not need many vertices for a convex polygon.
typedef struct b2Polygon
{
	b2Vec2 vertices[b2_maxPolygonVertices];
	b2Vec2 normals[b2_maxPolygonVertices];
	float radius;
	int32_t count;
} b2Polygon;

/// A line segment with two-sided collision.
typedef struct b2Segment
{
	b2Vec2 point1, point2;
} b2Segment;

/// A smooth line segment with one-sided collision. Only collides on the right side.
/// Normally these are generated from a chain shape.
/// ghost1 -> point1 -> point2 -> ghost2
/// This is only relevant for contact manifolds, otherwise use a regular segment.
typedef struct b2SmoothSegment
{
	/// The tail ghost vertex
	b2Vec2 ghost1;

	/// The line segment
	b2Vec2 point1, point2;

	/// The head ghost vertex
	b2Vec2 ghost2;
} b2SmoothSegment;

BOX2D_API bool b2IsValidRay(const b2RayCastInput* input);

/// Helper functions to make convex polygons
BOX2D_API b2Polygon b2MakePolygon(const b2Hull* hull, float radius);
BOX2D_API b2Polygon b2MakeSquare(float h);
BOX2D_API b2Polygon b2MakeBox(float hx, float hy);
BOX2D_API b2Polygon b2MakeRoundedBox(float hx, float hy, float radius);
BOX2D_API b2Polygon b2MakeOffsetBox(float hx, float hy, b2Vec2 center, float angle);
BOX2D_API b2Polygon b2MakeCapsule(b2Vec2 p1, b2Vec2 p2, float radius);

/// Compute mass properties
BOX2D_API b2MassData b2ComputeCircleMass(const b2Circle* shape, float density);
BOX2D_API b2MassData b2ComputeCapsuleMass(const b2Capsule* shape, float density);
BOX2D_API b2MassData b2ComputePolygonMass(const b2Polygon* shape, float density);

/// These compute the bounding box in world space
BOX2D_API b2AABB b2ComputeCircleAABB(const b2Circle* shape, b2Transform xf);
BOX2D_API b2AABB b2ComputeCapsuleAABB(const b2Capsule* shape, b2Transform xf);
BOX2D_API b2AABB b2ComputePolygonAABB(const b2Polygon* shape, b2Transform xf);
BOX2D_API b2AABB b2ComputeSegmentAABB(const b2Segment* shape, b2Transform xf);

/// Test a point in local space
BOX2D_API bool b2PointInCircle(b2Vec2 point, const b2Circle* shape);
BOX2D_API bool b2PointInCapsule(b2Vec2 point, const b2Capsule* shape);
BOX2D_API bool b2PointInPolygon(b2Vec2 point, const b2Polygon* shape);

// Ray cast versus shape in shape local space. Initial overlap is treated as a miss.
BOX2D_API b2RayCastOutput b2RayCastCircle(const b2RayCastInput* input, const b2Circle* shape);
BOX2D_API b2RayCastOutput b2RayCastCapsule(const b2RayCastInput* input, const b2Capsule* shape);
BOX2D_API b2RayCastOutput b2RayCastSegment(const b2RayCastInput* input, const b2Segment* shape);
BOX2D_API b2RayCastOutput b2RayCastPolygon(const b2RayCastInput* input, const b2Polygon* shape);
