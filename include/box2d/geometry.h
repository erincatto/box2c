// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "box2d/api.h"
#include "box2d/constants.h"
#include "box2d/types.h"

typedef struct b2Hull b2Hull;
typedef struct b2CastOutput b2RayCastOutput;
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

	/// TODO_ERIN remove geometry info from this

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
///	@warning DO NOT fill this out manually, instead use a helper function like
///	b2MakePolygon or b2MakeBox.
typedef struct b2Polygon
{
	b2Vec2 vertices[b2_maxPolygonVertices];
	b2Vec2 normals[b2_maxPolygonVertices];
	b2Vec2 centroid;
	float radius;
	int32_t count;
} b2Polygon;

/// A line segment with two-sided collision.
typedef struct b2Segment
{
	b2Vec2 point1, point2;
} b2Segment;

/// A smooth line segment with one-sided collision. Only collides on the right side.
/// Several of these are generated for a chain shape.
/// ghost1 -> point1 -> point2 -> ghost2
typedef struct b2SmoothSegment
{
	/// The tail ghost vertex
	b2Vec2 ghost1;

	/// The line segment
	b2Segment segment;

	/// The head ghost vertex
	b2Vec2 ghost2;

	/// The owning chain shape index (internal usage only)
	int32_t chainIndex;
} b2SmoothSegment;

/// Validate ray cast input data (NaN, etc)
BOX2D_API bool b2IsValidRay(const b2RayCastInput* input);

/// Make a convex polygon from a convex hull. This will assert if the hull is not valid.
BOX2D_API b2Polygon b2MakePolygon(const b2Hull* hull, float radius);

/// Make an offset convex polygon from a convex hull. This will assert if the hull is not valid.
BOX2D_API b2Polygon b2MakeOffsetPolygon(const b2Hull* hull, float radius, b2Transform transform);

/// Make a square polygon, bypassing the need for a convex hull.
BOX2D_API b2Polygon b2MakeSquare(float h);

/// Make a box (rectangle) polygon, bypassing the need for a convex hull.
BOX2D_API b2Polygon b2MakeBox(float hx, float hy);

/// Make a rounded box, bypassing the need for a convex hull.
BOX2D_API b2Polygon b2MakeRoundedBox(float hx, float hy, float radius);

/// Make an offset box, bypassing the need for a convex hull.
BOX2D_API b2Polygon b2MakeOffsetBox(float hx, float hy, b2Vec2 center, float angle);

/// Transform a polygon. This is useful for transfering a shape from one body to another.
BOX2D_API b2Polygon b2TransformPolygon(b2Transform transform, const b2Polygon* polygon);

/// Compute mass properties of a circle
BOX2D_API b2MassData b2ComputeCircleMass(const b2Circle* shape, float density);

/// Compute mass properties of a capsule
BOX2D_API b2MassData b2ComputeCapsuleMass(const b2Capsule* shape, float density);

/// Compute mass properties of a polygon
BOX2D_API b2MassData b2ComputePolygonMass(const b2Polygon* shape, float density);

/// Compute the bounding box of a transformed circle
BOX2D_API b2AABB b2ComputeCircleAABB(const b2Circle* shape, b2Transform transform);

/// Compute the bounding box of a transformed capsule
BOX2D_API b2AABB b2ComputeCapsuleAABB(const b2Capsule* shape, b2Transform transform);

/// Compute the bounding box of a transformed polygon
BOX2D_API b2AABB b2ComputePolygonAABB(const b2Polygon* shape, b2Transform transform);

/// Compute the bounding box of a transformed line segment
BOX2D_API b2AABB b2ComputeSegmentAABB(const b2Segment* shape, b2Transform transform);

/// Test a point for overlap with a circle in local space
BOX2D_API bool b2PointInCircle(b2Vec2 point, const b2Circle* shape);

/// Test a point for overlap with a capsule in local space
BOX2D_API bool b2PointInCapsule(b2Vec2 point, const b2Capsule* shape);

/// Test a point for overlap with a convex polygon in local space
BOX2D_API bool b2PointInPolygon(b2Vec2 point, const b2Polygon* shape);

/// Ray cast versus circle in shape local space. Initial overlap is treated as a miss.
BOX2D_API b2RayCastOutput b2RayCastCircle(const b2RayCastInput* input, const b2Circle* shape);

/// Ray cast versus capsule in shape local space. Initial overlap is treated as a miss.
BOX2D_API b2RayCastOutput b2RayCastCapsule(const b2RayCastInput* input, const b2Capsule* shape);

/// Ray cast versus segment in shape local space. Optionally treat the segment as one-sided with hits from
/// the left side being treated as a miss.
BOX2D_API b2RayCastOutput b2RayCastSegment(const b2RayCastInput* input, const b2Segment* shape, bool oneSided);

/// Ray cast versus polygon in shape local space. Initial overlap is treated as a miss.
BOX2D_API b2RayCastOutput b2RayCastPolygon(const b2RayCastInput* input, const b2Polygon* shape);

/// Shape cast versus a circle. Initial overlap is treated as a miss.
BOX2D_API b2RayCastOutput b2ShapeCastCircle(const b2ShapeCastInput* input, const b2Circle* shape);

/// Shape cast versus a capsule. Initial overlap is treated as a miss.
BOX2D_API b2RayCastOutput b2ShapeCastCapsule(const b2ShapeCastInput* input, const b2Capsule* shape);

/// Shape cast versus a line segment. Initial overlap is treated as a miss.
BOX2D_API b2RayCastOutput b2ShapeCastSegment(const b2ShapeCastInput* input, const b2Segment* shape);

/// Shape cast versus a convex polygon. Initial overlap is treated as a miss.
BOX2D_API b2RayCastOutput b2ShapeCastPolygon(const b2ShapeCastInput* input, const b2Polygon* shape);
