// SPDX-FileCopyrightText: 2022 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "box2d/types.h"
#include "box2d/constants.h"

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

#if 0
/// A shape is used for collision detection. You can create a shape however you like.
/// Shapes used for simulation in b2World are created automatically when a b2Fixture
/// is created. Shapes may encapsulate a one or more child shapes.
class B2_API b2Shape
{
public:

	enum Type
	{
		e_circle = 0,
		e_edge = 1,
		e_polygon = 2,
		e_chain = 3,
		e_typeCount = 4
	};

	virtual ~b2Shape() {}

	/// Clone the concrete shape using the provided allocator.
	virtual b2Shape* Clone(b2BlockAllocator* allocator) const = 0;

	/// Get the type of this shape. You can use this to down cast to the concrete shape.
	/// @return the shape type.
	Type GetType() const;

	/// Get the number of child primitives.
	virtual int32 GetChildCount() const = 0;

	/// Test a point for containment in this shape. This only works for convex shapes.
	/// @param xf the shape world transform.
	/// @param p a point in world coordinates.
	virtual bool TestPoint(const b2Transform& xf, const b2Vec2& p) const = 0;

	/// Cast a ray against a child shape.
	/// @param output the ray-cast results.
	/// @param input the ray-cast input parameters.
	/// @param transform the transform to be applied to the shape.
	/// @param childIndex the child shape index
	virtual bool RayCast(b2RayCastOutput* output, const b2RayCastInput& input,
						const b2Transform& transform, int32 childIndex) const = 0;

	/// Given a transform, compute the associated axis aligned bounding box for a child shape.
	/// @param aabb returns the axis aligned box.
	/// @param xf the world transform of the shape.
	/// @param childIndex the child shape
	virtual void ComputeAABB(b2AABB* aabb, const b2Transform& xf, int32 childIndex) const = 0;

	/// Compute the mass properties of this shape using its dimensions and density.
	/// The inertia tensor is computed about the local origin.
	/// @param massData returns the mass data for this shape.
	/// @param density the density in kilograms per meter squared.
	virtual void ComputeMass(b2MassData* massData, float density) const = 0;

	/// @brief Get the 
	/// @return 
	virtual float GetMinExtent() const { return 0.0f; }

	Type m_type;

	/// Radius of a shape
	/// TODO only for circles
	float m_radius;
};

inline b2Shape::Type b2Shape::GetType() const
{
	return m_type;
}

#endif

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
	int32_t count;
} b2PolygonShape;

typedef struct b2SegmentShape
{
	b2Vec2 point1, point2;
} b2SegmentShape;

/// A chain shape is a free form sequence of line segments.
/// The chain has one-sided collision, with the surface normal pointing to the right of the edge.
/// This provides a counter-clockwise winding like the polygon shape.
/// @warning the chain will not collide properly if there are self-intersections.
typedef struct b2ChainShape
{
	b2Vec2* m_vertices;
	int32_t m_count;
} b2ChainShape;


b2PolygonShape b2MakePolygon(const b2Hull* hull);
b2PolygonShape b2MakeBox(float hx, float hy, b2Vec2 center, float angle);

b2MassData b2ComputeCircleMass(const b2CircleShape* shape);
b2MassData b2ComputeCapsuleMass(const b2CapsuleShape* shape);
b2MassData b2ComputePolygonMass(const b2PolygonShape* shape);

b2AABB b2ComputeCircleAABB(const b2CircleShape* shape, b2Transform transform);
b2AABB b2ComputeCapsuleAABB(const b2CapsuleShape* shape, b2Transform transform);
b2AABB b2ComputePolygonAABB(const b2PolygonShape* shape, b2Transform transform);
b2AABB b2ComputeSegmentAABB(const b2SegmentShape* shape, b2Transform transform);

bool b2PointTestCircle(b2Vec2 point, b2CircleShape* shape, b2Transform transform);
bool b2PointTestCapsule(b2Vec2 point, b2CapsuleShape* shape, b2Transform transform);
bool b2PointTestPolygon(b2Vec2 point, b2PolygonShape* shape, b2Transform transform);

bool b2RayCastCircle(b2RayCastOutput* output, const b2RayCastInput* input, b2CircleShape* shape, b2Transform transform);
bool b2RayCastCapsule(b2RayCastOutput* output, const b2RayCastInput* input, b2CapsuleShape* shape, b2Transform transform);
bool b2RayCastSegment(b2RayCastOutput* output, const b2RayCastInput* input, b2SegmentShape* shape, b2Transform transform);
bool b2RayCastPolygon(b2RayCastOutput* output, const b2RayCastInput* input, b2PolygonShape* shape, b2Transform transform);
