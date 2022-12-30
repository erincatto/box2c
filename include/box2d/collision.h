// SPDX-FileCopyrightText: 2022 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include <limits.h>

#include "constants.h"
#include "types.h"
#include "vec_math.h"

/// @file
/// Structures and functions used for computing contact points, distance
/// queries, and TOI queries.

typedef struct b2Shape b2Shape;
typedef struct b2CircleShape b2CircleShape;
typedef struct b2EdgeShape b2EdgeShape;
typedef struct b2PolygonShape b2PolygonShape;

#define b2_nullFeature UCHAR_MAX;

enum
{
	b2_vertexFeature = 0,
	b2_faceFeature = 1
};

/// The features that intersect to form the contact point
/// This must be 4 bytes or less.
typedef struct b2ContactFeature
{
	uint8_t indexA;		///< Feature index on shapeA
	uint8_t indexB;		///< Feature index on shapeB
	uint8_t typeA;		///< The feature type on shapeA
	uint8_t typeB;		///< The feature type on shapeB
} b2ContactFeature;

/// Contact ids to facilitate warm starting.
typedef union b2ContactID
{
	b2ContactFeature cf;

	///< Used to quickly compare contact ids.
	uint32_t key;
} b2ContactID;

/// A manifold point is a contact point belonging to a contact
/// manifold. It holds details related to the geometry and dynamics
/// of the contact points.
/// The local point usage depends on the manifold type:
/// -e_circles: the local center of circleB
/// -e_faceA: the local center of cirlceB or the clip point of polygonB
/// -e_faceB: the clip point of polygonA
/// This structure is stored across time steps, so we keep it small.
/// Note: the impulses are used for internal caching and may not
/// provide reliable contact forces, especially for high speed collisions.
typedef struct b2ManifoldPoint
{
	b2Vec2 localPoint;		///< usage depends on manifold type
	float normalImpulse;	///< the non-penetration impulse
	float tangentImpulse;	///< the friction impulse
	b2ContactID id;			///< uniquely identifies a contact point between two shapes
	bool persisted;			///< did this contact point exist the previous step?
} b2ManifoldPoint;

typedef enum
{
	b2_manifoldCircles,
	b2_manifoldFaceA,
	b2_manifoldFaceB
} b2Manifold_Type;

/// A manifold for two touching convex shapes.
/// Box2D supports multiple types of contact:
/// - clip point versus plane with radius
/// - point versus point with radius (circles)
/// The local point usage depends on the manifold type:
/// -e_circles: the local center of circleA
/// -e_faceA: the center of faceA
/// -e_faceB: the center of faceB
/// Similarly the local normal usage:
/// -e_circles: not used
/// -e_faceA: the normal on polygonA
/// -e_faceB: the normal on polygonB
/// We store contacts in this way so that position correction can
/// account for movement, which is critical for continuous physics.
/// All contact scenarios must be expressed in one of these types.
/// This structure is stored across time steps, so we keep it small.
typedef struct b2Manifold
{

	b2ManifoldPoint points[b2_maxManifoldPoints];	///< the points of contact
	b2Vec2 localNormal;								///< not use for Type::e_points
	b2Vec2 localPoint;								///< usage depends on manifold type
	b2Manifold_Type type;
	int32_t pointCount;								///< the number of manifold points
} b2Manifold;

#if 0
/// This is used to compute the current state of a contact manifold.
typedef struct b2WorldManifold
{

	b2Vec2 normal;								///< world vector pointing from A to B
	b2Vec2 points[b2_maxManifoldPoints];		///< world contact point (point of intersection)
	float separations[b2_maxManifoldPoints];	///< a negative value indicates overlap, in meters
} b2WorldManifold;

/// Evaluate the manifold with supplied transforms. This assumes
/// modest motion from the original state. This does not change the
/// point count, impulses, etc. The radii must come from the shapes
/// that generated the manifold.
void b2WorldManifold_Initialize(b2WorldManifold* worldManifold, const b2Manifold* manifold,
				const b2Transform& xfA, float radiusA,
				const b2Transform& xfB, float radiusB);
#endif

/// This is used for determining the state of contact points.
typedef enum
{
	b2_nullState,		///< point does not exist
	b2_addState,		///< point was added in the update
	b2_persistState,	///< point persisted across the update
	b2_removeState		///< point was removed in the update
} b2PointState;

/// Compute the point states given two manifolds. The states pertain to the transition from manifold1
/// to manifold2. So state1 is either persist or remove while state2 is either add or persist.
void b2GetPointStates(b2PointState state1[b2_maxManifoldPoints], b2PointState state2[b2_maxManifoldPoints],
					  const b2Manifold* manifold1, const b2Manifold* manifold2);

/// Used for computing contact manifolds.
typedef struct b2ClipVertex
{
	b2Vec2 v;
	b2ContactID id;
} b2ClipVertex;

/// Ray-cast input data. The ray extends from p1 to p1 + maxFraction * (p2 - p1).
typedef struct b2RayCastInput
{
	b2Vec2 p1, p2;
	float maxFraction;
} b2RayCastInput;

/// Ray-cast output data. The ray hits at p1 + fraction * (p2 - p1), where p1 and p2
/// come from b2RayCastInput.
typedef struct b2RayCastOutput
{
	b2Vec2 normal;
	float fraction;
} b2RayCastOutput;

/// Verify that the bounds are sorted.
bool b2AABB_IsValid(b2AABB a);

/// Ray cast an AABB
bool b2AABB_RayCast(b2AABB a, b2RayCastOutput* output, const b2RayCastInput* input);

/// Compute the collision manifold between two circles.
void b2CollideCircles(b2Manifold* manifold,
					  const b2CircleShape* circleA, b2Transform xfA,
					  const b2CircleShape* circleB, b2Transform xfB);

/// Compute the collision manifold between a polygon and a circle.
void b2CollidePolygonAndCircle(b2Manifold* manifold,
							   const b2PolygonShape* polygonA, b2Transform xfA,
							   const b2CircleShape* circleB, b2Transform xfB);

/// Compute the collision manifold between two polygons.
void b2CollidePolygons(b2Manifold* manifold,
					   const b2PolygonShape* polygonA, b2Transform xfA,
					   const b2PolygonShape* polygonB, b2Transform xfB);

/// Compute the collision manifold between an edge and a circle.
void b2CollideEdgeAndCircle(b2Manifold* manifold,
							   const b2EdgeShape* polygonA, b2Transform xfA,
							   const b2CircleShape* circleB, b2Transform xfB);

/// Compute the collision manifold between an edge and a polygon.
void b2CollideEdgeAndPolygon(b2Manifold* manifold,
							   const b2EdgeShape* edgeA, b2Transform xfA,
							   const b2PolygonShape* polygonB, b2Transform xfB);

/// Clipping for contact manifolds.
int32_t b2ClipSegmentToLine(b2ClipVertex vOut[2], const b2ClipVertex vIn[2],
							b2Vec2 normal, float offset, int32_t vertexIndexA);

/// Determine if two generic shapes overlap.
bool b2TestOverlap(	const b2Shape* shapeA, int32_t indexA,
					const b2Shape* shapeB, int32_t indexB,
					b2Transform xfA, b2Transform xfB);

/// Convex hull used for polygon collision
typedef struct b2Hull
{
	b2Vec2 points[b2_maxPolygonVertices];
	int32_t count;
} b2Hull;

/// Compute the convex hull of a set of points. Returns an empty hull if it fails.
/// Some failure cases:
/// - all points very close together
/// - all points on a line
/// - less than 3 points
/// - more than b2_maxPolygonVertices points
/// This welds close points and removes collinear points.
b2Hull b2ComputeHull(const b2Vec2* points, int32_t count);

/// This determines if a hull is valid. Checks for:
/// - convexity
/// - collinear points
/// This is expensive and should not be called at runtime.
bool b2ValidateHull(const b2Hull* hull);


// ---------------- Inline Functions ------------------------------------------

/// Get the center of the AABB.
static inline b2Vec2 b2AABB_Center(b2AABB a)
{
	b2Vec2 b = { 0.5f * (a.lowerBound.x + a.upperBound.x),
					0.5f * (a.lowerBound.y + a.upperBound.y) };
	return b;
}

/// Get the extents of the AABB (half-widths).
static inline b2Vec2 b2AABB_Extents(b2AABB a)
{
	b2Vec2 b = { 0.5f * (a.upperBound.x - a.lowerBound.x),
					0.5f * (a.upperBound.y + a.lowerBound.y) };
	return b;
}

/// Get the perimeter length
static inline float b2AABB_Perimeter(b2AABB a)
{
	float wx = a.upperBound.x - a.lowerBound.x;
	float wy = a.upperBound.y - a.lowerBound.y;
	return 2.0f * (wx + wy);
}

/// Union of two AABBs
static inline b2AABB b2AABB_Union(b2AABB a, b2AABB b)
{
	b2AABB c;
	c.lowerBound.x = B2_MIN(a.lowerBound.x, b.lowerBound.x);
	c.lowerBound.y = B2_MIN(a.lowerBound.y, b.lowerBound.y);
	c.upperBound.x = B2_MAX(a.upperBound.x, b.upperBound.x);
	c.upperBound.y = B2_MAX(a.upperBound.y, b.upperBound.y);
	return c;
}

/// Does a fully contain b
static inline bool b2AABB_Contains(b2AABB a, b2AABB b)
{
	bool s = true;
	s = s && a.lowerBound.x <= b.lowerBound.x;
	s = s && a.lowerBound.y <= b.lowerBound.y;
	s = s && b.upperBound.x <= a.upperBound.x;
	s = s && b.upperBound.y <= a.upperBound.y;
	return s;
}

/// Do a and b overlap
static inline bool b2AABB_Overlaps(b2AABB a, b2AABB b)
{
	b2Vec2 d1, d2;
	d1 = b2Sub(b.lowerBound, a.upperBound);
	d2 = b2Sub(a.lowerBound, b.upperBound);

	if (d1.x > 0.0f || d1.y > 0.0f)
		return false;

	if (d2.x > 0.0f || d2.y > 0.0f)
		return false;

	return true;
}
