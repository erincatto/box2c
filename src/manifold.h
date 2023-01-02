// SPDX-FileCopyrightText: 2022 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "box2d/types.h"

#define b2_nullFeature UCHAR_MAX
#define b2_maxManifoldPoints 2

typedef struct b2CircleShape b2CircleShape;
typedef struct b2CapsuleShape b2CapsuleShape;
typedef struct b2PolygonShape b2PolygonShape;
typedef struct b2SegmentShape b2SegmentShape;

enum
{
	b2_vertexFeature = 0,
	b2_faceFeature = 1
};

/// The features that intersect to form the contact point
/// This must be 4 bytes or less.
typedef struct b2ContactFeature
{
	uint8_t indexA; ///< Feature index on shapeA
	uint8_t indexB; ///< Feature index on shapeB
	uint8_t typeA;  ///< The feature type on shapeA
	uint8_t typeB;  ///< The feature type on shapeB
} b2ContactFeature;

/// Contact ids to facilitate warm starting.
typedef union b2ContactID {
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
	b2Vec2 localPoint;    ///< usage depends on manifold type
	float normalImpulse;  ///< the non-penetration impulse
	float tangentImpulse; ///< the friction impulse
	b2ContactID id;       ///< uniquely identifies a contact point between two shapes
	bool persisted;       ///< did this contact point exist the previous step?
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

	b2ManifoldPoint points[b2_maxManifoldPoints]; ///< the points of contact
	b2Vec2 localNormal;                           ///< not use for Type::e_points
	b2Vec2 localPoint;                            ///< usage depends on manifold type
	b2Manifold_Type type;
	int32_t pointCount; ///< the number of manifold points
} b2Manifold;

/// This is used to compute the current state of a contact manifold.
typedef struct b2WorldManifold
{

	b2Vec2 normal;                           ///< world vector pointing from A to B
	b2Vec2 points[b2_maxManifoldPoints];     ///< world contact point (point of intersection)
	float separations[b2_maxManifoldPoints]; ///< a negative value indicates overlap, in meters
} b2WorldManifold;

/// Evaluate the manifold with supplied transforms. This assumes
/// modest motion from the original state. This does not change the
/// point count, impulses, etc. The radii must come from the shapes
/// that generated the manifold.
void b2WorldManifold_Initialize(b2WorldManifold* worldManifold, const b2Manifold* manifold, b2Transform xfA,
                                float radiusA, b2Transform xfB, float radiusB);

/// This is used for determining the state of contact points.
typedef enum
{
	b2_nullState,    ///< point does not exist
	b2_addState,     ///< point was added in the update
	b2_persistState, ///< point persisted across the update
	b2_removeState   ///< point was removed in the update
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

/// Clipping for contact manifolds.
int32_t b2ClipSegmentToLine(b2ClipVertex vOut[2], const b2ClipVertex vIn[2], b2Vec2 normal, float offset,
                            int32_t vertexIndexA);

/// Compute the collision manifold between two circles.
void b2CollideCircles(b2Manifold* manifold, const b2CircleShape* circleA, b2Transform xfA, const b2CircleShape* circleB,
                      b2Transform xfB);

/// Compute the collision manifold between a polygon and a circle.
void b2CollidePolygonAndCircle(b2Manifold* manifold, const b2PolygonShape* polygonA, b2Transform xfA,
                               const b2CircleShape* circleB, b2Transform xfB);

/// Compute the collision manifold between two polygons.
void b2CollidePolygons(b2Manifold* manifold, const b2PolygonShape* polygonA, b2Transform xfA,
                       const b2PolygonShape* polygonB, b2Transform xfB);

/// Compute the collision manifold between an edge and a circle.
void b2CollideSegmentAndCircle(b2Manifold* manifold, const b2SegmentShape* polygonA, b2Transform xfA,
                               const b2CircleShape* circleB, b2Transform xfB);

/// Compute the collision manifold between an edge and a polygon.
void b2CollideSegmentAndPolygon(b2Manifold* manifold, const b2SegmentShape* edgeA, b2Transform xfA,
                                const b2PolygonShape* polygonB, b2Transform xfB);
