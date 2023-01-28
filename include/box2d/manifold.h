// SPDX-FileCopyrightText: 2022 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "box2d/types.h"

#define b2_nullFeature UCHAR_MAX
#define b2_maxManifoldPoints 2

typedef struct b2Circle b2Circle;
typedef struct b2Capsule b2Capsule;
typedef struct b2Polygon b2Polygon;
typedef struct b2Segment b2Segment;
typedef struct b2SmoothSegment b2SmoothSegment;

enum
{
	b2_vertexFeature = 0,
	b2_faceFeature = 1
};

/// The features that intersect to form the contact point
/// This must be 4 bytes or less.
/// TODO eliminate this generic solution. Instead
/// use a key that makes sense for each shape pair.
typedef struct b2ContactFeature
{
	uint8_t indexA; ///< Feature index on shapeA
	uint8_t indexB; ///< Feature index on shapeB
	uint8_t typeA;	///< The feature type on shapeA
	uint8_t typeB;	///< The feature type on shapeB
} b2ContactFeature;

/// Contact ids to facilitate warm starting.
/// Used to quickly compare contact ids via type punning.
/// TODO No need for this type punning. Just build the key during manifold generation
typedef union b2ContactID {
	b2ContactFeature cf;
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
	b2Vec2 localPoint;	  ///< usage depends on manifold type
	float normalImpulse;  ///< the non-penetration impulse
	float tangentImpulse; ///< the friction impulse
	b2ContactID id;		  ///< uniquely identifies a contact point between two shapes
	bool persisted;		  ///< did this contact point exist the previous step?
} b2ManifoldPoint;

typedef enum
{
	b2_manifoldCircles,
	b2_manifoldFaceA,
	b2_manifoldFaceB
} b2ManifoldType;

/// A local manifold for two touching convex shapes.
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
typedef struct b2Manifold
{
	b2ManifoldPoint points[b2_maxManifoldPoints];
	b2Vec2 localNormal;
	b2Vec2 localPoint;
	b2ManifoldType type;
	int32_t pointCount;
} b2Manifold;

/// This is used to compute the current state of a contact manifold.
typedef struct b2WorldManifold
{
	b2Vec2 normal;							 ///< world vector pointing from A to B
	b2Vec2 points[b2_maxManifoldPoints];	 ///< world contact point (point of intersection)
	float separations[b2_maxManifoldPoints]; ///< a negative value indicates overlap, in meters
} b2WorldManifold;

/// This is used for determining the state of contact points.
typedef enum
{
	b2_nullState,	 ///< point does not exist
	b2_addState,	 ///< point was added in the update
	b2_persistState, ///< point persisted across the update
	b2_removeState	 ///< point was removed in the update
} b2PointState;

#ifdef __cplusplus
extern "C"
{
#endif

/// Evaluate the manifold with supplied transforms. This assumes small motion from the original state. This does not
/// change the point count, impulses, etc. The radii must come from the shapes that generated the manifold.
b2WorldManifold b2ComputeWorldManifold(const b2Manifold* manifold, b2Transform xfA, float radiusA, b2Transform xfB,
									   float radiusB);

/// Compute the point states given two manifolds. The states pertain to the transition from manifold1
/// to manifold2. So state1 is either persist or remove while state2 is either add or persist.
void b2GetPointStates(b2PointState state1[b2_maxManifoldPoints], b2PointState state2[b2_maxManifoldPoints],
					  const b2Manifold* manifold1, const b2Manifold* manifold2);

/// Compute the collision manifold between two circles.
b2Manifold b2CollideCircles(const b2Circle* circleA, const b2Circle* circleB);

/// Compute the collision manifold between a capulse and circle
b2Manifold b2CollideCapsuleAndCircle(const b2Capsule* capsuleA, b2Transform xfA, const b2Circle* circleB,
									 b2Transform xfB);

/// Compute the collision manifold between an segment and a circle.
b2Manifold b2CollideSegmentAndCircle(const b2Segment* segmentA, b2Transform xfA, const b2Circle* circleB,
									 b2Transform xfB);

/// Compute the collision manifold between a smooth segment and a circle.
b2Manifold b2CollideSmoothSegmentAndCircle(const b2SmoothSegment* segmentA, b2Transform xfA,
										   const b2Circle* circleB, b2Transform xfB);

/// Compute the collision manifold between a polygon and a circle.
b2Manifold b2CollidePolygonAndCircle(const b2Polygon* polygonA, b2Transform xfA, const b2Circle* circleB,
									 b2Transform xfB);

/// Compute the collision manifold between a capulse and circle
b2Manifold b2CollideCapsules(const b2Capsule* capsuleA, b2Transform xfA, const b2Capsule* capsuleB,
									 b2Transform xfB);

// TODO temp
b2Manifold b2CollideCapsules2(const b2Capsule* capsuleA, b2Transform xfA, const b2Capsule* capsuleB,
							  b2Transform xfB);

	/// Compute the collision manifold between an segment and a capsule.
b2Manifold b2CollideSegmentAndCapsule(const b2Segment* segmentA, b2Transform xfA, const b2Capsule* capsuleB,
									 b2Transform xfB);

/// Compute the collision manifold between a smooth segment and a capsule.
b2Manifold b2CollideSmoothSegmentAndCapsule(const b2SmoothSegment* segmentA, b2Transform xfA,
										   const b2Circle* capsuleB, b2Transform xfB);

/// Compute the collision manifold between a polygon and capsule
b2Manifold b2CollidePolygonAndCapsule(const b2Polygon* polygonA, b2Transform xfA, const b2Capsule* capsuleB,
									 b2Transform xfB);

/// Compute the collision manifold between two polygons.
b2Manifold b2CollidePolygons(const b2Polygon* polygonA, b2Transform xfA, const b2Polygon* polygonB,
							 b2Transform xfB);

/// Compute the collision manifold between an segment and a polygon.
b2Manifold b2CollideSegmentAndPolygon(const b2Segment* segmentA, b2Transform xfA, const b2Polygon* polygonB,
									  b2Transform xfB);

/// Compute the collision manifold between a smooth segment and a polygon.
b2Manifold b2CollideSmoothSegmentAndPolygon(const b2SmoothSegment* segmentA, b2Transform xfA,
											const b2Polygon* polygonB, b2Transform xfB);

#ifdef __cplusplus
}
#endif
