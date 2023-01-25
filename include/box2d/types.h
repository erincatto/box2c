// SPDX-FileCopyrightText: 2022 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
#define B2_LITERAL(T) T
#else
#define B2_LITERAL(T) (T)
#endif

#define B2_ARRAY_COUNT(A) (sizeof(A) / sizeof(A[0]))
#define B2_MAYBE_UNUSED(x) ((void)(x))
#define B2_NULL_INDEX (-1)

/// 2D vector
typedef struct b2Vec2
{
	float x, y;
} b2Vec2;

/// 2D rotation
typedef struct b2Rot
{
	/// Sine and cosine
	float s, c;
} b2Rot;

/// A 2D rigid transform
typedef struct b2Transform
{
	b2Vec2 p;
	b2Rot q;
} b2Transform;

/// This describes the motion of a body/shape for TOI computation.
/// Shapes are defined with respect to the body origin, which may
/// not coincide with the center of mass. However, to support dynamics
/// we must interpolate the center of mass position.
typedef struct b2Sweep
{
	/// local center of mass position
	b2Vec2 localCenter;

	/// center world positions
	b2Vec2 c1, c2;

	/// world angles
	float a1, a2;
} b2Sweep;

/// Axis-aligned bounding box
typedef struct b2AABB
{
	b2Vec2 lowerBound;
	b2Vec2 upperBound;
} b2AABB;

/// Color for debug drawing. Each value has the range [0,1].
typedef struct b2Color
{
	float r, g, b, a;
} b2Color;

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
	bool hit;
} b2RayCastOutput;

typedef struct b2WorldDef
{
	/// Gravity vector. Box2D has no up-vector defined.
	b2Vec2 gravity;

	/// initial capacity for bodies
	int32_t bodyCapacity;

	/// initial capcity for circles
	int32_t circleCapacity;

	/// initial capacity for polygons
	int32_t polygonCapacity;
} b2WorldDef;

/// The body type.
/// static: zero mass, zero velocity, may be manually moved
/// kinematic: zero mass, non-zero velocity set by user, moved by solver
/// dynamic: positive mass, non-zero velocity determined by forces, moved by solver
typedef enum b2BodyType
{
	b2_staticBody = 0,
	b2_kinematicBody,
	b2_dynamicBody
} b2BodyType;

/// A body definition holds all the data needed to construct a rigid body.
/// You can safely re-use body definitions. Shapes are added to a body after construction.
typedef struct b2BodyDef
{
	/// The body type: static, kinematic, or dynamic.
	/// Note: if a dynamic body would have zero mass, the mass is set to one.
	b2BodyType type;

	/// The world position of the body. Avoid creating bodies at the origin
	/// since this can lead to many overlapping shapes.
	b2Vec2 position;

	/// The world angle of the body in radians.
	float angle;

	/// The linear velocity of the body's origin in world co-ordinates.
	b2Vec2 linearVelocity;

	/// The angular velocity of the body.
	float angularVelocity;

	/// Linear damping is use to reduce the linear velocity. The damping parameter
	/// can be larger than 1.0f but the damping effect becomes sensitive to the
	/// time step when the damping parameter is large.
	float linearDamping;

	/// Angular damping is use to reduce the angular velocity. The damping parameter
	/// can be larger than 1.0f but the damping effect becomes sensitive to the
	/// time step when the damping parameter is large.
	float angularDamping;

	/// Scale the gravity applied to this body.
	float gravityScale;

	/// Use this to store application specific body data.
	void* userData;

	/// Set this flag to false if this body should never fall asleep. Note that
	/// this increases CPU usage.
	bool canSleep;

	/// Is this body initially awake or sleeping?
	bool awake;

	/// Should this body be prevented from rotating? Useful for characters.
	bool fixedRotation;

	/// Does this body start out enabled?
	bool enabled;
} b2BodyDef;

#ifdef __cplusplus
extern "C"
{
#endif

/// Make a world definition with default values.
static inline b2WorldDef b2EmptyWorldDef()
{
	b2WorldDef def;
	def.gravity = B2_LITERAL(b2Vec2){0.0f, -10.0f};
	def.bodyCapacity = 8;
	def.circleCapacity = 8;
	def.polygonCapacity = 4;
	return def;
}

/// Make a body definition with default values.
static inline b2BodyDef b2EmptyBodyDef()
{
	b2BodyDef def;
	def.type = b2_staticBody;
	def.position = B2_LITERAL(b2Vec2){0.0f, 0.0f};
	def.angle = 0.0f;
	def.linearVelocity = B2_LITERAL(b2Vec2){0.0f, 0.0f};
	def.angularVelocity = 0.0f;
	def.linearDamping = 0.0f;
	def.angularDamping = 0.0f;
	def.gravityScale = 1.0f;
	def.userData = NULL;
	def.canSleep = true;
	def.awake = true;
	def.fixedRotation = false;
	def.enabled = true;
	return def;
}

#ifdef __cplusplus
}
#endif
