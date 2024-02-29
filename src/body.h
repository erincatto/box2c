// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "pool.h"

#include "box2d/distance.h"
#include "box2d/id.h"
#include "box2d/math.h"

typedef struct b2Polygon b2Polygon;
typedef struct b2World b2World;

// The body state is designed for fast conversion to and from SIMD via scatter-gather
// todo every non-static body gets a solver body.
// No solver bodies for static bodies to avoid cross thread sharing and the cache misses they bring.
// Keep two solver body arrays: awake and sleeping
// However, this makes it slower to access transform for collision/broad-phase?
//
// 32 bytes
typedef struct b2BodyState
{
	b2Vec2 linearVelocity; // 8
	float angularVelocity; // 4
	int flags;			   // 4

	// Using delta position reduces round-off error far from the origin
	b2Vec2 deltaPosition; // 8

	// Using delta rotation because I cannot access the full rotation on static bodies in
	// the solver and must use zero delta rotation for static bodies (s,c) = (0,1)
	b2Rot deltaRotation; // 8
} b2BodyState;

static const b2BodyState b2_identityBodyState = {{0.0f, 0.0f}, 0.0f, 0, {0.0f, 0.0f}, {0.0f, 1.0f}};

// Holds extra data needed by the constraint solver, but not in the SIMD contact solver
typedef struct b2BodyParam
{
	float invMass;
	float invI;
	float linearDamping;
	float angularDamping;
	// force, torque, and gravity to be applied each sub-step
	b2Vec2 linearVelocityDelta;
	float angularVelocityDelta;
	int bodyIndex;
} b2BodyParam;

// A rigid body
typedef struct b2Body
{
	b2Object object;

	enum b2BodyType type;

	// the body origin (not center of mass)
	b2Vec2 origin;

	// rotation
	b2Rot rotation;

	// center of mass position in world
	b2Vec2 position;

	// previous rotation and position for TOI
	b2Rot rotation0;
	b2Vec2 position0;

	// location of center of mass relative to the body origin
	b2Vec2 localCenter;

	b2Vec2 linearVelocity;
	float angularVelocity;

	b2Vec2 force;
	float torque;

	int32_t shapeList;
	int32_t shapeCount;

	int32_t chainList;

	// This is a key: [jointIndex:31, edgeIndex:1]
	int32_t jointList;
	int32_t jointCount;

	int32_t contactList;
	int32_t contactCount;

	// A non-static body is always in an island. B2_NULL_INDEX for static bodies.
	int32_t islandIndex;

	// Doubly linked island list
	int32_t islandPrev;
	int32_t islandNext;

	int32_t solverIndex;

	float mass, invMass;

	// Rotational inertia about the center of mass.
	float I, invI;

	float minExtent;
	float maxExtent;
	float linearDamping;
	float angularDamping;
	float gravityScale;

	float sleepTime;

	void* userData;
	int16_t world;

	bool enableSleep;
	bool fixedRotation;
	bool isEnabled;
	bool isMarked;
	bool isFast;
	bool isBullet;
	bool isSpeedCapped;
	bool enlargeAABB;
} b2Body;

b2Body* b2GetBody(b2World* world, b2BodyId id);
bool b2ShouldBodiesCollide(b2World* world, b2Body* bodyA, b2Body* bodyB);
bool b2IsBodyAwake(b2World* world, b2Body* body);
void b2WakeBody(b2World* world, b2Body* body);
void b2UpdateBodyMassData(b2World* world, b2Body* body);

static inline b2Transform b2MakeTransform(const b2Body* body)
{
	return (b2Transform){body->origin, body->rotation};
}

static inline b2Sweep b2MakeSweep(const b2Body* body)
{
	b2Sweep s;
	if (body->type == b2_staticBody)
	{
		s.c1 = body->position;
		s.c2 = body->position;
		s.q1 = body->rotation;
		s.q2 = body->rotation;
	}
	else
	{
		s.c1 = body->position0;
		s.c2 = body->position;
		s.q1 = body->rotation0;
		s.q2 = body->rotation;
	}

	s.localCenter = body->localCenter;
	return s;
}
