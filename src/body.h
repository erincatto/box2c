// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "box2d/distance.h"
#include "box2d/id.h"
#include "box2d/math.h"

#include "pool.h"

typedef struct b2Polygon b2Polygon;
typedef struct b2World b2World;

// A rigid body
typedef struct b2Body
{
	b2Object object;

	enum b2BodyType type;

	// the body origin transform (not center of mass)
	b2Transform transform;
	
	// center of mass position in world
	b2Vec2 position0;
	b2Vec2 position;

	// rotation in radians
	float angle0;
	float angle;

	// location of center of mass relative to the body origin
	b2Vec2 localCenter;

	b2Vec2 linearVelocity;
	float angularVelocity;

	// These are the change in position/angle that accumulate across constraint substeps
	b2Vec2 deltaPosition;
	float deltaAngle;

	b2Vec2 force;
	float torque;

	int32_t shapeList;
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
	bool isSpeedCapped;
	bool enlargeAABB;
} b2Body;

// TODO_ERIN every non-static body gets a solver body. No solver bodies for static bodies to avoid cross thread sharing and the cache misses they bring.
// Keep two solver body arrays: awake and sleeping
// 12 + 12 + 8 = 32 bytes
typedef struct b2SolverBody
{
	b2Vec2 linearVelocity; // 8
	float angularVelocity; // 4

	// These are the change in position/angle that accumulate across constraint substeps
	b2Vec2 deltaPosition; // 8
	float deltaAngle;     // 4

	float invMass; // 4
	float invI;    // 4
} b2SolverBody;

b2Body* b2GetBody(b2World* world, b2BodyId id);
bool b2ShouldBodiesCollide(b2World* world, b2Body* bodyA, b2Body* bodyB);
bool b2IsBodyAwake(b2World* world, b2Body* body);

static inline b2Sweep b2MakeSweep(const b2Body* body)
{
	b2Sweep s;
	if (body->type == b2_staticBody)
	{
		s.c1 = body->position;
		s.c2 = body->position;
		s.a1 = body->angle;
		s.a2 = body->angle;
	}
	else
	{
		s.c1 = body->position0;
		s.c2 = body->position;
		s.a1 = body->angle0;
		s.a2 = body->angle;
	}

	s.localCenter = body->localCenter;
	return s;
}
