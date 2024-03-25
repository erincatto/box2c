// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "contact_list.h"

#include "box2d/distance.h"
#include "box2d/id.h"
#include "box2d/math_functions.h"

typedef struct b2Polygon b2Polygon;
typedef struct b2World b2World;
typedef struct b2Joint b2Joint;
typedef struct b2Contact b2Contact;
typedef struct b2Shape b2Shape;

// map from b2BodyId/int to solver set and index
// todo consider moving island graph stuff to lookups
typedef struct b2BodyLookup
{
	// index of solver set stored in b2World
	// may be B2_NULL_INDEX
	int setIndex;

	// body index within set
	// may be B2_NULL_INDEX
	int bodyIndex;

	// This is monotonically advanced when a body is allocated in this slot
	// Used to check for invalid b2BodyId
	int revision;
} b2BodyLookup;

// The body state is designed for fast conversion to and from SIMD via scatter-gather
// Only awake dynamic and kinematic bodies have a body state.
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

// Identity body state, notice the deltaRotation is {0, 1}
static const b2BodyState b2_identityBodyState = {{0.0f, 0.0f}, 0.0f, 0, {0.0f, 0.0f}, {0.0f, 1.0f}};

// A rigid body
// todo perhaps split out the transform which is accessed frequently
typedef struct b2Body
{
	void* userData;

	// todo combine into b2Transform
	// 
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

	b2Vec2 force;
	float torque;

	b2ShapeList shapeList;
	b2ChainList chainList;
	b2ContactList contactList;

	// This is a key: [jointIndex:31, edgeIndex:1]
	int headJointKey;
	int jointCount;

	// All enabled bodies are in an island.
	// B2_NULL_INDEX disabled bodies.
	int islandId;

	// Doubly linked island list
	int islandPrev;
	int islandNext;
	
	float mass, invMass;

	// Rotational inertia about the center of mass.
	float I, invI;

	float minExtent;
	float maxExtent;
	float linearDamping;
	float angularDamping;
	float gravityScale;
	float sleepTime;

	// body data can be moved around, the id is stable (used in b2BodyId)
	int bodyId;
	uint16_t revision;
	int16_t world;

	bool enableSleep;
	bool fixedRotation;

	// todo redundant with body set index
	bool isEnabled;
	bool isKinematic;
	// todo eliminate
	bool isMarked;
	bool isFast;
	bool isBullet;
	bool isSpeedCapped;
	bool enlargeAABB;
} b2Body;

b2Body* b2GetBodyFullId(b2World* world, b2BodyId bodyId);
b2Body* b2GetBody(b2World* world, int bodyId);
b2Transform b2GetBodyTransform(b2World* world, int bodyId);

b2BodyId b2MakeBodyId(b2World* world, int bodyId);
b2BodyState* b2GetBodyState(b2World* world, int bodyId);

bool b2ShouldBodiesCollide(b2World* world, b2Body* bodyA, b2Body* bodyB);
bool b2IsBodyAwake(b2World* world, int bodyId);

// careful calling this because it can invalidate body, state, joint, and contact pointers
bool b2WakeBody(b2World* world, int bodyId);

void b2UpdateBodyMassData(b2World* world, b2Body* body);

static inline b2Transform b2MakeTransform(const b2Body* body)
{
	return (b2Transform){body->origin, body->rotation};
}

static inline b2Sweep b2MakeSweep(const b2Body* body)
{
	b2Sweep s;
	s.c1 = body->position0;
	s.c2 = body->position;
	s.q1 = body->rotation0;
	s.q2 = body->rotation;
	s.localCenter = body->localCenter;
	return s;
}
