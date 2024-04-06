// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "box2d/distance.h"
#include "box2d/math_functions.h"

typedef struct b2Polygon b2Polygon;
typedef struct b2World b2World;
typedef struct b2Joint b2Joint;
typedef struct b2Contact b2Contact;
typedef struct b2Shape b2Shape;
typedef struct b2Body b2Body;

// Body organizational details that are not used in the solver.
typedef struct b2Body
{
	void* userData;

	// index of solver set stored in b2World
	// may be B2_NULL_INDEX
	int setIndex;

	// body sim and state index within set
	// may be B2_NULL_INDEX
	int localIndex;

	// [31 : contactId | 1 : edgeIndex]
	int headContactKey;
	int contactCount;

	int headShapeId;
	int shapeCount;

	int headChainId;

	// [31 : contactId | 1 : edgeIndex]
	int headJointKey;
	int jointCount;

	// All enabled dynamic and kinematic sims are in an island.
	int islandId;

	// doubly-linked island list
	int islandPrev;
	int islandNext;

	// todo maybe remove this
	int bodyId;

	// This is monotonically advanced when a body is allocated in this slot
	// Used to check for invalid b2BodyId
	uint16_t revision;

	int16_t worldId;

} b2Body;

// The body state is designed for fast conversion to and from SIMD via scatter-gather.
// Only awake dynamic and kinematic sims have a body state.
// This is used in the performance critical constraint solver
//
// 32 bytes
typedef struct b2BodyState
{
	b2Vec2 linearVelocity; // 8
	float angularVelocity; // 4
	int flags;			   // 4

	// Using delta position reduces round-off error far from the origin
	b2Vec2 deltaPosition; // 8

	// Using delta rotation because I cannot access the full rotation on static sims in
	// the solver and must use zero delta rotation for static sims (s,c) = (0,1)
	b2Rot deltaRotation; // 8
} b2BodyState;

// Identity body state, notice the deltaRotation is {0, 1}
static const b2BodyState b2_identityBodyState = {{0.0f, 0.0f}, 0.0f, 0, {0.0f, 0.0f}, {0.0f, 1.0f}};

// Body simulation data used for integration of position and velocity
// Transform data used for collision and solver preparation.
typedef struct b2BodySim
{
	// transform for body origin
	b2Transform transform;

	// center of mass position in world
	b2Vec2 center;

	// previous rotation and COM for TOI
	b2Rot rotation0;
	b2Vec2 center0;

	// location of center of mass relative to the body origin
	b2Vec2 localCenter;

	b2Vec2 force;
	float torque;

	float mass, invMass;

	// Rotational inertia about the center of mass.
	float I, invI;

	float minExtent;
	float maxExtent;
	float linearDamping;
	float angularDamping;
	float gravityScale;
	float sleepTime;

	// stable id for graph coloring, B2_NULL_INDEX for static sims
	int graphColorId;

	// body data can be moved around, the id is stable (used in b2BodyId)
	int bodyId;

	b2BodyType type;

	bool enableSleep;

	// todo needed in sim?
	bool fixedRotation;

	// todo eliminate
	bool isMarked;
	bool isFast;
	bool isBullet;
	bool isSpeedCapped;
	bool enlargeAABB;
} b2BodySim;

b2Body* b2GetBodyFullId(b2World* world, b2BodyId bodyId);

b2Body* b2GetBody(b2World* world, int bodyId);

// Get a validated body from a world using an id.
// todo remove this function and instead use B2_ASSERT(b2Body_IsValid(bodyId))
b2Body* b2GetBodyFullId(b2World* world, b2BodyId bodyId);

b2Transform b2GetBodyTransformQuick(b2World* world, b2Body* body);
b2Transform b2GetBodyTransform(b2World* world, int bodyId);

// Create a b2BodyId from a raw id.
b2BodyId b2MakeBodyId(b2World* world, int bodyId);

bool b2ShouldBodiesCollide(b2World* world, b2Body* bodyA, b2Body* bodyB);
bool b2IsBodyAwake(b2World* world, b2Body* body);

b2BodySim* b2GetBodySim(b2World* world, b2Body* body);
b2BodyState* b2GetBodyState(b2World* world, b2Body* body);

// careful calling this because it can invalidate body, state, joint, and contact pointers
bool b2WakeBody(b2World* world, b2Body* body);

void b2UpdateBodyMassData(b2World* world, b2Body* body);

inline b2Sweep b2MakeSweep(const b2BodySim* bodySim)
{
	b2Sweep s;
	s.c1 = bodySim->center0;
	s.c2 = bodySim->center;
	s.q1 = bodySim->rotation0;
	s.q2 = bodySim->transform.q;
	s.localCenter = bodySim->localCenter;
	return s;
}
