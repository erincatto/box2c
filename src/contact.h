// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "pool.h"

#include "box2d/distance.h"
#include "box2d/manifold.h"
#include "box2d/types.h"

typedef struct b2Body b2Body;
typedef struct b2Shape b2Shape;
typedef struct b2World b2World;

// A contact edge is used to connect bodies and contacts together
// in a contact graph where each body is a node and each contact
// is an edge. A contact edge belongs to a doubly linked list
// maintained in each attached body. Each contact has two contact
// edges, one for each attached body.
typedef struct b2ContactEdge
{
	int32_t bodyIndex;
	int32_t prevKey;
	int32_t nextKey;
} b2ContactEdge;

// Flags stored in b2Contact::flags
enum b2ContactFlags
{
	// Set when the shapes are touching.
	// TODO_ERIN sensor only? Overlap?
	b2_contactTouchingFlag = 0x00000002,

	// This contact can be disabled (by user)
	b2_contactEnabledFlag = 0x00000004,

	// This contact needs filtering because a fixture filter was changed.
	// TODO_ERIN don't defer this anymore
	b2_contactFilterFlag = 0x00000008,

	// One of the shapes is a sensor
	b2_contactSensorFlag = 0x00000010,

	// This contact no longer has overlapping AABBs
	b2_contactDisjoint = 0x00000020,

	// This contact started touching
	b2_contactStartedTouching = 0x00000040,

	// This contact stopped touching
	b2_contactStoppedTouching = 0x00000080,
};

/// The class manages contact between two shapes. A contact exists for each overlapping
/// AABB in the broad-phase (except if filtered). Therefore a contact object may exist
/// that has no contact points.
typedef struct b2Contact
{
	b2Object object;

	uint32_t flags;

	// The color of this constraint in the graph coloring
	int32_t colorIndex;

	// Index of contact within color
	int32_t colorSubIndex;

	b2ContactEdge edges[2];

	int32_t shapeIndexA;
	int32_t shapeIndexB;

	b2DistanceCache cache;
	b2Manifold manifold;

	// A contact only belongs to an island if touching, otherwise B2_NULL_INDEX.
	int32_t islandPrev;
	int32_t islandNext;
	int32_t islandIndex;

	// Mixed friction and restitution
	float friction;
	float restitution;

	// For conveyor belts
	float tangentSpeed;

	bool isMarked;
} b2Contact;

void b2InitializeContactRegisters(void);

void b2CreateContact(b2World* world, b2Shape* shapeA, b2Shape* shapeB);
void b2DestroyContact(b2World* world, b2Contact* contact);

bool b2ShouldShapesCollide(b2Filter filterA, b2Filter filterB);

void b2UpdateContact(b2World* world, b2Contact* contact, b2Shape* shapeA, b2Body* bodyA, b2Shape* shapeB, b2Body* bodyB);
