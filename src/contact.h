// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "box2d/distance.h"
#include "box2d/manifold.h"
#include "box2d/types.h"

typedef struct b2Shape b2Shape;
typedef struct b2World b2World;

typedef struct b2ContactLookup
{
	// index of simulation set stored in b2World
	// B2_NULL_INDEX when slot is free
	int setIndex;

	// index into the constraint graph color array
	// B2_NULL_INDEX for non-touching or sleeping contacts
	// B2_NULL_INDEX when slot is free
	int colorIndex;

	// contact index within set or graph color
	// B2_NULL_INDEX when slot is free
	int localIndex;
} b2ContactLookup;

// A contact edge is used to connect sims and contacts together
// in a contact graph where each body is a node and each contact
// is an edge. A contact edge belongs to a doubly linked list
// maintained in each attached body. Each contact has two contact
// edges, one for each attached body.
typedef struct b2ContactEdge
{
	int bodyId;
	int prevKey;
	int nextKey;
} b2ContactEdge;

// Flags stored in b2Contact::flags
enum b2ContactFlags
{
	// Set when the shapes are touching.
	b2_contactTouchingFlag = 0x00000002,

	// Contact has a hit event
	b2_contactHitEventFlag = 0x00000004,

	// One of the shapes is a sensor
	b2_contactSensorFlag = 0x00000010,

	// This contact no longer has overlapping AABBs
	b2_contactDisjoint = 0x00000020,

	// This contact started touching
	b2_contactStartedTouching = 0x00000040,

	// This contact stopped touching
	b2_contactStoppedTouching = 0x00000080,

	// This contact wants sensor events
	b2_contactEnableSensorEvents = 0x00000100,

	// This contact wants contact events
	b2_contactEnableContactEvents = 0x00000200,

	// This contact wants presolve events
	b2_contactEnablePreSolveEvents = 0x00000400,
};

/// The class manages contact between two shapes. A contact exists for each overlapping
/// AABB in the broad-phase (except if filtered). Therefore a contact object may exist
/// that has no contact points.
typedef struct b2Contact
{
	int contactId;

	uint32_t flags;

	// The color of this constraint in the graph coloring
	//int colorIndex;

	// Index of contact within color or within b2SolverSet contact array (non-touching or sleeping)
	//int localIndex;

	b2ContactEdge edges[2];

	int shapeIdA;
	int shapeIdB;

	b2DistanceCache cache;
	b2Manifold manifold;

	// A contact only belongs to an island if touching, otherwise B2_NULL_INDEX.
	int islandPrev;
	int islandNext;
	int islandId;

	// Mixed friction and restitution
	float friction;
	float restitution;

	// For conveyor belts
	float tangentSpeed;

	bool isMarked;
} b2Contact;

void b2InitializeContactRegisters(void);

void b2CreateContact(b2World* world, b2Shape* shapeA, b2Shape* shapeB);
void b2DestroyContact(b2World* world, b2Contact* contact, bool wakeBodies);

b2Contact* b2GetContactFromRawId(b2World* world, int contactId);
b2Contact* b2GetContactFromLookup(b2World* world, b2ContactLookup* lookup);

bool b2ShouldShapesCollide(b2Filter filterA, b2Filter filterB);

void b2UpdateContact(b2World* world, b2Contact* contact, b2Shape* shapeA, b2Transform transformA, b2Shape* shapeB,
					 b2Transform transformB);
