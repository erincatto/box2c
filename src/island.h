// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include <stdint.h>

typedef struct b2Body b2Body;
typedef struct b2Contact b2Contact;
typedef struct b2Joint b2Joint;
typedef struct b2Profile b2Profile;
typedef struct b2TimeStep b2TimeStep;

typedef struct b2Island
{
	struct b2World* world;
	const b2TimeStep* step;

	b2Body** bodies;
	b2Contact** contacts;
	b2Joint** joints;

	struct b2ContactSolver* contactSolver;
	struct b2BodyData* bodyData;
	struct b2Position* positions;
	struct b2Velocity* velocities;

	struct b2Island* nextIsland;

	int32_t bodyCount;
	int32_t jointCount;
	int32_t contactCount;

	bool isAwake;
} b2Island;

b2Island* b2CreateIsland(b2Body** bodies, int32_t bodyCount, b2Contact** contacts, int32_t contactCount, b2Joint** joints, int32_t jointCount, struct b2World* world, const b2TimeStep* step);
void b2DestroyIsland(b2Island* island);

void b2Island_AddBody(b2Island* island, struct b2Body* body);
void b2Island_AddContact(b2Island* island, struct b2Contact* contact);
void b2Island_AddJoint(b2Island* island, struct b2Joint* joint);

// Finalize island after adding bodies/contacts/joints
void b2FinalizeIsland(b2Island* island);

void b2SolveIsland(b2Island* island);

void b2CompleteIsland(b2Island* island);
