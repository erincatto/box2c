// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include <stdint.h>

typedef struct b2Profile b2Profile;
typedef struct b2TimeStep b2TimeStep;

typedef struct b2Island
{
	struct b2World* world;

	struct b2Body** bodies;
	struct b2Contact** contacts;
	struct b2Joint** joints;

	struct b2Position* positions;
	struct b2Velocity* velocities;

	struct b2Island* nextIsland;

	int32_t bodyCount;
	int32_t jointCount;
	int32_t contactCount;

	int32_t bodyCapacity;
	int32_t contactCapacity;
	int32_t jointCapacity;
} b2Island;

b2Island* b2CreateIsland(int32_t bodyCapacity, int32_t contactCapacity, int32_t jointCapacity, struct b2World* world);

void b2DestroyIsland(b2Island* island);

void b2Island_AddBody(b2Island* island, struct b2Body* body);
void b2Island_AddContact(b2Island* island, struct b2Contact* contact);
void b2Island_AddJoint(b2Island* island, struct b2Joint* joint);

void b2SolveIsland(b2Island* island, b2Profile* profile, const b2TimeStep* step, b2Vec2 gravity);

