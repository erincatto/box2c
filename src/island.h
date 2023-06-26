// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "pool.h"
#include <stdint.h>

typedef struct b2IslandBuilder b2IslandBuilder;
typedef struct b2StepContext b2StepContext;

// Deterministic solver
// 
// Collide all awake contacts
// Use bit array to emit start/stop touching events in defined order, per thread. Try using contact index, assuming contacts are created in a deterministic order.
// bit-wise OR together bit arrays and issue changes:
// - start touching: merge islands - temporary linked list - mark root island dirty - wake all - largest island is root
// - stop touching: mark island dirty - wake island
// Reserve island jobs
// - island job does a DFS to merge/split islands. Mutex to allocate new islands. Split islands sent to different jobs.

// Persistent island
// https://en.wikipedia.org/wiki/Component_(graph_theory)
// https://en.wikipedia.org/wiki/Dynamic_connectivity
// TODO_ERIN eventually b2Island
typedef struct b2PersistentIsland
{
	b2Object object;

	struct b2World* world;

	int32_t headBody;
	int32_t tailBody;
	int32_t bodyCount;

	int32_t headContact;
	int32_t tailContact;
	int32_t contactCount;

	int32_t headJoint;
	int32_t tailJoint;
	int32_t jointCount;

	// Union find
	int32_t parentIsland;

	// Index into world awake island array, B2_NULL_INDEX if the island is sleeping
	int32_t awakeIndex;

	// A dirty island may need to be split
	bool isDirty;

	// Transient solver data
	b2StepContext* stepContext;
	struct b2ContactSolver* contactSolver;
} b2PersistentIsland;

void b2ClearIsland(b2PersistentIsland* island);

void b2LinkContact(b2PersistentIsland* island, b2Contact* contact);
void b2UnlinkContact(b2PersistentIsland* island, b2Contact* contact);

void b2LinkJoint(b2PersistentIsland* island, b2Joint* joint);
void b2UnlinkJoint(b2PersistentIsland* island, b2Joint* joint);

void b2PrepareIsland(b2PersistentIsland* island, b2StepContext* step);

void b2SolveIsland(b2PersistentIsland* island);
void b2CompleteIsland(b2PersistentIsland* island);
