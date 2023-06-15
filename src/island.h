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

	int32_t headBody;
	int32_t tailBody;
	int32_t bodyCount;

	int32_t headContact;
	int32_t tailContact;
	int32_t contactCount;

	// This allow islands to be linked during a merge
	int32_t nextIsland;

	// Index into world awake island array, B2_NULL_INDEX if the island is sleeping
	int32_t awakeIndex;

	// A dirty island may need to be split
	bool isDirty;
} b2PersistentIsland;

typedef struct b2Island
{
	struct b2World* world;
	b2StepContext* context;

	// Indices into b2World::bodies
	int32_t* bodyIndices;
	int32_t bodyCount;

	// Indices into b2World::activeContacts
	int32_t* contactIndices;
	int32_t contactCount;

	// Indices into b2StepContext::activeJoints
	int32_t* jointIndices;
	int32_t jointCount;

	struct b2ContactSolver* contactSolver;

	bool isAwake;
} b2Island;

b2Island* b2CreateIsland(b2IslandBuilder* builder, int32_t index, struct b2World* world, b2StepContext* step);
void b2DestroyIsland(b2Island* island);

void b2SolveIsland(b2Island* island);
void b2CompleteIsland(b2Island* island);
