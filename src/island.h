// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "pool.h"
#include <stdint.h>

typedef struct b2Contact b2Contact;
typedef struct b2Joint b2Joint;
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

	// Keeps track of how many contacts have been removed from this island.
	int32_t contactRemoveCount;

	// This island has been chosen to be split up into smaller islands because a sufficient
	// number of contacts have been removed.
	bool maySplit;

	// Transient solver data
	b2StepContext* stepContext;
	struct b2ContactSolver* contactSolver;
} b2PersistentIsland;

void b2ClearIsland(b2PersistentIsland* island);

void b2WakeIsland(b2PersistentIsland* island);

	// Link contacts into the island graph when it starts having contact points
void b2LinkContact(b2World* world, b2Contact* contact);

// Unlink contact from the island graph when it stops having contact points
void b2UnlinkContact(b2World* world, b2Contact* contact);

// Link a joint into the island graph when it is created
void b2LinkJoint(b2World* world, b2Joint* joint);

// Unlink a joint from the island graph when it is destroyed
void b2UnlinkJoint(b2World* world, b2Joint* joint);

void b2MergeAwakeIslands(b2World* world);
void b2SortIslands(b2World* world, b2PersistentIsland** islands, int32_t count);

void b2PrepareIsland(b2PersistentIsland* island, b2StepContext* stepContext);

void b2SolveIsland(b2PersistentIsland* island);

void b2CompleteIsland(b2PersistentIsland* island);
void b2CompleteBaseSplitIsland(b2PersistentIsland* island);
void b2CompleteSplitIsland(b2PersistentIsland* island, bool isAwake);
