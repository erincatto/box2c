// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "block_array.h"

typedef struct b2World b2World;

// This holds solver set data. The following sets are used:
// - static set for all static sims (no contacts or joints)
// - active set for all active sims with body states (no contacts or joints)
// - disabled set for disabled sims and their joints
// - all further sets are sleeping island sets along with their contacts and joints
typedef struct b2SolverSet
{
	// Body array. Empty for unused set.
	b2BodySimArray sims;

	// Body state only exists for active set
	b2BodyStateArray states;

	// This holds sleeping/disabled joints. Empty for static/active set.
	b2JointArray joints;

	// This holds all contacts for sleeping sets.
	// This holds non-touching contacts for the awake set.
	b2ContactArray contacts;

	// The awake set has an array of islands. Sleeping sets normally have a single islands. However, joints
	// created between sleeping sets causes the sets to merge, leaving them with multiple islands. These sleeping
	// islands will be naturally merged with the set is woken.
	// The static and disabled sets have no islands.
	// Islands live in the solver sets to limit the number of islands that need to be considered for sleeping.
	b2IslandArray islands;

	// Aligns with b2World::solverSetIdPool. Used to create a stable id for body/contact/joint/island lookups.
	int setId;
} b2SolverSet;

void b2WakeSolverSet(b2World* world, int setId);
void b2TrySleepIsland(b2World* world, int islandId);

void b2MergeSolverSets(b2World* world, int setId1, int setId2);

void b2MoveBody(b2World* world, int jointId, int targetSetIndex);
void b2MoveJoint(b2World* world, int jointId, int targetSetIndex);
