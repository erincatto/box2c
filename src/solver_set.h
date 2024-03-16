// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "block_array.h"

typedef struct b2Body b2Body;
typedef struct b2BodyState b2BodyState;
typedef struct b2Contact b2Contact;
typedef struct b2Joint b2Joint;
typedef struct b2World b2World;

// This holds solver set data. The following sets are used:
// - static set for all static bodies (no contacts or joints)
// - active set for all active bodies with body states (no contacts or joints)
// - disabled set for disabled bodies and their joints
// - all further sets are sleeping island sets along with their contacts and joints
typedef struct b2SolverSet
{
	// Body array. Empty for unused set.
	b2BodyArray bodies;

	// Body state only exists for active set
	b2BodyStateArray states;

	// This holds sleeping/disabled joints. Empty for static/active set.
	b2JointArray joints;

	// This holds sleeping contacts for sleeping sets.
	// For the awake set this holds the non-touching awake contacts.
	// For the
	b2ContactArray contacts;

	// The awake set has an array of islands. Sleeping sets have a single islands.
	// The static and disabled sets have no islands.
	b2IslandArray islands;
} b2SolverSet;

void b2WakeSolverSet(b2World* world, int setIndex);
void b2MergeSolverSets(b2World* world, int setIndex1, int setIndex2);

void b2ValidateSet(b2World* world, int setIndex);
void b2ValidateSets(b2World* world);
