// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include <stdint.h>

typedef struct b2ContactElement b2ContactElement;
typedef struct b2IslandBuilder b2IslandBuilder;
typedef struct b2StepContext b2StepContext;

typedef struct b2Island
{
	struct b2World* world;
	b2StepContext* context;

	// Indices into b2World::bodies
	int32_t* bodyIndices;
	int32_t bodyCount;

	// Indices into b2World::activeContacts
	b2ContactElement* contactElements;
	int32_t contactCount;

	// Indices into b2StepContext::activeJoints
	int32_t* jointIndices;
	int32_t jointCount;

	// Auxiliary array for counting sort (bodyCount + 1)
	int32_t* counters;

	// Sorted contact indices for determinism (countactCount)
	int32_t* sortedContacts;

	struct b2ContactSolver* contactSolver;

	bool isAwake;
} b2Island;

b2Island* b2CreateIsland(b2IslandBuilder* builder, int32_t index, struct b2World* world, b2StepContext* step);
void b2DestroyIsland(b2Island* island);

void b2SolveIsland(b2Island* island);
void b2CompleteIsland(b2Island* island);
