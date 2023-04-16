// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include <stdint.h>

typedef struct b2IslandBuilder b2IslandBuilder;
typedef struct b2TimeStep b2TimeStep;

typedef struct b2Island2
{
	struct b2World* world;
	const b2TimeStep* step;

	int32_t* bodyIndices;
	int32_t bodyCount;

	int32_t* contactIndices;
	int32_t contactCount;

	int32_t* jointIndices;
	int32_t jointCount;

	struct b2ContactSolver2* contactSolver;

	bool isAwake;
} b2Island2;

b2Island2* b2CreateIsland2(b2IslandBuilder* builder, int32_t index, struct b2World* world, const b2TimeStep* step);
void b2DestroyIsland2(b2Island2* island);

void b2SolveIsland2(b2Island2* island);
void b2CompleteIsland2(b2Island2* island);
