// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "pool.h"
#include <stdint.h>

typedef struct b2IslandBuilder b2IslandBuilder;
typedef struct b2StepContext b2StepContext;

typedef struct b2IslandNode
{
	b2Object object;

	// B2_NULL_INDEX if free
	int32_t bodyIndex;
	int32_t prevNode, nextNode;
	int32_t edgeList;
	int32_t islandIndex;
} b2IslandNode;

typedef struct b2IslandEdge
{
	b2Object object;

	// B2_NULL_INDEX if free
	int32_t contactIndex;

	// B2_NULL_INDEX if static
	int32_t node1, node2;
	int32_t prevEdge, nextEdge;
	int32_t 
} b2IslandEdge;

// TODO_ERIN eventually b2Island
typedef struct b2PersistentIsland
{
	b2Object object;

	// B2_NULL_INDEX if free
	int32_t nodeList;
	int32_t edgeList;
} b2PersistentIsland;

typedef struct b2IslandManager
{
	b2Pool nodePool;
	b2Pool edgePool;
	b2Pool islandPool;

	// Pointers into the pools above
	b2IslandNode* nodes;
	b2IslandEdge* edges;
	b2PersistentIsland* islands;

} b2IslandManager;

int32_t b2AddIslandNode(b2IslandManager* manager, int32_t bodyIndex);
void b2RemoveIslandNode(b2IslandManager* manager, int32_t nodeIndex);

int32_t b2AddIslandEdge(b2IslandManager* manager, int32_t contactIndex, int32_t node1, int32_t node2);
void b2RemoveIslandEdge(b2IslandManager* manager, int32_t edgeIndex);

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
