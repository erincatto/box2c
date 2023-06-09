// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "pool.h"
#include <stdint.h>

typedef struct b2IslandBuilder b2IslandBuilder;
typedef struct b2StepContext b2StepContext;

typedef struct b2IslandNode
{
	// B2_NULL_INDEX if free
	int32_t bodyIndex;
	int32_t prevNode, nextNode;
	int32_t halfEdgeList;
	int32_t islandIndex;
} b2IslandNode;

typedef struct b2IslandEdge
{
	// B2_NULL_INDEX if free
	int32_t contactIndex;

	// B2_NULL_INDEX if static
	int32_t node1, node2;
	int32_t prevEdge, nextEdge;
} b2IslandEdge;

// For node edge list
typedef struct b2IslandHalfEdge
{
	int32_t prevHalfEdge, nextHalfEdge;
} b2IslandHalfEdge;

// TODO_ERIN eventually b2Island
typedef struct b2PersistentIsland
{
	b2Object object;

	// B2_NULL_INDEX if free
	int32_t nodeList;
	int32_t edgeList;

	// This allow islands to be linked during a merge
	int32_t prevIsland;
	int32_t nextIsland;
} b2PersistentIsland;

typedef struct b2IslandManager
{
	b2Pool islandPool;
	b2PersistentIsland* islands;

	b2IndexPool nodeIndexPool;
	b2IslandNode* nodeArray;

	b2IndexPool edgeIndexPool;
	b2IslandEdge* edgeArray;
	b2IslandHalfEdge* halfEdgeArray;
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
