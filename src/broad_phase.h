// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "box2d/dynamic_tree.h"
#include "table.h"

#define BP_PAIR_SET 1

typedef struct b2Pair
{
	void* userDataA;
	void* userDataB;
} b2Pair;

typedef void b2AddPairFcn(void* userDataA, void* userDataB, void* context);

// Store the proxy type in the lower 4 bits of the proxy key. This leaves 28 bits for the id.
#define B2_PROXY_TYPE(KEY) ((b2BodyType)((KEY)&0xF))
#define B2_PROXY_ID(KEY) ((KEY) >> 4)
#define B2_PROXY_KEY(ID, TYPE) (((ID) << 4) | (TYPE))

/// The broad-phase is used for computing pairs and performing volume queries and ray casts.
/// This broad-phase does not persist pairs. Instead, this reports potentially new pairs.
/// It is up to the client to consume the new pairs and to track subsequent overlap.
typedef struct b2BroadPhase
{
	b2DynamicTree trees[b2_bodyTypeCount];
	b2DynamicTree rebuildTrees[b2_bodyTypeCount];
	bool dirtyFlags[b2_bodyTypeCount];
	struct b2ProxyMap* proxyMaps[b2_bodyTypeCount];
	int32_t proxyMapCapacities[b2_bodyTypeCount];
	
	int32_t proxyCount;

	int32_t* moveBuffer;
	int32_t moveCapacity;
	int32_t moveCount;

	b2Set pairSet;

	b2AddPairFcn* addPairFcn;
	void* fcnContext;

	b2BodyType queryTreeType;
	int32_t queryProxyKey;
	void* queryUserData;
} b2BroadPhase;

void b2BroadPhase_Create(b2BroadPhase* bp, b2AddPairFcn* fcn, void* fcnContext);
void b2BroadPhase_Destroy(b2BroadPhase* bp);
int32_t b2BroadPhase_CreateProxy(b2BroadPhase* bp, b2BodyType bodyType, b2AABB aabb, uint32_t categoryBits,
								 void* userData);
void b2BroadPhase_DestroyProxy(b2BroadPhase* bp, int32_t proxyKey);
void b2BroadPhase_MoveProxy(b2BroadPhase* bp, int32_t proxyKey, b2AABB aabb);
void b2BroadPhase_MoveProxy2(b2BroadPhase* bp, int32_t proxyKey, b2AABB aabb);

void b2BroadPhase_RebuildTrees(b2BroadPhase* bp);
void b2BroadPhase_SwapTrees(b2BroadPhase* bp);

void b2BroadPhase_UpdatePairs(b2BroadPhase* bp);
bool b2BroadPhase_TestOverlap(const b2BroadPhase* bp, int32_t proxyKeyA, int32_t proxyKeyB);

static inline b2AABB b2BroadPhase_GetFatAABB(b2BroadPhase* bp, int32_t proxyKey)
{
	b2BodyType type = B2_PROXY_TYPE(proxyKey);
	int32_t proxyId = B2_PROXY_ID(proxyKey);
	return b2DynamicTree_GetFatAABB(bp->trees + type, proxyId);
}
