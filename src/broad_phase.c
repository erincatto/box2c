// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "broad_phase.h"

#include "allocate.h"
#include "array.h"
#include "shape.h"
#include "box2d/aabb.h"
#include "box2d/timer.h"

#include <string.h>

void b2BroadPhase_Create(b2BroadPhase* bp, b2AddPairFcn* fcn, void* fcnContext)
{
	bp->proxyCount = 0;

	// TODO_ERIN maybe specify initial size in b2WorldDef?
	bp->moveCapacity = 16;
	bp->moveCount = 0;
	bp->moveBuffer = (int32_t*)b2Alloc(bp->moveCapacity * sizeof(int32_t));

	// TODO_ERIN initial size from b2WorldDef
	bp->pairSet = b2CreateSet(32);

	bp->addPairFcn = fcn;
	bp->fcnContext = fcnContext;

	bp->queryProxyKey = B2_NULL_INDEX;
	bp->queryTreeType = b2_dynamicBody;
	bp->queryUserData = NULL;

	for (int32_t i = 0; i < b2_bodyTypeCount; ++i)
	{
		bp->trees[i] = b2DynamicTree_Create();
		bp->dirtyFlags[i] = false;
	}
	
	bp->altDynamicTree = b2DynamicTree_Create();
	bp->altKinematicTree = b2DynamicTree_Create();
	bp->dynamicProxyMap = NULL;
	bp->kinematicProxyMap = NULL;
	bp->dynamicMapCapacity = 0;
	bp->kinematicMapCapacity = 0;
}

void b2BroadPhase_Destroy(b2BroadPhase* bp)
{
	for (int32_t i = 0; i < b2_bodyTypeCount; ++i)
	{
		b2DynamicTree_Destroy(bp->trees + i);
	}

	b2DynamicTree_Destroy(&bp->altDynamicTree);
	b2DynamicTree_Destroy(&bp->altKinematicTree);

	b2Free(bp->dynamicProxyMap, bp->dynamicMapCapacity * sizeof(struct b2ProxyMap));
	b2Free(bp->kinematicProxyMap, bp->kinematicMapCapacity * sizeof(struct b2ProxyMap));

	b2Free(bp->moveBuffer, bp->moveCapacity * sizeof(int32_t));
	
	b2DestroySet(&bp->pairSet);

	memset(bp, 0, sizeof(b2BroadPhase));
}

static void b2BufferMove(b2BroadPhase* bp, int32_t proxyKey)
{
	if (bp->moveCount == bp->moveCapacity)
	{
		int32_t* oldBuffer = bp->moveBuffer;
		int32_t oldCapacity = bp->moveCapacity;
		bp->moveCapacity += bp->moveCapacity >> 1;
		bp->moveBuffer = (int32_t*)b2Alloc(bp->moveCapacity * sizeof(int32_t));
		memcpy(bp->moveBuffer, oldBuffer, bp->moveCount * sizeof(int32_t));
		b2Free(oldBuffer, oldCapacity * sizeof(int32_t));
	}

	bp->moveBuffer[bp->moveCount] = proxyKey;
	++bp->moveCount;
}

int32_t b2BroadPhase_CreateProxy(b2BroadPhase* bp, b2BodyType bodyType, b2AABB aabb, uint32_t categoryBits, void* userData)
{
	assert(0 <= bodyType && bodyType < b2_bodyTypeCount);
	int32_t proxyId = b2DynamicTree_CreateProxy(bp->trees + bodyType, aabb, categoryBits, userData);
	int32_t proxyKey = B2_PROXY_KEY(proxyId, bodyType);
	b2BufferMove(bp, proxyKey);
	return proxyKey;
}

void b2BroadPhase_DestroyProxy(b2BroadPhase* bp, int32_t proxyKey)
{
	// Purge from move buffer. Linear search.
	for (int32_t i = 0; i < bp->moveCount; ++i)
	{
		if (bp->moveBuffer[i] == proxyKey)
		{
			bp->moveBuffer[i] = B2_NULL_INDEX;
			break;
		}
	}

	--bp->proxyCount;

	int32_t typeIndex = B2_PROXY_TYPE(proxyKey);
	int32_t proxyId = B2_PROXY_ID(proxyKey);

	assert(0 <= typeIndex && typeIndex <= b2_bodyTypeCount);
	b2DynamicTree_DestroyProxy(bp->trees + typeIndex, proxyId);
}

void b2BroadPhase_MoveProxy(b2BroadPhase* bp, int32_t proxyKey, b2AABB aabb)
{
	int32_t typeIndex = B2_PROXY_TYPE(proxyKey);
	int32_t proxyId = B2_PROXY_ID(proxyKey);

	assert(typeIndex == b2_dynamicBody || typeIndex == b2_kinematicBody);

	bool buffer = b2DynamicTree_MoveProxy(bp->trees + typeIndex, proxyId, aabb);
	if (buffer)
	{
		b2BufferMove(bp, proxyKey);
		bp->dirtyFlags[typeIndex] = true;
	}
}

void b2BroadPhase_GrowProxy(b2BroadPhase* bp, int32_t proxyKey, b2AABB aabb)
{
	int32_t typeIndex = B2_PROXY_TYPE(proxyKey);
	int32_t proxyId = B2_PROXY_ID(proxyKey);

	assert(typeIndex == b2_dynamicBody || typeIndex == b2_kinematicBody);

	bool buffer = b2DynamicTree_GrowProxy(bp->trees + typeIndex, proxyId, aabb);
	if (buffer)
	{
		b2BufferMove(bp, proxyKey);
		bp->dirtyFlags[typeIndex] = true;
	}
}

// This is called from b2DynamicTree::Query when we are gathering pairs.
static bool b2QueryCallback(int32_t proxyId, void* userData, void* context)
{
	b2BroadPhase* bp = (b2BroadPhase*)context;

	int32_t proxyKey = B2_PROXY_KEY(proxyId, bp->queryTreeType);

	// A proxy cannot form a pair with itself.
	if (proxyKey == bp->queryProxyKey)
	{
		return true;
	}

	bool moved = b2DynamicTree_WasMoved(bp->trees + bp->queryTreeType, proxyId);
	if (moved && proxyKey > bp->queryProxyKey)
	{
		// Both proxies are moving. Avoid duplicate pairs.
		return true;
	}

	uint64_t pairKey = B2_PROXY_PAIR_KEY(proxyKey, bp->queryProxyKey);
	if (b2ContainsKey(&bp->pairSet, pairKey))
	{
		// contact exists
		return true;
	}

	void* userDataA;
	void* userDataB;
	if (proxyKey < bp->queryProxyKey)
	{
		userDataA = userData;
		userDataB = bp->queryUserData;
	}
	else
	{
		userDataA = bp->queryUserData;
		userDataB = userData;
	}

	bp->addPairFcn(userDataA, userDataB, bp->fcnContext);

	// continue the query
	return true;
}

// TODO_ERIN these queries could be done in parallel in the island solver. Use a hash table to skip existing pairs.
// then any truly new pairs can be added in serially when island is completed.
void b2BroadPhase_UpdatePairs(b2BroadPhase* bp)
{
	// Perform tree queries for all moving proxies. This fills the pair buffer.
	for (int32_t i = 0; i < bp->moveCount; ++i)
	{
		int32_t proxyKey = bp->moveBuffer[i];
		if (proxyKey == B2_NULL_INDEX)
		{
			// proxy was destroyed after it moved
			continue;
		}

		int32_t proxyType = B2_PROXY_TYPE(proxyKey);
		int32_t proxyId = B2_PROXY_ID(proxyKey);
		bp->queryProxyKey = proxyKey;

		const b2DynamicTree* baseTree = bp->trees + proxyType;

		// We have to query the tree with the fat AABB so that
		// we don't fail to create a contact that may touch later.
		b2AABB fatAABB = b2DynamicTree_GetFatAABB(baseTree, proxyId);
		bp->queryUserData = b2DynamicTree_GetUserData(baseTree, proxyId);

		// Query tree and invoke b2AddPairFcn callback. A callback inside a callback.
		// TODO_ERIN test with filtering
		if (proxyType == b2_dynamicBody)
		{
			bp->queryTreeType = b2_dynamicBody;
			b2DynamicTree_Query(bp->trees + b2_dynamicBody, fatAABB, b2QueryCallback, bp);
			bp->queryTreeType = b2_kinematicBody;
			b2DynamicTree_Query(bp->trees + b2_kinematicBody, fatAABB, b2QueryCallback, bp);
			bp->queryTreeType = b2_staticBody;
			b2DynamicTree_Query(bp->trees + b2_staticBody, fatAABB, b2QueryCallback, bp);
		}
		else if (proxyType == b2_kinematicBody)
		{
			bp->queryTreeType = b2_dynamicBody;
			b2DynamicTree_Query(bp->trees + b2_dynamicBody, fatAABB, b2QueryCallback, bp);
		}
	}

	// Clear move flags
	for (int32_t i = 0; i < bp->moveCount; ++i)
	{
		int32_t proxyKey = bp->moveBuffer[i];
		if (proxyKey == B2_NULL_INDEX)
		{
			continue;
		}

		int32_t typeIndex = B2_PROXY_TYPE(proxyKey);
		int32_t proxyId = B2_PROXY_ID(proxyKey);

		b2DynamicTree_ClearMoved(bp->trees + typeIndex, proxyId);
	}

	// Reset move buffer
	bp->moveCount = 0;
}

bool b2BroadPhase_TestOverlap(const b2BroadPhase* bp, int32_t proxyKeyA, int32_t proxyKeyB)
{
	int32_t typeIndexA = B2_PROXY_TYPE(proxyKeyA);
	int32_t proxyIdA = B2_PROXY_ID(proxyKeyA);
	int32_t typeIndexB = B2_PROXY_TYPE(proxyKeyB);
	int32_t proxyIdB = B2_PROXY_ID(proxyKeyB);

	b2AABB aabbA = b2DynamicTree_GetFatAABB(bp->trees + typeIndexA, proxyIdA);
	b2AABB aabbB = b2DynamicTree_GetFatAABB(bp->trees + typeIndexB, proxyIdB);
	return b2AABB_Overlaps(aabbA, aabbB);
}

void b2BroadPhase_RebuildTrees(b2BroadPhase* bp)
{
	if (bp->dirtyFlags[b2_dynamicBody] == true)
	{
		b2DynamicTree_Clone(&bp->altDynamicTree, bp->trees + b2_dynamicBody);

		int32_t proxyCount = b2DynamicTree_GetProxyCount(bp->trees + b2_dynamicBody);
		if (proxyCount > bp->dynamicMapCapacity)
		{
			b2Free(bp->dynamicProxyMap, bp->dynamicMapCapacity * sizeof(struct b2ProxyMap));
			bp->dynamicMapCapacity = B2_MAX(16, bp->dynamicMapCapacity + bp->dynamicMapCapacity / 2);
			bp->dynamicProxyMap = b2Alloc(bp->dynamicMapCapacity * sizeof(struct b2ProxyMap));
		}

		b2DynamicTree_RebuildTopDownSAH(&bp->altDynamicTree, bp->dynamicProxyMap);

		// Update proxy keys
		for (int32_t i = 0; i < proxyCount; ++i)
		{
			b2ShapeProxy* proxy = bp->dynamicProxyMap[i].userData;
			proxy->proxyKey = B2_PROXY_KEY(i, b2_dynamicBody);
		}

		bp->dirtyFlags[b2_dynamicBody] = false;
	}

	if (bp->dirtyFlags[b2_kinematicBody] == true)
	{
		b2DynamicTree_Clone(&bp->altKinematicTree, bp->trees + b2_kinematicBody);

		int32_t proxyCount = b2DynamicTree_GetProxyCount(bp->trees + b2_kinematicBody);
		if (proxyCount > bp->kinematicMapCapacity)
		{
			b2Free(bp->kinematicProxyMap, bp->kinematicMapCapacity * sizeof(struct b2ProxyMap));
			bp->kinematicMapCapacity = B2_MAX(16, bp->kinematicMapCapacity + bp->kinematicMapCapacity / 2);
			bp->kinematicProxyMap = b2Alloc(bp->kinematicMapCapacity * sizeof(struct b2ProxyMap));
		}

		b2DynamicTree_RebuildTopDownSAH(&bp->altKinematicTree, bp->kinematicProxyMap);

		// Update proxy keys
		for (int32_t i = 0; i < proxyCount; ++i)
		{
			b2ShapeProxy* proxy = bp->kinematicProxyMap[i].userData;
			proxy->proxyKey = B2_PROXY_KEY(i, b2_kinematicBody);
		}

		bp->dirtyFlags[b2_kinematicBody] = false;
	}
}

void b2BroadPhase_SwapTrees(b2BroadPhase* bp)
{
	{
		b2DynamicTree temp = bp->trees[b2_dynamicBody];
		bp->trees[b2_dynamicBody] = bp->altDynamicTree;
		bp->altDynamicTree = temp;
	}

	{
		b2DynamicTree temp = bp->trees[b2_kinematicBody];
		bp->trees[b2_kinematicBody] = bp->altKinematicTree;
		bp->altKinematicTree = temp;
	}
}
