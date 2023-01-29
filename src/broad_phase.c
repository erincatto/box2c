// SPDX-FileCopyrightText: 2022 Erin Catto
// SPDX-License-Identifier: MIT

#include "box2d/aabb.h"
#include "box2d/allocate.h"

#include "broad_phase.h"
#include <string.h>

void b2BroadPhase_Create(b2BroadPhase* bp, b2AddPairFcn* fcn, void* fcnContext)
{
	bp->proxyCount = 0;

	// TODO_ERIN maybe specify initial size in b2WorldDef?
	bp->moveCapacity = 16;
	bp->moveCount = 0;
	bp->moveBuffer = (int32_t*)b2Alloc(bp->moveCapacity * sizeof(int32_t));

	bp->addPairFcn = fcn;
	bp->fcnContext = fcnContext;

	bp->queryProxyId = B2_NULL_INDEX;
	bp->queryTree = B2_NULL_INDEX;

	for (int32_t i = 0; i < b2_treeCount; ++i)
	{
		bp->trees[i] = b2DynamicTree_Create();
	}
}

void b2BroadPhase_Destroy(b2BroadPhase* bp)
{
	b2Free(bp->moveBuffer);
	memset(bp, 0, sizeof(b2BroadPhase));
}

static void b2BufferMove(b2BroadPhase* bp, int32_t proxyKey)
{
	if (bp->moveCount == bp->moveCapacity)
	{
		int32_t* oldBuffer = bp->moveBuffer;
		bp->moveCapacity += bp->moveCapacity >> 1;
		bp->moveBuffer = (int32_t*)b2Alloc(bp->moveCapacity * sizeof(int32_t));
		memcpy(bp->moveBuffer, oldBuffer, bp->moveCount * sizeof(int32_t));
		b2Free(oldBuffer);
	}

	bp->moveBuffer[bp->moveCount] = proxyKey;
	++bp->moveCount;
}

int32_t b2BroadPhase_CreateProxy(b2BroadPhase* bp, b2TreeType treeType, b2AABB aabb, uint32_t categoryBits,
								 uint64_t userData)
{
	assert(0 <= treeType && treeType < b2_treeCount);
	int32_t proxyId = b2DynamicTree_CreateProxy(bp->trees + treeType, aabb, categoryBits, userData);
	int32_t proxyKey = B2_PROXY_KEY(proxyId, treeType);
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

	int32_t treeIndex = B2_PROXY_TREE(proxyKey);
	int32_t proxyId = B2_PROXY_ID(proxyKey);

	assert(0 <= treeIndex && treeIndex <= b2_treeCount);
	b2DynamicTree_DestroyProxy(bp->trees + treeIndex, proxyId);
}

void b2BroadPhase_MoveProxy(b2BroadPhase* bp, int32_t proxyKey, b2AABB aabb)
{
	int32_t treeIndex = B2_PROXY_TREE(proxyKey);
	int32_t proxyId = B2_PROXY_ID(proxyKey);

	assert(0 <= treeIndex && treeIndex <= b2_treeCount);

	bool buffer = b2DynamicTree_MoveProxy(bp->trees + treeIndex, proxyId, aabb);
	if (buffer)
	{
		b2BufferMove(bp, proxyKey);
	}
}

void b2BroadPhase_TouchProxy(b2BroadPhase* bp, int32_t proxyKey)
{
	b2BufferMove(bp, proxyKey);
}

// This is called from b2DynamicTree::Query when we are gathering pairs.
static bool b2QueryCallback(int32_t proxyId, uint64_t userData, void* context)
{
	b2BroadPhase* bp = (b2BroadPhase*)context;

	// A proxy cannot form a pair with itself.
	if (proxyId == bp->queryProxyId)
	{
		return true;
	}

	bool moved = b2DynamicTree_WasMoved(bp->trees + bp->queryTree, proxyId);
	if (moved && proxyId > bp->queryProxyId)
	{
		// Both proxies are moving. Avoid duplicate pairs.
		return true;
	}

	uint64_t userDataA, userDataB;
	if (proxyId < bp->queryProxyId)
	{
		userDataA = userData;
		userDataB = bp->queryUserData;
	}
	else
	{
		userDataA = bp->queryUserData;
		userDataB = proxyId;
	}

	bp->addPairFcn(userDataA, userDataB, bp->fcnContext);

	// continue the query
	return true;
}

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

		bp->queryTree = B2_PROXY_TREE(proxyKey);
		bp->queryProxyId = B2_PROXY_ID(proxyKey);

		const b2DynamicTree* tree = bp->trees + bp->queryTree;

		// We have to query the tree with the fat AABB so that
		// we don't fail to create a contact that may touch later.
		b2AABB fatAABB = b2DynamicTree_GetFatAABB(tree, bp->queryProxyId);
		bp->queryUserData = b2DynamicTree_GetUserData(tree, bp->queryProxyId);

		// Query tree and invoke b2AddPairFcn callback. A callback inside a callback.
		// TODO_ERIN test with filtering
		b2DynamicTree_Query(tree, fatAABB, b2QueryCallback, bp);
	}

	// Clear move flags
	for (int32_t i = 0; i < bp->moveCount; ++i)
	{
		int32_t proxyKey = bp->moveBuffer[i];
		if (proxyKey == B2_NULL_INDEX)
		{
			continue;
		}

		int32_t treeIndex = B2_PROXY_TREE(proxyKey);
		int32_t proxyId = B2_PROXY_ID(proxyKey);

		b2DynamicTree_ClearMoved(bp->trees + treeIndex, proxyId);
	}

	// Reset move buffer
	bp->moveCount = 0;
}
