// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "broad_phase.h"

#include "allocate.h"
#include "array.h"
#include "core.h"

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
	bp->queryShapeIndex = B2_NULL_INDEX;

	for (int32_t i = 0; i < b2_bodyTypeCount; ++i)
	{
		bp->trees[i] = b2DynamicTree_Create();
	}
}

void b2BroadPhase_Destroy(b2BroadPhase* bp)
{
	for (int32_t i = 0; i < b2_bodyTypeCount; ++i)
	{
		b2DynamicTree_Destroy(bp->trees + i);
	}

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

int32_t b2BroadPhase_CreateProxy(b2BroadPhase* bp, b2BodyType bodyType, b2AABB aabb, uint32_t categoryBits, int32_t shapeIndex,
								 b2AABB* outFatAABB)
{
	B2_ASSERT(0 <= bodyType && bodyType < b2_bodyTypeCount);
	int32_t proxyId = b2DynamicTree_CreateProxy(bp->trees + bodyType, aabb, categoryBits, shapeIndex, outFatAABB);
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

	B2_ASSERT(0 <= typeIndex && typeIndex <= b2_bodyTypeCount);
	b2DynamicTree_DestroyProxy(bp->trees + typeIndex, proxyId);
}

void b2BroadPhase_MoveProxy(b2BroadPhase* bp, int32_t proxyKey, b2AABB aabb, b2AABB* outFatAABB)
{
	int32_t typeIndex = B2_PROXY_TYPE(proxyKey);
	int32_t proxyId = B2_PROXY_ID(proxyKey);

	B2_ASSERT(typeIndex == b2_dynamicBody || typeIndex == b2_kinematicBody);

	bool buffer = b2DynamicTree_MoveProxy(bp->trees + typeIndex, proxyId, aabb, outFatAABB);
	if (buffer)
	{
		b2BufferMove(bp, proxyKey);
	}
}

void b2BroadPhase_EnlargeProxy(b2BroadPhase* bp, int32_t proxyKey, b2AABB aabb, b2AABB* outFatAABB)
{
	int32_t typeIndex = B2_PROXY_TYPE(proxyKey);
	int32_t proxyId = B2_PROXY_ID(proxyKey);

	B2_ASSERT(typeIndex == b2_dynamicBody || typeIndex == b2_kinematicBody);

	bool buffer = b2DynamicTree_EnlargeProxy(bp->trees + typeIndex, proxyId, aabb, outFatAABB);
	if (buffer)
	{
		b2BufferMove(bp, proxyKey);
	}
}

// This is called from b2DynamicTree::Query when we are gathering pairs.
static bool b2QueryCallback(int32_t proxyId, int32_t shapeIndex, void* context)
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

	uint64_t pairKey = B2_SHAPE_PAIR_KEY(shapeIndex, bp->queryShapeIndex);
	if (b2ContainsKey(&bp->pairSet, pairKey))
	{
		// contact exists
		return true;
	}

	int32_t shapeIndexA, shapeIndexB;
	if (proxyKey < bp->queryProxyKey)
	{
		shapeIndexA = shapeIndex;
		shapeIndexB = bp->queryShapeIndex;
	}
	else
	{
		shapeIndexA = bp->queryShapeIndex;
		shapeIndexB = shapeIndex;
	}

	bp->addPairFcn(shapeIndexA, shapeIndexB, bp->fcnContext);

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
		bp->queryShapeIndex = b2DynamicTree_GetUserData(baseTree, proxyId);

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
	b2DynamicTree_Rebuild(bp->trees + b2_dynamicBody, false);
	b2DynamicTree_Rebuild(bp->trees + b2_kinematicBody, false);
}

int32_t b2BroadPhase_GetShapeIndex(b2BroadPhase* bp, int32_t proxyKey)
{
	int32_t typeIndex = B2_PROXY_TYPE(proxyKey);
	int32_t proxyId = B2_PROXY_ID(proxyKey);

	return b2DynamicTree_GetUserData(bp->trees + typeIndex, proxyId);
}
