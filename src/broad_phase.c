// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#define _CRT_SECURE_NO_WARNINGS

#include "broad_phase.h"

#include "allocate.h"
#include "array.h"
#include "body.h"
#include "contact.h"
#include "core.h"
#include "shape.h"
#include "stack_allocator.h"
#include "world.h"

#include "box2d/aabb.h"
#include "box2d/timer.h"

#include <stdatomic.h>
#include <stdbool.h>
#include <string.h>

//#include <stdio.h>

//static FILE* s_file = NULL;

void b2CreateBroadPhase(b2BroadPhase* bp)
{
	//if (s_file == NULL)
	//{
	//	s_file = fopen("pairs01.txt", "a");
	//	fprintf(s_file, "============\n\n");
	//}

	bp->proxyCount = 0;

	// TODO_ERIN initial size in b2WorldDef?
	bp->moveSet = b2CreateSet(16);
	bp->moveArray = b2CreateArray(sizeof(int32_t), 16);

	bp->moveResults = NULL;
	bp->movePairs = NULL;
	bp->movePairCapacity = 0;
	bp->movePairIndex = 0;

	// TODO_ERIN initial size from b2WorldDef
	bp->pairSet = b2CreateSet(32);

	for (int32_t i = 0; i < b2_bodyTypeCount; ++i)
	{
		bp->trees[i] = b2DynamicTree_Create();
	}
}

void b2DestroyBroadPhase(b2BroadPhase* bp)
{
	for (int32_t i = 0; i < b2_bodyTypeCount; ++i)
	{
		b2DynamicTree_Destroy(bp->trees + i);
	}

	b2DestroySet(&bp->moveSet);
	b2DestroyArray(bp->moveArray, sizeof(int32_t));
	b2DestroySet(&bp->pairSet);

	memset(bp, 0, sizeof(b2BroadPhase));

	//if (s_file != NULL)
	//{
	//	fclose(s_file);
	//	s_file = NULL;
	//}
}

static inline void b2UnBufferMove(b2BroadPhase* bp, int32_t proxyKey)
{
	bool found = b2RemoveKey(&bp->moveSet, proxyKey + 1);

	if (found)
	{
		// Purge from move buffer. Linear search.
		int32_t count = b2Array(bp->moveArray).count;
		for (int32_t i = 0; i < count; ++i)
		{
			if (bp->moveArray[i] == proxyKey)
			{
				b2Array_RemoveSwap(bp->moveArray, i);
				break;
			}
		}
	}
}

int32_t b2BroadPhase_CreateProxy(b2BroadPhase* bp, b2BodyType bodyType, b2AABB aabb, uint32_t categoryBits, int32_t shapeIndex)
{
	B2_ASSERT(0 <= bodyType && bodyType < b2_bodyTypeCount);
	int32_t proxyId = b2DynamicTree_CreateProxy(bp->trees + bodyType, aabb, categoryBits, shapeIndex);
	int32_t proxyKey = B2_PROXY_KEY(proxyId, bodyType);
	if (bodyType != b2_staticBody)
	{
		b2BufferMove(bp, proxyKey);
	}
	return proxyKey;
}

void b2BroadPhase_DestroyProxy(b2BroadPhase* bp, int32_t proxyKey)
{
	B2_ASSERT(b2Array(bp->moveArray).count == (int32_t)bp->moveSet.count);
	b2UnBufferMove(bp, proxyKey);

	--bp->proxyCount;

	int32_t typeIndex = B2_PROXY_TYPE(proxyKey);
	int32_t proxyId = B2_PROXY_ID(proxyKey);

	B2_ASSERT(0 <= typeIndex && typeIndex <= b2_bodyTypeCount);
	b2DynamicTree_DestroyProxy(bp->trees + typeIndex, proxyId);
}

void b2BroadPhase_MoveProxy(b2BroadPhase* bp, int32_t proxyKey, b2AABB aabb)
{
	b2BodyType bodyType = B2_PROXY_TYPE(proxyKey);
	int32_t proxyId = B2_PROXY_ID(proxyKey);

	b2DynamicTree_MoveProxy(bp->trees + bodyType, proxyId, aabb);
	if (bodyType != b2_staticBody)
	{
		b2BufferMove(bp, proxyKey);
	}
}

void b2BroadPhase_EnlargeProxy(b2BroadPhase* bp, int32_t proxyKey, b2AABB aabb)
{
	B2_ASSERT(proxyKey != B2_NULL_INDEX);
	int32_t typeIndex = B2_PROXY_TYPE(proxyKey);
	int32_t proxyId = B2_PROXY_ID(proxyKey);

	B2_ASSERT(typeIndex == b2_dynamicBody || typeIndex == b2_kinematicBody);

	b2DynamicTree_EnlargeProxy(bp->trees + typeIndex, proxyId, aabb);
	b2BufferMove(bp, proxyKey);
}

typedef struct b2MovePair
{
	int32_t shapeIndexA;
	int32_t shapeIndexB;
	b2MovePair* next;
	bool heap;
} b2MovePair;

typedef struct b2MoveResult
{
	b2MovePair* pairList;
} b2MoveResult;

typedef struct b2QueryPairContext
{
	b2World* world;
	b2MoveResult* moveResult;
	b2BodyType queryTreeType;
	int32_t queryProxyKey;
	int32_t queryShapeIndex;
} b2QueryPairContext;

// This is called from b2DynamicTree::Query when we are gathering pairs.
static bool b2PairQueryCallback(int32_t proxyId, int32_t shapeIndex, void* context)
{
	b2QueryPairContext* queryContext = context;
	b2BroadPhase* bp = &queryContext->world->broadPhase;

	int32_t proxyKey = B2_PROXY_KEY(proxyId, queryContext->queryTreeType);

	// A proxy cannot form a pair with itself.
	if (proxyKey == queryContext->queryProxyKey)
	{
		return true;
	}

	bool moved = b2ContainsKey(&bp->moveSet, proxyKey + 1);
	if (moved && proxyKey < queryContext->queryProxyKey)
	{
		// Both proxies are moving. Avoid duplicate pairs.
		return true;
	}

	uint64_t pairKey = B2_SHAPE_PAIR_KEY(shapeIndex, queryContext->queryShapeIndex);
	if (b2ContainsKey(&bp->pairSet, pairKey))
	{
		// contact exists
		return true;
	}

	int32_t shapeIndexA, shapeIndexB;
	if (proxyKey < queryContext->queryProxyKey)
	{
		shapeIndexA = shapeIndex;
		shapeIndexB = queryContext->queryShapeIndex;
	}
	else
	{
		shapeIndexA = queryContext->queryShapeIndex;
		shapeIndexB = shapeIndex;
	}

	b2World* world = queryContext->world;

	B2_ASSERT(0 <= shapeIndexA && shapeIndexA < world->shapePool.capacity);
	B2_ASSERT(0 <= shapeIndexB && shapeIndexB < world->shapePool.capacity);

	b2Shape* shapeA = world->shapes + shapeIndexA;
	b2Shape* shapeB = world->shapes + shapeIndexB;

	// Are the shapes on the same body?
	if (shapeA->bodyIndex == shapeB->bodyIndex)
	{
		return true;
	}

	if (b2ShouldShapesCollide(shapeA->filter, shapeB->filter) == false)
	{
		return true;
	}

	int32_t bodyIndexA = shapeA->bodyIndex;
	int32_t bodyIndexB = shapeB->bodyIndex;
	b2Body* bodyA = world->bodies + bodyIndexA;
	b2Body* bodyB = world->bodies + bodyIndexB;

	// Does a joint override collision? Is at least one body dynamic?
	// TODO_ERIN this could be a hash set
	if (b2ShouldBodiesCollide(world, bodyA, bodyB) == false)
	{
		return true;
	}

	// TODO_ERIN per thread to eliminate atomic?
	int pairIndex = atomic_fetch_add(&bp->movePairIndex, 1);

	b2MovePair* pair;
	if (pairIndex < bp->movePairCapacity)
	{
		pair = bp->movePairs + pairIndex;
		pair->heap = false;
	}
	else
	{
		pair = b2Alloc(sizeof(b2MovePair));
		pair->heap = true;
	}

	pair->shapeIndexA = shapeIndexA;
	pair->shapeIndexB = shapeIndexB;
	pair->next = queryContext->moveResult->pairList;
	queryContext->moveResult->pairList = pair;

	// continue the query
	return true;
}

void b2FindPairsTask(int32_t startIndex, int32_t endIndex, uint32_t threadIndex, void* context)
{
	b2TracyCZoneNC(pair_task, "Pair Task", b2_colorAquamarine3, true);

	B2_MAYBE_UNUSED(threadIndex);

	b2World* world = context;
	b2BroadPhase* bp = &world->broadPhase;

	b2QueryPairContext queryContext;
	queryContext.world = world;

	for (int32_t i = startIndex; i < endIndex; ++i)
	{
		// Initialize move result for this moved proxy
		queryContext.moveResult = bp->moveResults + i;
		queryContext.moveResult->pairList = NULL;

		int32_t proxyKey = bp->moveArray[i];
		if (proxyKey == B2_NULL_INDEX)
		{
			// proxy was destroyed after it moved
			continue;
		}

		b2BodyType proxyType = B2_PROXY_TYPE(proxyKey);
		int32_t proxyId = B2_PROXY_ID(proxyKey);
		queryContext.queryProxyKey = proxyKey;

		const b2DynamicTree* baseTree = bp->trees + proxyType;

		// We have to query the tree with the fat AABB so that
		// we don't fail to create a contact that may touch later.
		b2AABB fatAABB = b2DynamicTree_GetAABB(baseTree, proxyId);
		queryContext.queryShapeIndex = b2DynamicTree_GetUserData(baseTree, proxyId);

		// Query trees
		if (proxyType == b2_dynamicBody)
		{
			queryContext.queryTreeType = b2_staticBody;
			b2DynamicTree_Query(bp->trees + b2_staticBody, fatAABB, b2PairQueryCallback, &queryContext);
			queryContext.queryTreeType = b2_kinematicBody;
			b2DynamicTree_Query(bp->trees + b2_kinematicBody, fatAABB, b2PairQueryCallback, &queryContext);
			queryContext.queryTreeType = b2_dynamicBody;
			b2DynamicTree_Query(bp->trees + b2_dynamicBody, fatAABB, b2PairQueryCallback, &queryContext);
		}
		else if (proxyType == b2_kinematicBody)
		{
			queryContext.queryTreeType = b2_dynamicBody;
			b2DynamicTree_Query(bp->trees + b2_dynamicBody, fatAABB, b2PairQueryCallback, &queryContext);
		}
	}

	b2TracyCZoneEnd(pair_task);
}

extern bool b2_parallel;

void b2UpdateBroadPhasePairs(b2World* world)
{
	b2BroadPhase* bp = &world->broadPhase;

	int32_t moveCount = b2Array(bp->moveArray).count;
	B2_ASSERT(moveCount == (int32_t)bp->moveSet.count);

	if (moveCount == 0)
	{
		return;
	}

	b2TracyCZoneNC(update_pairs, "Pairs", b2_colorFuchsia, true);

	b2StackAllocator* alloc = world->stackAllocator;

	bp->moveResults = b2AllocateStackItem(alloc, moveCount * sizeof(b2MoveResult), "move results");
	bp->movePairCapacity = 16 * moveCount;
	bp->movePairs = b2AllocateStackItem(alloc, bp->movePairCapacity * sizeof(b2MovePair), "move pairs");
	bp->movePairIndex = 0;

	if (b2_parallel)
	{
		int32_t minRange = 64;
		void* userPairTask = world->enqueueTaskFcn(&b2FindPairsTask, moveCount, minRange, world, world->userTaskContext);
		world->taskCount += 1;
		world->finishTaskFcn(userPairTask, world->userTaskContext);
	}
	else
	{
		b2FindPairsTask(0, moveCount, 0, world);
	}

	b2TracyCZoneNC(create_contacts, "Create Contacts", b2_colorGold, true);

	// Single-threaded work
	// - Clear move flags
	// - Create contacts in deterministic order
	b2Shape* shapes = world->shapes;

	//int32_t pairCount = 0;

	for (int32_t i = 0; i < moveCount; ++i)
	{
		b2MoveResult* result = bp->moveResults + i;
		b2MovePair* pair = result->pairList;
		while (pair != NULL)
		{
			// TODO_ERIN Check user filtering.
			// if (m_contactFilter && m_contactFilter->ShouldCollide(shapeA, shapeB) == false)
			//{
			//	return;
			//}

			int32_t shapeIndexA = pair->shapeIndexA;
			int32_t shapeIndexB = pair->shapeIndexB;

			//if (s_file != NULL)
			//{
			//	fprintf(s_file, "%d %d\n", shapeIndexA, shapeIndexB);
			//}

			//++pairCount;

			B2_ASSERT(0 <= shapeIndexA && shapeIndexA < world->shapePool.capacity);
			B2_ASSERT(0 <= shapeIndexB && shapeIndexB < world->shapePool.capacity);

			b2CreateContact(world, shapes + shapeIndexA, shapes + shapeIndexB);

			if (pair->heap)
			{
				b2MovePair* temp = pair;
				pair = pair->next;
				b2Free(temp, sizeof(b2MovePair));
			}
			else
			{
				pair = pair->next;
			}
		}

		//if (s_file != NULL)
		//{
		//	fprintf(s_file, "\n");
		//}
	}

	//if (s_file != NULL)
	//{
	//	fprintf(s_file, "count = %d\n\n", pairCount);
	//}

	// Reset move buffer
	b2Array_Clear(bp->moveArray);
	b2ClearSet(&bp->moveSet);

	b2FreeStackItem(alloc, bp->movePairs);
	bp->movePairs = NULL;
	b2FreeStackItem(alloc, bp->moveResults);
	bp->moveResults = NULL;

	b2TracyCZoneEnd(create_contacts);

	b2TracyCZoneEnd(update_pairs);
}

bool b2BroadPhase_TestOverlap(const b2BroadPhase* bp, int32_t proxyKeyA, int32_t proxyKeyB)
{
	int32_t typeIndexA = B2_PROXY_TYPE(proxyKeyA);
	int32_t proxyIdA = B2_PROXY_ID(proxyKeyA);
	int32_t typeIndexB = B2_PROXY_TYPE(proxyKeyB);
	int32_t proxyIdB = B2_PROXY_ID(proxyKeyB);

	b2AABB aabbA = b2DynamicTree_GetAABB(bp->trees + typeIndexA, proxyIdA);
	b2AABB aabbB = b2DynamicTree_GetAABB(bp->trees + typeIndexB, proxyIdB);
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

void b2ValidateBroadphase(const b2BroadPhase* bp)
{
	b2DynamicTree_Validate(bp->trees + b2_dynamicBody);
	b2DynamicTree_Validate(bp->trees + b2_kinematicBody);

	// TODO_ERIN validate every shape AABB is contained in tree AABB
}

void b2ValidateNoEnlarged(const b2BroadPhase* bp)
{
#if B2_VALIDATE == 1
	for (int32_t j = 0; j < b2_bodyTypeCount; ++j)
	{
		const b2DynamicTree* tree = bp->trees + j;
		int32_t capacity = tree->nodeCapacity;
		const b2TreeNode* nodes = tree->nodes;
		for (int32_t i = 0; i < capacity; ++i)
		{
			const b2TreeNode* node = nodes + i;
			if (node->height < 0)
			{
				continue;
			}

			if (node->enlarged == true)
			{
				capacity += 0;
			}

			B2_ASSERT(node->enlarged == false);
		}
	}
#else
	B2_MAYBE_UNUSED(bp);
#endif
}
