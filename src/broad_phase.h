// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "array.h"
#include "table.h"

#include "box2d/dynamic_tree.h"

typedef struct b2Shape b2Shape;
typedef struct b2MovePair b2MovePair;
typedef struct b2MoveResult b2MoveResult;
typedef struct b2StackAllocator b2StackAllocator;
typedef struct b2World b2World;

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
	int32_t proxyCount;

	// The move set and array are used to track shapes that have moved significantly
	// and need a pair query for new contacts. The array has a deterministic order.
	// TODO_ERIN perhaps just a move set?
	b2Set moveSet;
	int32_t* moveArray;

	// These are the results from the pair query and are used to create new contacts
	// in deterministic order.
	b2MoveResult* moveResults;
	b2MovePair* movePairs;
	int32_t movePairCapacity;
	_Atomic int movePairIndex;

	b2Set pairSet;

} b2BroadPhase;

void b2CreateBroadPhase(b2BroadPhase* bp);
void b2DestroyBroadPhase(b2BroadPhase* bp);
int32_t b2BroadPhase_CreateProxy(b2BroadPhase* bp, b2BodyType bodyType, b2AABB aabb, uint32_t categoryBits, int32_t shapeIndex);
void b2BroadPhase_DestroyProxy(b2BroadPhase* bp, int32_t proxyKey);

void b2BroadPhase_MoveProxy(b2BroadPhase* bp, int32_t proxyKey, b2AABB aabb);
void b2BroadPhase_EnlargeProxy(b2BroadPhase* bp, int32_t proxyKey, b2AABB aabb);

void b2BroadPhase_RebuildTrees(b2BroadPhase* bp);

int32_t b2BroadPhase_GetShapeIndex(b2BroadPhase* bp, int32_t proxyKey);

void b2UpdateBroadPhasePairs(b2World* world);
bool b2BroadPhase_TestOverlap(const b2BroadPhase* bp, int32_t proxyKeyA, int32_t proxyKeyB);

void b2ValidateBroadphase(const b2BroadPhase* bp);
void b2ValidateNoEnlarged(const b2BroadPhase* bp);

// Warning: this must be called in deterministic order
static inline void b2BufferMove(b2BroadPhase* bp, int32_t proxyKey)
{
	bool alreadyAdded = b2AddKey(&bp->moveSet, proxyKey);
	if (alreadyAdded == false)
	{
		b2Array_Push(bp->moveArray, proxyKey);
	}
}
