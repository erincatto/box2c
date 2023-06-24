// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "thread.h"

#include <stdint.h>

typedef struct b2StackAllocator b2StackAllocator;
typedef struct b2Contact b2Contact;

// Union-find link with extra data
typedef struct b2BodyLink
{
	// Parent in union-find tree
	B2_ATOMIC long parent;
	int32_t islandIndex;
} b2BodyLink;

typedef struct b2IslandIndex
{
	int32_t index;
	int32_t constraintCount;
} b2IslandIndex;

typedef struct b2ContactElement
{
	b2Contact* contact;

	// sort key is shapeIndexA << 32 | shapeIndexB
	// needed to sort contacts for determinism
	uint64_t sortKey;
} b2ContactElement;

// Parallel Union-Find island builder based Jorrit Rouwe's presentation:
// https://gdcvault.com/play/1027560/Architecting-Jolt-Physics-for-Horizon
// and Jolt Physics: https://github.com/jrouwe/JoltPhysics
typedef struct b2IslandBuilder
{
	// Union-find data populated in narrow-phase

	// Array of body links one-to-one with b2World::activeBodies. Must be contiguous.
	b2BodyLink*	links;

	// Map joint index to active body index
	int32_t* jointLinks;

	// Map contact index to active body index
	int32_t* contactLinks;

	// Final island data

	// Body indices ordered by island. Index into b2World::bodies.
	int32_t* bodyIslands;

	// End body index of each island. Index into bodyIslands.
	int32_t* bodyIslandEnds;

	// Joint indices ordered by island
	int32_t* jointIslands;

	// End index of each joint island
	int32_t* jointIslandEnds;
	
	// Contact indices ordered by island. Index into b2World::activeContacts
	// TODO_ERIN makes this an array of b2Contact*?
	b2ContactElement* contactElements;
	b2ContactElement* contactIslands;

	// End contact index for each island
	int32_t* contactIslandEnds;

	// Island indices sorted by contact + joint count
	struct b2IslandIndex* sortedIslands;

	int32_t bodyCapacity;
	int32_t bodyCount;
	int32_t jointCapacity;
	int32_t jointCount;
	int32_t contactCapacity;
	int32_t contactCount;
	int32_t islandCount;

} b2IslandBuilder;

b2IslandBuilder b2CreateIslandBuilder(int32_t bodyCapacity);
void b2DestroyIslandBuilder(b2IslandBuilder* builder);

// single-thread call
void b2StartIslands(b2IslandBuilder* builder, int32_t bodyCapacity, int32_t jointCapacity, int32_t contactCapacity, b2StackAllocator* allocator);

// multi-thread calls

// Link a joint given the index into the active joint array and the awake body array indices
void b2LinkJoint(b2IslandBuilder* builder, int32_t jointIndex, int32_t awakeIndexA, int32_t awakeIndexB);

// Link a contact given the index into the active contact array and the awake body array indices
void b2LinkContact(b2IslandBuilder* builder, int32_t contactIndex, b2Contact* contact, uint64_t sortKey, int32_t awakeIndexA, int32_t awakeIndexB);

// single-thread calls
void b2FinishIslands(b2IslandBuilder* builder, const int32_t* awakeBodies, int32_t bodyCount, int32_t jointCount, int32_t contactCount, b2StackAllocator* allocator);
void b2ResetIslands(b2IslandBuilder* builder, b2StackAllocator* allocator);
