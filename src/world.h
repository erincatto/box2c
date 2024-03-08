// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "block_allocator.h"
#include "bitset.h"
#include "broad_phase.h"
#include "constraint_graph.h"
#include "id_pool.h"
#include "island.h"
#include "pool.h"

#include "box2d/callbacks.h"
#include "box2d/timer.h"

#define B2_GRAPH_COLOR 1

typedef struct b2Contact b2Contact;

enum b2BodySetType
{
	b2_staticBodySet = 0,
	b2_awakeBodySet = 1,
	b2_disabledBodySet = 2,
};

// Per thread task storage
typedef struct b2TaskContext
{
	// These bits align with the awake contact array and signal change in contact status
	// that affects the island graph.
	b2BitSet contactStateBitSet;

	// Used to prevent duplicate awake contacts
	b2BitSet awakeContactBitSet;

	// Used to sort shapes that have enlarged AABBs
	b2BitSet shapeBitSet;

	// Used to wake islands
	b2BitSet awakeIslandBitSet;
} b2TaskContext;

/// The world class manages all physics entities, dynamic simulation,
/// and asynchronous queries. The world also contains efficient memory
/// management facilities.
typedef struct b2World
{
	b2BlockAllocator blockAllocator;
	struct b2StackAllocator* stackAllocator;

	b2BroadPhase broadPhase;
	b2ConstraintGraph graph;

	// The body id pool is used to allocate and recycle body ids. Body ids
	// provide a stable identifier for users, but incur caches misses when used
	// to access body data.
	b2IdPool bodyIdPool;

	// This is a sparse array that maps body ids to the body data
	// stored in body sets. As bodies move within a set or across set
	struct b2BodyLookup* bodyLookupArray;

	// Body sets allow bodies to be stored in contiguous arrays. The first
	// set is all static bodies. The second set is active bodies. The remaining
	// sets are sleeping islands.
	struct b2BodySet* bodySetArray;

	b2Pool contactPool;
	b2Pool jointPool;
	b2Pool shapePool;
	b2Pool chainPool;
	b2Pool islandPool;

	// These are sparse arrays that point into the pools above
	struct b2Contact* contacts;
	struct b2Joint* joints;
	struct b2Shape* shapes;
	struct b2ChainShape* chains;
	struct b2Island* islands;

	// Per thread storage
	b2TaskContext* taskContextArray;

	// Awake island array holds indices into the island array (islandPool).
	// This is a dense array that is rebuilt every time step.
	// todo should not be needed with body sets
	int32_t* awakeIslandArray;

	// Awake contact array holds contacts that should be updated.
	// This is a dense array that is rebuilt every time step. Order doesn't matter for determinism
	// but a bit set is used to prevent duplicates
	int32_t* awakeContactArray;

	// Hot data split from b2Contact. Used when a contact is destroyed and needs to be removed from the awake contact array.
	// A contact is destroyed when a shape/body is destroyed or when the shape AABBs stop overlapping.
	// TODO_ERIN use a bit array somehow?
	int32_t* contactAwakeIndexArray;

	struct b2BodyMoveEvent* bodyMoveEventArray;
	struct b2SensorBeginTouchEvent* sensorBeginEventArray;
	struct b2SensorEndTouchEvent* sensorEndEventArray;
	struct b2ContactBeginTouchEvent* contactBeginArray;
	struct b2ContactEndTouchEvent* contactEndArray;

	// Array of fast bodies that need continuous collision handling
	int32_t* fastBodies;
	_Atomic int fastBodyCount;

	// Array of bullet bodies that need continuous collision handling
	int32_t* bulletBodies;
	_Atomic int bulletBodyCount;

	// Id that is incremented every time step
	uint64_t stepId;

	// sub-step info from the most recent time step
	int32_t subStepCount;
	float inv_h;

	b2Vec2 gravity;
	float restitutionThreshold;
	float contactPushoutVelocity;
	float contactHertz;
	float contactDampingRatio;
	float jointHertz;
	float jointDampingRatio;

	uint16_t revision;

	b2Profile profile;

	b2PreSolveFcn* preSolveFcn;
	void* preSolveContext;

	uint32_t workerCount;
	b2EnqueueTaskCallback* enqueueTaskFcn;
	b2AddPinnedTaskCallback* addPinnedTaskFcn;
	b2FinishTaskCallback* finishTaskFcn;
	b2FinishPinnedTaskCallback* finishPinnedTaskFcn;
	void* userTaskContext;

	void* userTreeTask;

	int32_t splitIslandIndex;

	int32_t activeTaskCount;
	int32_t taskCount;

	uint16_t worldIndex;

	bool enableSleep;
	bool locked;
	bool enableWarmStarting;
	bool enableContinuous;
} b2World;

b2World* b2GetWorldFromId(b2WorldId id);
b2World* b2GetWorldFromIndex(uint16_t index);
b2World* b2GetWorldFromIndexLocked(uint16_t index);
