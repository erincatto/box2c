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

enum b2SetType
{
	b2_staticSet = 0,
	b2_awakeSet = 1,
	b2_disabledSet = 2,
	b2_firstSleepingSet = 3,
};

// Per thread task storage
typedef struct b2TaskContext
{
	// These bits align with the b2ConstraingGraph::contactBlocks and signal a change in contact status
	b2BitSet contactStateBitSet;

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

	// The body id pool is used to allocate and recycle body ids. Body ids
	// provide a stable identifier for users, but incur caches misses when used
	// to access body data. Aligns with b2BodyLookup.
	b2IdPool bodyIdPool;

	// This is a sparse array that maps body ids to the body data
	// stored in body sets. As bodies move within a set or across set.
	// Indices come from id pool.
	struct b2BodyLookup* bodyLookupArray;

	// Provides free list for solver sets.
	b2IdPool solverSetIdPool;

	// Solvers sets allow bodies to be stored in contiguous arrays. The first
	// set is all static bodies. The second set is active bodies. The remaining
	// sets are sleeping islands.
	struct b2SolverSet* solverSetArray;

	b2ConstraintGraph constraintGraph;

	// Used to create stable ids for joints
	b2IdPool jointIdPool;

	// This is a sparse array that maps joint ids to the joint data stored in the constraint graph
	// or in the simulation sets.
	struct b2JointLookup* jointLookupArray;

	// Used to create stable ids for contacts
	b2IdPool contactIdPool;

	// This is a sparse array that maps contact ids to the contact data stored in the constraint graph
	// or in the simulation sets.
	struct b2ContactLookup* contactLookupArray;

	b2Pool shapePool;
	b2Pool chainPool;
	b2Pool islandPool;

	// These are sparse arrays that point into the pools above
	struct b2Shape* shapes;
	struct b2ChainShape* chains;
	struct b2Island* islands;

	// Per thread storage
	b2TaskContext* taskContextArray;

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

	uint16_t worldId;

	bool enableSleep;
	bool locked;
	bool enableWarmStarting;
	bool enableContinuous;
} b2World;

b2World* b2GetWorldFromId(b2WorldId id);
b2World* b2GetWorldFromIndex(int index);
b2World* b2GetWorldFromIndexLocked(int index);
