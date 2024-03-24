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
#include "stack_allocator.h"

#include "box2d/callbacks.h"
#include "box2d/timer.h"

#define B2_GRAPH_COLOR 1

typedef struct b2Contact b2Contact;

enum b2SetType
{
	b2_awakeSet = 0,
	b2_disabledSet = 1,
	b2_firstSleepingSet = 2,
};

// Per thread task storage
typedef struct b2TaskContext
{
	// These bits align with the b2ConstraintGraph::contactBlocks and signal a change in contact status
	b2BitSet contactStateBitSet;

	// Used to indicate bodies with shapes that have enlarged AABBs
	b2BitSet enlargedBodyBitSet;

	// Used to wake islands
	b2BitSet awakeIslandBitSet;
} b2TaskContext;

/// The world class manages all physics entities, dynamic simulation,
/// and asynchronous queries. The world also contains efficient memory
/// management facilities.
typedef struct b2World
{
	b2BlockAllocator blockAllocator;
	b2StackAllocator stackAllocator;
	b2BroadPhase broadPhase;
	b2ConstraintGraph constraintGraph;

	b2IdPool staticBodyIdPool;
	struct b2StaticBody* staticBodyArray;

	// The body id pool is used to allocate and recycle body ids. Body ids
	// provide a stable identifier for users, but incur caches misses when used
	// to access body data. Aligns with b2BodyLookup.
	b2IdPool bodyIdPool;

	// This is a sparse array that maps body ids to the body data
	// stored in solver sets. As bodies move within a set or across set.
	// Indices come from id pool.
	struct b2BodyLookup* bodyLookupArray;

	// Provides free list for solver sets.
	b2IdPool solverSetIdPool;

	// Solvers sets allow bodies to be stored in contiguous arrays. The first
	// set is all static bodies. The second set is active bodies. The third set is disabled
	// bodies. The remaining sets are sleeping islands.
	struct b2SolverSet* solverSetArray;

	// Used to create stable ids for joints
	b2IdPool jointIdPool;

	// This is a sparse array that maps joint ids to the joint data stored in the constraint graph
	// or in the solver sets.
	struct b2JointLookup* jointLookupArray;

	// Used to create stable ids for contacts
	b2IdPool contactIdPool;

	// This is a sparse array that maps contact ids to the contact data stored in the constraint graph
	// or in the solver sets.
	struct b2ContactLookup* contactLookupArray;

	// Used to create stable ids for islands
	b2IdPool islandIdPool;

	// This is a sparse array that maps island ids to the island data stored in the solver sets.
	struct b2IslandLookup* islandLookupArray;

	b2Pool shapePool;
	b2Pool chainPool;

	// These are sparse arrays that point into the pools above
	struct b2Shape* shapes;
	struct b2ChainShape* chains;

	// Per thread storage
	b2TaskContext* taskContextArray;

	struct b2BodyMoveEvent* bodyMoveEventArray;
	struct b2SensorBeginTouchEvent* sensorBeginEventArray;
	struct b2SensorEndTouchEvent* sensorEndEventArray;
	struct b2ContactBeginTouchEvent* contactBeginArray;
	struct b2ContactEndTouchEvent* contactEndArray;

	// Id that is incremented every time step
	uint64_t stepIndex;

	int splitIslandId;
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
	b2FinishTaskCallback* finishTaskFcn;
	void* userTaskContext;
	void* userTreeTask;

	// Remember type step used for reporting forces and torques
	float inv_h;

	int32_t activeTaskCount;
	int32_t taskCount;

	uint16_t worldId;

	bool enableSleep;
	bool locked;
	bool enableWarmStarting;
	bool enableContinuous;
	bool inUse;
} b2World;

b2World* b2GetWorldFromId(b2WorldId id);
b2World* b2GetWorld(int index);
b2World* b2GetWorldLocked(int index);
void b2ValidateWorld(b2World* world);
