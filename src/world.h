// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "bitset.h"
#include "broad_phase.h"
#include "island.h"
#include "graph.h"
#include "pool.h"

#include "box2d/callbacks.h"
#include "box2d/timer.h"

#define B2_GRAPH_COLOR 1

typedef struct b2Contact b2Contact;

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
	int16_t index;

	struct b2BlockAllocator* blockAllocator;
	struct b2StackAllocator* stackAllocator;

	b2BroadPhase broadPhase;
	b2Graph graph;

	b2Pool bodyPool;
	b2Pool contactPool;
	b2Pool jointPool;
	b2Pool shapePool;
	b2Pool chainPool;
	b2Pool islandPool;

	// These are sparse arrays that point into the pools above
	struct b2Body* bodies;
	struct b2Contact* contacts;
	struct b2Joint* joints;
	struct b2Shape* shapes;
	struct b2ChainShape* chains;
	struct b2Island* islands;

	// Per thread storage
	b2TaskContext* taskContextArray;

	// Awake island array holds indices into the island array (islandPool).
	// This is a dense array that is rebuilt every time step.
	int32_t* awakeIslandArray;

	// Awake contact array holds contacts that should be updated.
	// This is a dense array that is rebuilt every time step. Order doesn't matter for determinism
	// but a bit set is used to prevent duplicates
	int32_t* awakeContactArray;

	// Hot data split from b2Contact. Used when a contact is destroyed and needs to be removed from the awake contact array.
	// A contact is destroyed when a shape/body is destroyed or when the shape AABBs stop overlapping.
	// TODO_ERIN use a bit array somehow?
	int32_t* contactAwakeIndexArray;

	struct b2SensorBeginTouchEvent* sensorBeginEventArray;
	struct b2SensorEndTouchEvent* sensorEndEventArray;

	// Array of fast bodies that need continuous collision handling
	int32_t* fastBodies;
	_Atomic int fastBodyCount;

	// Id that is incremented every time step
	uint64_t stepId;

	b2Vec2 gravity;
	float restitutionThreshold;
	float contactPushoutVelocity;
	float contactHertz;
	float contactDampingRatio;

	// This is used to compute the time step ratio to support a variable time step.
	float inv_dt0;

	uint16_t revision;

	b2Profile profile;

	b2PreSolveFcn* preSolveFcn;
	void* preSolveContext;

	b2PostSolveFcn* postSolveFcn;
	void* postSolveContext;

	uint32_t workerCount;
	b2EnqueueTaskCallback* enqueueTaskFcn;
	b2FinishTaskCallback* finishTaskFcn;
	b2FinishAllTasksCallback* finishAllTasksFcn;
	void* userTaskContext;

	void* userTreeTask;

	int32_t splitIslandIndex;

	bool enableSleep;
	bool locked;
	bool enableWarmStarting;
	bool enableContinuous;
} b2World;

b2World* b2GetWorldFromId(b2WorldId id);
b2World* b2GetWorldFromIndex(int16_t index);

bool b2IsBodyIdValid(b2World* world, b2BodyId id);
