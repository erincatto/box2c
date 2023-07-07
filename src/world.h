// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "broad_phase.h"
#include "island.h"
#include "pool.h"
#include "thread.h"

#include "box2d/callbacks.h"
#include "box2d/timer.h"

typedef struct b2Contact b2Contact;

/// The world class manages all physics entities, dynamic simulation,
/// and asynchronous queries. The world also contains efficient memory
/// management facilities.
typedef struct b2World
{
	int16_t index;

	struct b2BlockAllocator* blockAllocator;
	struct b2StackAllocator* stackAllocator;

	b2BroadPhase broadPhase;

	b2Pool bodyPool;
	b2Pool contactPool;
	b2Pool jointPool;
	b2Pool shapePool;
	b2Pool islandPool;

	// These are sparse arrays that point into the pools above
	struct b2Body* bodies;
	struct b2Contact* contacts;
	struct b2Joint* joints;
	struct b2Shape* shapes;
	struct b2Island* islands;

	// Per thread storage
	struct b2TaskContext* taskContextArray;

	// Awake island array holds indices into the island array (islandPool).
	// This is a dense array that is rebuilt every time step.
	int32_t* awakeIslandArray;

	// Awake contact array holds contacts that should be updated.
	// This is a dense array that is rebuilt every time step.
	int32_t* awakeContactArray;

	// This transient array holds islands created from splitting a larger island.
	int32_t* splitIslandArray;

	// Transient index of the island being split this time step. May be B2_NULL_INDEX.
	int32_t splitIslandIndex;

	// Id that is incremented every time step
	uint64_t stepId;

	b2Vec2 gravity;
	float restitutionThreshold;

	// b2DestructionListener* m_destructionListener;

	// This is used to compute the time step ratio to support a variable time step.
	float inv_dt0;

	uint16_t revision;

	b2Profile profile;
	B2_ATOMIC long contactPointCount;

	b2PreSolveFcn* preSolveFcn;
	void* preSolveContext;

	b2PostSolveFcn* postSolveFcn;
	void* postSolveContext;

	uint32_t workerCount;
	b2EnqueueTaskCallback* enqueueTask;
	b2FinishTasksCallback* finishTasks;
	void* userTaskContext;

	bool enableSleep;
	bool newContacts;
	bool locked;
	bool warmStarting;
} b2World;

b2World* b2GetWorldFromId(b2WorldId id);
b2World* b2GetWorldFromIndex(int16_t index);

bool b2IsBodyIdValid(b2World* world, b2BodyId id);
