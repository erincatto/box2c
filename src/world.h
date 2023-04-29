// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "broad_phase.h"
#include "island_builder.h"
#include "pool.h"
#include "thread.h"

#include "box2d/callbacks.h"
#include "box2d/timer.h"

/// The world class manages all physics entities, dynamic simulation,
/// and asynchronous queries. The world also contains efficient memory
/// management facilities.
typedef struct b2World
{
	int16_t index;

	struct b2BlockAllocator* blockAllocator;
	struct b2StackAllocator* stackAllocator;

	b2BroadPhase broadPhase;
	b2IslandBuilder islandBuilder;

	b2Pool bodyPool;
	b2Pool jointPool;
	b2Pool shapePool;

	// These are sparse arrays that point into the pools above
	struct b2Body* bodies;
	struct b2Joint* joints;
	struct b2Shape* shapes;

	int32_t bodyCapacity;
	int32_t contactCapacity;

	// This is a dense dmArray
	struct b2Contact** contactArray;

	// Fixed capacity array allocated at world creation and rebuilt every time step.
	// These are solid contacts that are touching and awake. b2IslandBuilder builds islands
	// that index into this array.
	struct b2Contact** activeContacts;

	// This atomic allows contacts to be added to the active contact array and to b2IslandBuilder
	// from multiple threads.
	B2_ATOMIC long activeContactCount;

	// Fixed capacity array of contacts with shapes that no longer have overlapping bounding boxes
	struct b2Contact** invalidContacts;
	B2_ATOMIC long invalidContactCount;

	// Awake body array holds indices into bodies array (bodyPool).
	// This is a dense fixed capacity array (bodyCapacity) that is rebuilt every time step.
	int32_t* awakeBodies;
	int32_t awakeCount;
	struct b2Mutex* awakeMutex;

	b2Vec2 gravity;
	float restitutionThreshold;

	// b2DestructionListener* m_destructionListener;

	// This is used to compute the time step ratio to support a variable time step.
	float inv_dt0;

	uint64_t islandId;
	uint16_t revision;
	int32_t groundBodyIndex;

	b2Profile profile;
	int32_t contactPointCount;

	b2PreSolveFcn* preSolveFcn;
	void* preSolveContext;

	b2PostSolveFcn* postSolveFcn;
	void* postSolveContext;

	int32_t workerCount;
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
