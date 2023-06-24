// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "broad_phase.h"
#include "island_builder.h"
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
	b2Contact** contactArray;

	// Fixed capacity array allocated at world creation and rebuilt every time step.
	// These are solid contacts that are touching and awake. b2IslandBuilder builds islands
	// that index into this array.
	b2Contact** activeContacts;

	// Fixed capacity array of contacts with shapes that no longer have overlapping bounding boxes
	b2Contact** invalidContacts;

	// This atomic allows active contacts to be added to the active contact array and to b2IslandBuilder
	// from multiple threads. Padding to avoid sharing cache lines with other atomics.
	B2_ATOMIC long activeContactCount;
	char padding1[64 - sizeof(long)];

	// This atomic allows invalid contacts to be added to the invalid contact array from multiple threads
	B2_ATOMIC long invalidContactCount;
	char padding2[64 - sizeof(long)];

	// Atomic count of the number of awake bodies. Recomputed each step.
	B2_ATOMIC long awakeCount;
	char padding3[64 - sizeof(long)];
	
	// Atomic count of the number of awake bodies sent to collision tasks.
	B2_ATOMIC long baseAwakeCount;
	char padding4[64 - sizeof(long)];

	// Awake body array holds indices into bodies array (bodyPool).
	// This is a dense fixed capacity array (bodyCapacity) that is rebuilt every time step.
	int32_t* awakeBodies;
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
	B2_ATOMIC long contactPointCount;

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
