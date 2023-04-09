// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "broad_phase.h"
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

	b2Pool bodyPool;
	b2Pool jointPool;
	b2Pool shapePool;

	struct b2Body* bodies;
	struct b2Joint* joints;
	struct b2Shape* shapes;

	struct b2Contact* contacts;
	int32_t contactCount;

	// array of contacts with shapes that no longer have overlapping bounding boxes
	struct b2Contact** invalidContacts;
	b2Mutex* invalidContactMutex;

	// double buffered awake body array (not safe to copy pointers)
	int32_t* awakeBodies;
	int32_t* seedBodies;

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
