// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "box2d/types.h"

// Context for a time step. Recreated each time step.
typedef struct b2StepContext
{
	// time step
	float dt;

	// inverse time step (0 if dt == 0).
	float inv_dt;

	// TODO_ERIN eliminate support for variable time step
	// ratio between current and previous time step (dt * inv_dt0)
	float dtRatio;

	// Velocity iterations for constraint solver. Controls the accuracy of internal forces.
	int32_t velocityIterations;

	// Position iterations for constraint solver. Controls the accuracy of shape overlap and joint alignment.
	int32_t positionIterations;

	float restitutionThreshold;

	// TODO_ERIN for joints
	struct b2Body* bodies;
	int32_t bodyCapacity;

	// Map from world body pool index to solver body
	const int32_t* bodyToSolverMap;

	// Map from solver body to world body
	const int32_t* solverToBodyMap;

	struct b2SolverBody* solverBodies;
	int32_t solverBodyCount;

	bool enableWarmStarting;
} b2StepContext;

typedef enum b2SolverStageType
{
	b2_stageIntegrateVelocities = 0,
	b2_stagePrepareContacts,
	b2_stageWarmStartContacts,
	b2_stagePrepareJoints,
	b2_stageSolveJoints,
	b2_stageSolveContacts,
	b2_stageIntegratePositions,
	b2_stageCalmJoints,
	b2_stageCalmContacts,
	b2_stageStoreImpulses
} b2SolverStageType;

// Each block of work has a sync index that gets incremented when a worker claims the block. This ensures only a single worker claims a
// block, yet lets work be distributed dynamically across multiple workers (work stealing). This also reduces contention on a single block
// index atomic. For non-iterative stages the sync index is simply set to one. For iterative stages (solver iteration) the same block of
// work is executed once per iteration and the atomic sync index is shared across iterations, so it increases monotonically.
typedef struct b2SolverBlock
{
	int32_t startIndex;
	int32_t endIndex;
	_Atomic int syncIndex;
} b2SolverBlock;

// Each stage must be completed before going to the next stage.
// Non-iterative stages use a stage instance once while iterative stages re-use the same instance each iteration.
typedef struct b2SolverStage
{
	b2SolverStageType type;
	b2SolverBlock* blocks;
	int32_t blockCount;
	int32_t colorIndex;
	_Atomic int completionCount;
} b2SolverStage;

typedef struct b2SolverTaskContext
{
	struct b2World* world;
	struct b2Graph* graph;
	struct b2Body** awakeBodies;
	struct b2SolverBody* solverBodies;
	int32_t* bodyToSolverMap;
	int32_t* solverToBodyMap;
	int32_t* contactIndices;

	b2StepContext* stepContext;
	struct b2ContactConstraint* constraints;
	struct b2ContactConstraintAVX* constraintAVXs;
	int32_t activeColorCount;
	int32_t velocityIterations;
	int32_t calmIterations;
	int32_t workerCount;

	float timeStep;
	float invTimeStep;
	float subStep;
	float invSubStep;

	b2SolverStage* stages;
	int32_t stageCount;

	// sync index (16-bits) | stage type (16-bits)
	_Atomic unsigned int syncBits;
} b2SolverTaskContext;
