// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "box2d/callbacks.h"

#include "solver_data.h"

typedef struct b2StackAllocator b2StackAllocator;

typedef struct b2ContactSolverDef
{
	const b2StepContext* step;
	struct b2World* world;
	int32_t* contactIndices;
	int32_t count;
} b2ContactSolverDef;

typedef struct b2ContactSolver
{
	const b2StepContext* step;
	struct b2World* world;
	struct b2ContactPositionConstraint* positionConstraints;
	struct b2ContactVelocityConstraint* velocityConstraints;
	int32_t* contactIndices;
	int32_t count;
} b2ContactSolver;

b2ContactSolver* b2CreateContactSolver(b2StackAllocator* alloc, b2ContactSolverDef* def);
void b2DestroyContactSolver(b2StackAllocator* alloc, b2ContactSolver* solver);

void b2ContactSolver_Initialize(b2ContactSolver* solver);
void b2ContactSolver_SolveVelocityConstraints(b2ContactSolver* solver);
void b2ContactSolver_ApplyRestitution(b2ContactSolver* solver);
void b2ContactSolver_StoreImpulses(b2ContactSolver* solver);
bool b2ContactSolver_SolvePositionConstraintsBlock(b2ContactSolver* solver);
