// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "solver_data.h"
#include "stack_allocator.h"

#include "box2d/callbacks.h"

typedef struct b2ContactSolverDef
{
	const b2StepContext* context;
	struct b2World* world;
	int32_t contactList;
	int32_t contactCount;
} b2ContactSolverDef;

typedef struct b2ContactSolver
{
	const b2StepContext* context;
	struct b2World* world;
	struct b2ContactPositionConstraint* positionConstraints;
	struct b2ContactVelocityConstraint* velocityConstraints;
	int32_t contactList;
	int32_t contactCount;
	int32_t constraintCount;
} b2ContactSolver;

b2ContactSolver* b2CreateContactSolver(b2ContactSolverDef* def);

static inline void b2DestroyContactSolver(b2ContactSolver* solver, b2StackAllocator* alloc)
{
	b2FreeStackItem(alloc, solver->velocityConstraints);
	b2FreeStackItem(alloc, solver->positionConstraints);
	b2FreeStackItem(alloc, solver);
}

void b2ContactSolver_Initialize(b2ContactSolver* solver);
void b2ContactSolver_SolveVelocityConstraints(b2ContactSolver* solver);
void b2ContactSolver_ApplyRestitution(b2ContactSolver* solver);
void b2ContactSolver_StoreImpulses(b2ContactSolver* solver);
bool b2ContactSolver_SolvePositionConstraintsBlock(b2ContactSolver* solver);
