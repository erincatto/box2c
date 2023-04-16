// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "box2d/callbacks.h"

#include "solver_data.h"

typedef struct b2StackAllocator b2StackAllocator;

typedef struct b2ContactSolverDef2
{
	const b2TimeStep* step;
	struct b2World* world;
	int32_t* contactIndices;
	int32_t count;
} b2ContactSolverDef2;

typedef struct b2ContactSolver2
{
	const b2TimeStep* step;
	struct b2World* world;
	struct b2ContactPositionConstraint2* positionConstraints;
	struct b2ContactVelocityConstraint2* velocityConstraints;
	int32_t* contactIndices;
	int32_t count;
} b2ContactSolver2;

b2ContactSolver2* b2CreateContactSolver2(b2StackAllocator* alloc, b2ContactSolverDef2* def);
void b2DestroyContactSolver2(b2StackAllocator* alloc, b2ContactSolver2* solver);

void b2ContactSolver_Initialize2(b2ContactSolver2* solver);
void b2ContactSolver_SolveVelocityConstraints2(b2ContactSolver2* solver);
void b2ContactSolver_ApplyRestitution2(b2ContactSolver2* solver);
void b2ContactSolver_StoreImpulses2(b2ContactSolver2* solver);
bool b2ContactSolver_SolvePositionConstraintsBlock2(b2ContactSolver2* solver);
