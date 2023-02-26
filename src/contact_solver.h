// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "box2d/callbacks.h"
#include "box2d/types.h"

#include "solver_data.h"

typedef struct b2ContactSolverDef
{
	b2TimeStep step;
	struct b2Contact** contacts;
	int32_t count;
	b2Position* positions;
	b2Velocity* velocities;
	struct b2World* world;
} b2ContactSolverDef;

typedef struct b2ContactSolver
{
	b2TimeStep step;
	b2Position* positions;
	b2Velocity* velocities;
	struct b2World* world;
	struct b2ContactPositionConstraint* positionConstraints;
	struct b2ContactVelocityConstraint* velocityConstraints;
	struct b2Contact** contacts;
	int32_t count;
} b2ContactSolver;

b2ContactSolver b2CreateContactSolver(b2ContactSolverDef* def);
void b2DestroyContactSolver(b2ContactSolver* solver);

void b2ContactSolver_WarmStart(b2ContactSolver* solver);
void b2ContactSolver_SolveVelocityConstraints(b2ContactSolver* solver);
void b2ContactSolver_ApplyRestitution(b2ContactSolver* solver);
void b2ContactSolver_StoreImpulses(b2ContactSolver* solver);
bool b2ContactSolver_SolvePositionConstraints(b2ContactSolver* solver);
b2ContactImpulse b2ContactSolver_GetImpulse(b2ContactSolver* solver, int32_t index);
