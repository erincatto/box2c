// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "box2d/types.h"

#include "solver_data.h"

typedef struct b2VelocityConstraintPoint
{
	b2Vec2 rA;
	b2Vec2 rB;
	float normalImpulse;
	float tangentImpulse;
	float normalMass;
	float tangentMass;
	float velocityBias;
	float relativeVelocity;
} b2VelocityConstraintPoint;

// TODO_ERIN why public?
typedef struct b2ContactVelocityConstraint
{
	b2VelocityConstraintPoint points[2];
	b2Vec2 normal;
	b2Mat22 normalMass;
	b2Mat22 K;
	int32_t indexA;
	int32_t indexB;
	float invMassA, invMassB;
	float invIA, invIB;
	float friction;
	float restitution;
	float tangentSpeed;
	int32_t pointCount;
	int32_t contactIndex;
} b2ContactVelocityConstraint;

typedef struct b2ContactSolverDef
{
	b2TimeStep step;
	struct b2Contact** contacts;
	int32_t count;
	b2Position* positions;
	b2Velocity* velocities;
	struct b2StackAllocator* allocator;
	struct b2World* world;
} b2ContactSolverDef;

typedef struct b2ContactSolver
{
	b2TimeStep step;
	b2Position* positions;
	b2Velocity* velocities;
	struct b2World* world;
	struct b2ContactPositionConstraint* positionConstraints;
	b2ContactVelocityConstraint* velocityConstraints;
	struct b2Contact** contacts;
	int32_t count;
} b2ContactSolver;

b2ContactSolver b2CreateContactSolver(b2ContactSolverDef* def);
void b2DestroyContactSolver(b2ContactSolver* solver);

void b2ContactSolver_InitializeVelocityConstraints(b2ContactSolver* solver);
void b2ContactSolver_WarmStart(b2ContactSolver* solver);
void b2ContactSolver_SolveVelocityConstraints(b2ContactSolver* solver);
void b2ContactSolver_ApplyRestitution(b2ContactSolver* solver);
void b2ContactSolver_StoreImpulses(b2ContactSolver* solver);
bool b2ContactSolver_SolvePositionConstraints(b2ContactSolver* solver);
