// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "box2d/types.h"

typedef struct b2TimeStep
{
	// time step
	float dt;

	// inverse time step (0 if dt == 0).
	float inv_dt;

	// ratio between current and previous time step (dt * inv_dt0)
	float dtRatio;

	// Velocity iterations for constraint solver. Controls the accuracy of internal forces.
	int32_t velocityIterations;

	// Position iterations for constraint solver. Controls the accuracy of shape overlap and joint alignment.
	int32_t positionIterations;

	// For testing
	bool warmStarting;
} b2TimeStep;

typedef struct b2Position
{
	b2Vec2 c;
	float a;
} b2Position;

typedef struct b2Velocity
{
	b2Vec2 v;
	float w;
} b2Velocity;

//	TODO_ERIN b2SolverContext?
typedef struct b2SolverData
{
	b2TimeStep step;
	b2Position* positions;
	b2Velocity* velocities;
} b2SolverData;
