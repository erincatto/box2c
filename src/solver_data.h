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

	float restitutionThreshold;

	// For testing
	bool warmStarting;
} b2TimeStep;

typedef struct b2BodyData
{
	b2Vec2 localCenter;
	float invMass;
	float invI;
} b2BodyData;

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

typedef struct b2SolverContext
{
	const b2TimeStep* step;
	const b2BodyData* bodyData;
	b2Position* positions;
	b2Velocity* velocities;
} b2SolverContext;
