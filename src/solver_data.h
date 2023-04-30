// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "box2d/types.h"

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

// Context for a time step. Recreated each time step.
typedef struct b2StepContext
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

	// Stack allocated array of active joints
	struct b2Joint** activeJoints;

	// Atomic count of the number of active joints. Recomputed each step.
	B2_ATOMIC long activeJointCount;
	char padding4[64 - sizeof(long)];

	const b2BodyData* bodyData;
	b2Position* positions;
	b2Velocity* velocities;

	// For testing
	bool warmStarting;
} b2StepContext;
