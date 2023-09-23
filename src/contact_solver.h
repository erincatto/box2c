// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "solver_data.h"

typedef struct b2Contact b2Contact;

typedef struct b2ContactConstraintPoint
{
	b2Vec2 rA, rB;
	float separation;
	float normalImpulse;
	float tangentImpulse;
	float normalMass;
	float tangentMass;
} b2ContactConstraintPoint;

typedef enum b2ContactConstraintType
{
	b2_onePointType,
	b2_twoPointType,
	b2_onePointStaticType,
	b2_twoPointStaticType,
} b2ContactConstraintType;

typedef struct b2ContactConstraint
{
	b2Contact* contact;
	int32_t indexA;
	int32_t indexB;
	b2ContactConstraintPoint points[2];
	b2Vec2 normal;
	float friction;
	float massCoefficient;
	float biasCoefficient;
	float impulseCoefficient;
	b2ContactConstraintType type;
} b2ContactConstraint;

void b2PrepareContactsTask(int32_t startIndex, int32_t endIndex, b2SolverTaskContext* context, int32_t colorIndex);
void b2SolveContactsTask(int32_t startIndex, int32_t endIndex, b2SolverTaskContext* context, int32_t colorIndex, bool useBias);
void b2StoreImpulsesTask(int32_t startIndex, int32_t endIndex, b2SolverTaskContext* context);
