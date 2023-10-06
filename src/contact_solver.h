// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "solver_data.h"

#include <immintrin.h>

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

typedef __m256 b2Float8;

typedef struct b2Vec2W
{
	b2Float8 X, Y;
} b2Vec2W;

typedef struct b2ContactConstraintAVX
{
	int32_t indexA[8];
	int32_t indexB[8];

	b2Vec2W normal;
	__m256 friction;
	b2Vec2W rA1, rB1;
	b2Vec2W rA2, rB2;
	__m256 separation1, separation2;
	__m256 normalImpulse1, normalImpulse2;
	__m256 tangentImpulse1, tangentImpulse2;
	__m256 normalMass1, tangentMass1;
	__m256 normalMass2, tangentMass2;
	__m256 massCoefficient;
	__m256 biasCoefficient;
	__m256 impulseCoefficient;
} b2ContactConstraintAVX;

void b2PrepareContactsTaskAVX(int32_t startIndex, int32_t endIndex, b2SolverTaskContext* context);
void b2WarmStartContactConstraints(int32_t startIndex, int32_t endIndex, b2SolverTaskContext* context, int32_t colorIndex);
void b2SolveContactAVXsTask(int32_t startIndex, int32_t endIndex, b2SolverTaskContext* context, int32_t colorIndex, bool useBias);
void b2StoreImpulsesTaskAVX(int32_t startIndex, int32_t endIndex, b2SolverTaskContext* context);

void b2PrepareContactsTask(int32_t startIndex, int32_t endIndex, b2SolverTaskContext* context, int32_t colorIndex);
void b2SolveContactsTask(int32_t startIndex, int32_t endIndex, b2SolverTaskContext* context, int32_t colorIndex, bool useBias);
void b2StoreImpulsesTask(int32_t startIndex, int32_t endIndex, b2SolverTaskContext* context);
