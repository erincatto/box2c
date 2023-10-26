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
	float relativeVelocity;
	float normalImpulse;
	float tangentImpulse;
	float normalMass;
	float tangentMass;
} b2ContactConstraintPoint;

typedef struct b2ContactConstraint
{
	b2Contact* contact;
	int32_t indexA;
	int32_t indexB;
	b2ContactConstraintPoint points[2];
	b2Vec2 normal;
	float friction;
	float restitution;
	float massCoefficient;
	float biasCoefficient;
	float impulseCoefficient;
	int32_t pointCount;
} b2ContactConstraint;

// Wide float
typedef __m256 b2FloatW;

// Wide vec2
typedef struct b2Vec2W
{
	b2FloatW X, Y;
} b2Vec2W;

typedef struct b2ContactConstraintSIMD
{
	int32_t indexA[8];
	int32_t indexB[8];

	b2Vec2W normal;
	__m256 friction;
	__m256 restitution;
	b2Vec2W rA1, rB1;
	b2Vec2W rA2, rB2;
	__m256 separation1, separation2;
	__m256 relativeVelocity1, relativeVelocity2;
	__m256 normalImpulse1, normalImpulse2;
	__m256 tangentImpulse1, tangentImpulse2;
	__m256 normalMass1, tangentMass1;
	__m256 normalMass2, tangentMass2;
	__m256 massCoefficient;
	__m256 biasCoefficient;
	__m256 impulseCoefficient;
} b2ContactConstraintAVX;

// Scalar
void b2PrepareOverflowContacts(b2SolverTaskContext* context);
void b2SolveOverflowContacts(b2SolverTaskContext* context, bool useBias);
void b2ApplyOverflowRestitution(b2SolverTaskContext* context);
void b2StoreOverflowImpulses(b2SolverTaskContext* context);

// AVX versions
void b2PrepareContactsSIMD(int32_t startIndex, int32_t endIndex, b2SolverTaskContext* context);
void b2WarmStartContactsSIMD(int32_t startIndex, int32_t endIndex, b2SolverTaskContext* context, int32_t colorIndex);
void b2SolveContactsSIMD(int32_t startIndex, int32_t endIndex, b2SolverTaskContext* context, int32_t colorIndex, bool useBias);
void b2ApplyRestitutionSIMD(int32_t startIndex, int32_t endIndex, b2SolverTaskContext* context, int32_t colorIndex);
void b2StoreImpulsesSIMD(int32_t startIndex, int32_t endIndex, b2SolverTaskContext* context);
