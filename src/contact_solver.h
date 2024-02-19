// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "solver_data.h"

// todo this could be hidden in contact_solver.c, then graph.c just needs to know the sizeof(b2ContactConstraintSIMD)
#include "x86/avx.h"

#include "box2d/math.h"

typedef struct b2Contact b2Contact;

typedef struct b2ContactConstraintPoint
{
	b2Vec2 anchorA, anchorB;
	float baseSeparation;
	float relativeVelocity;
	float normalImpulse;
	float tangentImpulse;
	float maxNormalImpulse;
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
	float invMassA, invMassB;
	float invIA, invIB;
	float friction;
	float restitution;
	b2Softness softness;
	int32_t pointCount;
} b2ContactConstraint;

// Wide float
typedef simde__m256 b2FloatW;

// Wide vec2
typedef struct b2Vec2W
{
	b2FloatW X, Y;
} b2Vec2W;

// Wide rotation
typedef struct b2RotW
{
	b2FloatW S, C;
} b2RotW;

typedef struct b2ContactConstraintSIMD
{
	int32_t indexA[8];
	int32_t indexB[8];

	b2FloatW invMassA, invMassB;
	b2FloatW invIA, invIB;
	b2Vec2W normal;
	b2FloatW friction;
	b2FloatW restitution;
	b2FloatW biasRate;
	b2FloatW massScale;
	b2FloatW impulseScale;
	b2Vec2W anchorA1, anchorB1;
	b2Vec2W anchorA2, anchorB2;
	b2FloatW baseSeparation1, baseSeparation2;
	b2FloatW normalImpulse1, normalImpulse2;
	b2FloatW tangentImpulse1, tangentImpulse2;
	b2FloatW maxNormalImpulse1, maxNormalImpulse2;
	b2FloatW normalMass1, tangentMass1;
	b2FloatW normalMass2, tangentMass2;
	b2FloatW relativeVelocity1, relativeVelocity2;
} b2ContactConstraintSIMD;

// Overflow contacts don't fit into the constraint graph coloring
void b2PrepareOverflowContacts(b2StepContext* context);
void b2WarmStartOverflowContacts(b2StepContext* context);
void b2SolveOverflowContacts(b2StepContext* context, bool useBias);
void b2ApplyOverflowRestitution(b2StepContext* context);
void b2StoreOverflowImpulses(b2StepContext* context);

// Contacts that live within the constraint graph coloring
void b2PrepareContactsTask(int32_t startIndex, int32_t endIndex, b2StepContext* context);
void b2WarmStartContactsTask(int32_t startIndex, int32_t endIndex, b2StepContext* context, int32_t colorIndex);
void b2SolveContactsTask(int32_t startIndex, int32_t endIndex, b2StepContext* context, int32_t colorIndex, bool useBias);
void b2ApplyRestitutionTask(int32_t startIndex, int32_t endIndex, b2StepContext* context, int32_t colorIndex);
void b2StoreImpulsesTask(int32_t startIndex, int32_t endIndex, b2StepContext* context);
