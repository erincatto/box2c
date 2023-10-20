// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "array.h"
#include "bitset.h"
#include "box2d/constants.h"

typedef struct b2Contact b2Contact;
typedef struct b2ContactConstraint b2ContactConstraint;
typedef struct b2ContactConstraintAVX b2ContactConstraintAVX;
typedef struct b2Joint b2Joint;
typedef struct b2StepContext b2StepContext;
typedef struct b2World b2World;

#define b2_overflowIndex b2_graphColorCount

typedef struct b2GraphColor
{
	b2BitSet bodySet;
	int32_t* contactArray;
	int32_t* jointArray;

	// transient
	b2ContactConstraint* contactConstraints;

	// TODO_ERIN these could be split up by worker so that workers get a contiguous array of constraints across colors
	b2ContactConstraintAVX* contactConstraintAVXs;
} b2GraphColor;

// This holds constraints that cannot fit the graph color limit. This happens when a single dynamic body
// is touching many other bodies.
typedef struct
{
	int32_t* contactArray;
	int32_t* jointArray;
	b2ContactConstraint* contactConstraints;
} b2GraphOverflow;

typedef struct b2Graph
{
	b2GraphColor colors[b2_graphColorCount];
	int32_t colorCount;

	// debug info
	int32_t occupancy[b2_graphColorCount + 1];

	b2GraphOverflow overflow;
} b2Graph;

void b2CreateGraph(b2Graph* graph, int32_t bodyCapacity, int32_t contactCapacity, int32_t jointCapacity);
void b2DestroyGraph(b2Graph* graph);

void b2AddContactToGraph(b2World* world, b2Contact* contact);
void b2RemoveContactFromGraph(b2World* world, b2Contact* contact);

void b2AddJointToGraph(b2World* world, b2Joint* joint);
void b2RemoveJointFromGraph(b2World* world, b2Joint* joint);

void b2SolveGraph(b2World* world, b2StepContext* stepContext);
