// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "array.h"
#include "bitset.h"
#include "table.h"

#include "box2d/dynamic_tree.h"

typedef struct b2Contact b2Contact;
typedef struct b2Joint b2Joint;
typedef struct b2StepContext b2StepContext;
typedef struct b2World b2World;
typedef struct b2SolverTaskEntry b2SolverTaskEntry;

// TODO_ERIN fixme
#define b2_graphColorCount 64

typedef struct b2GraphColor
{
	b2BitSet bodySet;
	int32_t* contactArray;
	int32_t* jointArray;

	// transient
	struct b2Constraint* contacts;
} b2GraphColor;

typedef struct b2Graph
{
	b2GraphColor colors[b2_graphColorCount];
	int32_t colorCount;

	b2SolverTaskEntry* solverTaskEntries;
} b2Graph;

void b2CreateGraph(b2Graph* graph, int32_t bodyCapacity, int32_t contactCapacity);
void b2DestroyGraph(b2Graph* graph);

void b2AddContactToGraph(b2World* world, b2Contact* contact);
void b2RemoveContactFromGraph(b2World* world, b2Contact* contact);

void b2AddJointToGraph(b2World* world, b2Joint* contact);
void b2RemoveJointFromGraph(b2World* world, b2Joint* contact);

void b2SolveGraphPGS(b2World* world, const b2StepContext* stepContext);
void b2SolveGraphSoftPGS(b2World* world, const b2StepContext* stepContext);
void b2SolveGraphSoftStep(b2World* world, const b2StepContext* stepContext);
void b2SolveGraphStickyTGS(b2World* world, const b2StepContext* stepContext);
