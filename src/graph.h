// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "array.h"
#include "bitset.h"
#include "table.h"

#include "box2d/dynamic_tree.h"

typedef struct b2Contact b2Contact;
typedef struct b2StepContext b2StepContext;
typedef struct b2World b2World;

#define b2_graphColorCount 16

typedef struct b2GraphColor
{
	b2BitSet bodySet;
	int32_t* contactArray;
	struct b2Constraint* constraints;
} b2GraphColor;

typedef struct b2Graph
{
	b2GraphColor colors[b2_graphColorCount];

} b2Graph;

void b2CreateGraph(b2Graph* graph, int32_t bodyCapacity, int32_t contactCapacity);
void b2DestroyGraph(b2Graph* graph);

void b2AddContactToGraph(b2World* world, b2Contact* contact);
void b2RemoveContactFromGraph(b2World* world, b2Contact* contact);

void b2SolveGraphPGS(b2World* world, const b2StepContext* stepContext);
void b2SolveGraphTGS(b2World* world, const b2StepContext* stepContext);
