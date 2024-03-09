// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "block_array.h"
#include "bitset.h"
#include "box2d/constants.h"

typedef struct b2Contact b2Contact;
typedef struct b2ContactConstraint b2ContactConstraint;
typedef struct b2ContactConstraintSIMD b2ContactConstraintSIMD;
typedef struct b2Joint b2Joint;
typedef struct b2StepContext b2StepContext;
typedef struct b2World b2World;

#define b2_overflowIndex b2_graphColorCount

typedef struct b2GraphColor
{
	b2BitSet bodySet;
	b2ContactArray contacts;
	b2JointArray joints;

	// transient
	b2ContactConstraintSIMD* contactConstraints;
} b2GraphColor;

// This holds constraints that cannot fit the graph color limit. This happens when a single dynamic body
// is touching many other bodies.
typedef struct b2GraphOverflow
{
	b2ContactArray contacts;
	b2JointArray joints;

	// transient
	b2ContactConstraint* contactConstraints;
} b2GraphOverflow;

typedef struct b2ContactBlock
{
	b2Contact* contacts;
	int count;
} b2ContactBlock;

typedef struct b2ConstraintGraph
{
	b2GraphColor colors[b2_graphColorCount];
	int colorCount;

	// debug info
	int occupancy[b2_graphColorCount + 1];

	b2GraphOverflow overflow;

	// transient blocks for parallel-for, pointers copied from graph colors
	b2ContactBlock contactBlocks[b2_graphColorCount + 1];
	int contactBlockCount;
} b2ConstraintGraph;

void b2CreateGraph(b2ConstraintGraph* graph, int32_t bodyCapacity, int32_t contactCapacity, int32_t jointCapacity);
void b2DestroyGraph(b2ConstraintGraph* graph);

void b2AddContactToGraph(b2World* world, b2Contact* contact);
void b2RemoveContactFromGraph(b2World* world, b2Contact* contact);

void b2AddJointToGraph(b2World* world, b2Joint* joint);
void b2RemoveJointFromGraph(b2World* world, b2Joint* joint);
