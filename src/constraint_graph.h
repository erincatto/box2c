// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "bitset.h"
#include "block_array.h"
#include "id_pool.h"

#include "box2d/constants.h"

typedef struct b2Contact b2Contact;
typedef struct b2ContactConstraint b2ContactConstraint;
typedef struct b2ContactConstraintSIMD b2ContactConstraintSIMD;
typedef struct b2Joint b2Joint;
typedef struct b2JointLookup b2JointLookup;
typedef struct b2StepContext b2StepContext;
typedef struct b2World b2World;

// This holds constraints that cannot fit the graph color limit. This happens when a single dynamic body
// is touching many other sims.
#define b2_overflowIndex b2_graphColorCount - 1

typedef struct b2GraphColor
{
	// todo this is allowing static sims too
	b2BitSet bodySet;

	// cache friendly arrays
	b2ContactArray contacts;
	b2JointArray joints;

	// transient
	union
	{
		b2ContactConstraintSIMD* simdConstraints;
		b2ContactConstraint* overflowConstraints;
	};
} b2GraphColor;

typedef struct b2ConstraintGraph
{
	// including overflow at the end
	b2GraphColor colors[b2_graphColorCount];

	// used to assign stable graph color ids to dynamic sims
	b2IdPool colorIdPool;

} b2ConstraintGraph;

void b2CreateGraph(b2ConstraintGraph* graph, b2BlockAllocator* allocator, int bodyCapacity);
void b2DestroyGraph(b2ConstraintGraph* graph);

b2Contact* b2AddContactToGraph(b2World* world, b2Contact* contact);
void b2RemoveContactFromGraph(b2World* world, b2Contact* contact);

b2Joint* b2AddJointToGraph(b2World* world, int bodyColorIdA, int bodyColorIdB, b2JointLookup* lookup);
void b2RemoveJointFromGraph(b2World* world, b2Joint* joint);
