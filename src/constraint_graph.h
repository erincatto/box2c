// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "bitset.h"
#include "block_array.h"

#include "box2d/constants.h"

typedef struct b2Contact b2Contact;
typedef struct b2ContactLookup b2ContactLookup;
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
	// base on bodyId so this is over-sized to encompass static bodies
	// however I never traverse these bits or use the bit count for anything
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
} b2ConstraintGraph;

void b2CreateGraph(b2ConstraintGraph* graph, b2BlockAllocator* allocator, int bodyCapacity);
void b2DestroyGraph(b2ConstraintGraph* graph);

b2Contact* b2AddContactToGraph(b2World* world, b2Contact* contact, b2ContactLookup* contactLookup);
void b2RemoveContactFromGraph(b2World* world, b2Contact* contact);

b2Joint* b2CreateJointInGraph(b2World* world, int bodyColorIdA, int bodyColorIdB, b2JointLookup* jointLookup);
void b2AddJointToGraph(b2World* world, b2Joint* joint, b2JointLookup* jointLookup);
void b2RemoveJointFromGraph(b2World* world, b2Joint* joint);
