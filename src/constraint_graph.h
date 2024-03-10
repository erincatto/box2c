// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "bitset.h"
#include "block_array.h"

#include "box2d/constants.h"

typedef struct b2Contact b2Contact;
typedef struct b2ContactConstraint b2ContactConstraint;
typedef struct b2ContactConstraintSIMD b2ContactConstraintSIMD;
typedef struct b2Joint b2Joint;
typedef struct b2StepContext b2StepContext;
typedef struct b2World b2World;

#define b2_graphColorCount 12

// This holds constraints that cannot fit the graph color limit. This happens when a single dynamic body
// is touching many other bodies.
#define b2_overflowIndex b2_graphColorCount - 1

typedef struct b2GraphColor
{
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

	// debug info
	int occupancy[b2_graphColorCount];
} b2ConstraintGraph;

void b2CreateGraph(b2ConstraintGraph* graph, b2BlockAllocator* allocator, int bodyCapacity);
void b2DestroyGraph(b2ConstraintGraph* graph);

//void b2AddContactToGraph(b2World* world, b2Contact* contact);
b2Contact* b2EmplaceContactInGraph(b2World* world, b2Body* bodyA, b2Body* bodyB);
void b2RemoveContactFromGraph(b2World* world, b2Contact* contact);

//void b2AddJointToGraph(b2World* world, b2Joint* joint);
b2Joint* b2EmplaceJointInGraph(b2World* world, b2Body* bodyA, b2Body* bodyB);
void b2RemoveJointFromGraph(b2World* world, b2Joint* joint);
