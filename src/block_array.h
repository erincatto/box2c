// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

typedef struct b2BlockAllocator b2BlockAllocator;
typedef struct b2BodySim b2BodySim;
typedef struct b2BodyState b2BodyState;
typedef struct b2Contact b2Contact;
typedef struct b2IslandSim b2IslandSim;
typedef struct b2Joint b2Joint;

typedef struct b2BodySimArray
{
	b2BodySim* data;
	int count;
	int capacity;
} b2BodySimArray;

typedef struct b2BodyStateArray
{
	b2BodyState* data;
	int count;
	int capacity;
} b2BodyStateArray;

typedef struct b2ContactArray
{
	b2Contact* data;
	int count;
	int capacity;
} b2ContactArray;

typedef struct b2IslandArray
{
	b2IslandSim* data;
	int count;
	int capacity;
} b2IslandArray;

typedef struct b2JointArray
{
	b2Joint* data;
	int count;
	int capacity;
} b2JointArray;

// These provide a way to create an array with a specified capacity. If the capacity is not
// known, you may use zero initialization.
b2BodySimArray b2CreateBodySimArray(b2BlockAllocator* allocator, int capacity);
b2BodyStateArray b2CreateBodyStateArray(b2BlockAllocator* allocator, int capacity);
b2ContactArray b2CreateContactArray(b2BlockAllocator* allocator, int capacity);
b2IslandArray b2CreateIslandArray(b2BlockAllocator* allocator, int capacity);
b2JointArray b2CreateJointArray(b2BlockAllocator* allocator, int capacity);

void b2DestroyBodySimArray(b2BlockAllocator* allocator, b2BodySimArray* array);
void b2DestroyBodyStateArray(b2BlockAllocator* allocator, b2BodyStateArray* array);
void b2DestroyContactArray(b2BlockAllocator* allocator, b2ContactArray* array);
void b2DestroyIslandArray(b2BlockAllocator* allocator, b2IslandArray* array);
void b2DestroyJointArray(b2BlockAllocator* allocator, b2JointArray* array);

b2BodySim* b2AddBodySim(b2BlockAllocator* allocator, b2BodySimArray* array);
b2BodyState* b2AddBodyState(b2BlockAllocator* allocator, b2BodyStateArray* array);
b2Contact* b2AddContact(b2BlockAllocator* allocator, b2ContactArray* array);
b2IslandSim* b2AddIsland(b2BlockAllocator* allocator, b2IslandArray* array);
b2Joint* b2AddJoint(b2BlockAllocator* allocator, b2JointArray* array);

// Returns the index of the element moved into the empty slot (or B2_NULL_INDEX)
// todo have these return the id directly?
int b2RemoveBodySim(b2BodySimArray* array, int index);
int b2RemoveBodyState(b2BodyStateArray* array, int index);
int b2RemoveContact(b2ContactArray* array, int index);
int b2RemoveIsland(b2IslandArray* array, int index);
int b2RemoveJoint(b2JointArray* array, int index);
