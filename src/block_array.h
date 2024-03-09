// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

typedef struct b2BlockAllocator b2BlockAllocator;
typedef struct b2Body b2Body;
typedef struct b2Contact b2Contact;
typedef struct b2Joint b2Joint;

typedef struct b2BodyArray
{
	b2Body* bodies;
	int count;
	int capacity;
} b2BodyArray;

typedef struct b2ContactArray
{
	b2Contact* contacts;
	int count;
	int capacity;
} b2ContactArray;

typedef struct b2JointArray
{
	b2Joint* joints;
	int count;
	int capacity;
} b2JointArray;

b2BodyArray b2CreateBodyArray(b2BlockAllocator* allocator, int capacity);
b2ContactArray b2CreateContactArray(b2BlockAllocator* allocator, int capacity);
b2JointArray b2CreateJointArray(b2BlockAllocator* allocator, int capacity);

void b2DestroyBodyArray(b2BlockAllocator* allocator, b2BodyArray* array);
void b2DestroyContactArray(b2BlockAllocator* allocator, b2ContactArray* array);
void b2DestroyJointArray(b2BlockAllocator* allocator, b2JointArray* array);

b2Body* b2AllocateBody(b2BlockAllocator* allocator, b2BodyArray* array);
b2Contact* b2AllocateContact(b2BlockAllocator* allocator, b2ContactArray* array);
b2Joint* b2AllocateJoint(b2BlockAllocator* allocator, b2JointArray* array);

// Returns the index of the element moved into the empty slot (or B2_NULL_INDEX)
int b2RemoveBody(b2BlockAllocator* allocator, b2BodyArray* array, int index);
int b2RemoveContact(b2BlockAllocator* allocator, b2ContactArray* array, int index);
int b2RemoveJoint(b2BlockAllocator* allocator, b2JointArray* array, int index);
