// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "block_allocator.h"
#include "block_array.h"

#include "body.h"
#include "contact.h"
#include "joint.h"

#include <string.h>

b2BodyArray b2CreateBodyArray(b2BlockAllocator* allocator, int capacity)
{
	// ensure memcpy works later
	B2_ASSERT(capacity > 0);
	return (b2BodyArray){b2AllocBlock(allocator, capacity * sizeof(b2Body)), 0, capacity};
}

b2ContactArray b2CreateContactArray(b2BlockAllocator* allocator, int capacity)
{
	B2_ASSERT(capacity > 0);
	return (b2ContactArray){b2AllocBlock(allocator, capacity * sizeof(b2Contact)), 0, capacity};
}

b2JointArray b2CreateJointArray(b2BlockAllocator* allocator, int capacity)
{
	B2_ASSERT(capacity > 0);
	return (b2JointArray){b2AllocBlock(allocator, capacity * sizeof(b2Joint)), 0, capacity};
}

void b2DestroyBodyArray(b2BlockAllocator* allocator, b2BodyArray* array)
{
	b2FreeBlock(allocator, array->bodies, array->capacity * sizeof(b2Body));
}

void b2DestroyContactArray(b2BlockAllocator* allocator, b2ContactArray* array)
{
	b2FreeBlock(allocator, array->contacts, array->capacity * sizeof(b2Contact));
}

void b2DestroyJointArray(b2BlockAllocator* allocator, b2JointArray* array)
{
	b2FreeBlock(allocator, array->joints, array->capacity * sizeof(b2Joint));
}

b2Body* b2AllocateBody(b2BlockAllocator* allocator, b2BodyArray* array)
{
	if (array->count == array->capacity)
	{
		B2_ASSERT(array->capacity > 0);
		int elementSize = sizeof(b2Body);
		int newCapacity = 2 * array->capacity;
		b2Body* newElements = b2AllocBlock(allocator, newCapacity * elementSize);
		memcpy(newElements, array->bodies, array->capacity * elementSize);
		b2FreeBlock(allocator, array->bodies, array->capacity * elementSize);
		array->bodies = newElements;
		array->capacity = newCapacity;
	}

	b2Body* element = array->bodies + array->count;
	array->count += 1;
	return element;
}

b2Contact* b2AllocateContact(b2BlockAllocator* allocator, b2ContactArray* array)
{
	if (array->count == array->capacity)
	{
		B2_ASSERT(array->capacity > 0);
		int elementSize = sizeof(b2Contact);
		int newCapacity = 2 * array->capacity;
		b2Contact* newElements = b2AllocBlock(allocator, newCapacity * elementSize);
		memcpy(newElements, array->contacts, array->capacity * elementSize);
		b2FreeBlock(allocator, array->contacts, array->capacity * elementSize);
		array->contacts = newElements;
		array->capacity = newCapacity;
	}

	b2Contact* element = array->contacts + array->count;
	array->count += 1;
	return element;
}

b2Joint* b2AllocateJoint(b2BlockAllocator* allocator, b2JointArray* array)
{
	if (array->count == array->capacity)
	{
		B2_ASSERT(array->capacity > 0);
		int elementSize = sizeof(b2Joint);
		int newCapacity = 2 * array->capacity;
		b2Joint* newElements = b2AllocBlock(allocator, newCapacity * elementSize);
		memcpy(newElements, array->joints, array->capacity * elementSize);
		b2FreeBlock(allocator, array->joints, array->capacity * elementSize);
		array->joints = newElements;
		array->capacity = newCapacity;
	}

	b2Joint* element = array->joints + array->count;
	array->count += 1;
	return element;
}

// Returns the index of the element moved into the empty slot (or B2_NULL_INDEX)
int b2RemoveBody(b2BlockAllocator* allocator, b2BodyArray* array, int index)
{
	B2_ASSERT(0 <= index && index < array->count);
	if (index < array->count - 1)
	{
		int removed = array->count - 1;
		array->bodies[index] = array->bodies[removed];
		array->count -= 1;
		return removed;
	}

	array->count -= 1;
	return B2_NULL_INDEX;
}

int b2RemoveContact(b2BlockAllocator* allocator, b2ContactArray* array, int index)
{
	B2_ASSERT(0 <= index && index < array->count);
	if (index < array->count - 1)
	{
		int removed = array->count - 1;
		array->contacts[index] = array->contacts[removed];
		array->count -= 1;
		return removed;
	}

	array->count -= 1;
	return B2_NULL_INDEX;
}

int b2RemoveJoint(b2BlockAllocator* allocator, b2JointArray* array, int index)
{
	B2_ASSERT(0 <= index && index < array->count);
	if (index < array->count - 1)
	{
		int removed = array->count - 1;
		array->joints[index] = array->joints[removed];
		array->count -= 1;
		return removed;
	}

	array->count -= 1;
	return B2_NULL_INDEX;
}
