// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "block_allocator.h"
#include "block_array.h"

#include "body.h"
#include "contact.h"
#include "island.h"
#include "joint.h"

#include <string.h>

b2BodyArray b2CreateBodyArray(b2BlockAllocator* allocator, int capacity)
{
	// ensure memcpy works later
	B2_ASSERT(capacity > 0);
	return (b2BodyArray){b2AllocBlock(allocator, capacity * sizeof(b2Body)), 0, capacity};
}

b2BodyStateArray b2CreateBodyStateArray(b2BlockAllocator* allocator, int capacity)
{
	// ensure memcpy works later
	B2_ASSERT(capacity > 0);
	return (b2BodyStateArray){b2AllocBlock(allocator, capacity * sizeof(b2BodyState)), 0, capacity};
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

b2IslandArray b2CreateIslandArray(b2BlockAllocator* allocator, int capacity)
{
	B2_ASSERT(capacity > 0);
	return (b2IslandArray){b2AllocBlock(allocator, capacity * sizeof(b2Island)), 0, capacity};
}

void b2DestroyBodyArray(b2BlockAllocator* allocator, b2BodyArray* array)
{
	b2FreeBlock(allocator, array->data, array->capacity * sizeof(b2Body));
}

void b2DestroyBodyStateArray(b2BlockAllocator* allocator, b2BodyStateArray* array)
{
	b2FreeBlock(allocator, array->data, array->capacity * sizeof(b2BodyState));
}

void b2DestroyContactArray(b2BlockAllocator* allocator, b2ContactArray* array)
{
	b2FreeBlock(allocator, array->data, array->capacity * sizeof(b2Contact));
}

void b2DestroyJointArray(b2BlockAllocator* allocator, b2JointArray* array)
{
	b2FreeBlock(allocator, array->data, array->capacity * sizeof(b2Joint));
}

void b2DestroyIslandArray(b2BlockAllocator* allocator, b2IslandArray* array)
{
	b2FreeBlock(allocator, array->data, array->capacity * sizeof(b2Island));
}

b2Body* b2AddBody(b2BlockAllocator* allocator, b2BodyArray* array)
{
	if (array->count == array->capacity)
	{
		int elementSize = sizeof(b2Body);
		int newCapacity = B2_MAX(16, 2 * array->capacity);
		b2Body* newElements = b2AllocBlock(allocator, newCapacity * elementSize);
		memcpy(newElements, array->data, array->capacity * elementSize);
		b2FreeBlock(allocator, array->data, array->capacity * elementSize);
		array->data = newElements;
		array->capacity = newCapacity;
	}

	b2Body* element = array->data + array->count;
	array->count += 1;
	return element;
}

b2BodyState* b2AddBodyState(b2BlockAllocator* allocator, b2BodyStateArray* array)
{
	if (array->count == array->capacity)
	{
		int elementSize = sizeof(b2BodyState);
		int newCapacity = B2_MAX(16, 2 * array->capacity);
		b2BodyState* newElements = b2AllocBlock(allocator, newCapacity * elementSize);
		memcpy(newElements, array->data, array->capacity * elementSize);
		b2FreeBlock(allocator, array->data, array->capacity * elementSize);
		array->data = newElements;
		array->capacity = newCapacity;
	}

	b2BodyState* element = array->data + array->count;
	array->count += 1;
	return element;
}

b2Contact* b2AddContact(b2BlockAllocator* allocator, b2ContactArray* array)
{
	if (array->count == array->capacity)
	{
		int elementSize = sizeof(b2Contact);
		int newCapacity = B2_MAX(16, 2 * array->capacity);
		b2Contact* newElements = b2AllocBlock(allocator, newCapacity * elementSize);
		memcpy(newElements, array->data, array->capacity * elementSize);
		b2FreeBlock(allocator, array->data, array->capacity * elementSize);
		array->data = newElements;
		array->capacity = newCapacity;
	}

	b2Contact* element = array->data + array->count;
	array->count += 1;
	return element;
}

b2Joint* b2AddJoint(b2BlockAllocator* allocator, b2JointArray* array)
{
	if (array->count == array->capacity)
	{
		int elementSize = sizeof(b2Joint);
		int newCapacity = B2_MAX(16, 2 * array->capacity);
		b2Joint* newElements = b2AllocBlock(allocator, newCapacity * elementSize);
		memcpy(newElements, array->data, array->capacity * elementSize);
		b2FreeBlock(allocator, array->data, array->capacity * elementSize);
		array->data = newElements;
		array->capacity = newCapacity;
	}

	b2Joint* element = array->data + array->count;
	array->count += 1;
	return element;
}

b2Island* b2AddIsland(b2BlockAllocator* allocator, b2IslandArray* array)
{
	if (array->count == array->capacity)
	{
		int elementSize = sizeof(b2Island);
		int newCapacity = B2_MAX(16, 2 * array->capacity);
		b2Island* newElements = b2AllocBlock(allocator, newCapacity * elementSize);
		memcpy(newElements, array->data, array->capacity * elementSize);
		b2FreeBlock(allocator, array->data, array->capacity * elementSize);
		array->data = newElements;
		array->capacity = newCapacity;
	}

	b2Island* element = array->data + array->count;
	element->islandId = B2_NULL_INDEX;
	array->count += 1;
	return element;
}

// Returns the index of the element moved into the empty slot (or B2_NULL_INDEX)
int b2RemoveBody(b2BodyArray* array, int index)
{
	B2_ASSERT(0 <= index && index < array->count);
	if (index < array->count - 1)
	{
		int removed = array->count - 1;
		array->data[index] = array->data[removed];
		array->count -= 1;
		return removed;
	}

	array->count -= 1;
	return B2_NULL_INDEX;
}

int b2RemoveBodyState(b2BodyStateArray* array, int index)
{
	B2_ASSERT(0 <= index && index < array->count);
	if (index < array->count - 1)
	{
		int removed = array->count - 1;
		array->data[index] = array->data[removed];
		array->count -= 1;
		return removed;
	}

	array->count -= 1;
	return B2_NULL_INDEX;
}

int b2RemoveContact(b2ContactArray* array, int index)
{
	B2_ASSERT(0 <= index && index < array->count);
	if (index < array->count - 1)
	{
		int removed = array->count - 1;
		array->data[index] = array->data[removed];
		array->count -= 1;
		return removed;
	}

	array->count -= 1;
	return B2_NULL_INDEX;
}

int b2RemoveJoint(b2JointArray* array, int index)
{
	B2_ASSERT(0 <= index && index < array->count);
	if (index < array->count - 1)
	{
		int removed = array->count - 1;
		array->data[index] = array->data[removed];
		array->count -= 1;
		return removed;
	}

	array->count -= 1;
	return B2_NULL_INDEX;
}

int b2RemoveIsland(b2IslandArray* array, int index)
{
	B2_ASSERT(0 <= index && index < array->count);
	if (index < array->count - 1)
	{
		int removed = array->count - 1;
		array->data[index] = array->data[removed];
		array->count -= 1;
		return removed;
	}

	array->count -= 1;
	return B2_NULL_INDEX;
}
