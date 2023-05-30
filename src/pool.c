// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "box2d/allocate.h"

#include "pool.h"

#include <assert.h>
#include <string.h>

#define B2_NULL_INDEX (-1)
#define B2_VALIDATE 1

b2IndexPool b2CreateIndexPool(int32_t capacity)
{
	// The free index array starts empty
	b2IndexPool pool;
	pool.freeIndexArray = b2CreateArray(sizeof(int32_t), capacity);
	pool.index = 0;
	return pool;
}

void b2DestroyIndexPool(b2IndexPool* pool)
{
	b2DestroyArray(pool->freeIndexArray);
}

int32_t b2AllocIndex(b2IndexPool* pool)
{
	// First check free list
	if (b2Array(pool->freeIndexArray).count > 0)
	{
		int32_t index = b2Array_Last(pool->freeIndexArray);
		b2Array_Pop(pool->freeIndexArray);
		return index;
	}

	// Free list is empty
	int32_t index = pool->index;
	pool->index += 1;
	return index;
}

void b2FreeIndex(b2IndexPool* pool, int32_t index)
{
	assert(index < pool->index);
	b2Array_Push(pool->freeIndexArray, index);
}

b2Pool b2CreatePool(int32_t objectSize, int32_t capacity)
{
	assert(objectSize >= sizeof(b2Object));

	b2Pool pool;
	pool.objectSize = objectSize;
	pool.capacity = capacity > 1 ? capacity : 1;
	pool.count = 0;
	pool.memory = (char*)b2Alloc(pool.capacity * objectSize);
	
	pool.freeList = 0;
	for (int32_t i = 0; i < pool.capacity - 1; ++i)
	{
		b2Object* object = (b2Object*)(pool.memory + i * objectSize);
		object->index = i;
		object->next = i + 1;
		object->revision = 0;
	}

	b2Object* object = (b2Object*)(pool.memory + (pool.capacity - 1) * objectSize);
	object->index = pool.capacity - 1;
	object->next = B2_NULL_INDEX;
	object->revision = 0;

	return pool;
}

void b2DestroyPool(b2Pool* pool)
{
	b2Free(pool->memory);
	pool->memory = NULL;
	pool->capacity = 0;
	pool->count = 0;
	pool->freeList = B2_NULL_INDEX;
	pool->objectSize = 0;
}

b2Object* b2AllocObject(b2Pool* pool)
{
	b2Object* newObject = NULL;
	if (pool->freeList != B2_NULL_INDEX)
	{
		newObject = (b2Object*)(pool->memory + pool->freeList * pool->objectSize);
		newObject->index = pool->freeList;
		newObject->revision += 1;
		pool->freeList = newObject->next;
		newObject->next = newObject->index;
	}
	else
	{
		int32_t oldCapacity = pool->capacity;
		int32_t newCapacity = oldCapacity + oldCapacity / 2;
		newCapacity = newCapacity > 2 ? newCapacity : 2;
		pool->capacity = newCapacity;
		char* newMemory = (char*)b2Alloc(pool->capacity * pool->objectSize);
		memcpy(newMemory, pool->memory, oldCapacity * pool->objectSize);
		b2Free(pool->memory);
		pool->memory = newMemory;

		newObject = (b2Object*)(pool->memory + oldCapacity * pool->objectSize);
		newObject->index = oldCapacity;
		newObject->revision = 0;
		newObject->next = newObject->index;

		pool->freeList = oldCapacity + 1;
		for (int32_t i = oldCapacity + 1; i < newCapacity - 1; ++i)
		{
			b2Object* object = (b2Object*)(pool->memory + i * pool->objectSize);
			object->index = i;
			object->next = i + 1;
			object->revision = 0;
		}

		b2Object* object = (b2Object*)(pool->memory + (newCapacity - 1) * pool->objectSize);
		object->index = newCapacity - 1;
		object->next = B2_NULL_INDEX;
		object->revision = 0;
	}

	pool->count += 1;
	return newObject;
}

void b2FreeObject(b2Pool* pool, b2Object* object)
{
	assert(pool->memory <= (char*)object && (char*)object < pool->memory + pool->capacity * pool->objectSize);
	assert(object->index == object->next);
	assert(object->index < pool->capacity);

	object->next = pool->freeList;
	pool->freeList = object->index;
	pool->count -= 1;
}
