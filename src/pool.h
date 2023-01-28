// SPDX-FileCopyrightText: 2022 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "box2d/allocate.h"
#include "box2d/types.h"

#include <assert.h>
#include <string.h>

struct b2PoolObject
{
	int32_t index;
	int32_t next;	
	uint16_t revision;
};

struct b2Pool
{
	char* memory;
	int32_t objectSize;
	int32_t capacity;
	int32_t count;
	int32_t freeList;
};

#define B2_POOL_GET(TYPE, POOL, INDEX) ((TYPE*)(POOL)->memory + (INDEX))

static inline b2Pool b2CreatePool(int32_t objectSize, int32_t capacity)
{
	assert(objectSize >= sizeof(b2PoolObject));

	b2Pool pool;
	pool.objectSize = objectSize;
	pool.capacity = capacity > 1 ? capacity : 1;
	pool.count = 0;
	pool.memory = (char*)b2Alloc(pool.capacity * objectSize);
	
	pool.freeList = 0;
	for (int32_t i = 0; i < pool.capacity - 1; ++i)
	{
		b2PoolObject* object = (b2PoolObject*)(pool.memory + i * objectSize);
		object->index = i;
		object->next = i + 1;
		object->revision = 0;
	}

	b2PoolObject* object = (b2PoolObject*)(pool.memory + (pool.capacity - 1) * objectSize);
	object->index = pool.capacity - 1;
	object->next = B2_NULL_INDEX;
	object->revision = 0;

	return pool;
}

static inline void b2DestroyPool(b2Pool* pool)
{
	b2Free(pool->memory);
	pool->memory = NULL;
	pool->capacity = 0;
	pool->count = 0;
	pool->freeList = B2_NULL_INDEX;
	pool->objectSize = 0;
}

static inline b2PoolObject* b2AllocPoolObject(b2Pool* pool)
{
	b2PoolObject* newObject = NULL;
	if (pool->freeList != B2_NULL_INDEX)
	{
		newObject = (b2PoolObject*)(pool->memory + pool->freeList * pool->objectSize);
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

		newObject = (b2PoolObject*)(pool->memory + oldCapacity * pool->objectSize);
		newObject->index = oldCapacity;
		newObject->revision = 0;
		newObject->next = newObject->index;

		pool->freeList = oldCapacity + 1;
		for (int32_t i = oldCapacity + 1; i < newCapacity - 1; ++i)
		{
			b2PoolObject* object = (b2PoolObject*)(pool->memory + i * pool->objectSize);
			object->index = i;
			object->next = i + 1;
			object->revision = 0;
		}

		b2PoolObject* object = (b2PoolObject*)(pool->memory + (newCapacity - 1) * pool->objectSize);
		object->index = newCapacity - 1;
		object->next = B2_NULL_INDEX;
		object->revision = 0;

		return object;
	}

	return newObject;
}

static inline void b2FreePoolObject(b2Pool* pool, b2PoolObject* object)
{
	assert(pool->memory <= (char*)object && (char*)object < pool->memory + pool->capacity * pool->objectSize);
	assert(object->index == object->next);
	assert(object->index < pool->capacity);

	object->next = pool->freeList;
	pool->freeList = object->index;
}
