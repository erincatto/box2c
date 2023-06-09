// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "array.h"

#include <stdbool.h>
#include <stdint.h>

// Index pool allows for SoA pools, the only requirement
// is that the associated arrays have sufficient capacity.
typedef struct b2IndexPool
{
	int32_t* freeIndexArray;

	// This is the highest index to be allocated + 1. This allows many indices
	// to be allocated without using the array.
	int32_t index;
} b2IndexPool;

b2IndexPool b2CreateIndexPool(int32_t capacity);
void b2DestroyIndexPool(b2IndexPool* pool);

int32_t b2AllocIndex(b2IndexPool* pool);
void b2FreeIndex(b2IndexPool* pool, int32_t index);

// Any pooled struct must have this as the first member.
typedef struct b2Object
{
	int32_t index;
	int32_t next;	
	uint16_t revision;
} b2Object;

typedef struct b2Pool
{
	char* memory;
	int32_t objectSize;
	int32_t capacity;
	int32_t count;
	int32_t freeList;
} b2Pool;

b2Pool b2CreatePool(int32_t objectSize, int32_t capacity);
void b2DestroyPool(b2Pool* pool);

b2Object* b2AllocObject(b2Pool* pool);
void b2FreeObject(b2Pool* pool, b2Object* object);

static inline bool b2ObjectValid(const b2Object* object)
{
	// this means the object is not on the free list
	return object->index == object->next;
}

