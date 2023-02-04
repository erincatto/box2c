// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include <stdint.h>

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
