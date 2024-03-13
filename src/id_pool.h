// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "array.h"

typedef struct b2IdPool
{
	int* freeArray;
	int nextIndex;
} b2IdPool;

b2IdPool b2CreateIdPool();
void b2DestroyIdPool(b2IdPool* pool);

int b2AllocateId(b2IdPool* pool);
void b2FreeId(b2IdPool* pool, int id);

inline int b2GetIdCount(b2IdPool* pool)
{
	return pool->nextIndex - b2Array(pool->freeArray).count;
}
