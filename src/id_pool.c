// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "id_pool.h"

#include "array.h"

b2IdPool b2CreateIdPool()
{
	b2IdPool pool = {0};
	pool.freeArray = b2CreateArray(sizeof(int), 32);
	return pool;
}

void b2DestroyIdPool(b2IdPool* pool)
{
	b2DestroyArray(pool->freeArray, sizeof(int));
	*pool = (b2IdPool){0};
}

int b2AllocId(b2IdPool* pool)
{
	if (b2Array(pool->freeArray).count > 0)
	{
		int id = b2Array_Last(pool->freeArray);
		b2Array_Pop(pool->freeArray);
		return id;
	}

	int id = pool->nextIndex;
	pool->nextIndex += 1;
	return id;
}

void b2FreeId(b2IdPool* pool, int id)
{
	B2_ASSERT(pool->nextIndex > 0);
	B2_ASSERT(0 <= id && id < pool->nextIndex);

	if (id == pool->nextIndex)
	{
		pool->nextIndex -= 1;
		return;
	}

	b2Array_Push(pool->freeArray, id);
}
