// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "box2d/allocate.h"

#if defined(_WIN32)
#define _CRTDBG_MAP_ALLOC
#include <stdlib.h>
#include <crtdbg.h>
#else
#include <stdlib.h>
#endif

b2AllocFcn* b2_allocFcn = NULL;
b2FreeFcn* b2_freeFcn = NULL;

int32_t b2_byteCount;

void b2SetAllocator(b2AllocFcn* allocFcn, b2FreeFcn* freeFcn)
{
	b2_allocFcn = allocFcn;
	b2_freeFcn = freeFcn;
}

void* b2Alloc(int32_t size)
{
	// TODO_ERIN atomic
	b2_byteCount += size;

	if (b2_allocFcn != NULL)
	{
		return b2_allocFcn(size);
	}

	return malloc(size);
}

void b2Free(void* mem)
{
	if (b2_freeFcn != NULL)
	{
		b2_freeFcn(mem);
	}
	else
	{
		free(mem);
	}

	// TODO_ERIN atomic
	--b2_byteCount;
}
