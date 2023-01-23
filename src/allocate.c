// SPDX-FileCopyrightText: 2022 Erin Catto
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

void b2SetAllocator(b2AllocFcn* allocFcn, b2FreeFcn* freeFcn)
{
	b2_allocFcn = allocFcn;
	b2_freeFcn = freeFcn;
}

void* b2Alloc(int32_t size)
{
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
}
