// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "allocate.h"
#include "box2d/api.h"

#if defined(_WIN32)
#define _CRTDBG_MAP_ALLOC
#include <stdlib.h>
#include <crtdbg.h>
#else
#include <stdlib.h>
#endif

#include <stdint.h>
#include <stdatomic.h>

#ifdef BOX2D_PROFILE

#include <tracy/TracyC.h>
#define b2TracyCAlloc(ptr, size) TracyCAlloc(ptr, size)
#define b2TracyCFree(ptr) TracyCFree(ptr)

#else

#define b2TracyCAlloc(ptr, size)
#define b2TracyCFree(ptr)

#endif

b2AllocFcn* b2_allocFcn = NULL;
b2FreeFcn* b2_freeFcn = NULL;

_Atomic int32_t b2_byteCount;

void b2SetAllocator(b2AllocFcn* allocFcn, b2FreeFcn* freeFcn)
{
	b2_allocFcn = allocFcn;
	b2_freeFcn = freeFcn;
}

void* b2Alloc(int32_t size)
{
	atomic_fetch_add_explicit(&b2_byteCount, size, memory_order_relaxed);

	if (b2_allocFcn != NULL)
	{
		void* ptr = b2_allocFcn(size);
		b2TracyCAlloc(ptr, size);
		return ptr;
	}

	void* ptr = malloc(size);
	b2TracyCAlloc(ptr, size);
	return ptr;
}

void b2Free(void* mem, int32_t size)
{
	b2TracyCFree(mem);

	if (b2_freeFcn != NULL)
	{
		b2_freeFcn(mem);
	}
	else
	{
		free(mem);
	}

	atomic_fetch_sub_explicit(&b2_byteCount, size, memory_order_relaxed);
}

int32_t b2GetByteCount()
{
	return atomic_load_explicit(&b2_byteCount, memory_order_relaxed);
}
