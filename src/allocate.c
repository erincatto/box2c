// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "allocate.h"

#include "core.h"

#include "box2d/api.h"

#if defined(B2_COMPILER_MSVC)
#define _CRTDBG_MAP_ALLOC
#include <crtdbg.h>
#include <stdlib.h>
#else
#include <stdlib.h>
#endif

#include <stdatomic.h>
#include <stdint.h>

#ifdef BOX2D_PROFILE

#include <tracy/TracyC.h>
#define b2TracyCAlloc(ptr, size) TracyCAlloc(ptr, size)
#define b2TracyCFree(ptr) TracyCFree(ptr)

#else

#define b2TracyCAlloc(ptr, size)
#define b2TracyCFree(ptr)

#endif

static b2AllocFcn* b2_allocFcn = NULL;
static b2FreeFcn* b2_freeFcn = NULL;

static _Atomic uint32_t b2_byteCount;

void b2SetAllocator(b2AllocFcn* allocFcn, b2FreeFcn* freeFcn)
{
	b2_allocFcn = allocFcn;
	b2_freeFcn = freeFcn;
}

void* b2Alloc(uint32_t size)
{
	atomic_fetch_add_explicit(&b2_byteCount, size, memory_order_relaxed);

	if (b2_allocFcn != NULL)
	{
		void* ptr = b2_allocFcn(size);
		b2TracyCAlloc(ptr, size);
		return ptr;
	}

	uint32_t size16 = ((size - 1) | 0xF) + 1;
#ifdef B2_PLATFORM_WINDOWS
	void* ptr = _aligned_malloc(size16, 16);
#else
	void* ptr = aligned_alloc(16, size16);
#endif

	b2TracyCAlloc(ptr, size);
	return ptr;
}

void b2Free(void* mem, uint32_t size)
{
	if (mem == NULL)
	{
		return;
	}

	b2TracyCFree(mem);

	if (b2_freeFcn != NULL)
	{
		b2_freeFcn(mem);
	}
	else
	{
#ifdef B2_PLATFORM_WINDOWS
		_aligned_free(mem);
#else
		free(mem);
#endif
	}

	atomic_fetch_sub_explicit(&b2_byteCount, size, memory_order_relaxed);
}

uint32_t b2GetByteCount(void)
{
	return atomic_load_explicit(&b2_byteCount, memory_order_relaxed);
}
