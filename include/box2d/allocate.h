// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once
#include <stdint.h>

typedef void* b2AllocFcn(int32_t size);
typedef void b2FreeFcn(void* mem);

#ifdef __cplusplus
extern "C"
{
#endif

/// Default allocation functions
void b2SetAllocator(b2AllocFcn* allocFcn, b2FreeFcn* freeFcn);

/// Implement this function to use your own memory allocator.
void* b2Alloc(int32_t size);

/// If you implement b2Alloc, you should also implement this function.
void b2Free(void* mem);

/// Total bytes allocated by Box2D
extern int32_t b2_byteCount;

#ifdef __cplusplus
}
#endif
