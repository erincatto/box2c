// SPDX-FileCopyrightText: 2022 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once
#include "types.h"

typedef void* b2AllocFcn(int32_t size);
typedef void b2FreeFcn(void* mem);

#ifdef __cplusplus
extern "C"
{
#endif

/// Default allocation functions
void b2SetAlloc(b2AllocFcn* allocFcn, b2FreeFcn* freeFcn);

/// Implement this function to use your own memory allocator.
void* b2Alloc(int32_t size);

/// If you implement b2Alloc, you should also implement this function.
void b2Free(void* mem);

#ifdef __cplusplus
}
#endif
