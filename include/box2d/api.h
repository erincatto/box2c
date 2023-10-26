// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include <stdint.h>

#ifdef __cplusplus
#define BOX2D_CPP extern "C"
#else
#define BOX2D_CPP
#endif

#if defined(_WIN32) && defined(BOX2D_BUILD_DLL)
// Building Box2D as a DLL
#define BOX2D_API BOX2D_CPP __declspec(dllexport)
#elif defined(_WIN32) && defined(BOX2D_DLL)
// Using Box2D as a DLL
#define BOX2D_API BOX2D_CPP __declspec(dllimport)
#elif defined(__GNUC__) && defined(BOX2D_BUILD_DLL)
// Building Box2D as a shared library
#define BOX2D_API BOX2D_CPP __attribute__((visibility("default")))
#else
#define BOX2D_API BOX2D_CPP
#endif

typedef void* b2AllocFcn(uint32_t size);
typedef void b2FreeFcn(void* mem);

// Return 0 to
typedef int b2AssertFcn(const char* condition, const char* fileName, int lineNumber);

#ifdef __cplusplus
extern "C"
{
#endif

/// Default allocation functions
void b2SetAllocator(b2AllocFcn* allocFcn, b2FreeFcn* freeFcn);

/// Total bytes allocated by Box2D
uint32_t b2GetByteCount(void);

extern b2AssertFcn* Box2DAssertCallback;

#ifdef __cplusplus
}
#endif
