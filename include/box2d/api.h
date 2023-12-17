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

/// Default allocation functions
BOX2D_API void b2SetAllocator(b2AllocFcn* allocFcn, b2FreeFcn* freeFcn);

/// Total bytes allocated by Box2D
BOX2D_API uint32_t b2GetByteCount(void);

#ifdef __cplusplus
extern "C" b2AssertFcn* Box2DAssertCallback;
#else
extern b2AssertFcn* Box2DAssertCallback;
#endif

