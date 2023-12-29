// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include <stdint.h>

#if defined(_WIN32) && defined(box2d_EXPORTS)
	// building the Box2D DLL
	#define BOX2D_EXPORT __declspec(dllexport)
#elif defined(_WIN32) && defined(BOX2D_DLL)
	// using the Box2D DLL
	#define BOX2D_EXPORT __declspec(dllimport)
#elif defined(__GNUC__) && defined(box2d_EXPORTS)
	// building the Box2D shared library
	#define BOX2D_EXPORT __attribute__((visibility("default")))
#else
	// static library
	#define BOX2D_EXPORT
#endif

#ifdef __cplusplus
	#define B2_API extern "C" BOX2D_EXPORT
#else
	#define B2_API BOX2D_EXPORT
#endif

/// Prototype for user allocation function.
///	@param size the allocation size in bytes
///	@param alignment the required alignment, guaranteed to be a power of 2
typedef void* b2AllocFcn(uint32_t size, int32_t alignment);

/// Prototype for user free function.
///	@param mem the memory previously allocated through `b2AllocFcn`
typedef void b2FreeFcn(void* mem);

/// This allows the user to override the allocation functions. These should be
///	set during application startup.
B2_API void b2SetAllocator(b2AllocFcn* allocFcn, b2FreeFcn* freeFcn);

/// Total bytes allocated by Box2D
B2_API uint32_t b2GetByteCount(void);

/// Prototype for the user assert callback. Return 0 to skip the debugger break.
typedef int b2AssertFcn(const char* condition, const char* fileName, int lineNumber);

/// Override the default assert callback.
///	@param assertFcn a non-null assert callback
B2_API void b2SetAssertFcn(b2AssertFcn* assertFcn);
