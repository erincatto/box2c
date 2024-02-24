// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include <stdint.h>

#if defined(_WIN32) && defined(box2d_EXPORTS)
	// build the Windows DLL
	#define BOX2D_EXPORT __declspec(dllexport)
#elif defined(_WIN32) && defined(BOX2D_DLL)
	// using the Windows DLL
	#define BOX2D_EXPORT __declspec(dllimport)
#elif defined(box2d_EXPORTS)
	// building or using the Box2D shared library
	#define BOX2D_EXPORT __attribute__((visibility("default")))
#else
	// static library
	#define BOX2D_EXPORT
#endif

#if defined(BOX2D_IMPLEMENTATION)
	#pragma message("BOX2D_IMPLEMENTATION")
	#if defined(_WIN32) && defined(box2d_EXPORTS)
		// build the Windows DLL
		#define BOX2D_INLINE __declspec(dllexport) extern inline
	#elif defined(_WIN32) && defined(BOX2D_DLL)
		// using the Windows DLL
		#define BOX2D_INLINE __declspec(dllimport)
	#elif defined(box2d_EXPORTS)
		#define BOX2D_INLINE __attribute__((visibility("default")))
	#else
		#define BOX2D_INLINE extern inline
	#endif
#else
	// #pragma message("BOX2D inline")
	#define BOX2D_INLINE inline
#endif

#ifdef __cplusplus
	#define B2_API extern "C" BOX2D_EXPORT
	#define B2_INLINE extern "C" BOX2D_INLINE
	#define B2_LITERAL(T) T
#else
	#define B2_API BOX2D_EXPORT
	#define B2_INLINE BOX2D_INLINE
	/// Used for C literals like (b2Vec2){1.0f, 2.0f} where C++ requires b2Vec2{1.0f, 2.0f}
	#define B2_LITERAL(T) (T)
#endif

/// Prototype for user allocation function.
///	@param size the allocation size in bytes
///	@param alignment the required alignment, guaranteed to be a power of 2
typedef void* b2AllocFcn(uint32_t size, int32_t alignment);

/// Prototype for user free function.
///	@param mem the memory previously allocated through `b2AllocFcn`
typedef void b2FreeFcn(void* mem);

/// Prototype for the user assert callback. Return 0 to skip the debugger break.
typedef int b2AssertFcn(const char* condition, const char* fileName, int lineNumber);

/// This allows the user to override the allocation functions. These should be
///	set during application startup.
B2_API void b2SetAllocator(b2AllocFcn* allocFcn, b2FreeFcn* freeFcn);

/// Total bytes allocated by Box2D
B2_API uint32_t b2GetByteCount(void);

/// Override the default assert callback.
///	@param assertFcn a non-null assert callback
B2_API void b2SetAssertFcn(b2AssertFcn* assertFcn);
