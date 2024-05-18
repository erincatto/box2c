// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

// Shared library macros
#if defined(_MSC_VER) && defined(box2d_EXPORTS)
	// build the Windows DLL
	#define BOX2D_EXPORT __declspec(dllexport)
#elif defined(_MSC_VER) && defined(BOX2D_DLL)
	// using the Windows DLL
	#define BOX2D_EXPORT __declspec(dllimport)
#elif defined(box2d_EXPORTS)
	// building or using the Box2D shared library
	#define BOX2D_EXPORT __attribute__((visibility("default")))
#else
	// static library
	#define BOX2D_EXPORT
#endif

// C++ macros
#ifdef __cplusplus
	#define B2_API extern "C" BOX2D_EXPORT
	#define B2_INLINE inline
	#define B2_LITERAL(T) T
#else
	#define B2_API BOX2D_EXPORT
	#define B2_INLINE static inline
	/// Used for C literals like (b2Vec2){1.0f, 2.0f} where C++ requires b2Vec2{1.0f, 2.0f}
	#define B2_LITERAL(T) (T)
#endif

/// Prototype for user allocation function
///	@param size the allocation size in bytes
///	@param alignment the required alignment, guaranteed to be a power of 2
typedef void* b2AllocFcn(unsigned int size, int alignment);

/// Prototype for user free function
///	@param mem the memory previously allocated through `b2AllocFcn`
typedef void b2FreeFcn(void* mem);

/// Prototype for the user assert callback. Return 0 to skip the debugger break.
typedef int b2AssertFcn(const char* condition, const char* fileName, int lineNumber);

/// This allows the user to override the allocation functions. These should be
///	set during application startup.
B2_API void b2SetAllocator(b2AllocFcn* allocFcn, b2FreeFcn* freeFcn);

/// @return the total bytes allocated by Box2D
B2_API int b2GetByteCount(void);

/// Override the default assert callback
///	@param assertFcn a non-null assert callback
B2_API void b2SetAssertFcn(b2AssertFcn* assertFcn);
