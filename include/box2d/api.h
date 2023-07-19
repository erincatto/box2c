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

#if defined(_DEBUG) && !defined(B2_ENABLE_ASSERTS)
	#define B2_ENABLE_ASSERTS
#endif

typedef void* b2AllocFcn(int32_t size);
typedef void b2FreeFcn(void* mem);
typedef void b2AssertFcn(const char* condition, const char* fileName, int lineNumber);

#ifdef __cplusplus
extern "C"
{
#endif

	/// Default allocation functions
	void b2SetAllocator(b2AllocFcn* allocFcn, b2FreeFcn* freeFcn);

	/// Total bytes allocated by Box2D
	int32_t b2GetByteCount();

#ifdef B2_ENABLE_ASSERTS
	extern b2AssertFcn AssertFailed;

	#define B2_ASSERT(condition) \
		do \
		{ \
			if (!(condition) && AssertFailed(#condition, __FILE__, int(__LINE__))) \
				B2_BREAKPOINT; \
		} while (false)
#else
	#define B2_ASSERT(...) ((void)0)
#endif

#ifdef __cplusplus
}
#endif
