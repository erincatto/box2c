// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "box2d/api.h"

#include <stdint.h>

// Define platform
#if defined(_WIN64)
#define B2_PLATFORM_WINDOWS
#elif defined(__ANDROID__)
#define B2_PLATFORM_ANDROID
#elif defined(__linux__)
#define B2_PLATFORM_LINUX
#elif defined(__APPLE__)
#include <TargetConditionals.h>
#if defined(TARGET_OS_IPHONE) && !TARGET_OS_IPHONE
#define B2_PLATFORM_MACOS
#else
#define B2_PLATFORM_IOS
#endif
#elif defined(__EMSCRIPTEN__)
#define B2_PLATFORM_WASM
#else
#error Unsupported platform
#endif

// Define CPU
#if defined(__x86_64__) || defined(_M_X64)
#define B2_CPU_X64
#elif defined(__aarch64__) || defined(_M_ARM64)
#define B2_CPU_ARM
#elif defined(__EMSCRIPTEN__)
#define B2_CPU_WASM
#else
#error Unsupported CPU
#endif

// Define compiler
#if defined(__clang__)
#define B2_COMPILER_CLANG
#elif defined(__GNUC__)
#define B2_COMPILER_GCC
#elif defined(_MSC_VER)
#define B2_COMPILER_MSVC
#endif

#if defined(B2_PLATFORM_WINDOWS)
#define B2_BREAKPOINT __debugbreak()
#elif defined(B2_PLATFORM_LINUX) || defined(B2_PLATFORM_ANDROID) || defined(B2_PLATFORM_MACOS) || defined(B2_PLATFORM_IOS)
#if defined(B2_CPU_X64)
#define B2_BREAKPOINT __asm volatile("int $0x3")
#elif defined(B2_CPU_ARM)
#define B2_BREAKPOINT __builtin_trap()
#endif
#elif defined(B2_PLATFORM_WASM)
#define B2_BREAKPOINT                                                                                                                      \
	do                                                                                                                                     \
	{                                                                                                                                      \
	} while (0)
#else
#error Unknown platform
#endif

#if !defined(NDEBUG) || defined(B2_ENABLE_ASSERT)
#define B2_ASSERT(condition)                                                                                                               \
	do                                                                                                                                     \
	{                                                                                                                                      \
		if (!(condition) && Box2DAssertCallback(#condition, __FILE__, (int)__LINE__))                                                      \
			B2_BREAKPOINT;                                                                                                                 \
	} while (0)
#else
#define B2_ASSERT(...) ((void)0)
#endif

#if defined(NDEBUG)
#define B2_VALIDATE 0
#else
#define B2_VALIDATE 1
#endif
