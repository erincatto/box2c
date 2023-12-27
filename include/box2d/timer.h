// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "types.h"

/// Profiling data. Times are in milliseconds.
typedef struct b2Profile
{
	float step;
	float pairs;
	float collide;
	float solve;
	float buildIslands;
	float solveConstraints;
	float broadphase;
	float continuous;
} b2Profile;

/// Use this to initialize your profile
static const b2Profile b2_emptyProfile = B2_ZERO_INIT;

/// Counters that give details of the simulation size
typedef struct b2Statistics
{
	int32_t islandCount;
	int32_t bodyCount;
	int32_t contactCount;
	int32_t jointCount;
	int32_t proxyCount;
	int32_t pairCount;
	int32_t treeHeight;
	int32_t stackCapacity;
	int32_t stackUsed;
	int32_t byteCount;
	int32_t taskCount;
	int32_t colorCounts[b2_graphColorCount + 1];
} b2Statistics;

/// Timer for profiling. This has platform specific code and may not work on every platform.
typedef struct b2Timer
{
#if defined(_WIN32)
	int64_t start;
#elif defined(__linux__) || defined(__APPLE__)
	unsigned long long start_sec;
	unsigned long long start_usec;
#else
	int dummy;
#endif
} b2Timer;

BOX2D_API b2Timer b2CreateTimer(void);
BOX2D_API int64_t b2GetTicks(b2Timer* timer);
BOX2D_API float b2GetMilliseconds(const b2Timer* timer);
BOX2D_API float b2GetMillisecondsAndReset(b2Timer* timer);
BOX2D_API void b2SleepMilliseconds(float milliseconds);

/// Tracy profiler instrumentation
///	https://github.com/wolfpld/tracy
#ifdef BOX2D_PROFILE

#include <tracy/TracyC.h>
#define b2TracyCZoneC(ctx, color, active) TracyCZoneC(ctx, color, active)
#define b2TracyCZoneNC(ctx, name, color, active) TracyCZoneNC(ctx, name, color, active)
#define b2TracyCZoneEnd(ctx) TracyCZoneEnd(ctx)

#else

#define b2TracyCZoneC(ctx, color, active)
#define b2TracyCZoneNC(ctx, name, color, active)
#define b2TracyCZoneEnd(ctx)

#endif
