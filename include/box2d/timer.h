// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "types.h"

/// Profiling data. Times are in milliseconds.
typedef struct b2Profile
{
	float step;
	float collide;
	float solve;
	float island;
	float solveInit;
	float solveVelocity;
	float solvePosition;
	float completion;
	float broadphase;
	float solveTOI;
} b2Profile;

static const b2Profile b2_emptyProfile = {0};

typedef struct b2Statistics
{
	int32_t bodyCount;
	int32_t contactCount;
	int32_t jointCount;
	int32_t proxyCount;
	int32_t treeHeight;
	int32_t contactPointCount;
} b2Statistics;

/// Timer for profiling. This has platform specific code and may
/// not work on every platform.
typedef struct b2Timer
{
#if defined(_WIN32)
	int64_t start;
#elif defined(__linux__) || defined (__APPLE__)
	unsigned long long start_sec;
	unsigned long long start_usec;
#endif
} b2Timer;

#ifdef __cplusplus
extern "C" {
#endif

b2Timer b2CreateTimer();
int64_t b2GetTicks(b2Timer* timer);
float b2GetMilliseconds(const b2Timer* timer);
float b2GetMillisecondsAndReset(b2Timer* timer);
void b2SleepMilliseconds(float milliseconds);

#ifdef __cplusplus
}
#endif
