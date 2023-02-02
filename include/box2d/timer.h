// SPDX-FileCopyrightText: 2022 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "types.h"

/// Profiling data. Times are in milliseconds.
typedef struct b2Profile
{
	float step;
	float collide;
	float solve;
	float solveInit;
	float solveVelocity;
	float solvePosition;
	float broadphase;
	float solveTOI;
} b2Profile;

/// Timer for profiling. This has platform specific code and may
/// not work on every platform.
typedef struct b2Timer
{
#if defined(_WIN32)
	double start;
#elif defined(__linux__) || defined (__APPLE__)
	unsigned long long start_sec;
	unsigned long long start_usec;
#endif
} b2Timer;


#ifdef __cplusplus
extern "C" {
#endif

b2Timer b2CreateTimer();
float b2GetMilliseconds(const b2Timer* timer);
float b2GetMillisecondsAndReset(b2Timer* timer);
void b2SleepMilliseconds(float milliseconds);

#ifdef __cplusplus
}
#endif
