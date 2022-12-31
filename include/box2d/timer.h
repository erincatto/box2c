// SPDX-FileCopyrightText: 2022 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "types.h"

#ifdef __cplusplus
extern "C" {
#endif

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

b2Timer b2CreateTimer();
float b2GetMilliseconds(const b2Timer* timer);
float b2GetMillisecondsAndReset(b2Timer* timer);

#ifdef __cplusplus
}
#endif
