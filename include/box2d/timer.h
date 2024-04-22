// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "api.h"

#include <stdint.h>

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

B2_API b2Timer b2CreateTimer(void);
B2_API int64_t b2GetTicks(b2Timer* timer);
B2_API float b2GetMilliseconds(const b2Timer* timer);
B2_API float b2GetMillisecondsAndReset(b2Timer* timer);
B2_API void b2SleepMilliseconds(int milliseconds);
B2_API void b2Yield();
