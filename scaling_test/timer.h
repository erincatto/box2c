// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include <stdint.h>

/// Timer for profiling. This has platform specific code and may not work on every platform.
typedef struct Timer
{
#if defined(_WIN32)
	int64_t start;
#elif defined(__linux__) || defined(__APPLE__)
	unsigned long long start_sec;
	unsigned long long start_usec;
#else
	int dummy;
#endif
} Timer;

Timer CreateTimer(void);
float GetMilliseconds(const Timer* timer);
