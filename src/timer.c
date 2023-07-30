// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "box2d/timer.h"

#if defined(_WIN32)

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif

#include <windows.h>

static double s_invFrequency = 0.0;

b2Timer b2CreateTimer(void)
{
	LARGE_INTEGER largeInteger;

	if (s_invFrequency == 0.0)
	{
		QueryPerformanceFrequency(&largeInteger);

		s_invFrequency = (double)largeInteger.QuadPart;
		if (s_invFrequency > 0.0)
		{
			s_invFrequency = 1000.0 / s_invFrequency;
		}
	}

	QueryPerformanceCounter(&largeInteger);
	b2Timer timer;
	timer.start = largeInteger.QuadPart;
	return timer;
}

int64_t b2GetTicks(b2Timer* timer)
{
	LARGE_INTEGER largeInteger;
	QueryPerformanceCounter(&largeInteger);
	int64_t ticks = largeInteger.QuadPart;
	int64_t count = ticks - timer->start;
	timer->start = ticks;
	return count;
}

float b2GetMilliseconds(const b2Timer* timer)
{
	LARGE_INTEGER largeInteger;
	QueryPerformanceCounter(&largeInteger);
	int64_t count = largeInteger.QuadPart;
	float ms = (float)(s_invFrequency * (count - timer->start));
	return ms;
}

float b2GetMillisecondsAndReset(b2Timer* timer)
{
	LARGE_INTEGER largeInteger;
	QueryPerformanceCounter(&largeInteger);
	int64_t count = largeInteger.QuadPart;
	float ms = (float)(s_invFrequency * (count - timer->start));
	timer->start = count;
	return ms;
}

void b2SleepMilliseconds(float milliseconds)
{
	Sleep((DWORD)(milliseconds));
}

#elif defined(__linux__) || defined (__APPLE__)

#include <sys/time.h>

b2Timer b2CreateTimer(void)
{
	b2Timer timer;
	struct timeval t;
	gettimeofday(&t, 0);
	timer.start_sec = t.tv_sec;
	timer.start_usec = t.tv_usec;
	return timer;
}

float b2GetMilliseconds(const b2Timer* timer)
{
	struct timeval t;
	gettimeofday(&t, 0);
	time_t start_sec = timer->start_sec;
	suseconds_t start_usec = timer->start_usec;
	
	// http://www.gnu.org/software/libc/manual/html_node/Elapsed-Time.html
	if (t.tv_usec < start_usec)
	{
		int nsec = (start_usec - t.tv_usec) / 1000000 + 1;
		start_usec -= 1000000 * nsec;
		start_sec += nsec;
	}
	
	if (t.tv_usec - start_usec > 1000000)
	{
		int nsec = (t.tv_usec - start_usec) / 1000000;
		start_usec += 1000000 * nsec;
		start_sec -= nsec;
	}
	return 1000.0f * (t.tv_sec - start_sec) + 0.001f * (t.tv_usec - start_usec);
}

float b2GetMillisecondsAndReset(b2Timer* timer)
{
	struct timeval t;
	gettimeofday(&t, 0);
	time_t start_sec = timer->start_sec;
	suseconds_t start_usec = timer->start_usec;

	// http://www.gnu.org/software/libc/manual/html_node/Elapsed-Time.html
	if (t.tv_usec < start_usec)
	{
		int nsec = (start_usec - t.tv_usec) / 1000000 + 1;
		start_usec -= 1000000 * nsec;
		start_sec += nsec;
	}

	if (t.tv_usec - start_usec > 1000000)
	{
		int nsec = (t.tv_usec - start_usec) / 1000000;
		start_usec += 1000000 * nsec;
		start_sec -= nsec;
	}
	
	timer->start_sec = t.tv_sec;
	timer->start_usec = t.tv_usec;

	return 1000.0f * (t.tv_sec - start_sec) + 0.001f * (t.tv_usec - start_usec);
}

void b2SleepMilliseconds(float milliseconds)
{
	B2_MAYBE_UNUSED(milliseconds);
	// TODO couldn't get this to compile on gcc
	// usleep((uint32_t)(1000.0f * milliseconds + 0.5f));
}

#else

b2Timer b2CreateTimer(void)
{
	return b2Timer;
}

float b2GetMilliseconds(const b2Timer*)
{
	return 0.0f;
}

float b2GetMillisecondsAndReset(b2Timer*)
{
	return 0.0f;
}

void b2SleepMilliseconds(float)
{
}

#endif
