// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include <stdbool.h>

#if defined(_MSC_VER) && !defined(__clang__)
#include <intrin.h>

// https://en.wikipedia.org/wiki/Find_first_set

static inline uint32_t b2CTZ32(uint32_t block)
{
	unsigned long index;
	_BitScanForward(&index, block);
	return index;
}

static inline uint32_t b2CTZ64(uint64_t block)
{
	unsigned long index;

#ifdef _WIN64
	_BitScanForward64(&index, block);
#else
	// 32-bit fall back
	if ((uint32_t)block != 0)
	{
		_BitScanForward(&index, (uint32_t)block);
	}
	else
	{
		_BitScanForward(&index, (uint32_t)(block >> 32));
		index += 32;
	}
#endif

	return index;
}

#else

static inline uint32_t b2CTZ32(uint32_t block)
{
	return __builtin_ctz(block);
}

static inline uint32_t b2CTZ64(uint64_t block)
{
	return __builtin_ctzll(block);
}

#endif

inline bool b2IsPowerOf2(int x)
{
	return (x & (x - 1)) == 0;
}

inline int b2BoundingPowerOf2(int x)
{
	if (x <= 1)
	{
		return 1;	
	}

	return 32 - (int)b2CTZ32((uint32_t)x - 1);
}

inline int b2RoundUpPowerOf2(int x)
{
	if (x <= 1)
	{
		return 1;
	}
	
	return 1 << (32 - (int)b2CTZ32((uint32_t)x - 1));
}

