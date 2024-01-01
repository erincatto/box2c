// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "bitset.h"

// separate inline file for intrinsics to reduce build time

#if defined(_MSC_VER) && !defined(__clang__)
#include <intrin.h>

// https://en.wikipedia.org/wiki/Find_first_set
static inline uint32_t b2CTZ(uint64_t block)
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

static inline uint32_t b2CTZ(uint64_t block)
{
	return __builtin_ctzll(block);
}

#endif
