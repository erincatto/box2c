// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "bitset.h"

// separate inline file for intrinsics to reduce build time

#if defined(_MSC_VER) && !defined(__clang__)
#include <intrin.h>

// https://en.wikipedia.org/wiki/Find_first_set
static inline uint32_t b2CTZ(uint64_t word)
{
	unsigned long index;

#ifdef _WIN64
	_BitScanForward64(&index, word);
#else
	// 32-bit fall back
	if ((uint32_t)word != 0)
	{
		_BitScanForward(&index, (uint32_t)word);
	}
	else
	{
		_BitScanForward(&index, (uint32_t)(word >> 32));
		index += 32;
	}
#endif

	return index;
}

#else

static inline uint32_t b2CTZ(uint64_t word)
{
	return __builtin_ctzll(word);
}

#endif
