// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "box2d/allocate.h"

#include "bitset.h"

#include <assert.h>
#include <string.h>

b2BitSet b2CreateBitSet(uint32_t bitCapacity)
{
	b2BitSet bitSet = {0};

	bitSet.wordCapacity = (bitCapacity + sizeof(uint64_t) * 8 - 1) / (sizeof(uint64_t) * 8);
	bitSet.wordCount = 0;
	bitSet.bits = b2Alloc(bitSet.wordCapacity * sizeof(uint64_t));

	return bitSet;
}

void b2DestroyBitSet(b2BitSet* bitSet)
{
	b2Free(bitSet->bits);
	bitSet->wordCapacity = 0;
	bitSet->wordCount = 0;
	bitSet->bits = NULL;
}

void b2SetBitCountAndClear(b2BitSet* bitSet, uint32_t bitCount)
{
	uint32_t wordCount = (bitCount + sizeof(uint64_t) * 8 - 1) / (sizeof(uint64_t) * 8);
	if (bitSet->wordCapacity < wordCount)
	{
		b2DestroyBitSet(bitSet);
		uint32_t newBitCapacity = bitCount + (bitCount >> 1);
		*bitSet = b2CreateBitSet(newBitCapacity);
		return;
	}

	bitSet->wordCount = wordCount;
	memset(bitSet->bits, 0, bitSet->wordCount * sizeof(uint64_t));
}

void b2InPlaceUnion(b2BitSet* setA, const b2BitSet* setB)
{
	assert(setA->wordCount == setB->wordCount);
	uint32_t wordCount = setA->wordCount;
	for (uint32_t i = 0; i < wordCount; ++i)
	{
		setA->bits[i] |= setB->bits[i];
	}
}


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


// Iterate over the set bits
// https://lemire.me/blog/2018/02/21/iterating-over-set-bits-quickly/
bool b2GetNextSetBitIndex(const b2BitSet* bitset, uint32_t* bitIndexPtr)
{
	uint32_t bitIndex = *bitIndexPtr;
	uint32_t wordIndex = bitIndex / 64;
	if (wordIndex >= bitset->wordCount)
	{
		return false;
	}

	uint64_t word = bitset->bits[wordIndex];
	word >>= (bitIndex & 63);

	if (word != 0)
	{
		*bitIndexPtr += b2CTZ(word);
		return true;
	}

	wordIndex += 1;

	while (wordIndex < bitset->wordCount)
	{
		word = bitset->bits[wordIndex];
		if (word != 0)
		{
			*bitIndexPtr = 64 * wordIndex + b2CTZ(word);
			return true;
		}

		wordIndex += 1;
	}

	return false;
}

