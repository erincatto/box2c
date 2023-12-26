// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "core.h"

#include <stdbool.h>
#include <stdint.h>

// Bit set provides fast operations on large arrays of bits
typedef struct b2BitSet
{
	uint64_t* bits;
	uint32_t wordCapacity;
	uint32_t wordCount;
} b2BitSet;

b2BitSet b2CreateBitSet(uint32_t bitCapacity);
void b2DestroyBitSet(b2BitSet* bitSet);
void b2SetBitCountAndClear(b2BitSet* bitset, uint32_t bitCount);
void b2InPlaceUnion(b2BitSet* setA, const b2BitSet* setB);
void b2GrowBitSet(b2BitSet* set, uint32_t wordCount);

static inline void b2SetBit(b2BitSet* bitSet, uint32_t bitIndex)
{
	uint32_t wordIndex = bitIndex / 64;
	// TODO_ERIN support growing
	B2_ASSERT(wordIndex < bitSet->wordCount);
	bitSet->bits[wordIndex] |= ((uint64_t)1 << bitIndex % 64);
}

static inline void b2SetBitGrow(b2BitSet* bitSet, uint32_t bitIndex)
{
	uint32_t wordIndex = bitIndex / 64;
	if (wordIndex >= bitSet->wordCount)
	{
		b2GrowBitSet(bitSet, wordIndex + 1);
	}
	bitSet->bits[wordIndex] |= ((uint64_t)1 << bitIndex % 64);
}

static inline void b2ClearBit(b2BitSet* bitSet, uint32_t bitIndex)
{
	uint32_t wordIndex = bitIndex / 64;
	if (wordIndex >= bitSet->wordCount)
	{
		return;
	}
	bitSet->bits[wordIndex] &= ~((uint64_t)1 << bitIndex % 64);
}

static inline bool b2GetBit(const b2BitSet* bitSet, uint32_t bitIndex)
{
	uint32_t wordIndex = bitIndex / 64;
	if (wordIndex >= bitSet->wordCount)
	{
		return false;
	}
	return (bitSet->bits[wordIndex] & ((uint64_t)1 << bitIndex % 64)) != 0;
}
