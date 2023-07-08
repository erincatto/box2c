// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include <assert.h>
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
bool b2GetNextSetBitIndex(const b2BitSet* bitset, uint32_t* bitIndexPtr);

static inline void b2SetBit(b2BitSet* bitSet, uint32_t bitIndex)
{
	uint32_t wordIndex = bitIndex / 64;
	assert(wordIndex < bitSet->wordCount);
	bitSet->bits[wordIndex] |= ((uint64_t)1) << (bitIndex % 64);
}
