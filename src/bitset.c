// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "bitset.h"

#include "allocate.h"

#include <string.h>

b2BitSet b2CreateBitSet(uint32_t bitCapacity)
{
	b2BitSet bitSet = {0};

	bitSet.wordCapacity = (bitCapacity + sizeof(uint64_t) * 8 - 1) / (sizeof(uint64_t) * 8);
	bitSet.wordCount = 0;
	bitSet.bits = b2Alloc(bitSet.wordCapacity * sizeof(uint64_t));
	memset(bitSet.bits, 0, bitSet.wordCapacity * sizeof(uint64_t));
	return bitSet;
}

void b2DestroyBitSet(b2BitSet* bitSet)
{
	b2Free(bitSet->bits, bitSet->wordCapacity * sizeof(uint64_t));
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
	}

	bitSet->wordCount = wordCount;
	memset(bitSet->bits, 0, bitSet->wordCount * sizeof(uint64_t));
}

void b2GrowBitSet(b2BitSet* bitSet, uint32_t wordCount)
{
	B2_ASSERT(wordCount > bitSet->wordCount);
	if (wordCount > bitSet->wordCapacity)
	{
		uint32_t oldCapacity = bitSet->wordCapacity;
		bitSet->wordCapacity = wordCount + wordCount / 2;
		uint64_t* newBits = b2Alloc(bitSet->wordCapacity * sizeof(uint64_t));
		memset(newBits, 0, bitSet->wordCapacity * sizeof(uint64_t));
		memcpy(newBits, bitSet->bits, bitSet->wordCount * sizeof(uint64_t));
		b2Free(bitSet->bits, oldCapacity * sizeof(uint64_t));
		bitSet->bits = newBits;
	}

	bitSet->wordCount = wordCount;
}

void b2InPlaceUnion(b2BitSet* setA, const b2BitSet* setB)
{
	B2_ASSERT(setA->wordCount == setB->wordCount);
	uint32_t wordCount = setA->wordCount;
	for (uint32_t i = 0; i < wordCount; ++i)
	{
		setA->bits[i] |= setB->bits[i];
	}
}
