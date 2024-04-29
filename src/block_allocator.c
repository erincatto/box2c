// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "block_allocator.h"
#include "allocate.h"
#include "array.h"
#include "core.h"
#include "ctz.h"

#include <limits.h>
#include <stdbool.h>
#include <string.h>

#define b2_targetChunkSize (1 << 18)
#define b2_maxBlockSize (1 << b2_maxBlockPower)

// This is a helper struct for building a linked list of memory blocks
typedef struct b2Block
{
	struct b2Block* next;
} b2Block;

// This holds a contiguous block list from a single malloc call. The blocks are all of a the same size (a power of 2).
// When a chunk is allocated, its blocks are installed in the block allocator free list for the specific power of 2 size.
typedef struct b2Chunk
{
	b2Block* blockList;

	// size of each block
	int blockSize;

	// size of the entire list
	int totalSize;
} b2Chunk;

b2BlockAllocator b2CreateBlockAllocator(void)
{
	_Static_assert(b2_blockPowerCount == b2_maxBlockPower - b2_minBlockPower + 1, "wrong size");
	_Static_assert(b2_targetChunkSize <= (1 << b2_maxBlockPower), "wrong size");
	_Static_assert(b2_targetChunkSize >= (1 << b2_minBlockPower), "wrong size");

	b2BlockAllocator allocator = {0};
	allocator.chunkArray = b2CreateArray(sizeof(b2Chunk), 128);
	memset(allocator.chunkArray, 0, b2Array(allocator.chunkArray).capacity * sizeof(b2Chunk));
	return allocator;
}

void b2DestroyBlockAllocator(b2BlockAllocator* allocator)
{
	int chunkCount = b2Array(allocator->chunkArray).count;
	for (int32_t i = 0; i < chunkCount; ++i)
	{
		b2Chunk* chunk = allocator->chunkArray + i;
		b2Free(chunk->blockList, chunk->totalSize);
	}

	b2DestroyArray(allocator->chunkArray, sizeof(b2Chunk));
}

// todo this would leak on shutdown
#define B2_USE_SYSTEM_ALLOC 0

#if B2_USE_SYSTEM_ALLOC
void* b2AllocBlock(b2BlockAllocator* allocator, int size)
{
	((void)allocator);
	return b2Alloc(size);
}

void b2FreeBlock(b2BlockAllocator* allocator, void* memory, int size)
{
	((void)allocator);
	b2Free(memory, size);
}

#else

void* b2AllocBlock(b2BlockAllocator* allocator, int size)
{
	B2_ASSERT(size >= 0);

	if (size == 0)
	{
		return NULL;
	}

	if (size > b2_maxBlockSize)
	{
		B2_ASSERT(false);
		return NULL;
	}

	int power = b2BoundingPowerOf2(size);
	power = power < b2_minBlockPower ? b2_minBlockPower : power;

	int index = power - b2_minBlockPower;
	B2_ASSERT(0 <= index && index < b2_blockPowerCount);

	if (allocator->freeLists[index] != NULL)
	{
		b2Block* block = allocator->freeLists[index];
		allocator->freeLists[index] = block->next;
		return block;
	}

	// free list is empty, allocate more blocks for this power
	b2Chunk chunk;
	chunk.blockSize = (1 << power);
	// if the block size is very large then the chunk will hold a single block
	if (chunk.blockSize > b2_targetChunkSize)
	{
		chunk.totalSize = chunk.blockSize;
	}
	else
	{
		chunk.totalSize = b2_targetChunkSize;
	}
	chunk.blockList = b2Alloc(chunk.totalSize);

#if B2_DEBUG
	memset(chunk.blockList, 0xcd, chunk.totalSize);
#endif

	// build linked list
	int blockCount = chunk.totalSize / chunk.blockSize;
	B2_ASSERT(blockCount * chunk.blockSize == chunk.totalSize);
	for (int i = 0; i < blockCount - 1; ++i)
	{
		b2Block* block = (b2Block*)((int8_t*)chunk.blockList + chunk.blockSize * i);
		b2Block* next = (b2Block*)((int8_t*)chunk.blockList + chunk.blockSize * (i + 1));
		block->next = next;
	}
	b2Block* last = (b2Block*)((int8_t*)chunk.blockList + chunk.blockSize * (blockCount - 1));
	last->next = NULL;

	allocator->freeLists[index] = chunk.blockList->next;

	b2Array_Push(allocator->chunkArray, chunk);

	return chunk.blockList;
}

void b2FreeBlock(b2BlockAllocator* allocator, void* memory, int size)
{
	B2_ASSERT(size >= 0);

	if (size == 0)
	{
		return;
	}

	B2_ASSERT(size <= b2_maxBlockSize);

	int power = b2BoundingPowerOf2(size);
	power = power < b2_minBlockPower ? b2_minBlockPower : power;

	int index = power - b2_minBlockPower;
	B2_ASSERT(0 <= index && index < b2_blockPowerCount);

#if B2_VALIDATE
	// verify the memory address and size are valid
	int blockSize = (1 << power);
	bool found = false;
	int chunkCount = b2Array(allocator->chunkArray).count;
	for (int i = 0; i < chunkCount; ++i)
	{
		b2Chunk* chunk = allocator->chunkArray + i;
		if (chunk->blockSize != blockSize)
		{
			// this chunk is not the right size, so make sure it does not overlap the freed memory
			B2_ASSERT((int8_t*)memory + blockSize <= (int8_t*)chunk->blockList ||
					  (int8_t*)chunk->blockList + chunk->totalSize <= (int8_t*)memory);
		}
		else
		{
			// does the freed memory overlap this chunk's block list?
			if ((int8_t*)chunk->blockList <= (int8_t*)memory &&
				(int8_t*)memory + blockSize <= (int8_t*)chunk->blockList + chunk->totalSize)
			{
				found = true;
			}
		}
	}

	B2_ASSERT(found);
	memset(memory, 0xfd, blockSize);
#endif

	// add to free list
	b2Block* block = memory;
	block->next = allocator->freeLists[index];
	allocator->freeLists[index] = block;
}
#endif

bool b2ValidateBlockAllocator(b2BlockAllocator* allocator)
{
	for (int i = 0; i < b2_blockPowerCount; ++i)
	{
		int count = 0;
		b2Block* block = allocator->freeLists[i];
		while (block != NULL)
		{
			block = block->next;
			count += 1;

			if (count == 10000000)
			{
				return false;
			}
		}
	}

	return true;
}
