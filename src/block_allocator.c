// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "block_allocator.h"
#include "allocate.h"
#include "core.h"

#include <limits.h>
#include <string.h>

#define b2_chunkSize (16 * 1024)
#define b2_maxBlockSize 640
#define b2_chunkArrayIncrement 128

// These are the supported object sizes. Actual allocations are rounded up the next size.
static const int32_t b2_blockSizes[] = {
	16,	 // 0
	32,	 // 1
	64,	 // 2
	96,	 // 3
	128, // 4
	160, // 5
	192, // 6
	224, // 7
	256, // 8
	320, // 9
	384, // 10
	448, // 11
	512, // 12
	640, // 13
};

#define b2_blockSizeCount B2_ARRAY_COUNT(b2_blockSizes)

// This maps an arbitrary allocation size to a suitable slot in b2_blockSizes.
typedef struct b2SizeMap
{
	uint8_t values[b2_maxBlockSize + 1];
} b2SizeMap;

static b2SizeMap b2_sizeMap;

void b2SizeMap_Initialize()
{
	int32_t j = 0;
	b2_sizeMap.values[0] = 0;
	for (int32_t i = 1; i <= b2_maxBlockSize; ++i)
	{
		B2_ASSERT(j < b2_blockSizeCount);
		if (i <= b2_blockSizes[j])
		{
			b2_sizeMap.values[i] = (uint8_t)j;
		}
		else
		{
			++j;
			b2_sizeMap.values[i] = (uint8_t)j;
		}
	}
}

static bool b2_sizeMapInitialized = false;

typedef struct b2Chunk
{
	int32_t blockSize;
	struct b2Block* blocks;
} b2Chunk;

typedef struct b2Block
{
	struct b2Block* next;
} b2Block;

// This is a small object allocator used for allocating small objects that persist for more than one time step.
// See: http://www.codeproject.com/useritems/Small_Block_Allocator.asp
typedef struct b2BlockAllocator
{
	b2Chunk* chunks;
	int32_t chunkCount;
	int32_t chunkSpace;

	b2Block* freeLists[b2_blockSizeCount];
} b2BlockAllocator;

b2BlockAllocator* b2CreateBlockAllocator()
{
	if (b2_sizeMapInitialized == false)
	{
		b2SizeMap_Initialize();
		b2_sizeMapInitialized = true;
	}

	B2_ASSERT(b2_blockSizeCount < UCHAR_MAX);

	b2BlockAllocator* allocator = (b2BlockAllocator*)b2Alloc(sizeof(b2BlockAllocator));
	allocator->chunkSpace = b2_chunkArrayIncrement;
	allocator->chunkCount = 0;
	allocator->chunks = (b2Chunk*)b2Alloc(allocator->chunkSpace * sizeof(b2Chunk));

	memset(allocator->chunks, 0, allocator->chunkSpace * sizeof(b2Chunk));
	memset(allocator->freeLists, 0, sizeof(allocator->freeLists));

	return allocator;
}

void b2DestroyBlockAllocator(b2BlockAllocator* allocator)
{
	for (int32_t i = 0; i < allocator->chunkCount; ++i)
	{
		b2Free(allocator->chunks[i].blocks, b2_chunkSize);
	}

	b2Free(allocator->chunks, allocator->chunkSpace * sizeof(b2Chunk));
	b2Free(allocator, sizeof(b2BlockAllocator));
}

void* b2AllocBlock(b2BlockAllocator* allocator, int32_t size)
{
	if (size == 0)
	{
		return NULL;
	}

	B2_ASSERT(0 < size);

	if (size > b2_maxBlockSize)
	{
		return b2Alloc(size);
	}

	int32_t index = b2_sizeMap.values[size];
	B2_ASSERT(0 <= index && index < b2_blockSizeCount);

	if (allocator->freeLists[index])
	{
		b2Block* block = allocator->freeLists[index];
		allocator->freeLists[index] = block->next;
		return block;
	}
	else
	{
		if (allocator->chunkCount == allocator->chunkSpace)
		{
			b2Chunk* oldChunks = allocator->chunks;
			int32_t oldSize = allocator->chunkSpace * sizeof(b2Chunk);
			allocator->chunkSpace += b2_chunkArrayIncrement;
			allocator->chunks = (b2Chunk*)b2Alloc(allocator->chunkSpace * sizeof(b2Chunk));
			memcpy(allocator->chunks, oldChunks, allocator->chunkCount * sizeof(b2Chunk));
			memset(allocator->chunks + allocator->chunkCount, 0, b2_chunkArrayIncrement * sizeof(b2Chunk));
			b2Free(oldChunks, oldSize);
		}

		b2Chunk* chunk = allocator->chunks + allocator->chunkCount;
		chunk->blocks = (b2Block*)b2Alloc(b2_chunkSize);
#if defined(_DEBUG)
		memset(chunk->blocks, 0xcd, b2_chunkSize);
#endif
		int32_t blockSize = b2_blockSizes[index];
		chunk->blockSize = blockSize;
		int32_t blockCount = b2_chunkSize / blockSize;
		B2_ASSERT(blockCount * blockSize <= b2_chunkSize);
		for (int32_t i = 0; i < blockCount - 1; ++i)
		{
			b2Block* block = (b2Block*)((int8_t*)chunk->blocks + blockSize * i);
			b2Block* next = (b2Block*)((int8_t*)chunk->blocks + blockSize * (i + 1));
			block->next = next;
		}
		b2Block* last = (b2Block*)((int8_t*)chunk->blocks + blockSize * (blockCount - 1));
		last->next = NULL;

		allocator->freeLists[index] = chunk->blocks->next;
		++allocator->chunkCount;

		return chunk->blocks;
	}
}

void b2FreeBlock(b2BlockAllocator* allocator, void* p, int32_t size)
{
	if (size == 0)
	{
		return;
	}

	B2_ASSERT(0 < size);

	if (size > b2_maxBlockSize)
	{
		b2Free(p, size);
		return;
	}

	int32_t index = b2_sizeMap.values[size];
	B2_ASSERT(0 <= index && index < b2_blockSizeCount);

#if defined(_DEBUG)
	// Verify the memory address and size is valid.
	int32_t blockSize = b2_blockSizes[index];
	bool found = false;
	for (int32_t i = 0; i < allocator->chunkCount; ++i)
	{
		b2Chunk* chunk = allocator->chunks + i;
		if (chunk->blockSize != blockSize)
		{
			B2_ASSERT((int8_t*)p + blockSize <= (int8_t*)chunk->blocks || (int8_t*)chunk->blocks + b2_chunkSize <= (int8_t*)p);
		}
		else
		{
			if ((int8_t*)chunk->blocks <= (int8_t*)p && (int8_t*)p + blockSize <= (int8_t*)chunk->blocks + b2_chunkSize)
			{
				found = true;
			}
		}
	}

	B2_ASSERT(found);

	memset(p, 0xfd, blockSize);
#endif

	b2Block* block = (b2Block*)p;
	block->next = allocator->freeLists[index];
	allocator->freeLists[index] = block;
}
