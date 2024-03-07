// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#define b2_blockPowerCount 11

// This is a block allocator used for allocating objects that persist for more than one time step. Allocations
// rounded up to a power of 2. Free lists are maintained for a range of powers of two. Not thread-safe.
typedef struct b2BlockAllocator
{
	// Array of all the chunks of memory from malloc
	struct b2Chunk* chunkArray;

	// List of free blocks, one list for each power of 2
	struct b2Block* freeLists[b2_blockPowerCount];
} b2BlockAllocator;

// Create an allocator suitable for allocating and freeing objects quickly.
// Does not return memory to the heap.
b2BlockAllocator b2CreateBlockAllocator(void);

// Destroy a block allocator instance
void b2DestroyBlockAllocator(b2BlockAllocator* allocator);

// Allocate memory. This will use b2Alloc if the size is larger than b2_maxBlockSize.
void* b2AllocBlock(b2BlockAllocator* allocator, int size);

// Free memory. This will use b2Free if the size is larger than b2_maxBlockSize.
void b2FreeBlock(b2BlockAllocator* allocator, void* memory, int size);
