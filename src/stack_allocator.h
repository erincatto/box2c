// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "box2d/types.h"

// 100k
#define b2_stackSize (100 * 1024)
#define b2_maxStackEntries 32

typedef struct b2StackEntry
{
	char* data;
	int32_t size;
	bool usedMalloc;
} b2StackEntry;

// This is a stack allocator used for fast per step allocations.
// You must nest allocate/free pairs. The code will assert
// if you try to interleave multiple allocate/free pairs.
typedef struct b2StackAllocator
{
	char data[b2_stackSize];
	int32_t index;

	int32_t allocation;
	int32_t maxAllocation;

	b2StackEntry entries[b2_maxStackEntries];
	int32_t entryCount;
} b2StackAllocator;

void* b2AllocateStackItem(b2StackAllocator* alloc, int32_t size);
void b2FreeStackItem(b2StackAllocator* alloc, void* mem);

