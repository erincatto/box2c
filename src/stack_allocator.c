// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "box2d/allocate.h"

#include "stack_allocator.h"

#include <assert.h>
#include <stdbool.h>

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

b2StackAllocator* b2CreateStackAllocator()
{
	b2StackAllocator* allocator = b2Alloc(sizeof(b2StackAllocator));
	allocator->allocation = 0;
	allocator->maxAllocation = 0;
	allocator->entryCount = 0;
	allocator->index = 0;

	return allocator;
}

void b2DestroyStackAllocator(b2StackAllocator* allocator)
{
	assert(allocator->entryCount == 0);
	b2Free(allocator);
}

void* b2AllocateStackItem(b2StackAllocator* alloc, int32_t size)
{
	assert(alloc->entryCount < b2_maxStackEntries);

	b2StackEntry* entry = alloc->entries + alloc->entryCount;
	entry->size = size;
	if (alloc->index + size > b2_stackSize)
	{
		entry->data = (char*)b2Alloc(size);
		entry->usedMalloc = true;
	}
	else
	{
		entry->data = alloc->data + alloc->index;
		entry->usedMalloc = false;
		alloc->index += size;
	}

	alloc->allocation += size;
	if (alloc->allocation > alloc->maxAllocation)
	{
		alloc->maxAllocation = alloc->allocation;
	}
	++alloc->entryCount;

	return entry->data;
}

void b2FreeStackItem(b2StackAllocator* alloc, void* mem)
{
	assert(alloc->entryCount > 0);
	b2StackEntry* entry = alloc->entries + alloc->entryCount - 1;
	assert(mem == entry->data);
	if (entry->usedMalloc)
	{
		b2Free(mem);
	}
	else
	{
		alloc->index -= entry->size;
	}
	alloc->allocation -= entry->size;
	--alloc->entryCount;
}
