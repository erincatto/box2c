// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "stack_allocator.h"

#include "allocate.h"
#include "array.h"
#include "core.h"

#include <stdbool.h>

typedef struct b2StackEntry
{
	char* data;
	const char* name;
	int32_t size;
	bool usedMalloc;
} b2StackEntry;

// This is a stack allocator used for fast per step allocations.
// You must nest allocate/free pairs. The code will B2_ASSERT
// if you try to interleave multiple allocate/free pairs.
// Unlike a scratch allocator, this lets me use the heap if the allocator
// space is insufficient.
typedef struct b2StackAllocator
{
	char* data;
	int32_t capacity;
	int32_t index;

	int32_t allocation;
	int32_t maxAllocation;

	b2StackEntry* entries;
} b2StackAllocator;

b2StackAllocator* b2CreateStackAllocator(int32_t capacity)
{
	B2_ASSERT(capacity >= 0);
	b2StackAllocator* allocator = b2Alloc(sizeof(b2StackAllocator));
	allocator->capacity = capacity;
	allocator->data = b2Alloc(capacity);
	allocator->allocation = 0;
	allocator->maxAllocation = 0;
	allocator->index = 0;
	allocator->entries = b2CreateArray(sizeof(b2StackEntry), 32);
	return allocator;
}

void b2DestroyStackAllocator(b2StackAllocator* allocator)
{
	b2DestroyArray(allocator->entries, sizeof(b2StackEntry));
	b2Free(allocator->data, allocator->capacity);
	b2Free(allocator, sizeof(b2StackAllocator));
}

void* b2AllocateStackItem(b2StackAllocator* alloc, int32_t size, const char* name)
{
	int32_t size32 = ((size - 1) | 0x1F) + 1;

	b2StackEntry entry;
	entry.size = size32;
	entry.name = name;
	if (alloc->index + size32 > alloc->capacity)
	{
		// fall back to the heap (undesirable)
		entry.data = (char*)b2Alloc(size32);
		entry.usedMalloc = true;

		B2_ASSERT(((uintptr_t)entry.data & 0x1F) == 0);
	}
	else
	{
		entry.data = alloc->data + alloc->index;
		entry.usedMalloc = false;
		alloc->index += size32;

		B2_ASSERT(((uintptr_t)entry.data & 0x1F) == 0);
	}

	alloc->allocation += size32;
	if (alloc->allocation > alloc->maxAllocation)
	{
		alloc->maxAllocation = alloc->allocation;
	}

	b2Array_Push(alloc->entries, entry);
	return entry.data;
}

void b2FreeStackItem(b2StackAllocator* alloc, void* mem)
{
	int32_t entryCount = b2Array(alloc->entries).count;
	B2_ASSERT(entryCount > 0);
	b2StackEntry* entry = alloc->entries + (entryCount - 1);
	B2_ASSERT(mem == entry->data);
	if (entry->usedMalloc)
	{
		b2Free(mem, entry->size);
	}
	else
	{
		alloc->index -= entry->size;
	}
	alloc->allocation -= entry->size;
	b2Array_Pop(alloc->entries);
}

void b2GrowStack(b2StackAllocator* alloc)
{
	// Stack must not be in use
	B2_ASSERT(alloc->allocation == 0);

	if (alloc->maxAllocation > alloc->capacity)
	{
		b2Free(alloc->data, alloc->capacity);
		alloc->capacity = alloc->maxAllocation + alloc->maxAllocation / 2;
		alloc->data = b2Alloc(alloc->capacity);
	}
}

int32_t b2GetStackCapacity(b2StackAllocator* alloc)
{
	return alloc->capacity;
}

int32_t b2GetStackAllocation(b2StackAllocator* alloc)
{
	return alloc->allocation;
}

int32_t b2GetMaxStackAllocation(b2StackAllocator* alloc)
{
	return alloc->maxAllocation;
}
