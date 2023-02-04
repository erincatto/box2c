// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "box2d/allocate.h"

#include "stack_allocator.h"

#include <assert.h>

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
