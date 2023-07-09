// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include <stdint.h>

typedef struct b2StackAllocator b2StackAllocator;

b2StackAllocator* b2CreateStackAllocator(int32_t capacity);
void b2DestroyStackAllocator(b2StackAllocator* allocator);

void* b2AllocateStackItem(b2StackAllocator* alloc, int32_t size, const char* name);
void b2FreeStackItem(b2StackAllocator* alloc, void* mem);

int32_t b2GetStackCapacity(b2StackAllocator* alloc);
int32_t b2GetStackAllocation(b2StackAllocator* alloc);
int32_t b2GetMaxStackAllocation(b2StackAllocator* alloc);
