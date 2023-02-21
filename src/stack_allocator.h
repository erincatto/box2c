// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include <stdint.h>

typedef struct b2StackAllocator b2StackAllocator;

b2StackAllocator* b2CreateStackAllocator();
void b2DestroyStackAllocator(b2StackAllocator* allocator);

void* b2AllocateStackItem(b2StackAllocator* alloc, int32_t size);
void b2FreeStackItem(b2StackAllocator* alloc, void* mem);

