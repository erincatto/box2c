// SPDX-FileCopyrightText: 2022 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "box2d/types.h"

typedef struct b2BlockAllocator b2BlockAllocator;

/// Create an allocator suitable for allocating and freeing small objects quickly.
/// Does not return memory to the heap.
b2BlockAllocator* b2CreateBlockAllocator();

/// Destroy a block alloctor instance
void b2DestroyBlockAllocator(b2BlockAllocator* allocator);

/// Allocate memory. This will use b2Alloc if the size is larger than b2_maxBlockSize.
void* b2BlockAllocate(b2BlockAllocator* allocator, int32_t size);

/// Free memory. This will use b2Free if the size is larger than b2_maxBlockSize.
void b2BlockFree(b2BlockAllocator* allocator,void* p, int32_t size);
