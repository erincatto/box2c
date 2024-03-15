// SPDX-FileCopyrightText: 2024 Erin Catto
// SPDX-License-Identifier: MIT

#include "block_allocator.h"
#include "test_macros.h"

int BlockAllocatorTest(void)
{
	b2BlockAllocator alloc = b2CreateBlockAllocator();
	
	void* data1[64];
	int size1 = 16;
	for (int i = 0; i < 64; ++i)
	{
		data1[i] = b2AllocBlock(&alloc, size1);
	}

	void* data2[64];
	int size2 = 3008;
	for (int i = 0; i < 64; ++i)
	{
		data2[i] = b2AllocBlock(&alloc, size2);
	}

	void* data3[64];
	int size3 = 16384;
	for (int i = 0; i < 64; ++i)
	{
		data3[i] = b2AllocBlock(&alloc, size3);
	}

	for (int i = 63; i >= 0; --i)
	{
		b2FreeBlock(&alloc, data2[i], size2);
	}

	for (int i = 63; i >= 0; --i)
	{
		b2FreeBlock(&alloc, data1[i], size1);
	}

	ENSURE(b2ValidateBlockAllocator(&alloc));

	for (int i = 0; i < 64; ++i)
	{
		b2FreeBlock(&alloc, data3[i], size3);
	}

	ENSURE(b2ValidateBlockAllocator(&alloc));

	b2DestroyBlockAllocator(&alloc);


	return 0;
}

int AllocatorTest(void)
{
	RUN_SUBTEST(BlockAllocatorTest);

	return 0;
}
