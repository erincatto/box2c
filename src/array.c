// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "box2d/allocate.h"

#include "array.h"

#include <assert.h>
#include <string.h>

#if 0
b2Array b2CreateArray(int32_t elementSize, int32_t capacity)
{
	assert(0 < elementSize && elementSize < 1024);
	b2Array array = {0};
	array.capacity = capacity > 2 ? capacity : 2;
	array.count = 0;
	array.elementSize = elementSize;
	array.memory = (char*)b2Alloc(array.capacity * array.elementSize);
	return array;
}

void b2DestroyArray(b2Array* array)
{
	b2Free(array->memory);
	array->memory = NULL;
}

void b2GrowArray(b2Array* array)
{
	// grow by 50%
	array->capacity += (array->capacity >> 1);
	char* mem = (char*)b2Alloc(array->capacity * array->elementSize);
	memcpy(mem, array->memory, array->count * array->elementSize);
	b2Free(array->memory);
	array->memory = mem;
}

void b2PushElement(b2Array* array, void* element)
{
	if (array->count == array->capacity)
	{
		b2GrowArray(array);
	}

	char* dst = array->memory + array->count * array->elementSize;
	memcpy(dst, element, array->elementSize);
	++array->count;
}
#endif

void* b2CreateArray(int32_t elementSize, int32_t capacity)
{
	void* result = (b2ArrayHeader*)b2Alloc(sizeof(b2ArrayHeader) + elementSize * capacity) + 1;
	b2Array(result).count = 0;
	b2Array(result).capacity = capacity;
	return result;
}

void b2DestroyArray(void* a)
{
	b2Free(((b2ArrayHeader*)a) - 1);
}

void b2Array_Grow(void** a, int32_t elementSize)
{
	int32_t capacity = b2Array(*a).capacity;
	assert(capacity == b2Array(*a).count);

	// grow by 50%
	int32_t newCapacity = capacity + (capacity >> 1);
	newCapacity = newCapacity >= 2 ? newCapacity : 2;
	void* tmp = *a;
	*a = (b2ArrayHeader*)b2Alloc(sizeof(b2ArrayHeader) + elementSize * newCapacity) + 1;
	b2Array(*a).capacity = newCapacity;
	b2Array(*a).count = capacity;
	memcpy(*a, tmp, capacity * elementSize);
	b2DestroyArray(tmp);
}