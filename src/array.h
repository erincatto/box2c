// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include <stdint.h>

#if 0
typedef struct b2Array
{
	char* memory;
	int32_t elementSize;
	int32_t capacity;
	int32_t count;
} b2Array;

b2Array b2CreateArray(int32_t elementSize, int32_t capacity);
void b2DestroyArray(b2Array* array);
void b2GrowArray(b2Array* array);
void b2PushElement(b2Array* array, void* element);
#endif

typedef struct b2ArrayHeader
{
	int32_t count;
	int32_t capacity;
} b2ArrayHeader;

#define b2Array(a) ((b2ArrayHeader*)a)[-1]

void* b2CreateArray(int32_t elementSize, int32_t capacity);
void b2DestroyArray(void* a);
void b2Array_Grow(void** a, int32_t elementSize);

#define b2Array_Clear(a) \
	b2Array(a).count = 0

#define b2Array_Push(a, element) \
	if (b2Array(a).count == b2Array(a).capacity) b2Array_Grow((void**)&a, sizeof(element)); \
	a[b2Array(a).count++] = element

#define b2Array_Remove(a, index) \
	assert(0 <= index && index < b2Array(a).count); \
	a[index] = a[b2Array(a).count - 1]; \
	b2Array(a).count--
