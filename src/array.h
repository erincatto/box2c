// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "core.h"

#include <stdint.h>

typedef struct b2ArrayHeader
{
	int32_t count;
	int32_t capacity;
} b2ArrayHeader;

#define b2Array(a) ((b2ArrayHeader*)(a))[-1]

void* b2CreateArray(int32_t elementSize, int32_t capacity);
void b2DestroyArray(void* a, int32_t elementSize);
void b2Array_Grow(void** a, int32_t elementSize);

#define b2Array_Check(a, index) B2_ASSERT(0 <= index && index < b2Array(a).count)

#define b2Array_Clear(a) b2Array(a).count = 0

#define b2Array_Push(a, element) \
	if (b2Array(a).count == b2Array(a).capacity) b2Array_Grow((void**)&a, sizeof(element)); \
	B2_ASSERT(b2Array(a).count < b2Array(a).capacity); \
	a[b2Array(a).count++] = element

#define b2Array_RemoveSwap(a, index) \
	B2_ASSERT(0 <= index && index < b2Array(a).count); \
	a[index] = a[b2Array(a).count - 1]; \
	b2Array(a).count--

#define b2Array_Last(a) (a)[b2Array(a).count - 1];

#define b2Array_Pop(a) \
	B2_ASSERT(0 < b2Array(a).count); \
	b2Array(a).count--

#define b2GetArrayCount(a) (b2Array(a).count)
#define b2GetArrayCapacity(a) (b2Array(a).capacity)
