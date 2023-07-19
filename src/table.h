// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include <stdbool.h>
#include <stdint.h>

#define B2_SHAPE_PAIR_KEY(K1, K2) K1 < K2 ? (uint64_t)K1 << 32 | (uint64_t)K2 : (uint64_t)K2 << 32 | (uint64_t)K1

typedef struct b2SetItem
{
	uint64_t key;
	uint32_t hash;
} b2SetItem;

typedef struct b2Set
{
	b2SetItem* items;
	uint32_t capacity;
	uint32_t count;
} b2Set;

b2Set b2CreateSet(int32_t capacity);
void b2DestroySet(b2Set* set);

void b2AddKey(b2Set* set, uint64_t key);
void b2RemoveKey(b2Set* set, uint64_t key);

bool b2ContainsKey(const b2Set* set, uint64_t key);
