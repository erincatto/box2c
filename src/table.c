// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "table.h"

#include "allocate.h"
#include "core.h"

#include "box2d/types.h"

#include <stdbool.h>
#include <string.h>

#if _DEBUG
int32_t g_probeCount;
#endif

static inline bool b2IsPowerOf2(uint32_t x)
{
	return (x & (x - 1)) == 0;
}

static inline uint32_t b2RoundUpPowerOf2(uint32_t x)
{
	if (b2IsPowerOf2(x))
	{
		return x;
	}

	// Hacker's Delight p48
	// Can also use ctz, but perf is not needed in this use case.
	x -= 1;
	x |= x >> 1;
	x |= x >> 2;
	x |= x >> 4;
	x |= x >> 8;
	x |= x >> 16;

	return x + 1;
}

b2HashSet b2CreateSet(int32_t capacity)
{
	b2HashSet set = {0};

	// Capacity must be a power of 2
	if (capacity > 16)
	{
		set.capacity = b2RoundUpPowerOf2(capacity);
	}
	else
	{
		set.capacity = 16;
	}

	set.count = 0;
	set.items = b2Alloc(capacity * sizeof(b2SetItem));
	memset(set.items, 0, capacity * sizeof(b2SetItem));

	return set;
}

void b2DestroySet(b2HashSet* set)
{
	b2Free(set->items, set->capacity * sizeof(b2SetItem));
	set->items = NULL;
	set->count = 0;
	set->capacity = 0;
}

void b2ClearSet(b2HashSet* set)
{
	set->count = 0;
	memset(set->items, 0, set->capacity * sizeof(b2SetItem));
}

// I need a good hash because the keys are built from pairs of increasing integers.
// A simple hash like hash = (integer1 XOR integer2) has many collisions.
// https://lemire.me/blog/2018/08/15/fast-strongly-universal-64-bit-hashing-everywhere/
// https://preshing.com/20130107/this-hash-set-is-faster-than-a-judy-array/
// TODO_ERIN try: https://www.jandrewrogers.com/2019/02/12/fast-perfect-hashing/
static inline uint32_t b2KeyHash(uint64_t key)
{
	uint64_t h = key;
	h ^= h >> 33;
	h *= 0xff51afd7ed558ccdL;
	h ^= h >> 33;
	h *= 0xc4ceb9fe1a85ec53L;
	h ^= h >> 33;

	return (uint32_t)h;
}

#if _DEBUG
int32_t g_probeCount;
#endif

int32_t b2FindSlot(const b2HashSet* set, uint64_t key, uint32_t hash)
{
	uint32_t capacity = set->capacity;
	int32_t index = hash & (capacity - 1);
	const b2SetItem* items = set->items;
	while (items[index].hash != 0 && items[index].key != key)
	{
#if _DEBUG
		g_probeCount += 1;
#endif
		index = (index + 1) & (capacity - 1);
	}

	return index;
}

static void b2AddKeyHaveCapacity(b2HashSet* set, uint64_t key, uint32_t hash)
{
	int32_t index = b2FindSlot(set, key, hash);
	b2SetItem* items = set->items;
	B2_ASSERT(items[index].hash == 0);

	items[index].key = key;
	items[index].hash = hash;
	set->count += 1;
}

static void b2GrowTable(b2HashSet* set)
{
	uint32_t oldCount = set->count;
	B2_MAYBE_UNUSED(oldCount);

	uint32_t oldCapacity = set->capacity;
	b2SetItem* oldItems = set->items;

	set->count = 0;
	// Capacity must be a power of 2
	set->capacity = 2 * oldCapacity;
	set->items = b2Alloc(set->capacity * sizeof(b2SetItem));
	memset(set->items, 0, set->capacity * sizeof(b2SetItem));

	// Transfer items into new array
	for (uint32_t i = 0; i < oldCapacity; ++i)
	{
		b2SetItem* item = oldItems + i;
		if (item->hash == 0)
		{
			// this item was empty
			continue;
		}

		b2AddKeyHaveCapacity(set, item->key, item->hash);
	}

	B2_ASSERT(set->count == oldCount);

	b2Free(oldItems, oldCapacity * sizeof(b2SetItem));
}

bool b2ContainsKey(const b2HashSet* set, uint64_t key)
{
	// key of zero is a sentinel
	B2_ASSERT(key != 0);
	uint32_t hash = b2KeyHash(key);
	int32_t index = b2FindSlot(set, key, hash);
	return set->items[index].key == key;
}

bool b2AddKey(b2HashSet* set, uint64_t key)
{
	// key of zero is a sentinel
	B2_ASSERT(key != 0);

	uint32_t hash = b2KeyHash(key);
	B2_ASSERT(hash != 0);

	int32_t index = b2FindSlot(set, key, hash);
	if (set->items[index].hash != 0)
	{
		// Already in set
		B2_ASSERT(set->items[index].hash == hash && set->items[index].key == key);
		return true;
	}

	if (2 * set->count >= set->capacity)
	{
		b2GrowTable(set);
	}

	b2AddKeyHaveCapacity(set, key, hash);
	return false;
}

// See https://en.wikipedia.org/wiki/Open_addressing
bool b2RemoveKey(b2HashSet* set, uint64_t key)
{
	uint32_t hash = b2KeyHash(key);
	int32_t i = b2FindSlot(set, key, hash);
	b2SetItem* items = set->items;
	if (items[i].hash == 0)
	{
		// Not in set
		return false;
	}

	// Mark item i as unoccupied
	items[i].key = 0;
	items[i].hash = 0;

	B2_ASSERT(set->count > 0);
	set->count -= 1;

	// Attempt to fill item i
	int32_t j = i;
	uint32_t capacity = set->capacity;
	for (;;)
	{
		j = (j + 1) & (capacity - 1);
		if (items[j].hash == 0)
		{
			break;
		}

		// k is the first item for the hash of j
		int32_t k = items[j].hash & (capacity - 1);

		// determine if k lies cyclically in (i,j]
		// i <= j: | i..k..j |
		// i > j: |.k..j  i....| or |....j     i..k.|
		if (i <= j)
		{
			if (i < k && k <= j)
			{
				continue;
			}
		}
		else
		{
			if (i < k || k <= j)
			{
				continue;
			}
		}

		// Move j into i
		items[i] = items[j];

		// Mark item j as unoccupied
		items[j].key = 0;
		items[j].hash = 0;

		i = j;
	}

	return true;
}
