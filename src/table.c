// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "allocate.h"
#include "shape.h"
#include "table.h"

#include <assert.h>
#include <string.h>

b2ProxyTable b2CreateProxyTable(int32_t capacity)
{
	b2ProxyTable table = {0};
	table.capacity = B2_MAX(capacity, 16);
	table.pairs = b2Alloc(capacity * sizeof(b2ProxyPair));
	memset(table.pairs, 0, capacity * sizeof(b2ProxyPair));
	table.count = 0;
}

void b2DestroyProxyTable(b2ProxyTable* table)
{
	b2Free(table->pairs, table->capacity * sizeof(b2ProxyPair));
	table->pairs = NULL;
	table->count = 0;
	table->capacity = 0;
}

// https://lemire.me/blog/2018/08/15/fast-strongly-universal-64-bit-hashing-everywhere/
static uint64_t b2PairHash(b2ProxyPair* pair)
{
	uint64_t key1 = pair->proxy1->proxyKey;
	uint64_t key2 = pair->proxy2->proxyKey;

	uint64_t h = key1 << 32 | key2;
	h ^= h >> 33;
	h *= 0xff51afd7ed558ccdL;
	h ^= h >> 33;
	h *= 0xc4ceb9fe1a85ec53L;
	h ^= h >> 33;

	// cache it
	pair->pairKey = h;
	return h;
}

static void b2AddProxyPairStatic(b2ProxyTable* table, b2ProxyPair* pair)
{
	uint32_t capacity = table->capacity;
	uint64_t hash = pair->pairKey;
	uint32_t index = (hash & (capacity - 1));
}

static void b2GrowTable(b2ProxyTable* table)
{
	int32_t oldCapacity = table->capacity;
	int32_t newCapacity = 2 * oldCapacity;
	b2ProxyPair* newPairs = b2Alloc(newCapacity * sizeof(b2ProxyPair));
	memset(newPairs, 0, newCapacity * sizeof(b2ProxyPair));

	b2ProxyPair* oldPairs = table->pairs;
	for (int32_t i = 0; i < oldCapacity; ++i)
	{
		b2ProxyPair* pair = oldPairs + i;
		if (pair->pairKey == 0)
		{
			continue;
		}



	}
}

void b2AddProxyPair(b2ProxyTable* table, b2ShapeProxy* proxy1, b2ShapeProxy* proxy2)
{
	if (2 * table->count >= table->capacity)
	{
		b2GrowTable(table);
	}

}

b2ProxyPair* b2DestroyProxyPair(b2ProxyTable* table, const b2ShapeProxy* proxy1, const b2ShapeProxy* proxy2)
{

}

b2ProxyPair* b2FindProxyPair(const b2ProxyTable* table, const b2ShapeProxy* proxy1, const b2ShapeProxy* proxy2)
{

}
