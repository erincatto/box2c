// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include <stdint.h>

typedef struct b2ProxyPair
{
	struct b2ShapeProxy* proxy1;
	struct b2ShapeProxy* proxy2;
	uint64_t pairKey;
} b2ProxyPair;

typedef struct b2ProxyTable
{
	b2ProxyPair* pairs;
	int32_t capacity;
	int32_t count;
} b2ProxyTable;

b2ProxyTable b2CreateProxyTable(int32_t capacity);
void b2DestroyProxyTable(b2ProxyTable* table);

void b2AddProxyPair(b2ProxyTable* table, b2ShapeProxy* proxy1, b2ShapeProxy* proxy2);
b2ProxyPair* b2DestroyProxyPair(b2ProxyTable* table, const b2ShapeProxy* proxy1, const b2ShapeProxy* proxy2);
b2ProxyPair* b2FindProxyPair(const b2ProxyTable* table, const b2ShapeProxy* proxy1, const b2ShapeProxy* proxy2);
