// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "body.h"
#include "contact.h"
#include "island_builder.h"

#include <algorithm>

struct b2CompareContacts
{
	bool operator()(const b2ContactElement& element1, const b2ContactElement& element2) const
	{
		return element1.sortKey < element2.sortKey;
	}
};

extern "C" void b2SortContacts(b2ContactElement* elements, int32_t count)
{
	b2CompareContacts compare;
	std::sort(elements, elements + count, compare);
}
