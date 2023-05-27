// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "body.h"
#include "contact.h"

#include <algorithm>

struct b2CompareContacts
{
	bool operator()(int32_t indexA, int32_t indexB)
	{
		b2Contact* contactA = contacts[indexA];
		b2Contact* contactB = contacts[indexB];

		int32_t minBodyA = B2_MIN(contactA->edgeA.otherBodyIndex, contactA->edgeB.otherBodyIndex);
		int32_t minBodyB = B2_MIN(contactB->edgeA.otherBodyIndex, contactB->edgeB.otherBodyIndex);

		if (minBodyA != minBodyB)
		{
			return minBodyA < minBodyB;
		}

		int32_t maxBodyA = B2_MAX(contactA->edgeA.otherBodyIndex, contactA->edgeB.otherBodyIndex);
		int32_t maxBodyB = B2_MAX(contactB->edgeA.otherBodyIndex, contactB->edgeB.otherBodyIndex);

		if (maxBodyA != maxBodyB)
		{
			return maxBodyA < maxBodyB;
		}

		return contactA->index < contactB->index;
	}

	b2Contact** contacts;
};

extern "C" void b2SortContacts(b2Contact** contacts, int32_t* indices, int32_t count)
{
	b2CompareContacts compare;
	compare.contacts = contacts;

	std::sort(indices, indices + count, compare);
}
