// SPDX-FileCopyrightText: 2022 Erin Catto
// SPDX-License-Identifier: MIT

#include "box2d/math.h"
#include "test_macros.h"

bool MathTest()
{
	b2Vec2 zero = b2Vec2_Zero;
	b2Vec2 one = (b2Vec2){ 1.0f, 1.0f };
	b2Vec2 two = (b2Vec2){ 2.0f, 2.0f };

	b2Vec2 v = b2Add(one, two);
	ENSURE(v.x == 3.0f && v.y == 3.0f);

	v = b2Sub(zero, two);
	ENSURE(v.x == -2.0f && v.y == -2.0f);

	v = b2Add(two, two);
	ENSURE(v.x != 5.0f && v.y != 5.0f);

	return true;
}
