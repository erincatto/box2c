// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "box2d/math.h"
#include "test_macros.h"

#include <float.h>

int MathTest()
{
	b2Vec2 zero = b2Vec2_zero;
	b2Vec2 one = {1.0f, 1.0f};
	b2Vec2 two = {2.0f, 2.0f};

	b2Vec2 v = b2Add(one, two);
	ENSURE(v.x == 3.0f && v.y == 3.0f);

	v = b2Sub(zero, two);
	ENSURE(v.x == -2.0f && v.y == -2.0f);

	v = b2Add(two, two);
	ENSURE(v.x != 5.0f && v.y != 5.0f);

	b2Transform xf1 = {{-2.0f, 3.0f}, b2MakeRot(1.0f)};
	b2Transform xf2 = {{1.0f, 0.0f}, b2MakeRot(-2.0f)};

	b2Transform xf = b2MulTransforms(xf2, xf1);

	v = b2TransformPoint(xf2, b2TransformPoint(xf1, two));

	b2Vec2 u = b2TransformPoint(xf, two);

	ENSURE_SMALL(u.x - v.x, FLT_EPSILON);
	ENSURE_SMALL(u.y - v.y, FLT_EPSILON);

	v = b2TransformPoint(xf1, two);
	v = b2InvTransformPoint(xf1, v);

	ENSURE_SMALL(v.x - two.x, 2.0f * FLT_EPSILON);
	ENSURE_SMALL(v.y - two.y, 2.0f * FLT_EPSILON);

	return 0;
}
