// SPDX-FileCopyrightText: 2022 Erin Catto
// SPDX-License-Identifier: MIT

#include "box2d/collision.h"
#include "box2d/vec_math.h"
#include "test_macros.h"

static int AABBTest()
{
	b2AABB a;
	a.lowerBound = (b2Vec2){ -1.0f, -1.0f };
	a.upperBound = (b2Vec2){ -2.0f, -2.0f };

	ENSURE(b2AABB_IsValid(a) == false);

	a.upperBound = (b2Vec2){ 1.0f, 1.0f };
	ENSURE(b2AABB_IsValid(a) == true);

	b2AABB b = { .lowerBound = {2.0f, 2.0f}, .upperBound = {4.0f, 4.0f} };
	ENSURE(b2AABB_Overlaps(a, b) == false);
	ENSURE(b2AABB_Contains(a, b) == false);

	b2RayCastInput input;
	input.p1 = (b2Vec2){ -2.0f, 0.0f };
	input.p2 = (b2Vec2){ 2.0f, 0.0f };
	input.maxFraction = 1.0f;

	b2RayCastOutput output;
	bool hit = b2AABB_RayCast(a, &output, &input);
	ENSURE(hit == true);
	ENSURE(0.1f < output.fraction && output.fraction < 0.9f);

	return 0;
}

int CollisionTest()
{
	RUN_SUBTEST(AABBTest);

	return 0;
}
