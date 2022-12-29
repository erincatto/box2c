// SPDX-FileCopyrightText: 2022 Erin Catto
// SPDX-License-Identifier: MIT

#include "box2d/constants.h"
#include "box2d/distance.h"
#include "box2d/vec_math.h"
#include "test_macros.h"

#include <float.h>

int ShapeDistanceTest()
{
	b2Vec2 vas[] = {
		(b2Vec2) { -1.0f, -1.0f },
		(b2Vec2) { 1.0f, -1.0f },
		(b2Vec2) { 1.0f, 1.0f },
		(b2Vec2) { -1.0f, 1.0f }
	};

	b2Vec2 vbs[] = {
		(b2Vec2) { 2.0f, -1.0f },
		(b2Vec2) { 2.0f, 1.0f },
	};

	b2DistanceInput input;
	input.proxyA = b2MakeProxy(vas, B2_ARRAY_COUNT(vas), 0.0f);
	input.proxyB = b2MakeProxy(vbs, B2_ARRAY_COUNT(vbs), 0.0f);
	input.transformA = b2Transform_identity;
	input.transformB = b2Transform_identity;
	input.useRadii = false;

	b2DistanceCache cache;
	cache.count = 0;
	b2DistanceOutput output;

	b2ShapeDistance(&output, &cache, &input);

	ENSURE_SMALL(output.distance - 1.0f, FLT_EPSILON);

	return 0;
}

int ShapeCastTest()
{
	b2Vec2 vas[] = {
		(b2Vec2) { -1.0f, -1.0f },
		(b2Vec2) { 1.0f, -1.0f },
		(b2Vec2) { 1.0f, 1.0f },
		(b2Vec2) { -1.0f, 1.0f }
	};

	b2Vec2 vbs[] = {
		(b2Vec2) { 2.0f, -1.0f },
		(b2Vec2) { 2.0f, 1.0f },
	};

	b2ShapeCastInput input;
	input.proxyA = b2MakeProxy(vas, B2_ARRAY_COUNT(vas), 0.0f);
	input.proxyB = b2MakeProxy(vbs, B2_ARRAY_COUNT(vbs), 0.0f);
	input.transformA = b2Transform_identity;
	input.transformB = b2Transform_identity;
	input.translationB = (b2Vec2){ -2.0f, 0.0f };

	b2ShapeCastOutput output;

	bool hit = b2ShapeCast(&output, &input);

	ENSURE(hit);
	ENSURE_SMALL(output.lambda - 0.5f, b2_linearSlop);

	return 0;
}

int DistanceTest()
{
	RUN_TEST(ShapeDistanceTest);
	RUN_TEST(ShapeCastTest);

	return 0;
}
