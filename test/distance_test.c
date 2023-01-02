// SPDX-FileCopyrightText: 2022 Erin Catto
// SPDX-License-Identifier: MIT

#include "box2d/constants.h"
#include "box2d/distance.h"
#include "box2d/math.h"
#include "test_macros.h"

#include <float.h>

static int ShapeDistanceTest()
{
	b2Vec2 vas[] = {
		(b2Vec2){-1.0f, -1.0f},
		(b2Vec2){1.0f, -1.0f},
		(b2Vec2){1.0f, 1.0f},
		(b2Vec2){-1.0f, 1.0f}};

	b2Vec2 vbs[] = {
		(b2Vec2){2.0f, -1.0f},
		(b2Vec2){2.0f, 1.0f},
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

static int ShapeCastTest()
{
	b2Vec2 vas[] = {
		(b2Vec2){-1.0f, -1.0f},
		(b2Vec2){1.0f, -1.0f},
		(b2Vec2){1.0f, 1.0f},
		(b2Vec2){-1.0f, 1.0f}};

	b2Vec2 vbs[] = {
		(b2Vec2){2.0f, -1.0f},
		(b2Vec2){2.0f, 1.0f},
	};

	b2ShapeCastInput input;
	input.proxyA = b2MakeProxy(vas, B2_ARRAY_COUNT(vas), 0.0f);
	input.proxyB = b2MakeProxy(vbs, B2_ARRAY_COUNT(vbs), 0.0f);
	input.transformA = b2Transform_identity;
	input.transformB = b2Transform_identity;
	input.translationB = (b2Vec2){-2.0f, 0.0f};

	b2ShapeCastOutput output;

	bool hit = b2ShapeCast(&output, &input);

	ENSURE(hit);
	ENSURE_SMALL(output.lambda - 0.5f, b2_linearSlop);

	return 0;
}

static int TimeOfImpactTest()
{
	b2Vec2 vas[] = {
		(b2Vec2){-1.0f, -1.0f},
		(b2Vec2){1.0f, -1.0f},
		(b2Vec2){1.0f, 1.0f},
		(b2Vec2){-1.0f, 1.0f}};

	b2Vec2 vbs[] = {
		(b2Vec2){2.0f, -1.0f},
		(b2Vec2){2.0f, 1.0f},
	};

	b2TOIInput input;
	input.proxyA = b2MakeProxy(vas, B2_ARRAY_COUNT(vas), 0.0f);
	input.proxyB = b2MakeProxy(vbs, B2_ARRAY_COUNT(vbs), 0.0f);
	input.sweepA = (b2Sweep){b2Vec2_zero, b2Vec2_zero, b2Vec2_zero, 0.0f, 0.0f};
	input.sweepB = (b2Sweep){b2Vec2_zero, b2Vec2_zero, (b2Vec2){-2.0f, 0.0f}, 0.0f, 0.0f};
	input.tMax = 1.0f;

	b2TOIOutput output;

	b2TimeOfImpact(&output, &input);

	ENSURE(output.state = b2_toiStateHit);
	ENSURE_SMALL(output.t - 0.5f, b2_linearSlop);

	return 0;
}

int DistanceTest()
{
	RUN_SUBTEST(ShapeDistanceTest);
	RUN_SUBTEST(ShapeCastTest);
	RUN_SUBTEST(TimeOfImpactTest);

	return 0;
}
