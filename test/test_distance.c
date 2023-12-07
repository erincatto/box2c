// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "box2d/constants.h"
#include "box2d/distance.h"
#include "box2d/math.h"
#include "test_macros.h"

#include <float.h>

static int SegmentDistanceTest()
{
	b2Vec2 p1 = {-1.0f, -1.0f};
	b2Vec2 q1 = {-1.0f, 1.0f};
	b2Vec2 p2 = {2.0f, 0.0f};
	b2Vec2 q2 = {1.0f, 0.0f};

	b2SegmentDistanceResult result = b2SegmentDistance(p1, q1, p2, q2);

	ENSURE_SMALL(result.fraction1 - 0.5f, FLT_EPSILON);
	ENSURE_SMALL(result.fraction2 - 1.0f, FLT_EPSILON);
	ENSURE_SMALL(result.closest1.x + 1.0f, FLT_EPSILON);
	ENSURE_SMALL(result.closest1.y, FLT_EPSILON);
	ENSURE_SMALL(result.closest2.x - 1.0f, FLT_EPSILON);
	ENSURE_SMALL(result.closest2.y, FLT_EPSILON);
	ENSURE_SMALL(result.distanceSquared - 4.0f, FLT_EPSILON);

	return 0;
}

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

	b2DistanceCache cache = {0};
	b2DistanceOutput output = b2ShapeDistance(&cache, &input);

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

	b2ShapeCastPairInput input;
	input.proxyA = b2MakeProxy(vas, B2_ARRAY_COUNT(vas), 0.0f);
	input.proxyB = b2MakeProxy(vbs, B2_ARRAY_COUNT(vbs), 0.0f);
	input.transformA = b2Transform_identity;
	input.transformB = b2Transform_identity;
	input.translationB = (b2Vec2){-2.0f, 0.0f};
	input.maxFraction = 1.0f;

	b2RayCastOutput output = b2ShapeCast(&input);

	ENSURE(output.hit);
	ENSURE_SMALL(output.fraction - 0.5f, b2_linearSlop);

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

	b2TOIOutput output = b2TimeOfImpact(&input);

	ENSURE(output.state = b2_toiStateHit);
	ENSURE_SMALL(output.t - 0.5f, b2_linearSlop);

	return 0;
}

int DistanceTest()
{
	RUN_SUBTEST(SegmentDistanceTest);
	RUN_SUBTEST(ShapeDistanceTest);
	RUN_SUBTEST(ShapeCastTest);
	RUN_SUBTEST(TimeOfImpactTest);

	return 0;
}
