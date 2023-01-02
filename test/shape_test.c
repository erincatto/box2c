// SPDX-FileCopyrightText: 2022 Erin Catto
// SPDX-License-Identifier: MIT

#include "box2d/shapes.h"
#include "box2d/math.h"
#include "test_macros.h"

#include <float.h>

static b2CapsuleShape capsule = {{-1.0f, 0.0f}, {1.0f, 0.0f}, 1.0f};
static b2CircleShape circle = {{1.0f, 0.0f}, 1.0f};
static b2PolygonShape box;
static b2SegmentShape segment = {{0.0f, 1.0f}, {0.0f, -1.0f}};

static int ShapeMassTest()
{
	{
		b2MassData md = b2ComputeCircleMass(&circle, 1.0f);
		ENSURE_SMALL(md.mass - b2_pi, FLT_EPSILON);
		ENSURE(md.center.x == 1.0f && md.center.y == 0.0f);
		ENSURE_SMALL(md.I - 1.5f * b2_pi, FLT_EPSILON);
	}

	{
		b2MassData md = b2ComputePolygonMass(&box, 1.0f);
		ENSURE_SMALL(md.mass - 4.0f, FLT_EPSILON);
		ENSURE_SMALL(md.center.x, FLT_EPSILON);
		ENSURE_SMALL(md.center.y, FLT_EPSILON);
		ENSURE_SMALL(md.I - 8.0f / 3.0f, 2.0f * FLT_EPSILON);
	}

	return 0;
}

static int ShapeAABBTest()
{
	{
		b2AABB b = b2ComputeCircleAABB(&circle, b2Transform_identity);
		ENSURE_SMALL(b.lowerBound.x, FLT_EPSILON);
		ENSURE_SMALL(b.lowerBound.y + 1.0f, FLT_EPSILON);
		ENSURE_SMALL(b.upperBound.x - 2.0f, FLT_EPSILON);
		ENSURE_SMALL(b.upperBound.y - 1.0f, FLT_EPSILON);
	}

	{
		b2AABB b = b2ComputePolygonAABB(&box, b2Transform_identity);
		ENSURE_SMALL(b.lowerBound.x + 1.0f, FLT_EPSILON);
		ENSURE_SMALL(b.lowerBound.y + 1.0f, FLT_EPSILON);
		ENSURE_SMALL(b.upperBound.x - 1.0f, FLT_EPSILON);
		ENSURE_SMALL(b.upperBound.y - 1.0f, FLT_EPSILON);
	}

	{
		b2AABB b = b2ComputeSegmentAABB(&segment, b2Transform_identity);
		ENSURE_SMALL(b.lowerBound.x, FLT_EPSILON);
		ENSURE_SMALL(b.lowerBound.y + 1.0f, FLT_EPSILON);
		ENSURE_SMALL(b.upperBound.x, FLT_EPSILON);
		ENSURE_SMALL(b.upperBound.y - 1.0f, FLT_EPSILON);
	}

	return 0;
}

static int PointInShapeTest()
{
	b2Vec2 p1 = {0.5f, 0.5f};
	b2Vec2 p2 = {4.0f, -4.0f};
	
	{
		bool hit;
		hit = b2PointInCircle(p1, &circle, b2Transform_identity);
		ENSURE(hit == true);
		hit = b2PointInCircle(p2, &circle, b2Transform_identity);
		ENSURE(hit == false);
	}

	{
		bool hit;
		hit = b2PointInPolygon(p1, &box, b2Transform_identity);
		ENSURE(hit == true);
		hit = b2PointInPolygon(p2, &box, b2Transform_identity);
		ENSURE(hit == false);
	}

	return 0;
}

static int RayCastShapeTest()
{
	b2RayCastInput input = {{-4.0f, 0.0f}, {4.0f, 0.0f}, 1.0f};
	
	{
		b2RayCastOutput output = b2RayCastCircle(&input, &circle, b2Transform_identity);
		ENSURE(output.hit);
		ENSURE_SMALL(output.normal.x + 1.0f, FLT_EPSILON);
		ENSURE_SMALL(output.normal.y, FLT_EPSILON);
		ENSURE_SMALL(output.fraction - 0.5f, FLT_EPSILON);
	}

	{
		b2RayCastOutput output = b2RayCastPolygon(&input, &box, b2Transform_identity);
		ENSURE(output.hit);
		ENSURE_SMALL(output.normal.x + 1.0f, FLT_EPSILON);
		ENSURE_SMALL(output.normal.y, FLT_EPSILON);
		ENSURE_SMALL(output.fraction - 3.0f / 8.0f, FLT_EPSILON);
	}

	{
		b2RayCastOutput output = b2RayCastSegment(&input, &segment, b2Transform_identity);
		ENSURE(output.hit);
		ENSURE_SMALL(output.normal.x + 1.0f, FLT_EPSILON);
		ENSURE_SMALL(output.normal.y, FLT_EPSILON);
		ENSURE_SMALL(output.fraction - 0.5f, FLT_EPSILON);
	}

	return 0;
}

int ShapeTest()
{
	box = b2MakeBox(1.0f, 1.0f, b2Vec2_zero, 0.0f);

	RUN_SUBTEST(ShapeMassTest);
	RUN_SUBTEST(ShapeAABBTest);
	RUN_SUBTEST(PointInShapeTest);
	RUN_SUBTEST(RayCastShapeTest);

	return 0;
}
