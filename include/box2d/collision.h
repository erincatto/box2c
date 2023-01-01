// SPDX-FileCopyrightText: 2022 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "types.h"
#include "vec_math.h"

#ifdef __cplusplus
extern "C"
{
#endif

/// Ray-cast input data. The ray extends from p1 to p1 + maxFraction * (p2 - p1).
typedef struct b2RayCastInput
{
	b2Vec2 p1, p2;
	float maxFraction;
} b2RayCastInput;

/// Ray-cast output data. The ray hits at p1 + fraction * (p2 - p1), where p1 and p2
/// come from b2RayCastInput.
typedef struct b2RayCastOutput
{
	b2Vec2 normal;
	float fraction;
	bool hit;
} b2RayCastOutput;

/// Verify that the bounds are sorted.
bool b2AABB_IsValid(b2AABB a);

/// Ray cast an AABB
bool b2AABB_RayCast(b2AABB a, b2RayCastOutput* output, const b2RayCastInput* input);

/// Get the center of the AABB.
static inline b2Vec2 b2AABB_Center(b2AABB a)
{
	b2Vec2 b = {0.5f * (a.lowerBound.x + a.upperBound.x), 0.5f * (a.lowerBound.y + a.upperBound.y)};
	return b;
}

/// Get the extents of the AABB (half-widths).
static inline b2Vec2 b2AABB_Extents(b2AABB a)
{
	b2Vec2 b = {0.5f * (a.upperBound.x - a.lowerBound.x), 0.5f * (a.upperBound.y + a.lowerBound.y)};
	return b;
}

/// Get the perimeter length
static inline float b2AABB_Perimeter(b2AABB a)
{
	float wx = a.upperBound.x - a.lowerBound.x;
	float wy = a.upperBound.y - a.lowerBound.y;
	return 2.0f * (wx + wy);
}

/// Union of two AABBs
static inline b2AABB b2AABB_Union(b2AABB a, b2AABB b)
{
	b2AABB c;
	c.lowerBound.x = B2_MIN(a.lowerBound.x, b.lowerBound.x);
	c.lowerBound.y = B2_MIN(a.lowerBound.y, b.lowerBound.y);
	c.upperBound.x = B2_MAX(a.upperBound.x, b.upperBound.x);
	c.upperBound.y = B2_MAX(a.upperBound.y, b.upperBound.y);
	return c;
}

/// Does a fully contain b
static inline bool b2AABB_Contains(b2AABB a, b2AABB b)
{
	bool s = true;
	s = s && a.lowerBound.x <= b.lowerBound.x;
	s = s && a.lowerBound.y <= b.lowerBound.y;
	s = s && b.upperBound.x <= a.upperBound.x;
	s = s && b.upperBound.y <= a.upperBound.y;
	return s;
}

/// Do a and b overlap
static inline bool b2AABB_Overlaps(b2AABB a, b2AABB b)
{
	b2Vec2 d1 = {b.lowerBound.x - a.upperBound.x, b.lowerBound.y - a.upperBound.y};
	b2Vec2 d2 = {a.lowerBound.x - b.upperBound.x, a.lowerBound.y - b.upperBound.y};

	if (d1.x > 0.0f || d1.y > 0.0f)
		return false;

	if (d2.x > 0.0f || d2.y > 0.0f)
		return false;

	return true;
}

#ifdef __cplusplus
}
#endif
