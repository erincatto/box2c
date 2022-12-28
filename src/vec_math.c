// SPDX-FileCopyrightText: 2022 Erin Catto
// SPDX-License-Identifier: MIT

#include "box2d/constants.h"
#include "box2d/vec_math.h"

#include <float.h>

float b2_lengthUnitsPerMeter = 1.0f;
b2Version b2_version = { 3, 0, 0 };

bool b2Vec2_IsValid(b2Vec2 v)
{
	if (isnan(v.x) || isnan(v.y))
	{
		return false;
	}

	if (isinf(v.x) || isinf(v.y))
	{
		return false;
	}

	return true;
}

b2Vec2 b2Normalize(b2Vec2 v)
{
	float length = b2Length(v);
	if (length < FLT_EPSILON)
	{
		return b2Vec2_Zero;
	}

	b2Vec2 n = v;
	float invLength = 1.0f / length;
	n.x *= invLength;
	n.y *= invLength;

	return n;
}

