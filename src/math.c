// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "core.h"

#include "box2d/constants.h"
#include "box2d/math.h"

#include <float.h>

bool b2IsValid(float a)
{
	if (isnan(a))
	{
		return false;
	}

	if (isinf(a))
	{
		return false;
	}

	return true;
}

bool b2IsValidVec2(b2Vec2 v)
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
		return b2Vec2_zero;
	}

	float invLength = 1.0f / length;
	b2Vec2 n = {invLength * v.x, invLength * v.y};
	return n;
}

b2Vec2 b2NormalizeChecked(b2Vec2 v)
{
	float length = b2Length(v);
	if (length < FLT_EPSILON)
	{
		B2_ASSERT(false);
		return b2Vec2_zero;
	}

	float invLength = 1.0f / length;
	b2Vec2 n = {invLength * v.x, invLength * v.y};
	return n;
}

b2Vec2 b2GetLengthAndNormalize(float* length, b2Vec2 v)
{
	*length = b2Length(v);
	if (*length < FLT_EPSILON)
	{
		return b2Vec2_zero;
	}

	float invLength = 1.0f / *length;
	b2Vec2 n = {invLength * v.x, invLength * v.y};
	return n;
}
