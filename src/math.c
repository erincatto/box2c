// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "box2d/constants.h"
#include "box2d/math.h"

#include <float.h>

float b2_lengthUnitsPerMeter = 1.0f;
float b2_timeToSleep = 0.5f;

b2Version b2_version = { 3, 0, 0 };

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

b2Transform b2GetSweepTransform(const b2Sweep* sweep, float time)
{
	// https://fgiesen.wordpress.com/2012/08/15/linear-interpolation-past-present-and-future/
	b2Transform xf;
	xf.p = b2Add(b2MulSV(1.0f - time, sweep->c1), b2MulSV(time, sweep->c2));
	float angle = (1.0f - time) * sweep->a1 + time * sweep->a2;
	xf.q = b2MakeRot(angle);

	// Shift to origin
	xf.p = b2Sub(xf.p, b2RotateVector(xf.q, sweep->localCenter));
	return xf;
}
