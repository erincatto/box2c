// SPDX-FileCopyrightText: 2022 Erin Catto
// SPDX-License-Identifier: MIT

#include "box2d/math.h"
#include <float.h>

float b2_lengthUnitsPerMeter = 1.0f;

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
