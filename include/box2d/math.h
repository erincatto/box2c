// SPDX-FileCopyrightText: 2022 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "types.h"

#include <math.h>

static const b2Vec2 b2Vec2_Zero = { 0.0f, 0.0f };

/// Convert this vector into a unit vector. Returns the length.
static b2Vec2 b2Normalize(b2Vec2 v);

/// Get the length of this vector (the norm).
static inline float b2Length(b2Vec2 v)
{
	return sqrtf(v.x * v.x + v.y * v.y);
}

/// Vector addition
static inline b2Vec2 b2Add(b2Vec2 a, b2Vec2 b)
{
	return B2_LITERAL(b2Vec2){ a.x + b.x, a.y + b.y };
}

/// Vector subtraction
static inline b2Vec2 b2Sub(b2Vec2 a, b2Vec2 b)
{
	return B2_LITERAL(b2Vec2){ a.x - b.x, a.y - b.y };
}
