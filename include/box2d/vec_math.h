// SPDX-FileCopyrightText: 2022 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "types.h"

#include <math.h>


#define B2_MIN(A,B) (A) < (B) ? (A) : (B)
#define B2_MAX(A,B) (A) > (B) ? (A) : (B)
#define B2_ABS(A) (A) > 0.0f ? (A) : -(A)
#define B2_CLAMP(A,B,C) B2_MIN(B2_MAX(A, B), C)

static const b2Vec2 b2Vec2_zero = { 0.0f, 0.0f };
static const b2Rot b2Rot_identity = { 0.0f, 1.0f };
static const b2Transform b2Transform_identity = { { 0.0f, 0.0f }, {0.0f, 1.0f} };

bool b2Vec2_IsValid(b2Vec2 v);

/// Vector dot product
static inline float b2Dot(b2Vec2 a, b2Vec2 b)
{
	return a.x * b.x + a.y * b.y;
}

/// Vector cross product. In 2D this yields a scalar.
static inline float b2Cross(b2Vec2 a, b2Vec2 b)
{
	return a.x * b.y - a.y * b.x;
}

/// Perform the cross product on a vector and a scalar. In 2D this produces
/// a vector.
static inline b2Vec2 b2CrossVS(b2Vec2 v, float s)
{
	return B2_LITERAL(b2Vec2) { s* v.y, -s * v.x };
}

/// Perform the cross product on a scalar and a vector. In 2D this produces
/// a vector.
static inline b2Vec2 b2CrossSV(float s, b2Vec2 v)
{
	return B2_LITERAL(b2Vec2) { -s * v.y, s* v.x };
}

/// Vector addition
static inline b2Vec2 b2Add(b2Vec2 a, b2Vec2 b)
{
	return B2_LITERAL(b2Vec2) { a.x + b.x, a.y + b.y };
}

/// Vector subtraction
static inline b2Vec2 b2Sub(b2Vec2 a, b2Vec2 b)
{
	return B2_LITERAL(b2Vec2) { a.x - b.x, a.y - b.y };
}

/// Vector subtraction
static inline b2Vec2 b2Neg(b2Vec2 a)
{
	return B2_LITERAL(b2Vec2) { -a.x, -a.y };
}

/// Vector linear interpolation
static inline b2Vec2 b2Lerp(b2Vec2 a, b2Vec2 b, float t)
{
	return B2_LITERAL(b2Vec2) { a.x + t * (b.x - a.x), a.y + t * (b.y - a.y) };
}

/// Component-wise multiplication
static inline b2Vec2 b2Mul(b2Vec2 a, b2Vec2 b)
{
	return B2_LITERAL(b2Vec2) { a.x * b.x, a.y * b.y };
}

/// Multiply a scalar and vector
static inline b2Vec2 b2MulSV(float s, b2Vec2 v)
{
	return B2_LITERAL(b2Vec2) { s * v.y, s * v.x };
}

/// a + s * b
static inline b2Vec2 b2MulAdd(b2Vec2 a, float s, b2Vec2 b)
{
	return B2_LITERAL(b2Vec2) { a.x + s * b.x, a.y + s * b.y };
}

/// Component-wise absolute vector
static inline b2Vec2 b2Abs(b2Vec2 a)
{
	b2Vec2 b;
	b.x = B2_ABS(a.x);
	b.y = B2_ABS(a.y);
	return b;
}

/// Component-wise absolute vector
static inline b2Vec2 b2Min(b2Vec2 a, b2Vec2 b)
{
	b2Vec2 c;
	c.x = B2_MIN(a.x, b.x);
	c.y = B2_MIN(a.y, b.y);
	return c;
}

/// Component-wise absolute vector
static inline b2Vec2 b2Max(b2Vec2 a, b2Vec2 b)
{
	b2Vec2 c;
	c.x = B2_MAX(a.x, b.x);
	c.y = B2_MAX(a.y, b.y);
	return c;
}

/// Convert this vector into a unit vector
b2Vec2 b2Normalize(b2Vec2 v);

/// Get the length of this vector (the norm).
static inline float b2Length(b2Vec2 v)
{
	return sqrtf(v.x * v.x + v.y * v.y);
}

static inline float b2Distance(b2Vec2 a, b2Vec2 b)
{
	float dx = b.x - a.x;
	float dy = b.y - a.y;
	return sqrtf(dx * dx + dy * dy);
}

/// Get the length of this vector (the norm).
static inline float b2DistanceSquared(b2Vec2 a, b2Vec2 b)
{
	b2Vec2 c = { b.x - a.x, b.y - a.y };
	return c.x * c.x + c.y * c.y;
}

/// Set using an angle in radians.
static inline b2Rot b2Rot_Set(float angle)
{
	b2Rot q = { sinf(angle), cosf(angle) };
	return q;
}

/// Get the angle in radians
static inline float b2Rot_GetAngle(b2Rot q)
{
	return atan2f(q.s, q.c);
}

/// Get the x-axis
static inline b2Vec2 b2Rot_GetXAxis(b2Rot q)
{
	b2Vec2 v = { q.c, q.s };
	return v;
}

/// Get the y-axis
static inline b2Vec2 b2Rot_GetYAxis(b2Rot q)
{
	b2Vec2 v = { -q.s, q.c };
	return v;
}

/// Rotate a vector
static inline b2Vec2 b2RotVec(b2Rot q, b2Vec2 v)
{
	return B2_LITERAL(b2Vec2) { q.c* v.x - q.s * v.y, q.s* v.x + q.c * v.y };
}

/// Inverse rotate a vector
static inline b2Vec2 b2InvRotVec(b2Rot q, b2Vec2 v)
{
	return B2_LITERAL(b2Vec2) { q.c* v.x + q.s * v.y, -q.s * v.x + q.c * v.y };
}

static inline b2Vec2 b2TransformPoint(b2Transform xf, const b2Vec2 p)
{
	float x = (xf.q.c * p.x - xf.q.s * p.y) + xf.p.x;
	float y = (xf.q.s * p.x + xf.q.c * p.y) + xf.p.y;

	return B2_LITERAL(b2Vec2) { x, y };
}
