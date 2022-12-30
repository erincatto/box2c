// SPDX-FileCopyrightText: 2022 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
#define B2_LITERAL(T) T
#else
#define B2_LITERAL(T) (T)
#endif

#define B2_ARRAY_COUNT(A) (sizeof(A) / sizeof(A[0]))

/// 2D vector
typedef struct b2Vec2
{
	float x, y;
} b2Vec2;

/// 2D rotation
typedef struct b2Rot
{
	/// Sine and cosine
	float s, c;
} b2Rot;

/// A 2D rigid transform
typedef struct b2Transform
{
	b2Vec2 p;
	b2Rot q;
} b2Transform;

/// Axis-aligned bounding box
typedef struct b2AABB
{
	b2Vec2 lowerBound;
	b2Vec2 upperBound;
} b2AABB;

/// Color for debug drawing. Each value has the range [0,1].
typedef struct b2Color
{
	float r, g, b, a;
} b2Color;
