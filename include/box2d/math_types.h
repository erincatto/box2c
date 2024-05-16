// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

/// 2D vector
/// This can be used to represent a point or free vector
typedef struct b2Vec2
{
	/// coordinates
	float x, y;
} b2Vec2;

/// 2D rotation
/// This is similar to using a complex number for rotation
typedef struct b2Rot
{
	/// cosine and sine
	float c, s;
} b2Rot;

/// A 2D rigid transform
typedef struct b2Transform
{
	b2Vec2 p;
	b2Rot q;
} b2Transform;

/// A 2-by-2 Matrix
typedef struct b2Mat22
{
	/// columns
	b2Vec2 cx, cy;
} b2Mat22;

/// Axis-aligned bounding box
typedef struct b2AABB
{
	b2Vec2 lowerBound;
	b2Vec2 upperBound;
} b2AABB;
