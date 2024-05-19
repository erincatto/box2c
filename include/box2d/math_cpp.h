// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "math_functions.h"

/**
 * @defgroup math_cpp C++ Math
 * @brief Math operator overloads for C++
 * @{
 */

/// Unary add one vector to another
inline void operator+=(b2Vec2& a, b2Vec2 b)
{
	a.x += b.x;
	a.y += b.y;
}

/// Unary subtract one vector from another
inline void operator-=(b2Vec2& a, b2Vec2 b)
{
	a.x -= b.x;
	a.y -= b.y;
}

/// Unary multiply a vector by a scalar
inline void operator*=(b2Vec2& a, float b)
{
	a.x *= b;
	a.y *= b;
}

/// Unary negate a vector
inline b2Vec2 operator-(b2Vec2 a)
{
	return {-a.x, -a.y};
}

/// Binary vector addition
inline b2Vec2 operator+(b2Vec2 a, b2Vec2 b)
{
	return {a.x + b.x, a.y + b.y};
}

/// Binary vector subtraction
inline b2Vec2 operator-(b2Vec2 a, b2Vec2 b)
{
	return {a.x - b.x, a.y - b.y};
}

/// Binary scalar and vector multiplication
inline b2Vec2 operator*(float a, b2Vec2 b)
{
	return {a * b.x, a * b.y};
}

/// Binary scalar and vector multiplication
inline b2Vec2 operator*(b2Vec2 a, float b)
{
	return {a.x * b, a.y * b};
}

/// Binary vector equality
inline bool operator==(b2Vec2 a, b2Vec2 b)
{
	return a.x == b.x && a.y == b.y;
}

/// Binary vector inequality
inline bool operator!=(b2Vec2 a, b2Vec2 b)
{
	return a.x != b.x || a.y != b.y;
}

/**@}*/
