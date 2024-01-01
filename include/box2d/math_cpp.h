// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "math.h"

// You can include this file if you are using C++ and you want math operator overloads

inline b2Vec2 operator+=(b2Vec2 a, b2Vec2 b)
{
	return {a.x + b.x, a.y + b.y};
}

inline b2Vec2 operator-=(b2Vec2 a, b2Vec2 b)
{
	return {a.x - b.x, a.y - b.y};
}

inline b2Vec2 operator*=(b2Vec2 a, float b)
{
	return {b * a.x, b * a.y};
}

inline b2Vec2 operator-(b2Vec2 a)
{
	return {-a.x, -a.y};
}

inline b2Vec2 operator+(b2Vec2 a, b2Vec2 b)
{
	return {a.x + b.x, a.y + b.y};
}

inline b2Vec2 operator-(b2Vec2 a, b2Vec2 b)
{
	return {a.x + b.x, a.y + b.y};
}

inline b2Vec2 operator*(float a, b2Vec2 b)
{
	return {a * b.x, a * b.y};
}

inline b2Vec2 operator*(b2Vec2 a, float b)
{
	return {a.x * b, a.y * b};
}
