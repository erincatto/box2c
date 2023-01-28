// SPDX-FileCopyrightText: 2022 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "box2d/constants.h"
#include "box2d/geometry.h"
#include "box2d/types.h"

enum b2ShapeType
{
	b2_circleShape,
	b2_polygonShape
};

typedef struct b2Shape
{
	b2Object object;
	int32_t bodyIndex;
	enum b2ShapeType type;
	float density;
	float friction;
	float restitution;

	int32_t proxyId;

	b2Filter filter;
	void* userData;

	bool isSensor;

	union
	{
		b2Circle circle;
		b2Polygon polygon;
	};
} b2Shape;
