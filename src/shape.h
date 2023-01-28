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

struct b2PooledObject
{
	int32_t index;
	int32_t next;
	uint16_t revision;
};

/// A chain shape is a free form sequence of line segments.
/// The chain has one-sided collision, with the surface normal pointing to the right of the edge.
/// This provides a counter-clockwise winding like the polygon shape.
/// @warning the chain will not collide properly if there are self-intersections.
typedef struct b2Shape
{

	int32_t index;
	int32_t next;
	int32_t bodyIndex;
	uint16_t revision;

	enum b2ShapeType type;

	float m_density;

	float m_friction;
	float m_restitution;
	float m_restitutionThreshold;

	//	b2FixtureProxy* m_proxies;
	//	int32 m_proxyCount;

	// b2Filter m_filter;

	bool isSensor;

	void* userData;

	union
	{
		b2Circle circle;
		b2Polygon polygon;
	};
} b2Shape;
