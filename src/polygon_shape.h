// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "box2d/types.h"
#include "box2d/constants.h"

/// A chain shape is a free form sequence of line segments.
/// The chain has one-sided collision, with the surface normal pointing to the right of the edge.
/// This provides a counter-clockwise winding like the polygon shape.
/// @warning the chain will not collide properly if there are self-intersections.
typedef struct b2ChainShape
{
	b2Vec2* m_vertices;
	int32_t m_count;
} b2ChainShape;
