// SPDX-FileCopyrightText: 2022 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "types.h"

/// This struct holds callbacks you can implement to draw a box2d world.
typedef struct b2DebugDraw
{
	/// Draw a closed polygon provided in CCW order.
	void (*DrawPolygon)(const b2Vec2* vertices, int vertexCount, b2Color color, void* context);

	/// Draw a solid closed polygon provided in CCW order.
	void (*DrawSolidPolygon)(const b2Vec2* vertices, int vertexCount, b2Color color, void* context);

	/// Draw a rounded polygon provided in CCW order.
	void (*DrawRoundedPolygon)(const b2Vec2* vertices, int vertexCount, float radius, b2Color lineColor, b2Color fillColor, void* context);

	/// Draw a circle.
	void (*DrawCircle)(b2Vec2 center, float radius, b2Color color, void* context);

	/// Draw a solid circle.
	void (*DrawSolidCircle)(b2Vec2 center, float radius, b2Vec2 axis, b2Color color, void* context);

	/// Draw a line segment.
	void (*DrawSegment)(b2Vec2 p1, b2Vec2 p2, b2Color color, void* context);

	/// Draw a transform. Choose your own length scale.
	/// @param xf a transform.
	void (*DrawTransform)(b2Transform xf, void* context);

	/// Draw a point.
	void (*DrawPoint)(b2Vec2 p, float size, b2Color color, void* context);

	bool drawShapes;
	bool drawJoints;
	bool drawAABBs;
	bool drawCOMs;
	void* context;
} b2DebugDraw;
