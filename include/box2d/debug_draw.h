// SPDX-FileCopyrightText: 2022 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "math_types.h"
#include "color.h"

/// This struct holds callbacks you can implement to draw a box2d world.
typedef struct b2DebugDraw
{
	/// Draw a closed polygon provided in CCW order.
	void (*DrawPolygon)(const b2Vec2* vertices, int vertexCount, b2HexColor color, void* context);

	/// Draw a solid closed polygon provided in CCW order.
	void (*DrawSolidPolygon)(b2Transform transform, const b2Vec2* vertices, int vertexCount, float radius, b2HexColor color,
							 void* context);

	/// Draw a circle.
	void (*DrawCircle)(b2Vec2 center, float radius, b2HexColor color, void* context);

	/// Draw a solid circle.
	void (*DrawSolidCircle)(b2Transform transform, float radius, b2HexColor color, void* context);

	/// Draw a capsule.
	void (*DrawCapsule)(b2Vec2 p1, b2Vec2 p2, float radius, b2HexColor color, void* context);

	/// Draw a solid capsule.
	void (*DrawSolidCapsule)(b2Vec2 p1, b2Vec2 p2, float radius, b2HexColor color, void* context);

	/// Draw a line segment.
	void (*DrawSegment)(b2Vec2 p1, b2Vec2 p2, b2HexColor color, void* context);

	/// Draw a transform. Choose your own length scale.
	void (*DrawTransform)(b2Transform transform, void* context);

	/// Draw a point.
	void (*DrawPoint)(b2Vec2 p, float size, b2HexColor color, void* context);

	/// Draw a string.
	void (*DrawString)(b2Vec2 p, const char* s, void* context);

	b2AABB drawingBounds;
	bool useDrawingBounds;

	bool drawShapes;
	bool drawJoints;
	bool drawJointExtras;
	bool drawAABBs;
	bool drawMass;
	bool drawContacts;
	bool drawGraphColors;
	bool drawContactNormals;
	bool drawContactImpulses;
	bool drawFrictionImpulses;
	void* context;
} b2DebugDraw;
