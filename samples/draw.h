// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "box2d/debug_draw.h"
#include "box2d/types.h"

//
struct Camera
{
	Camera();

	void ResetView();
	b2Vec2 ConvertScreenToWorld(b2Vec2 screenPoint);
	b2Vec2 ConvertWorldToScreen(b2Vec2 worldPoint);
	void BuildProjectionMatrix(float* m, float zBias);

	b2Vec2 m_center;
	float m_zoom;
	int32_t m_width;
	int32_t m_height;
};

// This class implements Box2D debug drawing callbacks
class Draw
{
public:
	Draw();
	~Draw();

	void Create();
	void Destroy();

	void DrawPolygon(const b2Vec2* vertices, int32_t vertexCount, b2Color color);

	void DrawSolidPolygon(const b2Vec2* vertices, int32_t vertexCount, b2Color color);

	void DrawRoundedPolygon(const b2Vec2* vertices, int32_t vertexCount, float radius, b2Color fillColor, b2Color outlineColor);

	void DrawCircle(b2Vec2 center, float radius, b2Color color);

	void DrawSolidCircle(b2Vec2 center, float radius, b2Vec2 axis, b2Color color);

	void DrawCapsule(b2Vec2 p1, b2Vec2 p2, float radius, b2Color color);

	void DrawSolidCapsule(b2Vec2 p1, b2Vec2 p2, float radius, b2Color color);

	void DrawSegment(b2Vec2 p1, b2Vec2 p2, b2Color color);

	void DrawTransform(b2Transform xf);

	void DrawPoint(b2Vec2 p, float size, b2Color color);

	void DrawString(int x, int y, const char* string, ...);

	void DrawString(b2Vec2 p, const char* string, ...);

	void DrawAABB(b2AABB aabb, b2Color color);

	void Flush();

	bool m_showUI;
	struct GLRenderPoints* m_points;
	struct GLRenderLines* m_lines;
	struct GLRenderTriangles* m_triangles;
	struct GLRenderRoundedTriangles* m_roundedTriangles;
	b2DebugDraw m_debugDraw;
};

extern Draw g_draw;
extern Camera g_camera;
extern struct GLFWwindow* g_mainWindow;
