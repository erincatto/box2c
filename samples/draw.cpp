// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "draw.h"
#include "shader.h"

#include "box2d/constants.h"
#include "box2d/math_cpp.h"
#include "box2d/math_functions.h"

#include <vector>
#include <stdarg.h>
#include <stdio.h>

#if defined(_WIN32)
	#define _CRTDBG_MAP_ALLOC
	#include <crtdbg.h>
	#include <stdlib.h>
#else
	#include <stdlib.h>
#endif

#include <glad/glad.h>
#include <imgui.h>

#define BUFFER_OFFSET(x) ((const void*)(x))

#define SHADER_TEXT(x) "#version 330\n" #x

struct RGBA8
{
	uint8_t r, g, b, a;
};

static inline RGBA8 MakeRGBA8(b2HexColor c, float alpha)
{
	return {uint8_t((c >> 16) & 0xFF), uint8_t((c >> 8) & 0xFF), uint8_t(c & 0xFF), uint8_t(0xFF * alpha)};
}

Draw g_draw;
Camera g_camera;

Camera::Camera()
{
	m_width = 1280;
	m_height = 800;
	ResetView();
}

void Camera::ResetView()
{
	m_center = {0.0f, 20.0f};
	m_zoom = 1.0f;
}

b2Vec2 Camera::ConvertScreenToWorld(b2Vec2 ps)
{
	float w = float(m_width);
	float h = float(m_height);
	float u = ps.x / w;
	float v = (h - ps.y) / h;

	float ratio = w / h;
	b2Vec2 extents = {m_zoom * ratio * 25.0f, m_zoom * 25.0f};

	b2Vec2 lower = b2Sub(m_center, extents);
	b2Vec2 upper = b2Add(m_center, extents);

	b2Vec2 pw = {(1.0f - u) * lower.x + u * upper.x, (1.0f - v) * lower.y + v * upper.y};
	return pw;
}

b2Vec2 Camera::ConvertWorldToScreen(b2Vec2 pw)
{
	float w = float(m_width);
	float h = float(m_height);
	float ratio = w / h;
	b2Vec2 extents = {m_zoom * ratio * 25.0f, m_zoom * 25.0f};

	b2Vec2 lower = b2Sub(m_center, extents);
	b2Vec2 upper = b2Add(m_center, extents);

	float u = (pw.x - lower.x) / (upper.x - lower.x);
	float v = (pw.y - lower.y) / (upper.y - lower.y);

	b2Vec2 ps = {u * w, (1.0f - v) * h};
	return ps;
}

// Convert from world coordinates to normalized device coordinates.
// http://www.songho.ca/opengl/gl_projectionmatrix.html
// This also includes the view transform
void Camera::BuildProjectionMatrix(float* m, float zBias)
{
	float ratio = float(m_width) / float(m_height);
	b2Vec2 extents = {m_zoom * ratio * 25.0f, m_zoom * 25.0f};

	b2Vec2 lower = b2Sub(m_center, extents);
	b2Vec2 upper = b2Add(m_center, extents);
	float w = upper.x - lower.x;
	float h = upper.y - lower.y;

	m[0] = 2.0f / w;
	m[1] = 0.0f;
	m[2] = 0.0f;
	m[3] = 0.0f;

	m[4] = 0.0f;
	m[5] = 2.0f / h;
	m[6] = 0.0f;
	m[7] = 0.0f;

	m[8] = 0.0f;
	m[9] = 0.0f;
	m[10] = -1.0f;
	m[11] = 0.0f;

	m[12] = -2.0f * m_center.x / w;
	m[13] = -2.0f * m_center.y / h;
	m[14] = zBias;
	m[15] = 1.0f;
}

b2AABB Camera::GetViewBounds()
{
	b2AABB bounds;
	bounds.lowerBound = ConvertScreenToWorld({0.0f, (float)m_height});
	bounds.upperBound = ConvertScreenToWorld({(float)m_width, 0.0f});
	return bounds;
}

struct PointData
{
	b2Vec2 position;
	float size;
	RGBA8 rgba;
};

struct GLRenderPoints
{
	void Create()
	{
		const char* vs = "#version 330\n"
						 "uniform mat4 projectionMatrix;\n"
						 "layout(location = 0) in vec2 v_position;\n"
						 "layout(location = 1) in float v_size;\n"
						 "layout(location = 2) in vec4 v_color;\n"
						 "out vec4 f_color;\n"
						 "void main(void)\n"
						 "{\n"
						 "	f_color = v_color;\n"
						 "	gl_Position = projectionMatrix * vec4(v_position, 0.0f, 1.0f);\n"
						 "   gl_PointSize = v_size;\n"
						 "}\n";

		const char* fs = "#version 330\n"
						 "in vec4 f_color;\n"
						 "out vec4 color;\n"
						 "void main(void)\n"
						 "{\n"
						 "	color = f_color;\n"
						 "}\n";

		m_programId = CreateProgramFromStrings(vs, fs);
		m_projectionUniform = glGetUniformLocation(m_programId, "projectionMatrix");
		int vertexAttribute = 0;
		int sizeAttribute = 1;
		int colorAttribute = 2;

		// Generate
		glGenVertexArrays(1, &m_vaoId);
		glGenBuffers(1, &m_vboId);

		glBindVertexArray(m_vaoId);
		glEnableVertexAttribArray(vertexAttribute);
		glEnableVertexAttribArray(sizeAttribute);
		glEnableVertexAttribArray(colorAttribute);

		// Vertex buffer
		glBindBuffer(GL_ARRAY_BUFFER, m_vboId);
		glBufferData(GL_ARRAY_BUFFER, e_maxCount * sizeof(PointData), NULL, GL_DYNAMIC_DRAW);

		glVertexAttribPointer(vertexAttribute, 2, GL_FLOAT, GL_FALSE, sizeof(PointData), (void*)offsetof(PointData, position));
		glVertexAttribPointer(sizeAttribute, 1, GL_FLOAT, GL_FALSE, sizeof(PointData), (void*)offsetof(PointData, size));
		// color will get automatically expanded to floats in the shader
		glVertexAttribPointer(colorAttribute, 4, GL_UNSIGNED_BYTE, GL_TRUE, sizeof(PointData), (void*)offsetof(PointData, rgba));

		CheckErrorGL();

		// Cleanup
		glBindBuffer(GL_ARRAY_BUFFER, 0);
		glBindVertexArray(0);
	}

	void Destroy()
	{
		if (m_vaoId)
		{
			glDeleteVertexArrays(1, &m_vaoId);
			glDeleteBuffers(1, &m_vboId);
			m_vaoId = 0;
			m_vboId = 0;
		}

		if (m_programId)
		{
			glDeleteProgram(m_programId);
			m_programId = 0;
		}
	}

	// todo instead of flushing, keep a growable array of data
	// this will prevent sorting problems.

	void AddPoint(b2Vec2 v, float size, b2HexColor c)
	{
		RGBA8 rgba = MakeRGBA8(c, 1.0f);
		m_points.push_back({v, size, rgba});
	}

	void Flush()
	{
		int count = m_points.size();
		if (count == 0)
		{
			return;
		}

		glUseProgram(m_programId);

		float proj[16] = {0.0f};
		g_camera.BuildProjectionMatrix(proj, 0.0f);

		glUniformMatrix4fv(m_projectionUniform, 1, GL_FALSE, proj);
		glBindVertexArray(m_vaoId);

		glBindBuffer(GL_ARRAY_BUFFER, m_vboId);
		glEnable(GL_PROGRAM_POINT_SIZE);

		int base = 0;
		while (count > 0)
		{
			int batchCount = b2MinInt(count, e_maxCount);
			glBufferSubData(GL_ARRAY_BUFFER, 0, batchCount * sizeof(PointData), &m_points[base]);
			glDrawArrays(GL_POINTS, 0, batchCount);

			CheckErrorGL();
		
			count -= e_maxCount;
			base += e_maxCount;
		}

		glDisable(GL_PROGRAM_POINT_SIZE);
		glBindBuffer(GL_ARRAY_BUFFER, 0);
		glBindVertexArray(0);
		glUseProgram(0);

		m_points.clear();
	}

	enum
	{
		e_maxCount = 2048
	};

	std::vector<PointData> m_points;

	GLuint m_vaoId;
	GLuint m_vboId;
	GLuint m_programId;
	GLint m_projectionUniform;
};

struct VertexData
{
	b2Vec2 position;
	RGBA8 rgba;
};

struct GLRenderLines
{
	void Create()
	{
		const char* vs = "#version 330\n"
						 "uniform mat4 projectionMatrix;\n"
						 "layout(location = 0) in vec2 v_position;\n"
						 "layout(location = 1) in vec4 v_color;\n"
						 "out vec4 f_color;\n"
						 "void main(void)\n"
						 "{\n"
						 "	f_color = v_color;\n"
						 "	gl_Position = projectionMatrix * vec4(v_position, 0.0f, 1.0f);\n"
						 "}\n";

		const char* fs = "#version 330\n"
						 "in vec4 f_color;\n"
						 "out vec4 color;\n"
						 "void main(void)\n"
						 "{\n"
						 "	color = f_color;\n"
						 "}\n";

		m_programId = CreateProgramFromStrings(vs, fs);
		m_projectionUniform = glGetUniformLocation(m_programId, "projectionMatrix");
		int vertexAttribute = 0;
		int colorAttribute = 1;

		// Generate
		glGenVertexArrays(1, &m_vaoId);
		glGenBuffers(1, &m_vboId);

		glBindVertexArray(m_vaoId);
		glEnableVertexAttribArray(vertexAttribute);
		glEnableVertexAttribArray(colorAttribute);

		// Vertex buffer
		glBindBuffer(GL_ARRAY_BUFFER, m_vboId);
		glBufferData(GL_ARRAY_BUFFER, e_maxCount * sizeof(VertexData), NULL, GL_DYNAMIC_DRAW);

		glVertexAttribPointer(vertexAttribute, 2, GL_FLOAT, GL_FALSE, sizeof(VertexData), (void*)offsetof(VertexData, position));
		// color will get automatically expanded to floats in the shader
		glVertexAttribPointer(colorAttribute, 4, GL_UNSIGNED_BYTE, GL_TRUE, sizeof(VertexData),
							  (void*)offsetof(VertexData, rgba));

		CheckErrorGL();

		// Cleanup
		glBindBuffer(GL_ARRAY_BUFFER, 0);
		glBindVertexArray(0);
	}

	void Destroy()
	{
		if (m_vaoId)
		{
			glDeleteVertexArrays(1, &m_vaoId);
			glDeleteBuffers(1, &m_vboId);
			m_vaoId = 0;
			m_vboId = 0;
		}

		if (m_programId)
		{
			glDeleteProgram(m_programId);
			m_programId = 0;
		}
	}

	void AddLine(b2Vec2 p1, b2Vec2 p2, b2HexColor c)
	{
		RGBA8 rgba = MakeRGBA8(c, 1.0f);
		m_points.push_back({p1, rgba});
		m_points.push_back({p2, rgba});
	}

	void Flush()
	{
		int count = m_points.size();
		if (count == 0)
		{
			return;
		}

		assert(count % 2 == 0);

		glUseProgram(m_programId);

		float proj[16] = {0.0f};
		g_camera.BuildProjectionMatrix(proj, 0.1f);

		glUniformMatrix4fv(m_projectionUniform, 1, GL_FALSE, proj);

		glBindVertexArray(m_vaoId);

		glBindBuffer(GL_ARRAY_BUFFER, m_vboId);

		int base = 0;
		while (count > 0)
		{
			int batchCount = b2MinInt(count, e_maxCount);
			glBufferSubData(GL_ARRAY_BUFFER, 0, batchCount * sizeof(VertexData), &m_points[base]);

			glDrawArrays(GL_LINES, 0, batchCount);

			CheckErrorGL();

			count -= e_maxCount;
			base += e_maxCount;
		}

		glBindBuffer(GL_ARRAY_BUFFER, 0);
		glBindVertexArray(0);
		glUseProgram(0);

		m_points.clear();
	}

	// need lots of space for lines so they draw last
	// could also consider disabling depth buffer
	enum
	{
		// must be multiple of 2
		e_maxCount = 2 * 2048
	};

	std::vector<VertexData> m_points;

	GLuint m_vaoId;
	GLuint m_vboId;
	GLuint m_programId;
	GLint m_projectionUniform;
};

// todo this is not used anymore and has untested changes
struct GLRenderTriangles
{
	void Create()
	{
		const char* vs = "#version 330\n"
						 "uniform mat4 projectionMatrix;\n"
						 "layout(location = 0) in vec2 v_position;\n"
						 "layout(location = 1) in vec4 v_color;\n"
						 "out vec4 f_color;\n"
						 "void main(void)\n"
						 "{\n"
						 "	f_color = v_color;\n"
						 "	gl_Position = projectionMatrix * vec4(v_position, 0.0f, 1.0f);\n"
						 "}\n";

		const char* fs = "#version 330\n"
						 "in vec4 f_color;\n"
						 "out vec4 color;\n"
						 "void main(void)\n"
						 "{\n"
						 "	color = f_color;\n"
						 "}\n";

		m_programId = CreateProgramFromStrings(vs, fs);
		m_projectionUniform = glGetUniformLocation(m_programId, "projectionMatrix");
		int vertexAttribute = 0;
		int colorAttribute = 1;

		// Generate
		glGenVertexArrays(1, &m_vaoId);
		glGenBuffers(1, &m_vboId);

		glBindVertexArray(m_vaoId);
		glEnableVertexAttribArray(vertexAttribute);
		glEnableVertexAttribArray(colorAttribute);

		// Vertex buffer
		glBindBuffer(GL_ARRAY_BUFFER, m_vboId);
		glBufferData(GL_ARRAY_BUFFER, sizeof(m_points), NULL, GL_DYNAMIC_DRAW);

		glVertexAttribPointer(vertexAttribute, 2, GL_FLOAT, GL_FALSE, sizeof(VertexData), (void*)offsetof(VertexData, position));
		// color will get automatically expanded to floats in the shader
		glVertexAttribPointer(colorAttribute, 4, GL_UNSIGNED_BYTE, GL_TRUE, sizeof(VertexData),
							  (void*)offsetof(VertexData, rgba));

		CheckErrorGL();

		// Cleanup
		glBindBuffer(GL_ARRAY_BUFFER, 0);
		glBindVertexArray(0);
	}

	void Destroy()
	{
		if (m_vaoId)
		{
			glDeleteVertexArrays(1, &m_vaoId);
			glDeleteBuffers(1, &m_vboId);
			m_vaoId = 0;
			m_vboId = 0;
		}

		if (m_programId)
		{
			glDeleteProgram(m_programId);
			m_programId = 0;
		}
	}

	void AddTriangle(b2Vec2 p1, b2Vec2 p2, b2Vec2 p3, b2HexColor c)
	{
		RGBA8 rgba = MakeRGBA8(c, 1.0f);
		m_points.push_back({p1, rgba});
		m_points.push_back({p2, rgba});
		m_points.push_back({p3, rgba});
	}

	void Flush()
	{
		int count = m_points.size();
		if (count == 0)
		{
			return;
		}

		assert(count % 3 == 0);

		glUseProgram(m_programId);

		float proj[16] = {0.0f};
		g_camera.BuildProjectionMatrix(proj, 0.2f);

		glUniformMatrix4fv(m_projectionUniform, 1, GL_FALSE, proj);

		glBindVertexArray(m_vaoId);

		glBindBuffer(GL_ARRAY_BUFFER, m_vboId);
		glEnable(GL_BLEND);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

		int base = 0;
		while (count > 0)
		{
			int batchCount = b2MinInt(count, e_maxCount);

			glBufferSubData(GL_ARRAY_BUFFER, 0, batchCount * sizeof(VertexData), &m_points);
			glDrawArrays(GL_TRIANGLES, 0, batchCount);

			CheckErrorGL();

			count -= e_maxCount;
			base += e_maxCount;
		}

		glDisable(GL_BLEND);

		glBindBuffer(GL_ARRAY_BUFFER, 0);
		glBindVertexArray(0);
		glUseProgram(0);

		m_points.clear();
	}

	enum
	{
		// must be multiple of 3
		e_maxCount = 3 * 512
	};

	std::vector<VertexData> m_points;

	GLuint m_vaoId;
	GLuint m_vboId;
	GLuint m_programId;
	GLint m_projectionUniform;
};

struct Transform
{
	float x, y, c, s;
};

struct CircleData
{
	Transform transform;
	float radius;
	RGBA8 rgba;
};

// Draws SDF circles using quad instancing. Apparently instancing of quads can be slow on older GPUs.
// https://www.reddit.com/r/opengl/comments/q7yikr/how_to_draw_several_quads_through_instancing/
// https://www.g-truc.net/post-0666.html
struct GLRenderCircles
{
	void Create()
	{
		m_programId = CreateProgramFromFiles("samples/data/circle.vs", "samples/data/circle.fs");
		m_projectionUniform = glGetUniformLocation(m_programId, "projectionMatrix");
		m_zoomUniform = glGetUniformLocation(m_programId, "zoom");
		int vertexAttribute = 0;
		int transformInstance = 1;
		int radiusInstance = 2;
		int colorInstance = 3;

		// Generate
		glGenVertexArrays(1, &m_vaoId);
		glGenBuffers(2, m_vboIds);

		glBindVertexArray(m_vaoId);
		glEnableVertexAttribArray(vertexAttribute);
		glEnableVertexAttribArray(transformInstance);
		glEnableVertexAttribArray(radiusInstance);
		glEnableVertexAttribArray(colorInstance);

		// Vertex buffer for single quad
		float a = 1.1f;
		b2Vec2 vertices[] = {{-a, -a}, {a, -a}, {-a, a}, {a, -a}, {a, a}, {-a, a}};
		glBindBuffer(GL_ARRAY_BUFFER, m_vboIds[0]);
		glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);
		glVertexAttribPointer(vertexAttribute, 2, GL_FLOAT, GL_FALSE, 0, BUFFER_OFFSET(0));

		// Circle buffer
		glBindBuffer(GL_ARRAY_BUFFER, m_vboIds[1]);
		glBufferData(GL_ARRAY_BUFFER, sizeof(m_circles), NULL, GL_DYNAMIC_DRAW);

		glVertexAttribPointer(transformInstance, 4, GL_FLOAT, GL_FALSE, sizeof(CircleData),
							  (void*)offsetof(CircleData, transform));
		glVertexAttribPointer(radiusInstance, 1, GL_FLOAT, GL_FALSE, sizeof(CircleData), (void*)offsetof(CircleData, radius));
		glVertexAttribPointer(colorInstance, 4, GL_UNSIGNED_BYTE, GL_TRUE, sizeof(CircleData), (void*)offsetof(CircleData, rgba));

		glVertexAttribDivisor(transformInstance, 1);
		glVertexAttribDivisor(radiusInstance, 1);
		glVertexAttribDivisor(colorInstance, 1);

		CheckErrorGL();

		// Cleanup
		glBindBuffer(GL_ARRAY_BUFFER, 0);
		glBindVertexArray(0);

		m_count = 0;
	}

	void Destroy()
	{
		if (m_vaoId)
		{
			glDeleteVertexArrays(1, &m_vaoId);
			glDeleteBuffers(2, m_vboIds);
			m_vaoId = 0;
			m_vboIds[0] = 0;
			m_vboIds[1] = 0;
		}

		if (m_programId)
		{
			glDeleteProgram(m_programId);
			m_programId = 0;
		}
	}

	void AddCircle(const b2Transform& transform, float radius, b2HexColor color)
	{
		if (m_count == e_maxCount)
		{
			Flush();
		}

		RGBA8 rgba = MakeRGBA8(color, 1.0f);
		m_circles[m_count] = {{transform.p.x, transform.p.y, transform.q.c, transform.q.s}, radius, rgba};
		++m_count;
	}

	void Flush()
	{
		if (m_count == 0)
		{
			return;
		}

		glUseProgram(m_programId);

		float proj[16] = {0.0f};
		g_camera.BuildProjectionMatrix(proj, 0.2f);

		glUniformMatrix4fv(m_projectionUniform, 1, GL_FALSE, proj);
		glUniform1f(m_zoomUniform, g_camera.m_zoom);

		glBindVertexArray(m_vaoId);

		glBindBuffer(GL_ARRAY_BUFFER, m_vboIds[1]);
		glBufferSubData(GL_ARRAY_BUFFER, 0, m_count * sizeof(CircleData), m_circles);
		CheckErrorGL();

		glEnable(GL_BLEND);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
		glDrawArraysInstanced(GL_TRIANGLES, 0, 6, m_count);
		glDisable(GL_BLEND);

		CheckErrorGL();

		glBindBuffer(GL_ARRAY_BUFFER, 0);
		glBindVertexArray(0);
		glUseProgram(0);

		m_count = 0;
	}

	enum
	{
		e_maxCount = 2048
	};

	CircleData m_circles[e_maxCount];

	int m_count;

	GLuint m_vaoId;
	GLuint m_vboIds[2];
	GLuint m_programId;
	GLint m_projectionUniform;
	GLint m_zoomUniform;
};

struct CapsuleData
{
	Transform transform;
	float radius;
	float length;
	RGBA8 rgba;
};

// Draw capsules using SDF-based shader
struct GLRenderCapsules
{
	void Create()
	{
		m_programId = CreateProgramFromFiles("samples/data/capsule.vs", "samples/data/capsule.fs");

		m_projectionUniform = glGetUniformLocation(m_programId, "projectionMatrix");
		m_zoomUniform = glGetUniformLocation(m_programId, "zoom");
		int vertexAttribute = 0;
		int transformInstance = 1;
		int radiusInstance = 2;
		int lengthInstance = 3;
		int colorInstance = 4;

		// Generate
		glGenVertexArrays(1, &m_vaoId);
		glGenBuffers(2, m_vboIds);

		glBindVertexArray(m_vaoId);
		glEnableVertexAttribArray(vertexAttribute);
		glEnableVertexAttribArray(transformInstance);
		glEnableVertexAttribArray(radiusInstance);
		glEnableVertexAttribArray(lengthInstance);
		glEnableVertexAttribArray(colorInstance);

		// Vertex buffer for single quad
		float a = 1.1f;
		b2Vec2 vertices[] = {{-a, -a}, {a, -a}, {-a, a}, {a, -a}, {a, a}, {-a, a}};
		glBindBuffer(GL_ARRAY_BUFFER, m_vboIds[0]);
		glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);
		glVertexAttribPointer(vertexAttribute, 2, GL_FLOAT, GL_FALSE, 0, BUFFER_OFFSET(0));

		// Capsule buffer
		glBindBuffer(GL_ARRAY_BUFFER, m_vboIds[1]);
		glBufferData(GL_ARRAY_BUFFER, sizeof(m_capsules), NULL, GL_DYNAMIC_DRAW);

		glVertexAttribPointer(transformInstance, 4, GL_FLOAT, GL_FALSE, sizeof(CapsuleData),
							  (void*)offsetof(CapsuleData, transform));
		glVertexAttribPointer(radiusInstance, 1, GL_FLOAT, GL_FALSE, sizeof(CapsuleData), (void*)offsetof(CapsuleData, radius));
		glVertexAttribPointer(lengthInstance, 1, GL_FLOAT, GL_FALSE, sizeof(CapsuleData), (void*)offsetof(CapsuleData, length));
		glVertexAttribPointer(colorInstance, 4, GL_UNSIGNED_BYTE, GL_TRUE, sizeof(CapsuleData),
							  (void*)offsetof(CapsuleData, rgba));

		glVertexAttribDivisor(transformInstance, 1);
		glVertexAttribDivisor(radiusInstance, 1);
		glVertexAttribDivisor(lengthInstance, 1);
		glVertexAttribDivisor(colorInstance, 1);

		CheckErrorGL();

		// Cleanup
		glBindBuffer(GL_ARRAY_BUFFER, 0);
		glBindVertexArray(0);

		m_count = 0;
	}

	void Destroy()
	{
		if (m_vaoId)
		{
			glDeleteVertexArrays(1, &m_vaoId);
			glDeleteBuffers(2, m_vboIds);
			m_vaoId = 0;
			m_vboIds[0] = 0;
			m_vboIds[1] = 0;
		}

		if (m_programId)
		{
			glDeleteProgram(m_programId);
			m_programId = 0;
		}
	}

	void AddCapsule(b2Vec2 p1, b2Vec2 p2, float radius, b2HexColor c)
	{
		if (m_count == e_maxCount)
		{
			Flush();
		}

		b2Vec2 d = p2 - p1;
		float length = b2Length(d);
		if (length < b2_linearSlop)
		{
			printf("WARNING: sample app: capsule too short!\n");
			return;
		}

		b2Vec2 axis = {d.x / length, d.y / length};
		b2Transform transform;
		transform.p = 0.5f * (p1 + p2);
		transform.q.c = axis.x;
		transform.q.s = axis.y;

		RGBA8 rgba = MakeRGBA8(c, 1.0f);

		m_capsules[m_count] = {{transform.p.x, transform.p.y, transform.q.c, transform.q.s}, radius, length, rgba};
		++m_count;
	}

	void Flush()
	{
		if (m_count == 0)
		{
			return;
		}

		glUseProgram(m_programId);

		float proj[16] = {0.0f};
		g_camera.BuildProjectionMatrix(proj, 0.2f);

		glUniformMatrix4fv(m_projectionUniform, 1, GL_FALSE, proj);
		glUniform1f(m_zoomUniform, g_camera.m_zoom);

		glBindVertexArray(m_vaoId);

		glBindBuffer(GL_ARRAY_BUFFER, m_vboIds[1]);
		glBufferSubData(GL_ARRAY_BUFFER, 0, m_count * sizeof(CapsuleData), m_capsules);
		CheckErrorGL();

		glEnable(GL_BLEND);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
		glDrawArraysInstanced(GL_TRIANGLES, 0, 6, m_count);
		glDisable(GL_BLEND);

		CheckErrorGL();

		glBindBuffer(GL_ARRAY_BUFFER, 0);
		glBindVertexArray(0);
		glUseProgram(0);

		m_count = 0;
	}

	enum
	{
		e_maxCount = 2048
	};

	CapsuleData m_capsules[e_maxCount];

	int m_count;

	GLuint m_vaoId;
	GLuint m_vboIds[2];
	GLuint m_programId;
	GLint m_projectionUniform;
	GLint m_zoomUniform;
};

struct PolygonData
{
	b2Transform transform;
	b2Vec2 p1, p2, p3, p4, p5, p6, p7, p8;
	int count;
	float radius;

	// keep color small
	RGBA8 color;
};

// Rounded and non-rounded convex polygons using an SDF-based shader.
struct GLRenderPolygons
{
	void Create()
	{
		#if 0
		const char* vs =
			SHADER_TEXT(uniform mat4 projectionMatrix; uniform float zoom;

						layout(location = 0) in vec2 v_localPosition; layout(location = 1) in vec4 v_instanceTransform;
						layout(location = 2) in vec4 v_instancePoints12; layout(location = 3) in vec4 v_instancePoints34;
						layout(location = 4) in vec4 v_instancePoints56; layout(location = 5) in vec4 v_instancePoints78;
						layout(location = 6) in int v_instanceCount; layout(location = 7) in float v_instanceRadius;
						layout(location = 8) in vec4 v_instanceColor;

						out vec2 f_position; out vec4 f_color; out vec2 f_points[8]; flat out int f_count; out float f_radius;
						out float f_zoom;

						void main() {
							f_position = v_localPosition;
							f_color = v_instanceColor;

							f_radius = v_instanceRadius;
							f_count = v_instanceCount;

							f_points[0] = v_instancePoints12.xy;
							f_points[1] = v_instancePoints12.zw;
							f_points[2] = v_instancePoints34.xy;
							f_points[3] = v_instancePoints34.zw;
							f_points[4] = v_instancePoints56.xy;
							f_points[5] = v_instancePoints56.zw;
							f_points[6] = v_instancePoints78.xy;
							f_points[7] = v_instancePoints78.zw;

							// Compute polygon AABB
							vec2 lower = f_points[0];
							vec2 upper = f_points[0];
							for (int i = 1; i < v_instanceCount; ++i)
							{
								lower = min(lower, f_points[i]);
								upper = max(upper, f_points[i]);
							}

							vec2 center = 0.5 * (lower + upper);
							vec2 width = upper - lower;
							float maxWidth = max(width.x, width.y);

							float scale = f_radius + 0.5 * maxWidth;
							float invScale = 1.0 / scale;

							// Shift and scale polygon points so they fit in 2x2 quad
							for (int i = 0; i < f_count; ++i)
							{
								f_points[i] = invScale * (f_points[i] - center);
							}

							// Scale radius as well
							f_radius = invScale * f_radius;

							// scale zoom so the border is fixed size
							f_zoom = invScale * zoom;

							// if (v_instanceCount == 4)
							//{
							//	f_color = vec4(0, 0, 1, 1);
							// }

							// scale up and transform quad to fit polygon
							float x = v_instanceTransform.x;
							float y = v_instanceTransform.y;
							float c = v_instanceTransform.z;
							float s = v_instanceTransform.w;
							vec2 p = vec2(scale * v_localPosition.x, scale * v_localPosition.y) + center;
							p = vec2((c * p.x - s * p.y) + x, (s * p.x + c * p.y) + y);
							gl_Position = projectionMatrix * vec4(p, 0.0f, 1.0f);
						});

		const char* fs = SHADER_TEXT(

			in vec2 f_position; in vec2 f_points[8]; flat in int f_count; in float f_radius; in vec4 f_color; in float f_zoom;

			out vec4 fragColor;

			// https://en.wikipedia.org/wiki/Alpha_compositing
			vec4 blend_colors(vec4 front, vec4 back) {
				vec3 cSrc = front.rgb;
				float alphaSrc = front.a;
				vec3 cDst = back.rgb;
				float alphaDst = back.a;

				vec3 cOut = cSrc * alphaSrc + cDst * alphaDst * (1.0 - alphaSrc);
				float alphaOut = alphaSrc + alphaDst * (1.0 - alphaSrc);

				// remove alpha from rgb
				cOut = cOut / alphaOut;

				return vec4(cOut, alphaOut);
			}

			float cross2d(in vec2 v1, in vec2 v2) { return v1.x * v2.y - v1.y * v2.x; }

			// Signed distance function for convex polygon
			float sdConvexPolygon(in vec2 p, in vec2[8] v, in int count) {
				// Initial squared distance
				float d = dot(p - v[0], p - v[0]);

				// Consider query point inside to start
				float side = -1.0;
				int j = count - 1;
				for (int i = 0; i < count; ++i)
				{
					// Distance to a polygon edge
					vec2 e = v[i] - v[j];
					vec2 w = p - v[j];
					float we = dot(w, e);
					vec2 b = w - e * clamp(we / dot(e, e), 0.0, 1.0);
					float bb = dot(b, b);

					// Track smallest distance
					if (bb < d)
					{
						d = bb;
					}

					// If the query point is outside any edge then it is outside the entire polygon.
					// This depends on the CCW winding order of points.
					float s = cross2d(w, e);
					if (s >= 0.0)
					{
						side = 1.0;
					}

					j = i;
				}

				return side * sqrt(d);
			}

			void main() {
				vec4 borderColor = f_color;
				vec4 fillColor = 0.6f * borderColor;

				float dw = sdConvexPolygon(f_position, f_points, f_count);
				float d = abs(dw - f_radius);

				float borderThickness = 0.07 * f_zoom;

				// roll the fill alpha down at the border
				vec4 back = vec4(fillColor.rgb, fillColor.a * (1.0 - smoothstep(f_radius, f_radius + borderThickness, dw)));

				// roll the border alpha down from 1 to 0 across the border thickness
				vec4 front = vec4(borderColor.rgb, 1.0 - smoothstep(0.0, borderThickness, d));

				fragColor = blend_colors(front, back);

				// fragColor = vec4(0.5);
			});

		m_programId = CreateProgramFromStrings(vs, fs);

		#else
		m_programId = CreateProgramFromFiles("samples/data/polygon.vs", "samples/data/polygon.fs");

		#endif

		m_projectionUniform = glGetUniformLocation(m_programId, "projectionMatrix");
		m_zoomUniform = glGetUniformLocation(m_programId, "zoom");
		int vertexAttribute = 0;
		int instanceTransform = 1;
		int instancePoint12 = 2;
		int instancePoint34 = 3;
		int instancePoint56 = 4;
		int instancePoint78 = 5;
		int instancePointCount = 6;
		int instanceRadius = 7;
		int instanceColor = 8;

		// Generate
		glGenVertexArrays(1, &m_vaoId);
		glGenBuffers(2, m_vboIds);

		glBindVertexArray(m_vaoId);
		glEnableVertexAttribArray(vertexAttribute);
		glEnableVertexAttribArray(instanceTransform);
		glEnableVertexAttribArray(instancePoint12);
		glEnableVertexAttribArray(instancePoint34);
		glEnableVertexAttribArray(instancePoint56);
		glEnableVertexAttribArray(instancePoint78);
		glEnableVertexAttribArray(instancePointCount);
		glEnableVertexAttribArray(instanceRadius);
		glEnableVertexAttribArray(instanceColor);

		// Vertex buffer for single quad
		float a = 1.1f;
		b2Vec2 vertices[] = {{-a, -a}, {a, -a}, {-a, a}, {a, -a}, {a, a}, {-a, a}};
		glBindBuffer(GL_ARRAY_BUFFER, m_vboIds[0]);
		glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);
		glVertexAttribPointer(vertexAttribute, 2, GL_FLOAT, GL_FALSE, 0, BUFFER_OFFSET(0));

		// Polygon buffer
		glBindBuffer(GL_ARRAY_BUFFER, m_vboIds[1]);
		glBufferData(GL_ARRAY_BUFFER, sizeof(m_polygons), NULL, GL_DYNAMIC_DRAW);
		glVertexAttribPointer(instanceTransform, 4, GL_FLOAT, GL_FALSE, sizeof(PolygonData),
							  (void*)offsetof(PolygonData, transform));
		glVertexAttribPointer(instancePoint12, 4, GL_FLOAT, GL_FALSE, sizeof(PolygonData), (void*)offsetof(PolygonData, p1));
		glVertexAttribPointer(instancePoint34, 4, GL_FLOAT, GL_FALSE, sizeof(PolygonData), (void*)offsetof(PolygonData, p3));
		glVertexAttribPointer(instancePoint56, 4, GL_FLOAT, GL_FALSE, sizeof(PolygonData), (void*)offsetof(PolygonData, p5));
		glVertexAttribPointer(instancePoint78, 4, GL_FLOAT, GL_FALSE, sizeof(PolygonData), (void*)offsetof(PolygonData, p7));
		glVertexAttribIPointer(instancePointCount, 1, GL_INT, sizeof(PolygonData), (void*)offsetof(PolygonData, count));
		glVertexAttribPointer(instanceRadius, 1, GL_FLOAT, GL_FALSE, sizeof(PolygonData), (void*)offsetof(PolygonData, radius));
		// color will get automatically expanded to floats in the shader
		glVertexAttribPointer(instanceColor, 4, GL_UNSIGNED_BYTE, GL_TRUE, sizeof(PolygonData),
							  (void*)offsetof(PolygonData, color));

		// These divisors tell glsl how to distribute per instance data
		glVertexAttribDivisor(instanceTransform, 1);
		glVertexAttribDivisor(instancePoint12, 1);
		glVertexAttribDivisor(instancePoint34, 1);
		glVertexAttribDivisor(instancePoint56, 1);
		glVertexAttribDivisor(instancePoint78, 1);
		glVertexAttribDivisor(instancePointCount, 1);
		glVertexAttribDivisor(instanceRadius, 1);
		glVertexAttribDivisor(instanceColor, 1);

		CheckErrorGL();

		// Cleanup
		glBindBuffer(GL_ARRAY_BUFFER, 0);
		glBindVertexArray(0);

		m_count = 0;
	}

	void Destroy()
	{
		if (m_vaoId)
		{
			glDeleteVertexArrays(1, &m_vaoId);
			glDeleteBuffers(2, m_vboIds);
			m_vaoId = 0;
		}

		if (m_programId)
		{
			glDeleteProgram(m_programId);
			m_programId = 0;
		}
	}

	void AddPolygon(const b2Transform& transform, const b2Vec2* points, int count, float radius, b2HexColor color)
	{
		if (m_count == e_maxCount)
		{
			Flush();
		}

		PolygonData* data = m_polygons + m_count;
		data->transform = transform;

		int n = count < 8 ? count : 8;
		b2Vec2* ps = &data->p1;
		for (int i = 0; i < n; ++i)
		{
			ps[i] = points[i];
		}

		for (int i = n; i < 8; ++i)
		{
			ps[i] = {0.0f, 0.0f};
		}

		data->count = n;
		data->radius = radius;
		data->color = MakeRGBA8(color, 1.0f);
		++m_count;
	}

	void Flush()
	{
		if (m_count == 0)
		{
			return;
		}

		glUseProgram(m_programId);

		float proj[16] = {0.0f};
		g_camera.BuildProjectionMatrix(proj, 0.2f);

		glUniformMatrix4fv(m_projectionUniform, 1, GL_FALSE, proj);
		glUniform1f(m_zoomUniform, g_camera.m_zoom);

		glBindVertexArray(m_vaoId);

		glBindBuffer(GL_ARRAY_BUFFER, m_vboIds[1]);
		glBufferSubData(GL_ARRAY_BUFFER, 0, m_count * sizeof(PolygonData), m_polygons);
		CheckErrorGL();

		glEnable(GL_BLEND);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
		glDrawArraysInstanced(GL_TRIANGLES, 0, 6, m_count);
		glDisable(GL_BLEND);

		CheckErrorGL();

		glBindBuffer(GL_ARRAY_BUFFER, 0);
		glBindVertexArray(0);
		glUseProgram(0);

		m_count = 0;
	}

	enum
	{
		e_maxCount = 512
	};

	PolygonData m_polygons[e_maxCount];

	int m_count;

	GLuint m_vaoId;
	GLuint m_vboIds[2];
	GLuint m_programId;
	GLint m_projectionUniform;
	GLint m_zoomUniform;
};

void DrawPolygonFcn(const b2Vec2* vertices, int vertexCount, b2HexColor color, void* context)
{
	static_cast<Draw*>(context)->DrawPolygon(vertices, vertexCount, color);
}

void DrawSolidPolygonFcn(b2Transform transform, const b2Vec2* vertices, int vertexCount, float radius, b2HexColor color,
						 void* context)
{
	static_cast<Draw*>(context)->DrawSolidPolygon(transform, vertices, vertexCount, radius, color);
}

void DrawCircleFcn(b2Vec2 center, float radius, b2HexColor color, void* context)
{
	static_cast<Draw*>(context)->DrawCircle(center, radius, color);
}

void DrawSolidCircleFcn(b2Transform transform, float radius, b2HexColor color, void* context)
{
	static_cast<Draw*>(context)->DrawSolidCircle(transform, b2Vec2_zero, radius, color);
}

void DrawCapsuleFcn(b2Vec2 p1, b2Vec2 p2, float radius, b2HexColor color, void* context)
{
	static_cast<Draw*>(context)->DrawCapsule(p1, p2, radius, color);
}

void DrawSolidCapsuleFcn(b2Vec2 p1, b2Vec2 p2, float radius, b2HexColor color, void* context)
{
	static_cast<Draw*>(context)->DrawSolidCapsule(p1, p2, radius, color);
}

void DrawSegmentFcn(b2Vec2 p1, b2Vec2 p2, b2HexColor color, void* context)
{
	static_cast<Draw*>(context)->DrawSegment(p1, p2, color);
}

void DrawTransformFcn(b2Transform xf, void* context)
{
	static_cast<Draw*>(context)->DrawTransform(xf);
}

void DrawPointFcn(b2Vec2 p, float size, b2HexColor color, void* context)
{
	static_cast<Draw*>(context)->DrawPoint(p, size, color);
}

void DrawStringFcn(b2Vec2 p, const char* s, void* context)
{
	static_cast<Draw*>(context)->DrawString(p, s);
}

Draw::Draw()
{
	m_showUI = true;
	m_points = nullptr;
	m_lines = nullptr;
	m_triangles = nullptr;
	m_circles = nullptr;
	m_capsules = nullptr;
	m_polygons = nullptr;
	m_debugDraw = {};
}

Draw::~Draw()
{
	assert(m_points == nullptr);
	assert(m_lines == nullptr);
	assert(m_triangles == nullptr);
	assert(m_circles == nullptr);
	assert(m_capsules == nullptr);
	assert(m_polygons == nullptr);
}

void Draw::Create()
{
	m_points = new GLRenderPoints;
	m_points->Create();
	m_lines = new GLRenderLines;
	m_lines->Create();
	m_triangles = new GLRenderTriangles;
	m_triangles->Create();
	m_circles = new GLRenderCircles;
	m_circles->Create();
	m_capsules = new GLRenderCapsules;
	m_capsules->Create();
	m_polygons = new GLRenderPolygons;
	m_polygons->Create();

	b2AABB bounds = {{-FLT_MAX, -FLT_MAX}, {FLT_MAX, FLT_MAX}};

	m_debugDraw = {DrawPolygonFcn,
				   DrawSolidPolygonFcn,
				   DrawCircleFcn,
				   DrawSolidCircleFcn,
				   DrawCapsuleFcn,
				   DrawSolidCapsuleFcn,
				   DrawSegmentFcn,
				   DrawTransformFcn,
				   DrawPointFcn,
				   DrawStringFcn,
				   bounds,
				   false, // drawUsingBounds
				   true,  // shapes
				   true,  // joints
				   false, // joint extras
				   false, // aabbs
				   false, // mass
				   false, // contacts
				   false, // colors
				   false, // normals
				   false, // impulse
				   false, // friction
				   this};
}

void Draw::Destroy()
{
	m_points->Destroy();
	delete m_points;
	m_points = nullptr;

	m_lines->Destroy();
	delete m_lines;
	m_lines = nullptr;

	m_triangles->Destroy();
	delete m_triangles;
	m_triangles = nullptr;

	m_circles->Destroy();
	delete m_circles;
	m_circles = nullptr;

	m_capsules->Destroy();
	delete m_capsules;
	m_capsules = nullptr;

	m_polygons->Destroy();
	delete m_polygons;
	m_polygons = nullptr;
}

void Draw::DrawPolygon(const b2Vec2* vertices, int vertexCount, b2HexColor color)
{
	b2Vec2 p1 = vertices[vertexCount - 1];
	for (int i = 0; i < vertexCount; ++i)
	{
		b2Vec2 p2 = vertices[i];
		m_lines->AddLine(p1, p2, color);
		p1 = p2;
	}
}

void Draw::DrawSolidPolygon(b2Transform transform, const b2Vec2* vertices, int vertexCount, float radius, b2HexColor color)
{
	m_polygons->AddPolygon(transform, vertices, vertexCount, radius, color);
}

// todo use SDF
void Draw::DrawCircle(b2Vec2 center, float radius, b2HexColor color)
{
	const float k_segments = 32.0f;
	const float k_increment = 2.0f * b2_pi / k_segments;
	float sinInc = sinf(k_increment);
	float cosInc = cosf(k_increment);
	b2Vec2 r1 = {1.0f, 0.0f};
	b2Vec2 v1 = b2MulAdd(center, radius, r1);
	for (int i = 0; i < k_segments; ++i)
	{
		// Perform rotation to avoid additional trigonometry.
		b2Vec2 r2;
		r2.x = cosInc * r1.x - sinInc * r1.y;
		r2.y = sinInc * r1.x + cosInc * r1.y;
		b2Vec2 v2 = b2MulAdd(center, radius, r2);
		m_lines->AddLine(v1, v2, color);
		r1 = r2;
		v1 = v2;
	}
}

void Draw::DrawSolidCircle(b2Transform transform, b2Vec2 center, float radius, b2HexColor color)
{
	b2Transform xf = transform;
	xf.p = b2TransformPoint(transform, center);
	m_circles->AddCircle(transform, radius, color);
}

// todo use SDF
void Draw::DrawCapsule(b2Vec2 p1, b2Vec2 p2, float radius, b2HexColor color)
{
	float length;
	b2Vec2 axis = b2GetLengthAndNormalize(&length, b2Sub(p2, p1));

	if (length == 0.0f)
	{
		DrawCircle(p1, radius, color);
	}

	const float k_segments = 16.0f;
	const float k_increment = b2_pi / k_segments;
	float sinInc = sinf(k_increment);
	float cosInc = cosf(k_increment);

	b2Vec2 r1 = {-axis.y, axis.x};
	b2Vec2 v1 = b2MulAdd(p1, radius, r1);
	b2Vec2 a = v1;
	for (int i = 0; i < k_segments; ++i)
	{
		// Perform rotation to avoid additional trigonometry.
		b2Vec2 r2;
		r2.x = cosInc * r1.x - sinInc * r1.y;
		r2.y = sinInc * r1.x + cosInc * r1.y;
		b2Vec2 v2 = b2MulAdd(p1, radius, r2);
		m_lines->AddLine(v1, v2, color);
		r1 = r2;
		v1 = v2;
	}
	b2Vec2 b = v1;

	r1 = {axis.y, -axis.x};
	v1 = b2MulAdd(p2, radius, r1);
	b2Vec2 c = v1;
	for (int i = 0; i < k_segments; ++i)
	{
		// Perform rotation to avoid additional trigonometry.
		b2Vec2 r2;
		r2.x = cosInc * r1.x - sinInc * r1.y;
		r2.y = sinInc * r1.x + cosInc * r1.y;
		b2Vec2 v2 = b2MulAdd(p2, radius, r2);
		m_lines->AddLine(v1, v2, color);
		r1 = r2;
		v1 = v2;
	}
	b2Vec2 d = v1;

	m_lines->AddLine(a, d, color);
	m_lines->AddLine(b, c, color);
	m_lines->AddLine(p1, p2, color);
}

void Draw::DrawSolidCapsule(b2Vec2 p1, b2Vec2 p2, float radius, b2HexColor color)
{
	m_capsules->AddCapsule(p1, p2, radius, color);
}

void Draw::DrawSegment(b2Vec2 p1, b2Vec2 p2, b2HexColor color)
{
	m_lines->AddLine(p1, p2, color);
}

void Draw::DrawTransform(b2Transform xf)
{
	const float k_axisScale = 0.2f;
	b2Vec2 p1 = xf.p;

	b2Vec2 p2 = b2MulAdd(p1, k_axisScale, b2Rot_GetXAxis(xf.q));
	m_lines->AddLine(p1, p2, b2_colorRed);

	p2 = b2MulAdd(p1, k_axisScale, b2Rot_GetYAxis(xf.q));
	m_lines->AddLine(p1, p2, b2_colorGreen);
}

void Draw::DrawPoint(b2Vec2 p, float size, b2HexColor color)
{
	m_points->AddPoint(p, size, color);
}

void Draw::DrawString(int x, int y, const char* string, ...)
{
	// if (m_showUI == false)
	//{
	//	return;
	// }

	va_list arg;
	va_start(arg, string);
	ImGui::Begin("Overlay", nullptr,
				 ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoInputs | ImGuiWindowFlags_AlwaysAutoResize |
					 ImGuiWindowFlags_NoScrollbar);
	ImGui::SetCursorPos(ImVec2(float(x), float(y)));
	ImGui::TextColoredV(ImColor(230, 153, 153, 255), string, arg);
	ImGui::End();
	va_end(arg);
}

void Draw::DrawString(b2Vec2 pw, const char* string, ...)
{
	b2Vec2 ps = g_camera.ConvertWorldToScreen(pw);

	va_list arg;
	va_start(arg, string);
	ImGui::Begin("Overlay", nullptr,
				 ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoInputs | ImGuiWindowFlags_AlwaysAutoResize |
					 ImGuiWindowFlags_NoScrollbar);
	ImGui::SetCursorPos(ImVec2(ps.x, ps.y));
	ImGui::TextColoredV(ImColor(230, 230, 230, 255), string, arg);
	ImGui::End();
	va_end(arg);
}

void Draw::DrawAABB(b2AABB aabb, b2HexColor c)
{
	b2Vec2 p1 = aabb.lowerBound;
	b2Vec2 p2 = {aabb.upperBound.x, aabb.lowerBound.y};
	b2Vec2 p3 = aabb.upperBound;
	b2Vec2 p4 = {aabb.lowerBound.x, aabb.upperBound.y};

	m_lines->AddLine(p1, p2, c);
	m_lines->AddLine(p2, p3, c);
	m_lines->AddLine(p3, p4, c);
	m_lines->AddLine(p4, p1, c);
}

void Draw::Flush()
{
	m_circles->Flush();
	m_capsules->Flush();
	m_polygons->Flush();
	m_triangles->Flush();
	m_lines->Flush();
	m_points->Flush();
	CheckErrorGL();
}
