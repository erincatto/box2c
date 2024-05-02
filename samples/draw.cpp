// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "draw.h"

#include "box2d/constants.h"
#include "box2d/math_cpp.h"
#include "box2d/math_functions.h"

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

static inline RGBA8 MakeRGBA8(b2Color c)
{
	return {uint8_t(0xFF * c.r), uint8_t(0xFF * c.g), uint8_t(0xFF * c.b), uint8_t(0xFF * c.a)};
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

static void CheckGLError()
{
	GLenum errCode = glGetError();
	if (errCode != GL_NO_ERROR)
	{
		printf("OpenGL error = %d\n", errCode);
		assert(false);
	}
}

// Prints shader compilation errors
static void PrintLog(GLuint object)
{
	GLint log_length = 0;
	if (glIsShader(object))
	{
		glGetShaderiv(object, GL_INFO_LOG_LENGTH, &log_length);
	}
	else if (glIsProgram(object))
	{
		glGetProgramiv(object, GL_INFO_LOG_LENGTH, &log_length);
	}
	else
	{
		printf("printlog: Not a shader or a program\n");
		return;
	}

	char* log = (char*)malloc(log_length);

	if (glIsShader(object))
	{
		glGetShaderInfoLog(object, log_length, nullptr, log);
	}
	else if (glIsProgram(object))
	{
		glGetProgramInfoLog(object, log_length, nullptr, log);
	}

	printf("%s", log);
	free(log);
}

//
static GLuint sCreateShaderFromString(const char* source, GLenum type)
{
	GLuint res = glCreateShader(type);
	const char* sources[] = {source};
	glShaderSource(res, 1, sources, nullptr);
	glCompileShader(res);
	GLint status = GL_FALSE;
	glGetShaderiv(res, GL_COMPILE_STATUS, &status);
	if (status == GL_FALSE)
	{
		printf("Error compiling shader of type %d!\n", type);
		PrintLog(res);
		glDeleteShader(res);
		return 0;
	}

	return res;
}

//
static GLuint sCreateShaderProgram(const char* vs, const char* fs)
{
	GLuint vsId = sCreateShaderFromString(vs, GL_VERTEX_SHADER);
	GLuint fsId = sCreateShaderFromString(fs, GL_FRAGMENT_SHADER);
	assert(vsId != 0 && fsId != 0);

	GLuint programId = glCreateProgram();
	glAttachShader(programId, vsId);
	glAttachShader(programId, fsId);
	glBindFragDataLocation(programId, 0, "color");
	glLinkProgram(programId);

	glDeleteShader(vsId);
	glDeleteShader(fsId);

	GLint status = GL_FALSE;
	glGetProgramiv(programId, GL_LINK_STATUS, &status);
	if (status == GL_FALSE)
	{
		printf("Error linking shader program!\n");
		PrintLog(programId);
		glDeleteProgram(programId);
		return 0;
	}

	return programId;
}

//
struct GLRenderPoints
{
	void Create()
	{
		const char* vs = "#version 330\n"
						 "uniform mat4 projectionMatrix;\n"
						 "layout(location = 0) in vec2 v_position;\n"
						 "layout(location = 1) in vec4 v_color;\n"
						 "layout(location = 2) in float v_size;\n"
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

		m_programId = sCreateShaderProgram(vs, fs);
		m_projectionUniform = glGetUniformLocation(m_programId, "projectionMatrix");
		m_vertexAttribute = 0;
		m_colorAttribute = 1;
		m_sizeAttribute = 2;

		// Generate
		glGenVertexArrays(1, &m_vaoId);
		glGenBuffers(3, m_vboIds);

		glBindVertexArray(m_vaoId);
		glEnableVertexAttribArray(m_vertexAttribute);
		glEnableVertexAttribArray(m_colorAttribute);
		glEnableVertexAttribArray(m_sizeAttribute);

		// Vertex buffer
		glBindBuffer(GL_ARRAY_BUFFER, m_vboIds[0]);
		glVertexAttribPointer(m_vertexAttribute, 2, GL_FLOAT, GL_FALSE, 0, BUFFER_OFFSET(0));
		glBufferData(GL_ARRAY_BUFFER, sizeof(m_vertices), m_vertices, GL_DYNAMIC_DRAW);

		glBindBuffer(GL_ARRAY_BUFFER, m_vboIds[1]);
		glVertexAttribPointer(m_colorAttribute, 4, GL_FLOAT, GL_FALSE, 0, BUFFER_OFFSET(0));
		glBufferData(GL_ARRAY_BUFFER, sizeof(m_colors), m_colors, GL_DYNAMIC_DRAW);

		glBindBuffer(GL_ARRAY_BUFFER, m_vboIds[2]);
		glVertexAttribPointer(m_sizeAttribute, 1, GL_FLOAT, GL_FALSE, 0, BUFFER_OFFSET(0));
		glBufferData(GL_ARRAY_BUFFER, sizeof(m_sizes), m_sizes, GL_DYNAMIC_DRAW);

		CheckGLError();

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
			glDeleteBuffers(3, m_vboIds);
			m_vaoId = 0;
		}

		if (m_programId)
		{
			glDeleteProgram(m_programId);
			m_programId = 0;
		}
	}

	void Vertex(const b2Vec2& v, const b2Color& c, float size)
	{
		if (m_count == e_maxVertices)
			Flush();

		m_vertices[m_count] = v;
		m_colors[m_count] = c;
		m_sizes[m_count] = size;
		++m_count;
	}

	void Flush()
	{
		if (m_count == 0)
			return;

		glUseProgram(m_programId);

		float proj[16] = {0.0f};
		g_camera.BuildProjectionMatrix(proj, 0.0f);

		glUniformMatrix4fv(m_projectionUniform, 1, GL_FALSE, proj);
		glBindVertexArray(m_vaoId);

		glBindBuffer(GL_ARRAY_BUFFER, m_vboIds[0]);
		glBufferSubData(GL_ARRAY_BUFFER, 0, m_count * sizeof(b2Vec2), m_vertices);

		glBindBuffer(GL_ARRAY_BUFFER, m_vboIds[1]);
		glBufferSubData(GL_ARRAY_BUFFER, 0, m_count * sizeof(b2Color), m_colors);

		glBindBuffer(GL_ARRAY_BUFFER, m_vboIds[2]);
		glBufferSubData(GL_ARRAY_BUFFER, 0, m_count * sizeof(float), m_sizes);

		glEnable(GL_PROGRAM_POINT_SIZE);
		glDrawArrays(GL_POINTS, 0, m_count);
		glDisable(GL_PROGRAM_POINT_SIZE);

		CheckGLError();

		glBindBuffer(GL_ARRAY_BUFFER, 0);
		glBindVertexArray(0);
		glUseProgram(0);

		m_count = 0;
	}

	enum
	{
		e_maxVertices = 512
	};
	b2Vec2 m_vertices[e_maxVertices];
	b2Color m_colors[e_maxVertices];
	float m_sizes[e_maxVertices];

	int m_count;

	GLuint m_vaoId;
	GLuint m_vboIds[3];
	GLuint m_programId;
	GLint m_projectionUniform;
	GLint m_vertexAttribute;
	GLint m_colorAttribute;
	GLint m_sizeAttribute;
};

//
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

		m_programId = sCreateShaderProgram(vs, fs);
		m_projectionUniform = glGetUniformLocation(m_programId, "projectionMatrix");
		m_vertexAttribute = 0;
		m_colorAttribute = 1;

		// Generate
		glGenVertexArrays(1, &m_vaoId);
		glGenBuffers(2, m_vboIds);

		glBindVertexArray(m_vaoId);
		glEnableVertexAttribArray(m_vertexAttribute);
		glEnableVertexAttribArray(m_colorAttribute);

		// Vertex buffer
		glBindBuffer(GL_ARRAY_BUFFER, m_vboIds[0]);
		glVertexAttribPointer(m_vertexAttribute, 2, GL_FLOAT, GL_FALSE, 0, BUFFER_OFFSET(0));
		glBufferData(GL_ARRAY_BUFFER, sizeof(m_vertices), m_vertices, GL_DYNAMIC_DRAW);

		glBindBuffer(GL_ARRAY_BUFFER, m_vboIds[1]);
		glVertexAttribPointer(m_colorAttribute, 4, GL_FLOAT, GL_FALSE, 0, BUFFER_OFFSET(0));
		glBufferData(GL_ARRAY_BUFFER, sizeof(m_colors), m_colors, GL_DYNAMIC_DRAW);

		CheckGLError();

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

	void Vertex(const b2Vec2& v, const b2Color& c)
	{
		if (m_count == e_maxVertices)
			Flush();

		m_vertices[m_count] = v;
		m_colors[m_count] = c;
		++m_count;
	}

	void Flush()
	{
		if (m_count == 0)
			return;

		glUseProgram(m_programId);

		float proj[16] = {0.0f};
		g_camera.BuildProjectionMatrix(proj, 0.1f);

		glUniformMatrix4fv(m_projectionUniform, 1, GL_FALSE, proj);

		glBindVertexArray(m_vaoId);

		glBindBuffer(GL_ARRAY_BUFFER, m_vboIds[0]);
		glBufferSubData(GL_ARRAY_BUFFER, 0, m_count * sizeof(b2Vec2), m_vertices);

		glBindBuffer(GL_ARRAY_BUFFER, m_vboIds[1]);
		glBufferSubData(GL_ARRAY_BUFFER, 0, m_count * sizeof(b2Color), m_colors);

		glDrawArrays(GL_LINES, 0, m_count);

		CheckGLError();

		glBindBuffer(GL_ARRAY_BUFFER, 0);
		glBindVertexArray(0);
		glUseProgram(0);

		m_count = 0;
	}

	// need lots of space for lines so they draw last
	// could also consider disabling depth buffer
	enum
	{
		e_maxVertices = 32 * 512
	};

	b2Vec2 m_vertices[e_maxVertices];
	b2Color m_colors[e_maxVertices];

	int m_count;

	GLuint m_vaoId;
	GLuint m_vboIds[2];
	GLuint m_programId;
	GLint m_projectionUniform;
	GLint m_vertexAttribute;
	GLint m_colorAttribute;
};

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

		m_programId = sCreateShaderProgram(vs, fs);
		m_projectionUniform = glGetUniformLocation(m_programId, "projectionMatrix");
		m_vertexAttribute = 0;
		m_colorAttribute = 1;

		// Generate
		glGenVertexArrays(1, &m_vaoId);
		glGenBuffers(2, m_vboIds);

		glBindVertexArray(m_vaoId);
		glEnableVertexAttribArray(m_vertexAttribute);
		glEnableVertexAttribArray(m_colorAttribute);

		// Vertex buffer
		glBindBuffer(GL_ARRAY_BUFFER, m_vboIds[0]);
		glVertexAttribPointer(m_vertexAttribute, 2, GL_FLOAT, GL_FALSE, 0, BUFFER_OFFSET(0));
		glBufferData(GL_ARRAY_BUFFER, sizeof(m_vertices), m_vertices, GL_DYNAMIC_DRAW);

		glBindBuffer(GL_ARRAY_BUFFER, m_vboIds[1]);
		glVertexAttribPointer(m_colorAttribute, 4, GL_FLOAT, GL_FALSE, 0, BUFFER_OFFSET(0));
		glBufferData(GL_ARRAY_BUFFER, sizeof(m_colors), m_colors, GL_DYNAMIC_DRAW);

		CheckGLError();

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

	void Vertex(const b2Vec2& v, const b2Color& c)
	{
		if (m_count == e_maxVertices)
		{
			Flush();
		}

		m_vertices[m_count] = v;
		m_colors[m_count] = c;
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

		glBindVertexArray(m_vaoId);

		glBindBuffer(GL_ARRAY_BUFFER, m_vboIds[0]);
		glBufferSubData(GL_ARRAY_BUFFER, 0, m_count * sizeof(b2Vec2), m_vertices);

		glBindBuffer(GL_ARRAY_BUFFER, m_vboIds[1]);
		glBufferSubData(GL_ARRAY_BUFFER, 0, m_count * sizeof(b2Color), m_colors);

		glEnable(GL_BLEND);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
		glDrawArrays(GL_TRIANGLES, 0, m_count);
		glDisable(GL_BLEND);

		CheckGLError();

		glBindBuffer(GL_ARRAY_BUFFER, 0);
		glBindVertexArray(0);
		glUseProgram(0);

		m_count = 0;
	}

	enum
	{
		e_maxVertices = 3 * 512
	};

	b2Vec2 m_vertices[e_maxVertices];
	b2Color m_colors[e_maxVertices];

	int m_count;

	GLuint m_vaoId;
	GLuint m_vboIds[2];
	GLuint m_programId;
	GLint m_projectionUniform;
	GLint m_vertexAttribute;
	GLint m_colorAttribute;
};

struct Transform
{
	float x, y, c, s;
};

// Draws SDF circles using quad instancing. Apparently instancing of quads can be slow on older GPUs.
// https://www.reddit.com/r/opengl/comments/q7yikr/how_to_draw_several_quads_through_instancing/
// https://www.g-truc.net/post-0666.html
struct GLRenderCircles
{
	void Create()
	{
		const char* vs = SHADER_TEXT(
		 uniform mat4 projectionMatrix;
		 uniform float zoom;
		 
		 layout(location = 0) in vec2 v_localPosition;
		 layout(location = 1) in vec4 v_instanceTransform;
		 layout(location = 2) in float v_instanceRadius;
		 layout(location = 3) in vec4 v_instanceColor;
		 
		 out vec2 f_position;
		 out vec4 f_color;
		 out float f_zoom;

		 void main()
		 {
			f_position = v_localPosition;
			f_color = v_instanceColor;
			float radius = v_instanceRadius;

			// scale zoom so the border is fixed size
			f_zoom = zoom / radius;

			float x = v_instanceTransform.x;
			float y = v_instanceTransform.y;
			float c = v_instanceTransform.z;
			float s = v_instanceTransform.w;
			vec2 p = vec2(radius * v_localPosition.x, radius * v_localPosition.y);
			p = vec2((c * p.x - s * p.y) + x, (s * p.x + c * p.y) + y);
			gl_Position = projectionMatrix * vec4(p, 0.0f, 1.0f);
		});

		// Thanks to baz! for help on this shader
		// todo this can be optimized a bit, keeping some terms for clarity
		const char* fs = SHADER_TEXT(
			in vec2 f_position;
			in vec4 f_color;
			in float f_zoom;

			out vec4 color;

			// https://en.wikipedia.org/wiki/Alpha_compositing
			vec4 blend_colors(vec4 front, vec4 back)
			{
				vec3 cSrc = front.rgb;
				float alphaSrc = front.a;
				vec3 cDst = back.rgb;
				float alphaDst = back.a;

				vec3 cOut = cSrc * alphaSrc + cDst * alphaDst * (1.0 - alphaSrc);
				float alphaOut = alphaSrc + alphaDst * (1.0 - alphaSrc);
				cOut = cOut / alphaOut;

				return vec4(cOut, alphaOut);
			}

			void main()
			{
				// radius in unit quad
				float radius = 1.0;

				// distance to axis line segment
				vec2 e = vec2(radius, 0);
				vec2 w = f_position;
				float we = dot(w, e);
				vec2 b = w - e * clamp(we / dot(e, e), 0.0, 1.0);
				float da = sqrt(dot(b, b));

				// distance to circle
				float dw = sqrt(dot(w, w));
				float dc = abs(dw - radius);

				// union of circle and axis
				float d = min(da, dc);

				vec4 borderColor = f_color;
				vec4 fillColor = 0.6f * borderColor;

				// scale border by zoom so the pixel width is constant
				float borderThickness = 0.07f * f_zoom;

				vec4 back = vec4(fillColor.rgb, fillColor.a * (1.0 - smoothstep(radius, radius + borderThickness, dw)));
				vec4 front = vec4(borderColor.rgb, 1.0 - smoothstep(0.0, borderThickness, d));
				color = blend_colors(front, back);
			});

		m_programId = sCreateShaderProgram(vs, fs);
		m_projectionUniform = glGetUniformLocation(m_programId, "projectionMatrix");
		m_zoomUniform = glGetUniformLocation(m_programId, "zoom");
		int vertexAttribute = 0;
		int instanceTransformAttribute = 1;
		int instanceRadiusAttribute = 2;
		int instanceColorAttribute = 3;

		// Generate
		glGenVertexArrays(1, &m_vaoId);
		glGenBuffers(4, m_vboIds);

		glBindVertexArray(m_vaoId);
		glEnableVertexAttribArray(vertexAttribute);
		glEnableVertexAttribArray(instanceTransformAttribute);
		glEnableVertexAttribArray(instanceRadiusAttribute);
		glEnableVertexAttribArray(instanceColorAttribute);

		// Vertex buffer for single quad
		float a = 1.1f;
		b2Vec2 vertices[] = {{-a, -a}, {a, -a}, {-a, a}, {a, -a}, {a, a}, {-a, a}};
		glBindBuffer(GL_ARRAY_BUFFER, m_vboIds[0]);
		glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);
		glVertexAttribPointer(vertexAttribute, 2, GL_FLOAT, GL_FALSE, 0, BUFFER_OFFSET(0));

		// Transform buffer
		glBindBuffer(GL_ARRAY_BUFFER, m_vboIds[1]);
		glBufferData(GL_ARRAY_BUFFER, sizeof(m_transforms), NULL, GL_DYNAMIC_DRAW);
		glVertexAttribPointer(instanceTransformAttribute, 4, GL_FLOAT, GL_FALSE, 0, BUFFER_OFFSET(0));
		glVertexAttribDivisor(instanceTransformAttribute, 1);

		// Radii buffer
		glBindBuffer(GL_ARRAY_BUFFER, m_vboIds[2]);
		glBufferData(GL_ARRAY_BUFFER, sizeof(m_radii), NULL, GL_DYNAMIC_DRAW);
		glVertexAttribPointer(instanceRadiusAttribute, 1, GL_FLOAT, GL_FALSE, 0, BUFFER_OFFSET(0));
		glVertexAttribDivisor(instanceRadiusAttribute, 1);

		// Color buffer
		glBindBuffer(GL_ARRAY_BUFFER, m_vboIds[3]);
		glBufferData(GL_ARRAY_BUFFER, sizeof(m_colors), NULL, GL_DYNAMIC_DRAW);
		glVertexAttribPointer(instanceColorAttribute, 4, GL_FLOAT, GL_FALSE, 0, BUFFER_OFFSET(0));
		glVertexAttribDivisor(instanceColorAttribute, 1);

		CheckGLError();

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
			glDeleteBuffers(4, m_vboIds);
			m_vaoId = 0;
		}

		if (m_programId)
		{
			glDeleteProgram(m_programId);
			m_programId = 0;
		}
	}

	void AddCircle(const b2Transform& transform, float radius, const b2Color& c)
	{
		if (m_count == e_maxCount)
		{
			Flush();
		}

		m_transforms[m_count] = {transform.p.x, transform.p.y, transform.q.c, transform.q.s};
		m_radii[m_count] = radius;
		m_colors[m_count] = c;
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
		glBufferSubData(GL_ARRAY_BUFFER, 0, m_count * sizeof(Transform), m_transforms);
		CheckGLError();

		glBindBuffer(GL_ARRAY_BUFFER, m_vboIds[2]);
		glBufferSubData(GL_ARRAY_BUFFER, 0, m_count * sizeof(float), m_radii);
		CheckGLError();

		glBindBuffer(GL_ARRAY_BUFFER, m_vboIds[3]);
		glBufferSubData(GL_ARRAY_BUFFER, 0, m_count * sizeof(b2Color), m_colors);
		CheckGLError();

		glEnable(GL_BLEND);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
		glDrawArraysInstanced(GL_TRIANGLES, 0, 6, m_count);
		glDisable(GL_BLEND);

		CheckGLError();

		glBindBuffer(GL_ARRAY_BUFFER, 0);
		glBindVertexArray(0);
		glUseProgram(0);

		m_count = 0;
	}

	enum
	{
		e_maxCount = 3 * 512
	};

	// todo combine into struct like PolygonData
	Transform m_transforms[e_maxCount];
	float m_radii[e_maxCount];
	b2Color m_colors[e_maxCount];

	int m_count;

	GLuint m_vaoId;
	GLuint m_vboIds[4];
	GLuint m_programId;
	GLint m_projectionUniform;
	GLint m_zoomUniform;
};

struct CapsuleData
{
	float radius;
	float length;
};

// Draw capsules using SDF-based shader
struct GLRenderCapsules
{
	void Create()
	{
		const char* vs = SHADER_TEXT(
		uniform mat4 projectionMatrix;
		uniform float zoom;

		layout(location = 0) in vec2 v_localPosition;
		layout(location = 1) in vec4 v_instanceTransform;
		layout(location = 2) in vec2 v_instanceRadiusAndLength;
		layout(location = 3) in vec4 v_instanceColor;
		
		out vec2 f_position;
		out vec4 f_color;
		out float f_length;
		out float f_zoom;
		
		void main()
		{
			f_position = v_localPosition;
			f_color = v_instanceColor;

			float radius = v_instanceRadiusAndLength.x;
			float length = v_instanceRadiusAndLength.y;

			// scale quad large enough to hold capsule
			float scale = radius + 0.5 * length;

			// quad range of [-1, 1] implies normalize radius and length
			f_length = 2.0 * length / (2.0 * radius + length);

			// scale zoom so the border is fixed size
			f_zoom = zoom / scale;

			float x = v_instanceTransform.x;
			float y = v_instanceTransform.y;
			float c = v_instanceTransform.z;
			float s = v_instanceTransform.w;
			vec2 p = vec2(scale * v_localPosition.x, scale * v_localPosition.y);
			p = vec2((c * p.x - s * p.y) + x, (s * p.x + c * p.y) + y);
			gl_Position = projectionMatrix * vec4(p, 0.0f, 1.0f);
		});

		// Thanks to baz! for help on this shader
		// todo this can be optimized a bit, keeping some terms for clarity
		const char* fs = SHADER_TEXT(
			
		in vec2 f_position;
		in vec4 f_color;
		in float f_length;
		in float f_zoom;

		out vec4 color;
		
		// https://en.wikipedia.org/wiki/Alpha_compositing
		vec4 blend_colors(vec4 front, vec4 back)
		{
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

		void main()
		{
			// radius in unit quad
			float radius = 0.5 * (2.0 - f_length);

			vec4 borderColor = f_color;
			vec4 fillColor = 0.6f * borderColor;

			vec2 v1 = vec2(-0.5 * f_length, 0);
			vec2 v2 = vec2(0.5 * f_length, 0);

			// distance to line segment
			vec2 e = v2 - v1;
			vec2 w = f_position - v1;
			float we = dot(w, e);
			vec2 b = w - e * clamp(we / dot(e, e), 0.0, 1.0);
			float dw = sqrt(dot(b, b));

			// SDF union of capsule and line segment
			float d = min(dw, abs(dw - radius));

			float borderThickness = 0.07 * f_zoom;

			// roll the fill alpha down at the border
			vec4 back = vec4(fillColor.rgb, fillColor.a * (1.0 - smoothstep(radius, radius + borderThickness, dw)));

			// roll the border alpha down from 1 to 0 across the border thickness
			vec4 front = vec4(borderColor.rgb, 1.0 - smoothstep(0.0, borderThickness, d));

			color = blend_colors(front, back);
		});

		m_programId = sCreateShaderProgram(vs, fs);
		m_projectionUniform = glGetUniformLocation(m_programId, "projectionMatrix");
		m_zoomUniform = glGetUniformLocation(m_programId, "zoom");
		int vertexAttribute = 0;
		int instanceTransformAttribute = 1;
		int instanceRadiusLengthAttribute = 2;
		int instanceColorAttribute = 3;

		// Generate
		glGenVertexArrays(1, &m_vaoId);
		glGenBuffers(4, m_vboIds);

		glBindVertexArray(m_vaoId);
		glEnableVertexAttribArray(vertexAttribute);
		glEnableVertexAttribArray(instanceTransformAttribute);
		glEnableVertexAttribArray(instanceRadiusLengthAttribute);
		glEnableVertexAttribArray(instanceColorAttribute);

		// Vertex buffer for single quad
		float a = 1.1f;
		b2Vec2 vertices[] = {{-a, -a}, {a, -a}, {-a, a}, {a, -a}, {a, a}, {-a, a}};
		glBindBuffer(GL_ARRAY_BUFFER, m_vboIds[0]);
		glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);
		glVertexAttribPointer(vertexAttribute, 2, GL_FLOAT, GL_FALSE, 0, BUFFER_OFFSET(0));

		// Transform buffer
		glBindBuffer(GL_ARRAY_BUFFER, m_vboIds[1]);
		glBufferData(GL_ARRAY_BUFFER, sizeof(m_transforms), NULL, GL_DYNAMIC_DRAW);
		glVertexAttribPointer(instanceTransformAttribute, 4, GL_FLOAT, GL_FALSE, 0, BUFFER_OFFSET(0));
		glVertexAttribDivisor(1, 1);

		// Radii buffer
		glBindBuffer(GL_ARRAY_BUFFER, m_vboIds[2]);
		glBufferData(GL_ARRAY_BUFFER, sizeof(m_capsuleData), NULL, GL_DYNAMIC_DRAW);
		glVertexAttribPointer(instanceRadiusLengthAttribute, 2, GL_FLOAT, GL_FALSE, 0, BUFFER_OFFSET(0));
		glVertexAttribDivisor(2, 1);

		// Color buffer
		glBindBuffer(GL_ARRAY_BUFFER, m_vboIds[3]);
		glBufferData(GL_ARRAY_BUFFER, sizeof(m_colors), NULL, GL_DYNAMIC_DRAW);
		glVertexAttribPointer(instanceColorAttribute, 4, GL_FLOAT, GL_FALSE, 0, BUFFER_OFFSET(0));
		glVertexAttribDivisor(3, 1);

		CheckGLError();

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
			glDeleteBuffers(4, m_vboIds);
			m_vaoId = 0;
		}

		if (m_programId)
		{
			glDeleteProgram(m_programId);
			m_programId = 0;
		}
	}

	void AddCapsule(b2Vec2 p1, b2Vec2 p2, float radius, const b2Color& c)
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

		m_transforms[m_count] = {transform.p.x, transform.p.y, transform.q.c, transform.q.s};
		m_capsuleData[m_count] = {radius, length};
		m_colors[m_count] = c;
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
		glBufferSubData(GL_ARRAY_BUFFER, 0, m_count * sizeof(Transform), m_transforms);
		CheckGLError();

		glBindBuffer(GL_ARRAY_BUFFER, m_vboIds[2]);
		glBufferSubData(GL_ARRAY_BUFFER, 0, m_count * sizeof(CapsuleData), m_capsuleData);
		CheckGLError();

		glBindBuffer(GL_ARRAY_BUFFER, m_vboIds[3]);
		glBufferSubData(GL_ARRAY_BUFFER, 0, m_count * sizeof(b2Color), m_colors);
		CheckGLError();

		glEnable(GL_BLEND);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
		glDrawArraysInstanced(GL_TRIANGLES, 0, 6, m_count);
		glDisable(GL_BLEND);

		CheckGLError();

		glBindBuffer(GL_ARRAY_BUFFER, 0);
		glBindVertexArray(0);
		glUseProgram(0);

		m_count = 0;
	}

	enum
	{
		e_maxCount = 3 * 512
	};

	// todo combine into struct like PolygonData
	Transform m_transforms[e_maxCount];
	CapsuleData m_capsuleData[e_maxCount];
	b2Color m_colors[e_maxCount];

	int m_count;

	GLuint m_vaoId;
	GLuint m_vboIds[4];
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
		const char* vs = SHADER_TEXT(
			uniform mat4 projectionMatrix;
			uniform float zoom;

			layout(location = 0) in vec2 v_localPosition;
			layout(location = 1) in vec4 v_instanceTransform;
			layout(location = 2) in vec4 v_instancePoints12;
			layout(location = 3) in vec4 v_instancePoints34;
			layout(location = 4) in vec4 v_instancePoints56;
			layout(location = 5) in vec4 v_instancePoints78;
			layout(location = 6) in int v_instanceCount;
			layout(location = 7) in float v_instanceRadius;
			layout(location = 8) in vec4 v_instanceColor;
			
			out vec2 f_position;
			out vec4 f_color;
			out vec2 f_points[8];
			flat out int f_count;
			out float f_radius;
			out float f_zoom;
			
			void main()
			{
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

				//if (v_instanceCount == 4)
				//{
				//	f_color = vec4(0, 0, 1, 1);
				//}

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
			
			in vec2 f_position;
			in vec2 f_points[8];
			flat in int f_count;
			in float f_radius;
			in vec4 f_color;
			in float f_zoom;

			out vec4 fragColor;
			
			// https://en.wikipedia.org/wiki/Alpha_compositing
			vec4 blend_colors(vec4 front, vec4 back)
			{
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

			float cross2d(in vec2 v1, in vec2 v2)
			{
				return v1.x * v2.y - v1.y * v2.x;
			}

			// Signed distance function for convex polygon
			float sdConvexPolygon(in vec2 p, in vec2[8] v, in int count)
			{
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

			void main()
			{
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

				//fragColor = vec4(0.5);
			});

		m_programId = sCreateShaderProgram(vs, fs);
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
		glVertexAttribDivisor(instanceTransform, 1);
		glVertexAttribDivisor(instancePoint12, 1);
		glVertexAttribDivisor(instancePoint34, 1);
		glVertexAttribDivisor(instancePoint56, 1);
		glVertexAttribDivisor(instancePoint78, 1);
		glVertexAttribDivisor(instancePointCount, 1);
		glVertexAttribDivisor(instanceRadius, 1);
		glVertexAttribDivisor(instanceColor, 1);

		CheckGLError();

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

	void AddPolygon(const b2Transform& transform, const b2Vec2* points, int count, float radius, const b2Color& color)
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
		data->color = MakeRGBA8(color);
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
		CheckGLError();

		glEnable(GL_BLEND);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
		glDrawArraysInstanced(GL_TRIANGLES, 0, 6, m_count);
		glDisable(GL_BLEND);

		CheckGLError();

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

void DrawPolygonFcn(const b2Vec2* vertices, int vertexCount, b2Color color, void* context)
{
	static_cast<Draw*>(context)->DrawPolygon(vertices, vertexCount, color);
}

void DrawSolidPolygonFcn(b2Transform transform, const b2Vec2* vertices, int vertexCount, float radius, b2Color color,
						 void* context)
{
	static_cast<Draw*>(context)->DrawSolidPolygon(transform, vertices, vertexCount, radius, color);
}

void DrawCircleFcn(b2Vec2 center, float radius, b2Color color, void* context)
{
	static_cast<Draw*>(context)->DrawCircle(center, radius, color);
}

void DrawSolidCircleFcn(b2Transform transform, float radius, b2Color color, void* context)
{
	static_cast<Draw*>(context)->DrawSolidCircle(transform, b2Vec2_zero, radius, color);
}

void DrawCapsuleFcn(b2Vec2 p1, b2Vec2 p2, float radius, b2Color color, void* context)
{
	static_cast<Draw*>(context)->DrawCapsule(p1, p2, radius, color);
}

void DrawSolidCapsuleFcn(b2Vec2 p1, b2Vec2 p2, float radius, b2Color color, void* context)
{
	static_cast<Draw*>(context)->DrawSolidCapsule(p1, p2, radius, color);
}

void DrawSegmentFcn(b2Vec2 p1, b2Vec2 p2, b2Color color, void* context)
{
	static_cast<Draw*>(context)->DrawSegment(p1, p2, color);
}

void DrawTransformFcn(b2Transform xf, void* context)
{
	static_cast<Draw*>(context)->DrawTransform(xf);
}

void DrawPointFcn(b2Vec2 p, float size, b2Color color, void* context)
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
	m_points = static_cast<GLRenderPoints*>(malloc(sizeof(GLRenderPoints)));
	m_points->Create();
	m_lines = static_cast<GLRenderLines*>(malloc(sizeof(GLRenderLines)));
	m_lines->Create();
	m_triangles = static_cast<GLRenderTriangles*>(malloc(sizeof(GLRenderTriangles)));
	m_triangles->Create();
	m_circles = static_cast<GLRenderCircles*>(malloc(sizeof(GLRenderCircles)));
	m_circles->Create();
	m_capsules = static_cast<GLRenderCapsules*>(malloc(sizeof(GLRenderCapsules)));
	m_capsules->Create();
	m_polygons = static_cast<GLRenderPolygons*>(malloc(sizeof(GLRenderPolygons)));
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
	free(m_points);
	m_points = nullptr;

	m_lines->Destroy();
	free(m_lines);
	m_lines = nullptr;

	m_triangles->Destroy();
	free(m_triangles);
	m_triangles = nullptr;

	m_circles->Destroy();
	free(m_circles);
	m_circles = nullptr;

	m_capsules->Destroy();
	free(m_capsules);
	m_capsules = nullptr;

	m_polygons->Destroy();
	free(m_polygons);
	m_polygons = nullptr;
}

void Draw::DrawPolygon(const b2Vec2* vertices, int vertexCount, b2Color color)
{
	b2Vec2 p1 = vertices[vertexCount - 1];
	for (int i = 0; i < vertexCount; ++i)
	{
		b2Vec2 p2 = vertices[i];
		m_lines->Vertex(p1, color);
		m_lines->Vertex(p2, color);
		p1 = p2;
	}
}

void Draw::DrawSolidPolygon(b2Transform transform, const b2Vec2* vertices, int vertexCount, float radius, b2Color color)
{
	m_polygons->AddPolygon(transform, vertices, vertexCount, radius, color);
}

// todo use SDF
void Draw::DrawCircle(b2Vec2 center, float radius, b2Color color)
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
		m_lines->Vertex(v1, color);
		m_lines->Vertex(v2, color);
		r1 = r2;
		v1 = v2;
	}
}

void Draw::DrawSolidCircle(b2Transform transform, b2Vec2 center, float radius, b2Color color)
{
	b2Transform xf = transform;
	xf.p = b2TransformPoint(transform, center);
	m_circles->AddCircle(transform, radius, color);
}

// todo use SDF
void Draw::DrawCapsule(b2Vec2 p1, b2Vec2 p2, float radius, b2Color color)
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
		m_lines->Vertex(v1, color);
		m_lines->Vertex(v2, color);
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
		m_lines->Vertex(v1, color);
		m_lines->Vertex(v2, color);
		r1 = r2;
		v1 = v2;
	}
	b2Vec2 d = v1;

	m_lines->Vertex(a, color);
	m_lines->Vertex(d, color);

	m_lines->Vertex(b, color);
	m_lines->Vertex(c, color);

	m_lines->Vertex(p1, color);
	m_lines->Vertex(p2, color);
}

void Draw::DrawSolidCapsule(b2Vec2 p1, b2Vec2 p2, float radius, b2Color color)
{
	m_capsules->AddCapsule(p1, p2, radius, color);
}

void Draw::DrawSegment(b2Vec2 p1, b2Vec2 p2, b2Color color)
{
	m_lines->Vertex(p1, color);
	m_lines->Vertex(p2, color);
}

void Draw::DrawTransform(b2Transform xf)
{
	const float k_axisScale = 0.2f;
	b2Color red = {1.0f, 0.0f, 0.0f, 1.0f};
	b2Color green = {0.0f, 1.0f, 0.0f, 1.0f};
	b2Vec2 p1 = xf.p;

	m_lines->Vertex(p1, red);
	b2Vec2 p2 = b2MulAdd(p1, k_axisScale, b2Rot_GetXAxis(xf.q));
	m_lines->Vertex(p2, red);

	m_lines->Vertex(p1, green);
	p2 = b2MulAdd(p1, k_axisScale, b2Rot_GetYAxis(xf.q));
	m_lines->Vertex(p2, green);
}

void Draw::DrawPoint(b2Vec2 p, float size, b2Color color)
{
	m_points->Vertex(p, color, size);
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

void Draw::DrawAABB(b2AABB aabb, b2Color c)
{
	b2Vec2 p1 = aabb.lowerBound;
	b2Vec2 p2 = {aabb.upperBound.x, aabb.lowerBound.y};
	b2Vec2 p3 = aabb.upperBound;
	b2Vec2 p4 = {aabb.lowerBound.x, aabb.upperBound.y};

	m_lines->Vertex(p1, c);
	m_lines->Vertex(p2, c);

	m_lines->Vertex(p2, c);
	m_lines->Vertex(p3, c);

	m_lines->Vertex(p3, c);
	m_lines->Vertex(p4, c);

	m_lines->Vertex(p4, c);
	m_lines->Vertex(p1, c);
}

void Draw::Flush()
{
	m_circles->Flush();
	m_capsules->Flush();
	m_polygons->Flush();
	m_triangles->Flush();
	m_lines->Flush();
	m_points->Flush();
	CheckGLError();
}
