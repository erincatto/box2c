// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "types.h"

#include <math.h>

#ifdef __cplusplus
extern "C"
{
#endif

#define B2_MIN(A, B) ((A) < (B) ? (A) : (B))
#define B2_MAX(A, B) ((A) > (B) ? (A) : (B))
#define B2_ABS(A) ((A) > 0.0f ? (A) : -(A))
#define B2_CLAMP(A, B, C) B2_MIN(B2_MAX(A, B), C)

static const b2Vec2 b2Vec2_zero = {0.0f, 0.0f};
static const b2Vec3 b2Vec3_zero = {0.0f, 0.0f, 0.0f};
static const b2Rot b2Rot_identity = {0.0f, 1.0f};
static const b2Transform b2Transform_identity = {{0.0f, 0.0f}, {0.0f, 1.0f}};
static const b2Mat22 b2Mat22_zero = {{0.0f, 0.0f}, {0.0f, 0.0f}};
static const b2Mat33 b2Mat33_zero = {{0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}};

bool b2IsValid(float a);
bool b2IsValidVec2(b2Vec2 v);

/// Make a vector
static inline b2Vec2 b2MakeVec2(float x, float y)
{
	return B2_LITERAL(b2Vec2){x, y};
}

/// Vector dot product
static inline float b2Dot(b2Vec2 a, b2Vec2 b)
{
	return a.x * b.x + a.y * b.y;
}

/// Vector cross product. In 2D this yields a scalar.
static inline float b2Cross(b2Vec2 a, b2Vec2 b)
{
	return a.x * b.y - a.y * b.x;
}

/// Perform the dot product on two 3-vectors.
static inline float b2Dot3(b2Vec3 a, b2Vec3 b)
{
	return a.x * b.x + a.y * b.y + a.z * b.z;
}

/// Perform the cross product on two 3-vectors.
static inline b2Vec3 b2Cross3(b2Vec3 a, b2Vec3 b)
{
	return B2_LITERAL(b2Vec3){a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}

/// Perform the cross product on a vector and a scalar. In 2D this produces
/// a vector.
static inline b2Vec2 b2CrossVS(b2Vec2 v, float s)
{
	return B2_LITERAL(b2Vec2){s * v.y, -s * v.x};
}

/// Perform the cross product on a scalar and a vector. In 2D this produces
/// a vector.
static inline b2Vec2 b2CrossSV(float s, b2Vec2 v)
{
	return B2_LITERAL(b2Vec2){-s * v.y, s * v.x};
}

/// Get a left pointing perpendicular vector. Equivalent to b2CrossSV(1.0f, v)
static inline b2Vec2 b2LeftPerp(b2Vec2 v)
{
	return B2_LITERAL(b2Vec2){-v.y, v.x};
}

/// Get a right pointing perpendicular vector. Equivalent to b2CrossVS(v, 1.0f)
static inline b2Vec2 b2RightPerp(b2Vec2 v)
{
	return B2_LITERAL(b2Vec2){v.y, -v.x};
}

/// Vector addition
static inline b2Vec2 b2Add(b2Vec2 a, b2Vec2 b)
{
	return B2_LITERAL(b2Vec2){a.x + b.x, a.y + b.y};
}

/// Vector subtraction
static inline b2Vec2 b2Sub(b2Vec2 a, b2Vec2 b)
{
	return B2_LITERAL(b2Vec2){a.x - b.x, a.y - b.y};
}

/// Vector subtraction
static inline b2Vec2 b2Neg(b2Vec2 a)
{
	return B2_LITERAL(b2Vec2){-a.x, -a.y};
}

/// Vector linear interpolation
static inline b2Vec2 b2Lerp(b2Vec2 a, b2Vec2 b, float t)
{
	return B2_LITERAL(b2Vec2){a.x + t * (b.x - a.x), a.y + t * (b.y - a.y)};
}

/// Component-wise multiplication
static inline b2Vec2 b2Mul(b2Vec2 a, b2Vec2 b)
{
	return B2_LITERAL(b2Vec2){a.x * b.x, a.y * b.y};
}

/// Multiply a scalar and vector
static inline b2Vec2 b2MulSV(float s, b2Vec2 v)
{
	return B2_LITERAL(b2Vec2){s * v.x, s * v.y};
}

/// a + s * b
static inline b2Vec2 b2MulAdd(b2Vec2 a, float s, b2Vec2 b)
{
	return B2_LITERAL(b2Vec2){a.x + s * b.x, a.y + s * b.y};
}

/// a - s * b
static inline b2Vec2 b2MulSub(b2Vec2 a, float s, b2Vec2 b)
{
	return B2_LITERAL(b2Vec2){a.x - s * b.x, a.y - s * b.y};
}

/// Component-wise absolute vector
static inline b2Vec2 b2Abs(b2Vec2 a)
{
	b2Vec2 b;
	b.x = B2_ABS(a.x);
	b.y = B2_ABS(a.y);
	return b;
}

/// Component-wise absolute vector
static inline b2Vec2 b2Min(b2Vec2 a, b2Vec2 b)
{
	b2Vec2 c;
	c.x = B2_MIN(a.x, b.x);
	c.y = B2_MIN(a.y, b.y);
	return c;
}

/// Component-wise absolute vector
static inline b2Vec2 b2Max(b2Vec2 a, b2Vec2 b)
{
	b2Vec2 c;
	c.x = B2_MAX(a.x, b.x);
	c.y = B2_MAX(a.y, b.y);
	return c;
}

/// Component-wise clamp vector so v into the range [a, b]
static inline b2Vec2 b2Clamp(b2Vec2 v, b2Vec2 a, b2Vec2 b)
{
	b2Vec2 c;
	c.x = B2_CLAMP(v.x, a.x, b.x);
	c.y = B2_CLAMP(v.y, a.y, b.y);
	return c;
}

/// Convert this vector into a unit vector
b2Vec2 b2Normalize(b2Vec2 v);

/// This asserts of the vector is too short
b2Vec2 b2NormalizeChecked(b2Vec2 v);

b2Vec2 b2GetLengthAndNormalize(float* length, b2Vec2 v);

/// Get the length of this vector (the norm).
static inline float b2Length(b2Vec2 v)
{
	return sqrtf(v.x * v.x + v.y * v.y);
}

/// Get the length of this vector (the norm).
static inline float b2LengthSquared(b2Vec2 v)
{
	return v.x * v.x + v.y * v.y;
}

static inline float b2Distance(b2Vec2 a, b2Vec2 b)
{
	float dx = b.x - a.x;
	float dy = b.y - a.y;
	return sqrtf(dx * dx + dy * dy);
}

/// Get the length of this vector (the norm).
static inline float b2DistanceSquared(b2Vec2 a, b2Vec2 b)
{
	b2Vec2 c = {b.x - a.x, b.y - a.y};
	return c.x * c.x + c.y * c.y;
}

/// Set using an angle in radians.
static inline b2Rot b2MakeRot(float angle)
{
	b2Rot q = {sinf(angle), cosf(angle)};
	return q;
}

/// Get the angle in radians
static inline float b2Rot_GetAngle(b2Rot q)
{
	return atan2f(q.s, q.c);
}

/// Get the x-axis
static inline b2Vec2 b2Rot_GetXAxis(b2Rot q)
{
	b2Vec2 v = {q.c, q.s};
	return v;
}

/// Get the y-axis
static inline b2Vec2 b2Rot_GetYAxis(b2Rot q)
{
	b2Vec2 v = {-q.s, q.c};
	return v;
}

/// Multiply two rotations: q * r
static inline b2Rot b2MulRot(b2Rot q, b2Rot r)
{
	// [qc -qs] * [rc -rs] = [qc*rc-qs*rs -qc*rs-qs*rc]
	// [qs  qc]   [rs  rc]   [qs*rc+qc*rs -qs*rs+qc*rc]
	// s = qs * rc + qc * rs
	// c = qc * rc - qs * rs
	b2Rot qr;
	qr.s = q.s * r.c + q.c * r.s;
	qr.c = q.c * r.c - q.s * r.s;
	return qr;
}

/// Transpose multiply two rotations: qT * r
static inline b2Rot b2InvMulRot(b2Rot q, b2Rot r)
{
	// [ qc qs] * [rc -rs] = [qc*rc+qs*rs -qc*rs+qs*rc]
	// [-qs qc]   [rs  rc]   [-qs*rc+qc*rs qs*rs+qc*rc]
	// s = qc * rs - qs * rc
	// c = qc * rc + qs * rs
	b2Rot qr;
	qr.s = q.c * r.s - q.s * r.c;
	qr.c = q.c * r.c + q.s * r.s;
	return qr;
}

/// Rotate a vector
static inline b2Vec2 b2RotateVector(b2Rot q, b2Vec2 v)
{
	return B2_LITERAL(b2Vec2){q.c * v.x - q.s * v.y, q.s * v.x + q.c * v.y};
}

/// Inverse rotate a vector
static inline b2Vec2 b2InvRotateVector(b2Rot q, b2Vec2 v)
{
	return B2_LITERAL(b2Vec2){q.c * v.x + q.s * v.y, -q.s * v.x + q.c * v.y};
}

/// Transform a point (e.g. local space to world space)
static inline b2Vec2 b2TransformPoint(b2Transform xf, const b2Vec2 p)
{
	float x = (xf.q.c * p.x - xf.q.s * p.y) + xf.p.x;
	float y = (xf.q.s * p.x + xf.q.c * p.y) + xf.p.y;

	return B2_LITERAL(b2Vec2){x, y};
}

// Inverse transform a point (e.g. world space to local space)
static inline b2Vec2 b2InvTransformPoint(b2Transform xf, const b2Vec2 p)
{
	float vx = p.x - xf.p.x;
	float vy = p.y - xf.p.y;
	return B2_LITERAL(b2Vec2){xf.q.c * vx + xf.q.s * vy, -xf.q.s * vx + xf.q.c * vy};
}

// v2 = A.q.Rot(B.q.Rot(v1) + B.p) + A.p
//    = (A.q * B.q).Rot(v1) + A.q.Rot(B.p) + A.p
static inline b2Transform b2MulTransforms(b2Transform A, b2Transform B)
{
	b2Transform C;
	C.q = b2MulRot(A.q, B.q);
	C.p = b2Add(b2RotateVector(A.q, B.p), A.p);
	return C;
}

// v2 = A.q' * (B.q * v1 + B.p - A.p)
//    = A.q' * B.q * v1 + A.q' * (B.p - A.p)
static inline b2Transform b2InvMulTransforms(b2Transform A, b2Transform B)
{
	b2Transform C;
	C.q = b2InvMulRot(A.q, B.q);
	C.p = b2InvRotateVector(A.q, b2Sub(B.p, A.p));
	return C;
}

static inline b2Vec2 b2MulMV(b2Mat22 A, b2Vec2 v)
{
	b2Vec2 u = {
		A.cx.x * v.x + A.cy.x * v.y,
		A.cx.y * v.x + A.cy.y * v.y,
	};
	return u;
}

static inline b2Vec3 b2MulMV33(b2Mat33 A, b2Vec3 v)
{
	b2Vec3 u = {
		A.cx.x * v.x + A.cy.x * v.y + A.cz.x * v.z,
		A.cx.y * v.x + A.cy.y * v.y + A.cz.y * v.z,
		A.cx.z * v.x + A.cy.z * v.y + A.cz.z * v.z,
	};
	return u;
}

static inline b2Mat22 b2GetInverse22(b2Mat22 A)
{
	float a = A.cx.x, b = A.cy.x, c = A.cx.y, d = A.cy.y;
	float det = a * d - b * c;
	if (det != 0.0f)
	{
		det = 1.0f / det;
	}

	b2Mat22 B = {
		{det * d, -det * c},
		{-det * b, det * a},
	};
	return B;
}

static inline b2Mat33 b2GetInverse33as22(b2Mat33 A)
{
	float a = A.cx.x, b = A.cy.x, c = A.cx.y, d = A.cy.y;
	float det = a * d - b * c;
	if (det != 0.0f)
	{
		det = 1.0f / det;
	}

	b2Mat33 B = {
		{det * d, -det * c, 0.0f},
		{-det * b, det * a, 0.0f},
		{0.0f, 0.0f, 0.0f},
	};
	return B;
}

/// Returns the zero matrix if singular.
static inline b2Mat33 b2GetSymInverse33(b2Mat33 A)
{
	float det = b2Dot3(A.cx, b2Cross3(A.cy, A.cz));
	if (det != 0.0f)
	{
		det = 1.0f / det;
	}

	float a11 = A.cx.x, a12 = A.cy.x, a13 = A.cz.x;
	float a22 = A.cy.y, a23 = A.cz.y;
	float a33 = A.cz.z;

	b2Mat33 B;
	B.cx.x = det * (a22 * a33 - a23 * a23);
	B.cx.y = det * (a13 * a23 - a12 * a33);
	B.cx.z = det * (a12 * a23 - a13 * a22);

	B.cy.x = B.cx.y;
	B.cy.y = det * (a11 * a33 - a13 * a13);
	B.cy.z = det * (a13 * a12 - a11 * a23);

	B.cz.x = B.cx.z;
	B.cz.y = B.cy.z;
	B.cz.z = det * (a11 * a22 - a12 * a12);
	return B;
}

/// Solve A * x = b, where b is a column vector.
static inline b2Vec2 b2Solve22(b2Mat22 A, b2Vec2 b)
{
	float a11 = A.cx.x, a12 = A.cy.x, a21 = A.cx.y, a22 = A.cy.y;
	float det = a11 * a22 - a12 * a21;
	if (det != 0.0f)
	{
		det = 1.0f / det;
	}
	b2Vec2 x = {
		det * (a22 * b.x - a12 * b.y),
		det * (a11 * b.y - a21 * b.x),
	};
	return x;
}

/// Solve A * x = b, where b is a column vector.
static inline b2Vec3 b2Solve33(b2Mat33 A, b2Vec3 b)
{
	float det = b2Dot3(A.cx, b2Cross3(A.cy, A.cz));
	if (det != 0.0f)
	{
		det = 1.0f / det;
	}
	b2Vec3 x;
	x.x = det * b2Dot3(b, b2Cross3(A.cy, A.cz));
	x.y = det * b2Dot3(A.cx, b2Cross3(b, A.cz));
	x.z = det * b2Dot3(A.cx, b2Cross3(A.cy, b));
	return x;
}

#ifdef __cplusplus
}
#endif
