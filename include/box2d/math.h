// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "api.h"
#include "types.h"

#include <float.h>
#include <math.h>

/// Macro to get the minimum of two values
#define B2_MIN(A, B) ((A) < (B) ? (A) : (B))

/// Macro to get the maximum of two values
#define B2_MAX(A, B) ((A) > (B) ? (A) : (B))

/// Macro to get the absolute value
#define B2_ABS(A) ((A) > 0.0f ? (A) : -(A))

/// Macro to clamp A to be between B and C, inclusive
#define B2_CLAMP(A, B, C) B2_MIN(B2_MAX(A, B), C)

static const b2Vec2 b2Vec2_zero = {0.0f, 0.0f};
static const b2Rot b2Rot_identity = {0.0f, 1.0f};
static const b2Transform b2Transform_identity = {{0.0f, 0.0f}, {0.0f, 1.0f}};
static const b2Mat22 b2Mat22_zero = {{0.0f, 0.0f}, {0.0f, 0.0f}};

/// Vector dot product
B2_INLINE float b2Dot(b2Vec2 a, b2Vec2 b)
{
	return a.x * b.x + a.y * b.y;
}

/// Vector cross product. In 2D this yields a scalar.
B2_INLINE float b2Cross(b2Vec2 a, b2Vec2 b)
{
	return a.x * b.y - a.y * b.x;
}

/// Perform the cross product on a vector and a scalar. In 2D this produces
/// a vector.
B2_INLINE b2Vec2 b2CrossVS(b2Vec2 v, float s)
{
	return B2_LITERAL(b2Vec2){s * v.y, -s * v.x};
}

/// Perform the cross product on a scalar and a vector. In 2D this produces
/// a vector.
B2_INLINE b2Vec2 b2CrossSV(float s, b2Vec2 v)
{
	return B2_LITERAL(b2Vec2){-s * v.y, s * v.x};
}

/// Get a left pointing perpendicular vector. Equivalent to b2CrossSV(1.0f, v)
B2_INLINE b2Vec2 b2LeftPerp(b2Vec2 v)
{
	return B2_LITERAL(b2Vec2){-v.y, v.x};
}

/// Get a right pointing perpendicular vector. Equivalent to b2CrossVS(v, 1.0f)
B2_INLINE b2Vec2 b2RightPerp(b2Vec2 v)
{
	return B2_LITERAL(b2Vec2){v.y, -v.x};
}

/// Vector addition
B2_INLINE b2Vec2 b2Add(b2Vec2 a, b2Vec2 b)
{
	return B2_LITERAL(b2Vec2){a.x + b.x, a.y + b.y};
}

/// Vector subtraction
B2_INLINE b2Vec2 b2Sub(b2Vec2 a, b2Vec2 b)
{
	return B2_LITERAL(b2Vec2){a.x - b.x, a.y - b.y};
}

/// Vector subtraction
B2_INLINE b2Vec2 b2Neg(b2Vec2 a)
{
	return B2_LITERAL(b2Vec2){-a.x, -a.y};
}

/// Vector linear interpolation
/// https://fgiesen.wordpress.com/2012/08/15/linear-interpolation-past-present-and-future/
B2_INLINE b2Vec2 b2Lerp(b2Vec2 a, b2Vec2 b, float t)
{
	return B2_LITERAL(b2Vec2){(1.0f - t) * a.x + t * b.x, (1.0f - t) * a.y + t * b.y};
}

/// Component-wise multiplication
B2_INLINE b2Vec2 b2Mul(b2Vec2 a, b2Vec2 b)
{
	return B2_LITERAL(b2Vec2){a.x * b.x, a.y * b.y};
}

/// Multiply a scalar and vector
B2_INLINE b2Vec2 b2MulSV(float s, b2Vec2 v)
{
	return B2_LITERAL(b2Vec2){s * v.x, s * v.y};
}

/// a + s * b
B2_INLINE b2Vec2 b2MulAdd(b2Vec2 a, float s, b2Vec2 b)
{
	return B2_LITERAL(b2Vec2){a.x + s * b.x, a.y + s * b.y};
}

/// a - s * b
B2_INLINE b2Vec2 b2MulSub(b2Vec2 a, float s, b2Vec2 b)
{
	return B2_LITERAL(b2Vec2){a.x - s * b.x, a.y - s * b.y};
}

/// Component-wise absolute vector
B2_INLINE b2Vec2 b2Abs(b2Vec2 a)
{
	b2Vec2 b;
	b.x = B2_ABS(a.x);
	b.y = B2_ABS(a.y);
	return b;
}

/// Component-wise absolute vector
B2_INLINE b2Vec2 b2Min(b2Vec2 a, b2Vec2 b)
{
	b2Vec2 c;
	c.x = B2_MIN(a.x, b.x);
	c.y = B2_MIN(a.y, b.y);
	return c;
}

/// Component-wise absolute vector
B2_INLINE b2Vec2 b2Max(b2Vec2 a, b2Vec2 b)
{
	b2Vec2 c;
	c.x = B2_MAX(a.x, b.x);
	c.y = B2_MAX(a.y, b.y);
	return c;
}

/// Component-wise clamp vector so v into the range [a, b]
B2_INLINE b2Vec2 b2Clamp(b2Vec2 v, b2Vec2 a, b2Vec2 b)
{
	b2Vec2 c;
	c.x = B2_CLAMP(v.x, a.x, b.x);
	c.y = B2_CLAMP(v.y, a.y, b.y);
	return c;
}

/// Get the length of this vector (the norm).
B2_INLINE float b2Length(b2Vec2 v)
{
	return sqrtf(v.x * v.x + v.y * v.y);
}

/// Get the length of this vector (the norm).
B2_INLINE float b2LengthSquared(b2Vec2 v)
{
	return v.x * v.x + v.y * v.y;
}

B2_INLINE float b2Distance(b2Vec2 a, b2Vec2 b)
{
	float dx = b.x - a.x;
	float dy = b.y - a.y;
	return sqrtf(dx * dx + dy * dy);
}

/// Get the length of this vector (the norm).
B2_INLINE float b2DistanceSquared(b2Vec2 a, b2Vec2 b)
{
	b2Vec2 c = {b.x - a.x, b.y - a.y};
	return c.x * c.x + c.y * c.y;
}

/// Set using an angle in radians.
B2_INLINE b2Rot b2MakeRot(float angle)
{
	// todo determinism
	b2Rot q = {sinf(angle), cosf(angle)};
	return q;
}

/// Normalize rotation
B2_INLINE b2Rot b2NormalizeRot(b2Rot q)
{
	float mag = sqrtf(q.s * q.s + q.c * q.c);
	float invMag = mag > 0.0 ? 1.0f / mag : 0.0f;
	b2Rot qn = {q.s * invMag, q.c * invMag};
	return qn;
}

/// Is this rotation normalized?
B2_INLINE bool b2IsNormalized(b2Rot q)
{
	float qq = q.s * q.s + q.c * q.c;
	return 1.0f - 0.0004f < qq && qq < 1.0f + 0.0004f;
}

/// Normalized linear interpolation
/// https://fgiesen.wordpress.com/2012/08/15/linear-interpolation-past-present-and-future/
B2_INLINE b2Rot b2NLerp(b2Rot q1, b2Rot q2, float t)
{
	float omt = 1.0f - t;
	b2Rot q = {
		omt * q1.s + t * q2.s,
		omt * q1.c + t * q2.c,
	};

	return b2NormalizeRot(q);
}

/// Integration rotation from angular velocity
///	@param q1 initial rotation
///	@param deltaAngle the angular displacement in radians
B2_INLINE b2Rot b2IntegrateRotation(b2Rot q1, float deltaAngle)
{
	// ds/dt = omega * cos(t)
	// dc/dt = -omega * sin(t)
	// s2 = s1 + omega * h * c1
	// c2 = c1 - omega * h * s1
	b2Rot q2 = {q1.s + deltaAngle * q1.c, q1.c - deltaAngle * q1.s};
	float mag = sqrtf(q2.s * q2.s + q2.c * q2.c);
	float invMag = mag > 0.0 ? 1.0f / mag : 0.0f;
	b2Rot qn = {q2.s * invMag, q2.c * invMag};
	return qn;
}

/// Compute the angular velocity necessary to rotate between two
///	rotations over a give time
///	@param q1 initial rotation
///	@param q2 final rotation
///	@param inv_h inverse time step
B2_INLINE float b2ComputeAngularVelocity(b2Rot q1, b2Rot q2, float inv_h)
{
	// ds/dt = omega * cos(t)
	// dc/dt = -omega * sin(t)
	// s2 = s1 + omega * h * c1
	// c2 = c1 - omega * h * s1

	// omega * h * s1 = c1 - c2
	// omega * h * c1 = s2 - s1
	// omega * h = (c1 - c2) * s1 + (s2 - s1) * c1;
	// omega * h = s1 * c1 - c2 * s1 + s2 * c1 - s1 * c1
	// omega * h = s2 * c1 - c2 * s1 = sin(a2 - a1) ~= a2 - a1 for small delta
	float omega = inv_h * (q2.s * q1.c - q2.c * q1.s);
	return omega;
}

/// Get the angle in radians
B2_INLINE float b2Rot_GetAngle(b2Rot q)
{
	// todo determinism
	return atan2f(q.s, q.c);
}

/// Get the x-axis
B2_INLINE b2Vec2 b2Rot_GetXAxis(b2Rot q)
{
	b2Vec2 v = {q.c, q.s};
	return v;
}

/// Get the y-axis
B2_INLINE b2Vec2 b2Rot_GetYAxis(b2Rot q)
{
	b2Vec2 v = {-q.s, q.c};
	return v;
}

/// Multiply two rotations: q * r
B2_INLINE b2Rot b2MulRot(b2Rot q, b2Rot r)
{
	// [qc -qs] * [rc -rs] = [qc*rc-qs*rs -qc*rs-qs*rc]
	// [qs  qc]   [rs  rc]   [qs*rc+qc*rs -qs*rs+qc*rc]
	// s(q + r) = qs * rc + qc * rs
	// c(q + r) = qc * rc - qs * rs
	b2Rot qr;
	qr.s = q.s * r.c + q.c * r.s;
	qr.c = q.c * r.c - q.s * r.s;
	return qr;
}

/// Transpose multiply two rotations: qT * r
B2_INLINE b2Rot b2InvMulRot(b2Rot q, b2Rot r)
{
	// [ qc qs] * [rc -rs] = [qc*rc+qs*rs -qc*rs+qs*rc]
	// [-qs qc]   [rs  rc]   [-qs*rc+qc*rs qs*rs+qc*rc]
	// s(q - r) = qc * rs - qs * rc
	// c(q - r) = qc * rc + qs * rs
	b2Rot qr;
	qr.s = q.c * r.s - q.s * r.c;
	qr.c = q.c * r.c + q.s * r.s;
	return qr;
}

// relative angle between b and a (rot_b * inv(rot_a))
B2_INLINE float b2RelativeAngle(b2Rot b, b2Rot a)
{
	// sin(b - a) = bs * ac - bc * as
	// cos(b - a) = bc * ac + bs * as
	float s = b.s * a.c - b.c * a.s;
	float c = b.c * a.c + b.s * a.s;
	return atan2f(s, c);
}

/// Rotate a vector
B2_INLINE b2Vec2 b2RotateVector(b2Rot q, b2Vec2 v)
{
	return B2_LITERAL(b2Vec2){q.c * v.x - q.s * v.y, q.s * v.x + q.c * v.y};
}

/// Inverse rotate a vector
B2_INLINE b2Vec2 b2InvRotateVector(b2Rot q, b2Vec2 v)
{
	return B2_LITERAL(b2Vec2){q.c * v.x + q.s * v.y, -q.s * v.x + q.c * v.y};
}

/// Transform a point (e.g. local space to world space)
B2_INLINE b2Vec2 b2TransformPoint(b2Transform xf, const b2Vec2 p)
{
	float x = (xf.q.c * p.x - xf.q.s * p.y) + xf.p.x;
	float y = (xf.q.s * p.x + xf.q.c * p.y) + xf.p.y;

	return B2_LITERAL(b2Vec2){x, y};
}

/// Inverse transform a point (e.g. world space to local space)
B2_INLINE b2Vec2 b2InvTransformPoint(b2Transform xf, const b2Vec2 p)
{
	float vx = p.x - xf.p.x;
	float vy = p.y - xf.p.y;
	return B2_LITERAL(b2Vec2){xf.q.c * vx + xf.q.s * vy, -xf.q.s * vx + xf.q.c * vy};
}

/// v2 = A.q.Rot(B.q.Rot(v1) + B.p) + A.p
///    = (A.q * B.q).Rot(v1) + A.q.Rot(B.p) + A.p
B2_INLINE b2Transform b2MulTransforms(b2Transform A, b2Transform B)
{
	b2Transform C;
	C.q = b2MulRot(A.q, B.q);
	C.p = b2Add(b2RotateVector(A.q, B.p), A.p);
	return C;
}

/// v2 = A.q' * (B.q * v1 + B.p - A.p)
///    = A.q' * B.q * v1 + A.q' * (B.p - A.p)
B2_INLINE b2Transform b2InvMulTransforms(b2Transform A, b2Transform B)
{
	b2Transform C;
	C.q = b2InvMulRot(A.q, B.q);
	C.p = b2InvRotateVector(A.q, b2Sub(B.p, A.p));
	return C;
}

/// Multiply a 2-by-2 matrix times a 2D vector
B2_INLINE b2Vec2 b2MulMV(b2Mat22 A, b2Vec2 v)
{
	b2Vec2 u = {
		A.cx.x * v.x + A.cy.x * v.y,
		A.cx.y * v.x + A.cy.y * v.y,
	};
	return u;
}

/// Get the inverse of a 2-by-2 matrix
B2_INLINE b2Mat22 b2GetInverse22(b2Mat22 A)
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

/// Solve A * x = b, where b is a column vector. This is more efficient
/// than computing the inverse in one-shot cases.
B2_INLINE b2Vec2 b2Solve22(b2Mat22 A, b2Vec2 b)
{
	float a11 = A.cx.x, a12 = A.cy.x, a21 = A.cx.y, a22 = A.cy.y;
	float det = a11 * a22 - a12 * a21;
	if (det != 0.0f)
	{
		det = 1.0f / det;
	}
	b2Vec2 x = {det * (a22 * b.x - a12 * b.y), det * (a11 * b.y - a21 * b.x)};
	return x;
}

/// Does a fully contain b
B2_INLINE bool b2AABB_Contains(b2AABB a, b2AABB b)
{
	bool s = true;
	s = s && a.lowerBound.x <= b.lowerBound.x;
	s = s && a.lowerBound.y <= b.lowerBound.y;
	s = s && b.upperBound.x <= a.upperBound.x;
	s = s && b.upperBound.y <= a.upperBound.y;
	return s;
}

/// Get the center of the AABB.
B2_INLINE b2Vec2 b2AABB_Center(b2AABB a)
{
	b2Vec2 b = {0.5f * (a.lowerBound.x + a.upperBound.x), 0.5f * (a.lowerBound.y + a.upperBound.y)};
	return b;
}

/// Get the extents of the AABB (half-widths).
B2_INLINE b2Vec2 b2AABB_Extents(b2AABB a)
{
	b2Vec2 b = {0.5f * (a.upperBound.x - a.lowerBound.x), 0.5f * (a.upperBound.y - a.lowerBound.y)};
	return b;
}

/// Union of two AABBs
B2_INLINE b2AABB b2AABB_Union(b2AABB a, b2AABB b)
{
	b2AABB c;
	c.lowerBound.x = B2_MIN(a.lowerBound.x, b.lowerBound.x);
	c.lowerBound.y = B2_MIN(a.lowerBound.y, b.lowerBound.y);
	c.upperBound.x = B2_MAX(a.upperBound.x, b.upperBound.x);
	c.upperBound.y = B2_MAX(a.upperBound.y, b.upperBound.y);
	return c;
}

B2_API bool b2IsValid(float a);
B2_API bool b2Vec2_IsValid(b2Vec2 v);
B2_API bool b2Rot_IsValid(b2Rot q);
B2_API bool b2AABB_IsValid(b2AABB aabb);

/// Convert this vector into a unit vector
B2_API b2Vec2 b2Normalize(b2Vec2 v);

/// This asserts of the vector is too short
B2_API b2Vec2 b2NormalizeChecked(b2Vec2 v);

B2_API b2Vec2 b2GetLengthAndNormalize(float* length, b2Vec2 v);
