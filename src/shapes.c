// SPDX-FileCopyrightText: 2022 Erin Catto
// SPDX-License-Identifier: MIT

#include "box2d/aabb.h"
#include "box2d/hull.h"
#include "box2d/math.h"
#include "box2d/shapes.h"

#include <assert.h>
#include <float.h>

b2PolygonShape b2MakePolygon(const b2Hull* hull)
{
	assert(hull->count >= 3);

	b2PolygonShape shape;
	shape.count = hull->count;

	// Copy vertices
	for (int32_t i = 0; i < shape.count; ++i)
	{
		shape.vertices[i] = hull->points[i];
	}

	// Compute normals. Ensure the edges have non-zero length.
	for (int32_t i = 0; i < shape.count; ++i)
	{
		int32_t i1 = i;
		int32_t i2 = i + 1 < shape.count ? i + 1 : 0;
		b2Vec2 edge = b2Sub(shape.vertices[i2], shape.vertices[i1]);
		assert(b2Dot(edge, edge) > FLT_EPSILON * FLT_EPSILON);
		shape.normals[i] = b2Normalize(b2CrossVS(edge, 1.0f));
	}

	// this is a bit wasteful and the centroid is only needed for smooth collision
	b2MassData massData = b2ComputePolygonMass(&shape, 1.0f);
	shape.centroid = massData.center;

	return shape;
}

b2PolygonShape b2MakeBox(float hx, float hy, b2Vec2 center, float angle)
{
	b2Transform xf;
	xf.p = center;
	xf.q = b2Rot_Set(angle);

	b2PolygonShape shape;
	shape.count = 4;
	shape.vertices[0] = b2TransformPoint(xf, (b2Vec2){-hx, -hy});
	shape.vertices[1] = b2TransformPoint(xf, (b2Vec2){hx, -hy});
	shape.vertices[2] = b2TransformPoint(xf, (b2Vec2){hx, hy});
	shape.vertices[3] = b2TransformPoint(xf, (b2Vec2){-hx, hy});
	shape.normals[0] = b2RotateVector(xf.q, (b2Vec2){0.0f, -1.0f});
	shape.normals[1] = b2RotateVector(xf.q, (b2Vec2){1.0f, 0.0f});
	shape.normals[2] = b2RotateVector(xf.q, (b2Vec2){0.0f, 1.0f});
	shape.normals[3] = b2RotateVector(xf.q, (b2Vec2){-1.0f, 0.0f});
	shape.centroid = center;

	return shape;
}

b2MassData b2ComputeCircleMass(const b2CircleShape* shape, float density)
{
	float rr = shape->radius * shape->radius;

	b2MassData massData;
	massData.mass = density * b2_pi * rr;
	massData.center = shape->point;

	// inertia about the local origin
	massData.I = massData.mass * (0.5f * rr + b2Dot(shape->point, shape->point));

	return massData;
}

//b2MassData b2ComputeCapsuleMass(const b2CapsuleShape* shape);

b2MassData b2ComputePolygonMass(const b2PolygonShape* shape, float density)
{
	// Polygon mass, centroid, and inertia.
	// Let rho be the polygon density in mass per unit area.
	// Then:
	// mass = rho * int(dA)
	// centroid.x = (1/mass) * rho * int(x * dA)
	// centroid.y = (1/mass) * rho * int(y * dA)
	// I = rho * int((x*x + y*y) * dA)
	//
	// We can compute these integrals by summing all the integrals
	// for each triangle of the polygon. To evaluate the integral
	// for a single triangle, we make a change of variables to
	// the (u,v) coordinates of the triangle:
	// x = x0 + e1x * u + e2x * v
	// y = y0 + e1y * u + e2y * v
	// where 0 <= u && 0 <= v && u + v <= 1.
	//
	// We integrate u from [0,1-v] and then v from [0,1].
	// We also need to use the Jacobian of the transformation:
	// D = cross(e1, e2)
	//
	// Simplification: triangle centroid = (1/3) * (p1 + p2 + p3)
	//
	// The rest of the derivation is handled by computer algebra.

	assert(shape->count >= 3);

	b2Vec2 center = {0.0f, 0.0f};
	float area = 0.0f;
	float I = 0.0f;

	// Get a reference point for forming triangles.
	// Use the first vertex to reduce round-off errors.
	b2Vec2 r = shape->vertices[0];

	const float inv3 = 1.0f / 3.0f;

	for (int32_t i = 1; i < shape->count - 1; ++i)
	{
		// Triangle edges
		b2Vec2 e1 = b2Sub(shape->vertices[i], r);
		b2Vec2 e2 = b2Sub(shape->vertices[i + 1], r);

		float D = b2Cross(e1, e2);

		float triangleArea = 0.5f * D;
		area += triangleArea;

		// Area weighted centroid, r at origin
		center = b2MulAdd(center, triangleArea * inv3, b2Add(e1, e2));

		float ex1 = e1.x, ey1 = e1.y;
		float ex2 = e2.x, ey2 = e2.y;

		float intx2 = ex1 * ex1 + ex2 * ex1 + ex2 * ex2;
		float inty2 = ey1 * ey1 + ey2 * ey1 + ey2 * ey2;

		I += (0.25f * inv3 * D) * (intx2 + inty2);
	}

	b2MassData massData;

	// Total mass
	massData.mass = density * area;

	// Center of mass, shift back from origin at r
	assert(area > FLT_EPSILON);
	massData.center = b2MulAdd(r, 1.0f / area, center);

	// Inertia tensor relative to the local origin (point s).
	massData.I = density * I;

	// Shift to center of mass then to original body origin.
	massData.I += massData.mass * (b2Dot(massData.center, massData.center) - b2Dot(center, center));

	return massData;
}

b2AABB b2ComputeCircleAABB(const b2CircleShape* shape, b2Transform xf)
{
	b2Vec2 p = b2TransformPoint(xf, shape->point);
	float r = shape->radius;

	b2AABB aabb = {{p.x - r, p.y - r}, {p.x + r, p.y + r}};
	return aabb;
}

//b2AABB b2ComputeCapsuleAABB(const b2CapsuleShape* shape, b2Transform xf);

b2AABB b2ComputePolygonAABB(const b2PolygonShape* shape, b2Transform xf)
{
	assert(shape->count > 0);
	b2Vec2 lower = b2TransformPoint(xf, shape->vertices[0]);
	b2Vec2 upper = lower;

	for (int32_t i = 1; i < shape->count; ++i)
	{
		b2Vec2 v = b2TransformPoint(xf, shape->vertices[i]);
		lower = b2Min(lower, v);
		upper = b2Max(upper, v);
	}

	b2AABB aabb = {lower, upper};
	return aabb;
}

b2AABB b2ComputeSegmentAABB(const b2SegmentShape* shape, b2Transform xf)
{
	b2Vec2 v1 = b2TransformPoint(xf, shape->point1);
	b2Vec2 v2 = b2TransformPoint(xf, shape->point2);

	b2Vec2 lower = b2Min(v1, v2);
	b2Vec2 upper = b2Max(v1, v2);

	b2AABB aabb = {lower, upper};
	return aabb;
}

bool b2PointInCircle(b2Vec2 point, const b2CircleShape* shape, b2Transform xf)
{
	b2Vec2 center = b2TransformPoint(xf, shape->point);
	b2Vec2 d = b2Sub(point, center);
	return b2Dot(d, d) <= shape->radius * shape->radius;
}

//bool b2PointInCapsule(b2Vec2 point, const b2CapsuleShape* shape, b2Transform xf);

bool b2PointInPolygon(b2Vec2 point, const b2PolygonShape* shape, b2Transform xf)
{
	b2Vec2 localPoint = b2InvRotateVector(xf.q, b2Sub(point, xf.p));

	for (int32_t i = 0; i < shape->count; ++i)
	{
		float dot = b2Dot(shape->normals[i], b2Sub(localPoint, shape->vertices[i]));
		if (dot > 0.0f)
		{
			return false;
		}
	}

	return true;
}

// Precision Improvements for Ray / Sphere Intersection - Ray Tracing Gems 2019
// http://www.codercorner.com/blog/?p=321
b2RayCastOutput b2RayCastCircle(const b2RayCastInput* input, const b2CircleShape* shape, b2Transform xf)
{
	b2Vec2 p = b2TransformPoint(xf, shape->point);

	b2RayCastOutput output = {{0.0f, 0.0f}, 0.0f, false};
	
	// Shift ray so circle center is the origin
	b2Vec2 s = b2Sub(input->p1, p);
	float length;
	b2Vec2 d = b2GetLengthAndNormalize(&length, b2Sub(input->p2, input->p1));
	if (length == 0.0f)
	{
		// zero length ray
		return output;
	}

	// Find closest point on ray to origin

	// solve: dot(s + t * d, d) = 0
	float t = -b2Dot(s, d);

	// c is the closest point on the line to the origin
	b2Vec2 c = b2MulAdd(s, t, d);

	float cc = b2Dot(c, c);
	float rr = shape->radius * shape->radius;

	if (cc > rr)
	{
		// closest point is outside the circle
		return output;
	}

	// Pythagorus
	float h = sqrtf(rr - cc);

	float fraction = t - h;

	if (fraction < 0.0f || input->maxFraction * length < fraction)
	{
		// outside the range of the ray segment
		return output;
	}

	b2Vec2 hitPoint = b2MulAdd(s, fraction, d);

	output.fraction = fraction / length;
	output.normal = b2Normalize(hitPoint);
	output.hit = true;

	return output;
}

//b2RayCastOutput b2RayCastCapsule(const b2RayCastInput* input, const b2CapsuleShape* shape, b2Transform xf);

// Ray vs line segment, ignores back-side collision (from the left).
//  p = p1 + t * d
//  v = v1 + s * e
//  p1 + t * d = v1 + s * e
//  s * e - t * d = p1 - v1
b2RayCastOutput b2RayCastSegment(const b2RayCastInput* input, const b2SegmentShape* shape, b2Transform xf)
{
	// Put the ray into the edge's frame of reference.
	b2Vec2 p1 = b2InvTransformPoint(xf, input->p1);
	b2Vec2 p2 = b2InvTransformPoint(xf, input->p2);
	b2Vec2 d = b2Sub(p2, p1);

	b2Vec2 v1 = shape->point1;
	b2Vec2 v2 = shape->point2;
	b2Vec2 e = b2Sub(v2, v1);

	// Normal points to the right, looking from v1 at v2
	b2Vec2 normal = b2Normalize((b2Vec2){e.y, -e.x});

	b2RayCastOutput output = {{0.0f, 0.0f}, 0.0f, false};

	// q = p1 + t * d
	// dot(normal, q - v1) = 0
	// dot(normal, p1 - v1) + t * dot(normal, d) = 0
	float numerator = b2Dot(normal, b2Sub(v1, p1));
	if (numerator > 0.0f)
	{
		// back-side
		return output;
	}

	float denominator = b2Dot(normal, d);

	if (denominator == 0.0f)
	{
		// parallel
		return output;
	}

	float t = numerator / denominator;
	if (t < 0.0f || input->maxFraction < t)
	{
		// out of ray range
		return output;
	}

	b2Vec2 q = b2MulAdd(p1, t, d);

	// q = v1 + s * r
	// s = dot(q - v1, r) / dot(r, r)
	b2Vec2 r = b2Sub(v2, v1);
	float rr = b2Dot(r, r);
	if (rr == 0.0f)
	{
		// zero length segment
		return output;
	}

	float s = b2Dot(b2Sub(q, v1), r) / rr;
	if (s < 0.0f || 1.0f < s)
	{
		// out of segment range
		return output;
	}

	output.fraction = t;
	output.normal = b2RotateVector(xf.q, normal);
	output.hit = true;

	return output;
}

b2RayCastOutput b2RayCastPolygon(const b2RayCastInput* input, const b2PolygonShape* shape, b2Transform xf)
{
	// Put the ray into the polygon's frame of reference.
	b2Vec2 p1 = b2InvRotateVector(xf.q, b2Sub(input->p1, xf.p));
	b2Vec2 p2 = b2InvRotateVector(xf.q, b2Sub(input->p2, xf.p));
	b2Vec2 d = b2Sub(p2, p1);

	float lower = 0.0f, upper = input->maxFraction;

	int32_t index = -1;

	b2RayCastOutput output = {{0.0f, 0.0f}, 0.0f, false};

	for (int32_t i = 0; i < shape->count; ++i)
	{
		// p = p1 + a * d
		// dot(normal, p - v) = 0
		// dot(normal, p1 - v) + a * dot(normal, d) = 0
		float numerator = b2Dot(shape->normals[i], b2Sub(shape->vertices[i], p1));
		float denominator = b2Dot(shape->normals[i], d);

		if (denominator == 0.0f)
		{
			if (numerator < 0.0f)
			{
				return output;
			}
		}
		else
		{
			// Note: we want this predicate without division:
			// lower < numerator / denominator, where denominator < 0
			// Since denominator < 0, we have to flip the inequality:
			// lower < numerator / denominator <==> denominator * lower > numerator.
			if (denominator < 0.0f && numerator < lower * denominator)
			{
				// Increase lower.
				// The segment enters this half-space.
				lower = numerator / denominator;
				index = i;
			}
			else if (denominator > 0.0f && numerator < upper * denominator)
			{
				// Decrease upper.
				// The segment exits this half-space.
				upper = numerator / denominator;
			}
		}

		// The use of epsilon here causes the assert on lower to trip
		// in some cases. Apparently the use of epsilon was to make edge
		// shapes work, but now those are handled separately.
		// if (upper < lower - b2_epsilon)
		if (upper < lower)
		{
			return output;
		}
	}

	assert(0.0f <= lower && lower <= input->maxFraction);

	if (index >= 0)
	{
		output.fraction = lower;
		output.normal = b2RotateVector(xf.q, shape->normals[index]);
		output.hit = true;
	}

	return output;
}
