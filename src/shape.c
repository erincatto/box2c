// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "shape.h"
#include "broad_phase.h"

b2AABB b2Shape_ComputeAABB(const b2Shape* shape, b2Transform xf, int32_t childIndex)
{
	B2_MAYBE_UNUSED(childIndex);

	switch (shape->type)
	{
		case b2_circleShape:
			return b2ComputeCircleAABB(&shape->circle, xf);
		case b2_polygonShape:
			return b2ComputePolygonAABB(&shape->polygon, xf);
		default:
		{
			assert(false);
			b2AABB empty = {xf.p, xf.p};
			return empty;
		}
	}
}

b2MassData b2Shape_ComputeMass(const b2Shape* shape)
{
	switch (shape->type)
	{
		case b2_circleShape:
			return b2ComputeCircleMass(&shape->circle, shape->density);
		case b2_polygonShape:
			return b2ComputePolygonMass(&shape->polygon, shape->density);
		default:
			return (b2MassData){0.0f, {0.0f, 0.0f}, 0.0f};
	}
}

void b2Shape_CreateProxies(b2Shape* shape, b2BroadPhase* bp, b2BodyType type, b2Transform xf)
{
	// Create proxies in the broad-phase.
	int32_t proxyCount = shape->proxyCount;

	for (int32_t i = 0; i < proxyCount; ++i)
	{
		b2ShapeProxy* proxy = shape->proxies + i;
		proxy->aabb = b2Shape_ComputeAABB(shape, xf, i);
		proxy->proxyKey = b2BroadPhase_CreateProxy(bp, type, proxy->aabb, shape->filter.categoryBits, proxy);
		proxy->shapeIndex = shape->object.index;
		proxy->childIndex = i;
	}
}

void b2Shape_DestroyProxies(b2Shape* shape, b2BroadPhase* bp)
{
	int32_t proxyCount = shape->proxyCount;

	for (int32_t i = 0; i < proxyCount; ++i)
	{
		b2ShapeProxy* proxy = shape->proxies + i;
		b2BroadPhase_DestroyProxy(bp, proxy->proxyKey);
		proxy->proxyKey = B2_NULL_INDEX;
	}
}

b2DistanceProxy b2Shape_MakeDistanceProxy(const b2Shape* shape, int32_t child)
{
	B2_MAYBE_UNUSED(child);

	switch (shape->type)
	{
		case b2_circleShape:
			return b2MakeProxy(&shape->circle.point, 1, shape->circle.radius);
		case b2_polygonShape:
			return b2MakeProxy(shape->polygon.vertices, shape->polygon.count, 0.0f);
		default:
		{
			assert(false);
			b2DistanceProxy empty = {0};
			return empty;
		}
	}
}
