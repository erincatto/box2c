// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "shape.h"
#include "body.h"
#include "broad_phase.h"
#include "world.h"

b2AABB b2Shape_ComputeAABB(const b2Shape* shape, b2Transform xf)
{
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

void b2Shape_CreateProxy(b2Shape* shape, b2BroadPhase* bp, b2BodyType type, b2Transform xf)
{
	// Create proxies in the broad-phase.
	shape->aabb = b2Shape_ComputeAABB(shape, xf);
	shape->proxyKey = b2BroadPhase_CreateProxy(bp, type, shape->aabb, shape->filter.categoryBits, shape->object.index, &shape->fatAABB);
	assert(B2_PROXY_TYPE(shape->proxyKey) < b2_bodyTypeCount);
}

void b2Shape_DestroyProxy(b2Shape* shape, b2BroadPhase* bp)
{
	b2BroadPhase_DestroyProxy(bp, shape->proxyKey);
	shape->proxyKey = B2_NULL_INDEX;
}

b2DistanceProxy b2Shape_MakeDistanceProxy(const b2Shape* shape)
{
	switch (shape->type)
	{
		case b2_circleShape:
			return b2MakeProxy(&shape->circle.point, 1, shape->circle.radius);
		case b2_polygonShape:
			return b2MakeProxy(shape->polygon.vertices, shape->polygon.count, shape->polygon.radius);
		default:
		{
			assert(false);
			b2DistanceProxy empty = {0};
			return empty;
		}
	}
}

float b2Shape_GetRadius(const b2Shape* shape)
{
	switch (shape->type)
	{
		case b2_circleShape:
			return shape->circle.radius;
		default:
			return 0.0f;
	}
}

b2BodyId b2Shape_GetBody(b2ShapeId shapeId)
{
	b2World* world = b2GetWorldFromIndex(shapeId.world);
	assert(0 <= shapeId.index && shapeId.index < world->shapePool.capacity);
	b2Shape* shape = world->shapes + shapeId.index;
	assert(b2ObjectValid(&shape->object));

	assert(0 <= shape->bodyIndex && shape->bodyIndex < world->bodyPool.capacity);
	b2Body* body = world->bodies + shape->bodyIndex;
	assert(b2ObjectValid(&body->object));

	b2BodyId bodyId = {body->object.index, shapeId.world, body->object.revision};
	return bodyId;
}

bool b2Shape_TestPoint(b2ShapeId shapeId, b2Vec2 point)
{
	b2World* world = b2GetWorldFromIndex(shapeId.world);
	assert(0 <= shapeId.index && shapeId.index < world->shapePool.capacity);
	b2Shape* shape = world->shapes + shapeId.index;
	assert(b2ObjectValid(&shape->object));

	assert(0 <= shape->bodyIndex && shape->bodyIndex < world->bodyPool.capacity);
	b2Body* body = world->bodies + shape->bodyIndex;
	assert(b2ObjectValid(&body->object));

	b2Vec2 localPoint = b2InvTransformPoint(body->transform, point);

	switch (shape->type)
	{
		case b2_circleShape:
			return b2PointInCircle(localPoint, &shape->circle);

		case b2_polygonShape:
			return b2PointInPolygon(localPoint, &shape->polygon);

		default:
			return false;
	}
}
