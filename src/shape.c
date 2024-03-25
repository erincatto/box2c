// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "shape.h"

#include "allocate.h"
#include "body.h"
#include "broad_phase.h"
#include "contact.h"
#include "static_body.h"
#include "world.h"

// needed for dll export
#include "box2d/box2d.h"
#include "box2d/event_types.h"

b2Shape* b2GetShape(b2World* world, b2ShapeId shapeId)
{
	int id = shapeId.index1 - 1;
	B2_ASSERT(0 <= id && id < world->shapePool.capacity);
	b2Shape* shape = world->shapes + id;
	B2_ASSERT(b2IsValidObject(&shape->object));
	B2_ASSERT(shape->object.revision == shapeId.revision);
	return shape;
}

b2Transform b2GetOwnerTransform(b2World* world, b2Shape* shape)
{
	int bodyId = shape->bodyKey >> 1;
	if (shape->bodyKey & 1)
	{
		return b2GetBodyTransform(world, bodyId);
	}

	return b2GetStaticBodyTransform(world, bodyId);
}

static b2ChainShape* b2GetChainShape(b2World* world, b2ChainId chainId)
{
	B2_ASSERT(1 <= chainId.index1 && chainId.index1 <= world->chainPool.capacity);
	b2ChainShape* chain = world->chains + (chainId.index1 - 1);
	B2_ASSERT(b2IsValidObject(&chain->object));
	B2_ASSERT(chain->object.revision == chainId.revision);
	return chain;
}

static b2Shape* b2CreateShape(b2World* world, int bodyKey, b2Transform transform, b2ShapeList* shapeList, bool isEnabled,
							  const b2ShapeDef* def, const void* geometry, b2ShapeType shapeType)
{
	b2Shape* shape = (b2Shape*)b2AllocObject(&world->shapePool);
	world->shapes = (b2Shape*)world->shapePool.memory;

	B2_ASSERT(b2IsValid(def->density) && def->density >= 0.0f);
	B2_ASSERT(b2IsValid(def->friction) && def->friction >= 0.0f);
	B2_ASSERT(b2IsValid(def->restitution) && def->restitution >= 0.0f);

	switch (shapeType)
	{
		case b2_capsuleShape:
			shape->capsule = *(const b2Capsule*)geometry;
			break;

		case b2_circleShape:
			shape->circle = *(const b2Circle*)geometry;
			break;

		case b2_polygonShape:
			shape->polygon = *(const b2Polygon*)geometry;
			break;

		case b2_segmentShape:
			shape->segment = *(const b2Segment*)geometry;
			break;

		case b2_smoothSegmentShape:
			shape->smoothSegment = *(const b2SmoothSegment*)geometry;
			break;

		default:
			B2_ASSERT(false);
			break;
	}

	shape->bodyKey = bodyKey;
	shape->type = shapeType;
	shape->density = def->density;
	shape->friction = def->friction;
	shape->restitution = def->restitution;
	shape->filter = def->filter;
	shape->userData = def->userData;
	shape->isSensor = def->isSensor;
	shape->enlargedAABB = false;
	shape->enableSensorEvents = def->enableSensorEvents;
	shape->enableContactEvents = def->enableContactEvents;
	shape->enablePreSolveEvents = def->enablePreSolveEvents;
	shape->isFast = false;
	shape->proxyKey = B2_NULL_INDEX;
	shape->localCentroid = b2GetShapeCentroid(shape);
	shape->aabb = (b2AABB){b2Vec2_zero, b2Vec2_zero};
	shape->fatAABB = (b2AABB){b2Vec2_zero, b2Vec2_zero};

	if (isEnabled)
	{
		b2ProxyType proxyType = (bodyKey & 1) == 0 ? b2_staticProxy : b2_movableProxy;
		b2CreateShapeProxy(shape, &world->broadPhase, proxyType, transform);
	}

	// Add to shape linked list
	shape->nextShapeId = shapeList->headShapeId;
	shapeList->headShapeId = shape->object.index;
	shapeList->shapeCount += 1;

	b2ValidateWorld(world);

	return shape;
}

b2ShapeId b2CreateDynamicShape(b2BodyId bodyId, const b2ShapeDef* def, const void* geometry, b2ShapeType shapeType)
{
	b2World* world = b2GetWorldLocked(bodyId.world0);
	if (world == NULL)
	{
		return (b2ShapeId){0};
	}

	b2Body* body = b2GetBodyFullId(world, bodyId);
	int bodyKey = (body->bodyId << 1) | 1;
	b2Transform transform = b2MakeTransform(body);

	b2Shape* shape = b2CreateShape(world, bodyKey, transform, &body->shapeList, body->isEnabled, def, geometry, shapeType);
	if (shape->density > 0.0f)
	{
		b2UpdateBodyMassData(world, body);
	}

	b2ShapeId id = {shape->object.index + 1, bodyId.world0, shape->object.revision};
	return id;
}

b2ShapeId b2CreateCircleShape(b2BodyId bodyId, const b2ShapeDef* def, const b2Circle* circle)
{
	return b2CreateDynamicShape(bodyId, def, circle, b2_circleShape);
}

b2ShapeId b2CreateCapsuleShape(b2BodyId bodyId, const b2ShapeDef* def, const b2Capsule* capsule)
{
	float lengthSqr = b2DistanceSquared(capsule->point1, capsule->point2);
	if (lengthSqr <= b2_linearSlop * b2_linearSlop)
	{
		B2_ASSERT(false);
		return (b2ShapeId){0};
	}

	return b2CreateDynamicShape(bodyId, def, capsule, b2_capsuleShape);
}

b2ShapeId b2CreatePolygonShape(b2BodyId bodyId, const b2ShapeDef* def, const b2Polygon* polygon)
{
	return b2CreateDynamicShape(bodyId, def, polygon, b2_polygonShape);
}

b2ShapeId b2CreateSegmentShape(b2BodyId bodyId, const b2ShapeDef* def, const b2Segment* segment)
{
	float lengthSqr = b2DistanceSquared(segment->point1, segment->point2);
	if (lengthSqr <= b2_linearSlop * b2_linearSlop)
	{
		B2_ASSERT(false);
		return b2_nullShapeId;
	}

	return b2CreateDynamicShape(bodyId, def, segment, b2_segmentShape);
}

b2ShapeId b2CreateStaticShape(b2StaticBodyId staticBodyId, const b2ShapeDef* def, const void* geometry, b2ShapeType shapeType)
{
	b2World* world = b2GetWorldLocked(staticBodyId.world0);
	if (world == NULL)
	{
		return (b2ShapeId){0};
	}

	b2StaticBody* staticBody = b2GetStaticBodyFullId(world, staticBodyId);
	int bodyKey = (staticBody->bodyId << 1) | 0;
	b2Transform transform = staticBody->transform;
	bool isEnabled = true;
	b2Shape* shape = b2CreateShape(world, bodyKey, transform, &staticBody->shapeList, isEnabled, def, geometry, shapeType);

	b2ShapeId id = {shape->object.index + 1, staticBodyId.world0, shape->object.revision};
	return id;
}

b2ShapeId b2CreateStaticCircleShape(b2StaticBodyId staticBodyId, const b2ShapeDef* def, const b2Circle* circle)
{
	return b2CreateStaticShape(staticBodyId, def, circle, b2_circleShape);
}

b2ShapeId b2CreateStaticCapsuleShape(b2StaticBodyId staticBodyId, const b2ShapeDef* def, const b2Capsule* capsule)
{
	float lengthSqr = b2DistanceSquared(capsule->point1, capsule->point2);
	if (lengthSqr <= b2_linearSlop * b2_linearSlop)
	{
		B2_ASSERT(false);
		return (b2ShapeId){0};
	}

	return b2CreateStaticShape(staticBodyId, def, capsule, b2_capsuleShape);
}

b2ShapeId b2CreateStaticPolygonShape(b2StaticBodyId staticBodyId, const b2ShapeDef* def, const b2Polygon* polygon)
{
	return b2CreateStaticShape(staticBodyId, def, polygon, b2_polygonShape);
}

b2ShapeId b2CreateStaticSegmentShape(b2StaticBodyId staticBodyId, const b2ShapeDef* def, const b2Segment* segment)
{
	float lengthSqr = b2DistanceSquared(segment->point1, segment->point2);
	if (lengthSqr <= b2_linearSlop * b2_linearSlop)
	{
		B2_ASSERT(false);
		return b2_nullShapeId;
	}

	return b2CreateStaticShape(staticBodyId, def, segment, b2_segmentShape);
}

// Destroy a shape on a body. This doesn't need to be called when destroying a body.
void b2DestroyShapeInternal(b2World* world, b2Shape* shape, b2ShapeList* shapeList, b2ContactList* contactList, bool wakeBodies)
{
	int shapeId = shape->object.index;

	// Remove the shape from the body's singly linked list.
	int* shapeIdPtr = &shapeList->headShapeId;
	bool found = false;
	while (*shapeIdPtr != B2_NULL_INDEX)
	{
		if (*shapeIdPtr == shape->object.index)
		{
			*shapeIdPtr = shape->nextShapeId;
			found = true;
			break;
		}

		shapeIdPtr = &(world->shapes[*shapeIdPtr].nextShapeId);
	}

	B2_ASSERT(found);
	if (found == false)
	{
		return;
	}

	shapeList->shapeCount -= 1;

	// Destroy any contacts associated with the shape
	int contactKey = contactList->headContactKey;
	while (contactKey != B2_NULL_INDEX)
	{
		int contactId = contactKey >> 1;
		int edgeIndex = contactKey & 1;

		b2Contact* contact = b2GetContactFromRawId(world, contactId);
		contactKey = contact->edges[edgeIndex].nextKey;

		if (contact->shapeIdA == shapeId || contact->shapeIdB == shapeId)
		{
			b2DestroyContact(world, contact, wakeBodies);
		}
	}

	b2DestroyShapeProxy(shape, &world->broadPhase);
	b2FreeObject(&world->shapePool, &shape->object);
}

void b2DestroyShape(b2ShapeId shapeId)
{
	b2CheckShapeId(shapeId);

	b2World* world = b2GetWorldLocked(shapeId.world0);
	b2Shape* shape = world->shapes + (shapeId.index1 - 1);
	bool wakeBodies = true;

	int bodyKey = shape->bodyKey;
	if (bodyKey & 1)
	{
		b2Body* body = b2GetBody(world, bodyKey >> 1);
		b2DestroyShapeInternal(world, shape, &body->shapeList, &body->contactList, wakeBodies);
		b2UpdateBodyMassData(world, body);
	}
	else
	{
		b2StaticBody* staticBody = b2GetStaticBody(world, bodyKey >> 1);
		b2DestroyShapeInternal(world, shape, &staticBody->shapeList, &staticBody->contactList, wakeBodies);
	}
}

b2ChainId b2CreateChainInternal(b2World* world, int bodyKey, b2Transform transform, b2ChainList* chainList,
								b2ShapeList* shapeList, bool isEnabled, const b2ChainDef* def)
{
	B2_ASSERT(b2IsValid(def->friction) && def->friction >= 0.0f);
	B2_ASSERT(b2IsValid(def->restitution) && def->restitution >= 0.0f);
	B2_ASSERT(def->count >= 4);

	b2ChainShape* chainShape = (b2ChainShape*)b2AllocObject(&world->chainPool);
	world->chains = (b2ChainShape*)world->chainPool.memory;

	int chainIndex = chainShape->object.index;
	chainShape->bodyKey = bodyKey;
	chainShape->nextIndex = chainList->headChainId;
	chainList->headChainId = chainShape->object.index;

	b2ShapeDef shapeDef = b2DefaultShapeDef();
	shapeDef.userData = def->userData;
	shapeDef.restitution = def->restitution;
	shapeDef.friction = def->friction;
	shapeDef.filter = def->filter;
	shapeDef.enableContactEvents = false;
	shapeDef.enableSensorEvents = false;

	int n = def->count;
	const b2Vec2* points = def->points;

	if (def->isLoop)
	{
		chainShape->count = n;
		chainShape->shapeIndices = b2Alloc(n * sizeof(int));

		b2SmoothSegment smoothSegment;

		int prevIndex = n - 1;
		for (int i = 0; i < n - 2; ++i)
		{
			smoothSegment.ghost1 = points[prevIndex];
			smoothSegment.segment.point1 = points[i];
			smoothSegment.segment.point2 = points[i + 1];
			smoothSegment.ghost2 = points[i + 2];
			smoothSegment.chainIndex = chainIndex;
			prevIndex = i;

			b2Shape* shape =
				b2CreateShape(world, bodyKey, transform, shapeList, isEnabled, &shapeDef, &smoothSegment, b2_smoothSegmentShape);
			chainShape->shapeIndices[i] = shape->object.index;
		}

		{
			smoothSegment.ghost1 = points[n - 3];
			smoothSegment.segment.point1 = points[n - 2];
			smoothSegment.segment.point2 = points[n - 1];
			smoothSegment.ghost2 = points[0];
			smoothSegment.chainIndex = chainIndex;
			b2Shape* shape =
				b2CreateShape(world, bodyKey, transform, shapeList, isEnabled, &shapeDef, &smoothSegment, b2_smoothSegmentShape);
			chainShape->shapeIndices[n - 2] = shape->object.index;
		}

		{
			smoothSegment.ghost1 = points[n - 2];
			smoothSegment.segment.point1 = points[n - 1];
			smoothSegment.segment.point2 = points[0];
			smoothSegment.ghost2 = points[1];
			smoothSegment.chainIndex = chainIndex;
			b2Shape* shape =
				b2CreateShape(world, bodyKey, transform, shapeList, isEnabled, &shapeDef, &smoothSegment, b2_smoothSegmentShape);
			chainShape->shapeIndices[n - 1] = shape->object.index;
		}
	}
	else
	{
		chainShape->count = n - 3;
		chainShape->shapeIndices = b2Alloc(n * sizeof(int));

		b2SmoothSegment smoothSegment;

		for (int i = 0; i < n - 3; ++i)
		{
			smoothSegment.ghost1 = points[i];
			smoothSegment.segment.point1 = points[i + 1];
			smoothSegment.segment.point2 = points[i + 2];
			smoothSegment.ghost2 = points[i + 3];
			smoothSegment.chainIndex = chainIndex;

			b2Shape* shape =
				b2CreateShape(world, bodyKey, transform, shapeList, isEnabled, &shapeDef, &smoothSegment, b2_smoothSegmentShape);
			chainShape->shapeIndices[i] = shape->object.index;
		}
	}

	b2ChainId id = {chainShape->object.index + 1, world->worldId, chainShape->object.revision};
	return id;
}

b2ChainId b2CreateStaticChain(b2StaticBodyId staticBodyId, const b2ChainDef* def)
{
	b2World* world = b2GetWorldLocked(staticBodyId.world0);
	if (world == NULL)
	{
		return (b2ChainId){0};
	}

	b2StaticBody* staticBody = b2GetStaticBodyFullId(world, staticBodyId);
	int bodyKey = (staticBody->bodyId << 1) | 0;
	b2Transform transform = staticBody->transform;
	bool isEnabled = true;
	return b2CreateChainInternal(world, bodyKey, transform, &staticBody->chainList, &staticBody->shapeList, isEnabled, def);
}

b2ChainId b2CreateChain(b2BodyId bodyId, const b2ChainDef* def)
{
	b2World* world = b2GetWorldLocked(bodyId.world0);
	if (world == NULL)
	{
		return (b2ChainId){0};
	}

	b2Body* body = b2GetBodyFullId(world, bodyId);
	int bodyKey = (body->bodyId << 1) | 1;
	b2Transform transform = b2MakeTransform(body);
	bool isEnabled = body->isEnabled;
	return b2CreateChainInternal(world, bodyKey, transform, &body->chainList, &body->shapeList, isEnabled, def);
}

void b2DestroyChainInternal(b2World* world, b2ChainShape* chain, b2ChainList* chainList, b2ShapeList* shapeList, b2ContactList* contactList, bool wakeBodies)
{
	// Remove the chain from the body's singly linked list.
	int* chainIdPtr = &chainList->headChainId;
	bool found = false;
	while (*chainIdPtr != B2_NULL_INDEX)
	{
		if (*chainIdPtr == chain->object.index)
		{
			*chainIdPtr = chain->nextIndex;
			found = true;
			break;
		}

		chainIdPtr = &(world->chains[*chainIdPtr].nextIndex);
	}

	B2_ASSERT(found == true);
	if (found == false)
	{
		return;
	}

	int count = chain->count;
	for (int i = 0; i < count; ++i)
	{
		int shapeIndex = chain->shapeIndices[i];
		B2_ASSERT(0 <= shapeIndex && shapeIndex < world->shapePool.count);
		b2Shape* shape = world->shapes + shapeIndex;
		b2DestroyShapeInternal(world, shape, shapeList, contactList, wakeBodies);
	}

	b2Free(chain->shapeIndices, count * sizeof(int));
	b2FreeObject(&world->chainPool, &chain->object);
}

void b2DestroyChain(b2ChainId chainId)
{
	b2CheckChainId(chainId);

	b2World* world = b2GetWorldLocked(chainId.world0);
	b2ChainShape* chain = world->chains + (chainId.index1 - 1);
	bool wakeBodies = true;

	int bodyKey = chain->bodyKey;
	if (bodyKey & 1)
	{
		b2Body* body = b2GetBody(world, bodyKey >> 1);
		b2DestroyChainInternal(world, chain, &body->chainList, & body->shapeList, &body->contactList, wakeBodies);
	}
	else
	{
		b2StaticBody* staticBody = b2GetStaticBody(world, bodyKey >> 1);
		b2DestroyChainInternal(world, chain, &staticBody->chainList, &staticBody->shapeList, &staticBody->contactList,
							   wakeBodies);
	}
}

b2AABB b2ComputeShapeAABB(const b2Shape* shape, b2Transform xf)
{
	switch (shape->type)
	{
		case b2_capsuleShape:
			return b2ComputeCapsuleAABB(&shape->capsule, xf);
		case b2_circleShape:
			return b2ComputeCircleAABB(&shape->circle, xf);
		case b2_polygonShape:
			return b2ComputePolygonAABB(&shape->polygon, xf);
		case b2_segmentShape:
			return b2ComputeSegmentAABB(&shape->segment, xf);
		case b2_smoothSegmentShape:
			return b2ComputeSegmentAABB(&shape->smoothSegment.segment, xf);
		default:
		{
			B2_ASSERT(false);
			b2AABB empty = {xf.p, xf.p};
			return empty;
		}
	}
}

b2Vec2 b2GetShapeCentroid(const b2Shape* shape)
{
	switch (shape->type)
	{
		case b2_capsuleShape:
			return b2Lerp(shape->capsule.point1, shape->capsule.point2, 0.5f);
		case b2_circleShape:
			return shape->circle.point;
		case b2_polygonShape:
			return shape->polygon.centroid;
		case b2_segmentShape:
			return b2Lerp(shape->segment.point1, shape->segment.point2, 0.5f);
		case b2_smoothSegmentShape:
			return b2Lerp(shape->smoothSegment.segment.point1, shape->smoothSegment.segment.point2, 0.5f);
		default:
			return b2Vec2_zero;
	}
}

b2MassData b2ComputeShapeMass(const b2Shape* shape)
{
	switch (shape->type)
	{
		case b2_capsuleShape:
			return b2ComputeCapsuleMass(&shape->capsule, shape->density);
		case b2_circleShape:
			return b2ComputeCircleMass(&shape->circle, shape->density);
		case b2_polygonShape:
			return b2ComputePolygonMass(&shape->polygon, shape->density);
		default:
		{
			return (b2MassData){0};
		}
	}
}

b2ShapeExtent b2ComputeShapeExtent(const b2Shape* shape)
{
	b2ShapeExtent extent = {0};

	switch (shape->type)
	{
		case b2_capsuleShape:
		{
			float radius = shape->capsule.radius;
			extent.minExtent = radius;
			extent.maxExtent = B2_MAX(b2Length(shape->capsule.point1), b2Length(shape->capsule.point2)) + radius;
		}
		break;

		case b2_circleShape:
		{
			float radius = shape->circle.radius;
			extent.minExtent = radius;
			extent.maxExtent = b2Length(shape->circle.point) + radius;
		}
		break;

		case b2_polygonShape:
		{
			const b2Polygon* poly = &shape->polygon;
			float minExtent = b2_huge;
			float maxExtentSqr = 0.0f;
			int count = poly->count;
			for (int i = 0; i < count; ++i)
			{
				float planeOffset = b2Dot(poly->normals[i], b2Sub(poly->vertices[i], poly->centroid));
				minExtent = B2_MIN(minExtent, planeOffset);

				float distanceSqr = b2LengthSquared(poly->vertices[i]);
				maxExtentSqr = B2_MAX(maxExtentSqr, distanceSqr);
			}

			extent.minExtent = minExtent + poly->radius;
			extent.maxExtent = sqrtf(maxExtentSqr) + poly->radius;
		}
		break;

		default:
			break;
	}

	return extent;
}

b2CastOutput b2RayCastShape(const b2RayCastInput* input, const b2Shape* shape, b2Transform xf)
{
	b2RayCastInput localInput = *input;
	localInput.origin = b2InvTransformPoint(xf, input->origin);
	localInput.translation = b2InvRotateVector(xf.q, input->translation);

	b2CastOutput output = {0};
	switch (shape->type)
	{
		case b2_capsuleShape:
			output = b2RayCastCapsule(&localInput, &shape->capsule);
			break;
		case b2_circleShape:
			output = b2RayCastCircle(&localInput, &shape->circle);
			break;
		case b2_polygonShape:
			output = b2RayCastPolygon(&localInput, &shape->polygon);
			break;
		case b2_segmentShape:
			output = b2RayCastSegment(&localInput, &shape->segment, false);
			break;
		case b2_smoothSegmentShape:
			output = b2RayCastSegment(&localInput, &shape->smoothSegment.segment, true);
			break;
		default:
			return output;
	}

	output.point = b2TransformPoint(xf, output.point);
	output.normal = b2RotateVector(xf.q, output.normal);
	return output;
}

b2CastOutput b2ShapeCastShape(const b2ShapeCastInput* input, const b2Shape* shape, b2Transform xf)
{
	b2ShapeCastInput localInput = *input;

	for (int i = 0; i < localInput.count; ++i)
	{
		localInput.points[i] = b2InvTransformPoint(xf, input->points[i]);
	}

	localInput.translation = b2InvRotateVector(xf.q, input->translation);

	b2CastOutput output = {0};
	switch (shape->type)
	{
		case b2_capsuleShape:
			output = b2ShapeCastCapsule(&localInput, &shape->capsule);
			break;
		case b2_circleShape:
			output = b2ShapeCastCircle(&localInput, &shape->circle);
			break;
		case b2_polygonShape:
			output = b2ShapeCastPolygon(&localInput, &shape->polygon);
			break;
		case b2_segmentShape:
			output = b2ShapeCastSegment(&localInput, &shape->segment);
			break;
		case b2_smoothSegmentShape:
			output = b2ShapeCastSegment(&localInput, &shape->smoothSegment.segment);
			break;
		default:
			return output;
	}

	output.point = b2TransformPoint(xf, output.point);
	output.normal = b2RotateVector(xf.q, output.normal);
	return output;
}

void b2CreateShapeProxy(b2Shape* shape, b2BroadPhase* bp, b2ProxyType type, b2Transform transform)
{
	B2_ASSERT(shape->proxyKey == B2_NULL_INDEX);

	// Compute a bounding box with a speculative margin
	b2AABB aabb = b2ComputeShapeAABB(shape, transform);
	aabb.lowerBound.x -= b2_speculativeDistance;
	aabb.lowerBound.y -= b2_speculativeDistance;
	aabb.upperBound.x += b2_speculativeDistance;
	aabb.upperBound.y += b2_speculativeDistance;
	shape->aabb = aabb;

	// Smaller margin for static bodies. Cannot be zero due to TOI tolerance.
	float margin = type == b2_staticProxy ? b2_speculativeDistance : b2_aabbMargin;
	b2AABB fatAABB;
	fatAABB.lowerBound.x = aabb.lowerBound.x - margin;
	fatAABB.lowerBound.y = aabb.lowerBound.y - margin;
	fatAABB.upperBound.x = aabb.upperBound.x + margin;
	fatAABB.upperBound.y = aabb.upperBound.y + margin;
	shape->fatAABB = fatAABB;

	// Create proxies in the broad-phase.
	shape->proxyKey = b2BroadPhase_CreateProxy(bp, type, fatAABB, shape->filter.categoryBits, shape->object.index);
	B2_ASSERT(B2_PROXY_TYPE(shape->proxyKey) < b2_proxyTypeCount);
}

void b2DestroyShapeProxy(b2Shape* shape, b2BroadPhase* bp)
{
	if (shape->proxyKey != B2_NULL_INDEX)
	{
		b2BroadPhase_DestroyProxy(bp, shape->proxyKey);
		shape->proxyKey = B2_NULL_INDEX;
	}
}

b2DistanceProxy b2MakeShapeDistanceProxy(const b2Shape* shape)
{
	switch (shape->type)
	{
		case b2_capsuleShape:
			return b2MakeProxy(&shape->capsule.point1, 2, shape->capsule.radius);
		case b2_circleShape:
			return b2MakeProxy(&shape->circle.point, 1, shape->circle.radius);
		case b2_polygonShape:
			return b2MakeProxy(shape->polygon.vertices, shape->polygon.count, shape->polygon.radius);
		case b2_segmentShape:
			return b2MakeProxy(&shape->segment.point1, 2, 0.0f);
		case b2_smoothSegmentShape:
			return b2MakeProxy(&shape->smoothSegment.segment.point1, 2, 0.0f);
		default:
		{
			B2_ASSERT(false);
			b2DistanceProxy empty = {0};
			return empty;
		}
	}
}

b2BodyId b2Shape_GetBody(b2ShapeId shapeId)
{
	b2World* world = b2GetWorld(shapeId.world0);
	b2Shape* shape = b2GetShape(world, shapeId);
	if (shape->bodyKey & 1)
	{
		return b2MakeBodyId(world, shape->bodyKey >> 1);
	}

	return (b2BodyId){0};
}

void b2Shape_SetUserData(b2ShapeId shapeId, void* userData)
{
	b2World* world = b2GetWorld(shapeId.world0);
	b2Shape* shape = b2GetShape(world, shapeId);
	shape->userData = userData;
}

void* b2Shape_GetUserData(b2ShapeId shapeId)
{
	b2World* world = b2GetWorld(shapeId.world0);
	b2Shape* shape = b2GetShape(world, shapeId);
	return shape->userData;
}

bool b2Shape_IsSensor(b2ShapeId shapeId)
{
	b2World* world = b2GetWorld(shapeId.world0);
	b2Shape* shape = b2GetShape(world, shapeId);
	return shape->isSensor;
}

bool b2Shape_TestPoint(b2ShapeId shapeId, b2Vec2 point)
{
	b2World* world = b2GetWorld(shapeId.world0);
	b2Shape* shape = b2GetShape(world, shapeId);

	b2Transform transform = b2GetOwnerTransform(world, shape);
	b2Vec2 localPoint = b2InvTransformPoint(transform, point);

	switch (shape->type)
	{
		case b2_capsuleShape:
			return b2PointInCapsule(localPoint, &shape->capsule);

		case b2_circleShape:
			return b2PointInCircle(localPoint, &shape->circle);

		case b2_polygonShape:
			return b2PointInPolygon(localPoint, &shape->polygon);

		default:
			return false;
	}
}

b2CastOutput b2Shape_RayCast(b2ShapeId shapeId, b2Vec2 origin, b2Vec2 translation)
{
	b2World* world = b2GetWorld(shapeId.world0);
	b2Shape* shape = b2GetShape(world, shapeId);

	b2Transform transform = b2GetOwnerTransform(world, shape);

	// input in local coordinates
	b2RayCastInput input;
	input.maxFraction = 1.0f;
	input.origin = b2InvTransformPoint(transform, origin);
	input.translation = b2InvRotateVector(transform.q, translation);

	b2CastOutput output = {0};
	switch (shape->type)
	{
		case b2_capsuleShape:
			output = b2RayCastCapsule(&input, &shape->capsule);
			break;

		case b2_circleShape:
			output = b2RayCastCircle(&input, &shape->circle);
			break;

		case b2_segmentShape:
			output = b2RayCastSegment(&input, &shape->segment, false);
			break;

		case b2_polygonShape:
			output = b2RayCastPolygon(&input, &shape->polygon);
			break;

		case b2_smoothSegmentShape:
			output = b2RayCastSegment(&input, &shape->smoothSegment.segment, true);
			break;

		default:
			B2_ASSERT(false);
			return output;
	}

	if (output.hit)
	{
		// convert to world coordinates
		output.normal = b2RotateVector(transform.q, output.normal);
		output.point = b2TransformPoint(transform, output.point);
	}
	return output;
}

void b2Shape_SetDensity(b2ShapeId shapeId, float density)
{
	B2_ASSERT(b2IsValid(density) && density >= 0.0f);

	b2World* world = b2GetWorldLocked(shapeId.world0);
	if (world == NULL)
	{
		return;
	}

	b2Shape* shape = b2GetShape(world, shapeId);
	if (density == shape->density)
	{
		// early return to avoid expensive function
		return;
	}

	shape->density = density;
}

float b2Shape_GetDensity(b2ShapeId shapeId)
{
	b2World* world = b2GetWorld(shapeId.world0);
	b2Shape* shape = b2GetShape(world, shapeId);
	return shape->density;
}

void b2Shape_SetFriction(b2ShapeId shapeId, float friction)
{
	B2_ASSERT(b2IsValid(friction) && friction >= 0.0f);

	b2World* world = b2GetWorld(shapeId.world0);
	B2_ASSERT(world->locked == false);
	if (world->locked)
	{
		return;
	}

	b2Shape* shape = b2GetShape(world, shapeId);
	shape->friction = friction;
}

float b2Shape_GetFriction(b2ShapeId shapeId)
{
	b2World* world = b2GetWorld(shapeId.world0);
	b2Shape* shape = b2GetShape(world, shapeId);
	return shape->friction;
}

void b2Shape_SetRestitution(b2ShapeId shapeId, float restitution)
{
	B2_ASSERT(b2IsValid(restitution) && restitution >= 0.0f);

	b2World* world = b2GetWorld(shapeId.world0);
	B2_ASSERT(world->locked == false);
	if (world->locked)
	{
		return;
	}

	b2Shape* shape = b2GetShape(world, shapeId);
	shape->restitution = restitution;
}

float b2Shape_GetRestitution(b2ShapeId shapeId)
{
	b2World* world = b2GetWorld(shapeId.world0);
	b2Shape* shape = b2GetShape(world, shapeId);
	return shape->restitution;
}

b2Filter b2Shape_GetFilter(b2ShapeId shapeId)
{
	b2World* world = b2GetWorld(shapeId.world0);
	b2Shape* shape = b2GetShape(world, shapeId);
	return shape->filter;
}

static void b2ResetProxy(b2World* world, b2Shape* shape)
{
	b2ContactList* contactList = b2GetContactList(world, shape->bodyKey);

	int shapeId = shape->object.index;
	bool wakeBodies = true;

	// destroy all contacts associated with this shape
	int contactKey = contactList->headContactKey;
	while (contactKey != B2_NULL_INDEX)
	{
		int contactId = contactKey >> 1;
		int edgeIndex = contactKey & 1;

		b2Contact* contact = b2GetContactFromRawId(world, contactId);
		contactKey = contact->edges[edgeIndex].nextKey;

		if (contact->shapeIdA == shapeId || contact->shapeIdB == shapeId)
		{
			b2DestroyContact(world, contact, wakeBodies);
		}
	}

	// Touch the broad-phase proxies to ensure the correct contacts get created.
	int proxyKey = shape->proxyKey;
	if (proxyKey != B2_NULL_INDEX && B2_PROXY_TYPE(proxyKey) == b2_movableProxy)
	{
		b2BufferMove(&world->broadPhase, proxyKey);
	}
}

void b2Shape_SetFilter(b2ShapeId shapeId, b2Filter filter)
{
	b2World* world = b2GetWorldLocked(shapeId.world0);
	if (world == NULL)
	{
		return;
	}

	b2Shape* shape = b2GetShape(world, shapeId);
	shape->filter = filter;
	b2ResetProxy(world, shape);
}

void b2Shape_EnableSensorEvents(b2ShapeId shapeId, bool flag)
{
	b2World* world = b2GetWorldLocked(shapeId.world0);
	if (world == NULL)
	{
		return;
	}

	b2Shape* shape = b2GetShape(world, shapeId);
	shape->enableSensorEvents = flag;
}

bool b2Shape_AreSensorEventsEnabled(b2ShapeId shapeId)
{
	b2World* world = b2GetWorld(shapeId.world0);
	b2Shape* shape = b2GetShape(world, shapeId);
	return shape->enableSensorEvents;
}

void b2Shape_EnableContactEvents(b2ShapeId shapeId, bool flag)
{
	b2World* world = b2GetWorldLocked(shapeId.world0);
	if (world == NULL)
	{
		return;
	}

	b2Shape* shape = b2GetShape(world, shapeId);
	shape->enableContactEvents = flag;
}

bool b2Shape_AreContactEventsEnabled(b2ShapeId shapeId)
{
	b2World* world = b2GetWorld(shapeId.world0);
	b2Shape* shape = b2GetShape(world, shapeId);
	return shape->enableContactEvents;
}

void b2Shape_EnablePreSolveEvents(b2ShapeId shapeId, bool flag)
{
	b2World* world = b2GetWorldLocked(shapeId.world0);
	if (world == NULL)
	{
		return;
	}

	b2Shape* shape = b2GetShape(world, shapeId);
	shape->enablePreSolveEvents = flag;
}

bool b2Shape_ArePreSolveEventsEnabled(b2ShapeId shapeId)
{
	b2World* world = b2GetWorld(shapeId.world0);
	b2Shape* shape = b2GetShape(world, shapeId);
	return shape->enablePreSolveEvents;
}

b2ShapeType b2Shape_GetType(b2ShapeId shapeId)
{
	b2World* world = b2GetWorld(shapeId.world0);
	b2Shape* shape = b2GetShape(world, shapeId);
	return shape->type;
}

b2Circle b2Shape_GetCircle(b2ShapeId shapeId)
{
	b2World* world = b2GetWorld(shapeId.world0);
	b2Shape* shape = b2GetShape(world, shapeId);
	B2_ASSERT(shape->type == b2_circleShape);
	return shape->circle;
}

b2Segment b2Shape_GetSegment(b2ShapeId shapeId)
{
	b2World* world = b2GetWorld(shapeId.world0);
	b2Shape* shape = b2GetShape(world, shapeId);
	B2_ASSERT(shape->type == b2_segmentShape);
	return shape->segment;
}

b2SmoothSegment b2Shape_GetSmoothSegment(b2ShapeId shapeId)
{
	b2World* world = b2GetWorld(shapeId.world0);
	b2Shape* shape = b2GetShape(world, shapeId);
	B2_ASSERT(shape->type == b2_smoothSegmentShape);
	return shape->smoothSegment;
}

b2Capsule b2Shape_GetCapsule(b2ShapeId shapeId)
{
	b2World* world = b2GetWorld(shapeId.world0);
	b2Shape* shape = b2GetShape(world, shapeId);
	B2_ASSERT(shape->type == b2_capsuleShape);
	return shape->capsule;
}

b2Polygon b2Shape_GetPolygon(b2ShapeId shapeId)
{
	b2World* world = b2GetWorld(shapeId.world0);
	b2Shape* shape = b2GetShape(world, shapeId);
	B2_ASSERT(shape->type == b2_polygonShape);
	return shape->polygon;
}

void b2Shape_SetCircle(b2ShapeId shapeId, const b2Circle* circle)
{
	b2World* world = b2GetWorldLocked(shapeId.world0);
	if (world == NULL)
	{
		return;
	}

	b2Shape* shape = b2GetShape(world, shapeId);
	shape->circle = *circle;
	shape->type = b2_circleShape;
	b2ResetProxy(world, shape);

	if (shape->bodyKey & 1)
	{
		int bodyId = shape->bodyKey >> 1;
		b2WakeBody(world, bodyId);
	}
}

void b2Shape_SetCapsule(b2ShapeId shapeId, const b2Capsule* capsule)
{
	b2World* world = b2GetWorldLocked(shapeId.world0);
	if (world == NULL)
	{
		return;
	}

	b2Shape* shape = b2GetShape(world, shapeId);
	shape->capsule = *capsule;
	shape->type = b2_capsuleShape;
	b2ResetProxy(world, shape);

	if (shape->bodyKey & 1)
	{
		int bodyId = shape->bodyKey >> 1;
		b2WakeBody(world, bodyId);
	}
}

void b2Shape_SetSegment(b2ShapeId shapeId, const b2Segment* segment)
{
	b2World* world = b2GetWorldLocked(shapeId.world0);
	if (world == NULL)
	{
		return;
	}

	b2Shape* shape = b2GetShape(world, shapeId);
	shape->segment = *segment;
	shape->type = b2_segmentShape;
	b2ResetProxy(world, shape);

	if (shape->bodyKey & 1)
	{
		int bodyId = shape->bodyKey >> 1;
		b2WakeBody(world, bodyId);
	}
}

void b2Shape_SetPolygon(b2ShapeId shapeId, const b2Polygon* polygon)
{
	b2World* world = b2GetWorldLocked(shapeId.world0);
	if (world == NULL)
	{
		return;
	}

	b2Shape* shape = b2GetShape(world, shapeId);
	shape->polygon = *polygon;
	shape->type = b2_polygonShape;
	b2ResetProxy(world, shape);

	if (shape->bodyKey & 1)
	{
		int bodyId = shape->bodyKey >> 1;
		b2WakeBody(world, bodyId);
	}
}

b2ChainId b2Shape_GetParentChain(b2ShapeId shapeId)
{
	b2World* world = b2GetWorld(shapeId.world0);
	b2Shape* shape = b2GetShape(world, shapeId);
	if (shape->type == b2_smoothSegmentShape)
	{
		int chainIndex = shape->smoothSegment.chainIndex;
		if (chainIndex != B2_NULL_INDEX)
		{
			B2_ASSERT(0 <= chainIndex && chainIndex < world->chainPool.capacity);
			b2ChainShape* chain = world->chains + chainIndex;
			B2_ASSERT(b2IsValidObject(&chain->object));
			b2ChainId chainId = {chainIndex + 1, shapeId.world0, chain->object.revision};
			return chainId;
		}
	}

	return b2_nullChainId;
}

void b2Chain_SetFriction(b2ChainId chainId, float friction)
{
	b2World* world = b2GetWorldLocked(chainId.world0);
	if (world == NULL)
	{
		return;
	}

	b2ChainShape* chainShape = b2GetChainShape(world, chainId);

	int count = chainShape->count;

	for (int i = 0; i < count; ++i)
	{
		int shapeIndex = chainShape->shapeIndices[i];
		B2_ASSERT(0 <= shapeIndex && shapeIndex < world->shapePool.count);
		b2Shape* shape = world->shapes + shapeIndex;
		shape->friction = friction;
	}
}

void b2Chain_SetRestitution(b2ChainId chainId, float restitution)
{
	b2World* world = b2GetWorldLocked(chainId.world0);
	if (world == NULL)
	{
		return;
	}

	b2ChainShape* chainShape = b2GetChainShape(world, chainId);

	int count = chainShape->count;

	for (int i = 0; i < count; ++i)
	{
		int shapeIndex = chainShape->shapeIndices[i];
		B2_ASSERT(0 <= shapeIndex && shapeIndex < world->shapePool.count);
		b2Shape* shape = world->shapes + shapeIndex;
		shape->restitution = restitution;
	}
}

int b2Shape_GetContactCapacity(b2ShapeId shapeId)
{
	b2World* world = b2GetWorldLocked(shapeId.world0);
	if (world == NULL)
	{
		return 0;
	}

	b2Shape* shape = b2GetShape(world, shapeId);
	b2ContactList* contactList = b2GetContactList(world, shape->bodyKey);

	// Conservative and fast
	return contactList->contactCount;
}

int b2Shape_GetContactData(b2ShapeId shapeId, b2ContactData* contactData, int capacity)
{
	b2World* world = b2GetWorldLocked(shapeId.world0);
	if (world == NULL)
	{
		return 0;
	}

	b2Shape* shape = b2GetShape(world, shapeId);

	b2ContactList* contactList = b2GetContactList(world, shape->bodyKey);
	int contactKey = contactList->headContactKey;
	int index = 0;
	while (contactKey != B2_NULL_INDEX && index < capacity)
	{
		int contactId = contactKey >> 1;
		int edgeIndex = contactKey & 1;

		b2Contact* contact = b2GetContactFromRawId(world, contactId);

		// Does contact involve this shape and is it touching?
		if ((contact->shapeIdA == shapeId.index1 - 1 || contact->shapeIdB == shapeId.index1 - 1) &&
			(contact->flags & b2_contactTouchingFlag) != 0)
		{
			b2Shape* shapeA = world->shapes + contact->shapeIdA;
			b2Shape* shapeB = world->shapes + contact->shapeIdB;

			contactData[index].shapeIdA = (b2ShapeId){shapeA->object.index + 1, shapeId.world0, shapeA->object.revision};
			contactData[index].shapeIdB = (b2ShapeId){shapeB->object.index + 1, shapeId.world0, shapeB->object.revision};
			contactData[index].manifold = contact->manifold;
			index += 1;
		}

		contactKey = contact->edges[edgeIndex].nextKey;
	}

	B2_ASSERT(index < capacity);

	return index;
}

b2AABB b2Shape_GetAABB(b2ShapeId shapeId)
{
	b2World* world = b2GetWorldLocked(shapeId.world0);
	if (world == NULL)
	{
		return (b2AABB){0};
	}

	b2Shape* shape = b2GetShape(world, shapeId);
	return shape->aabb;
}
