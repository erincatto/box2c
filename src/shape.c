// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "shape.h"

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
	B2_ASSERT(1 <= shapeId.index1 && shapeId.index1 <= world->shapePool.capacity);
	b2Shape* shape = world->shapes + (shapeId.index1 - 1);
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
			int32_t count = poly->count;
			for (int32_t i = 0; i < count; ++i)
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

void b2CreateShapeProxy(b2Shape* shape, b2BroadPhase* bp, b2BodyType type, b2Transform transform)
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
	float margin = type == b2_staticBody ? b2_speculativeDistance : b2_aabbMargin;
	b2AABB fatAABB;
	fatAABB.lowerBound.x = aabb.lowerBound.x - margin;
	fatAABB.lowerBound.y = aabb.lowerBound.y - margin;
	fatAABB.upperBound.x = aabb.upperBound.x + margin;
	fatAABB.upperBound.y = aabb.upperBound.y + margin;
	shape->fatAABB = fatAABB;

	// Create proxies in the broad-phase.
	shape->proxyKey = b2BroadPhase_CreateProxy(bp, type, fatAABB, shape->filter.categoryBits, shape->object.index);
	B2_ASSERT(B2_PROXY_TYPE(shape->proxyKey) < b2_bodyTypeCount);
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

static void b2ResetContactsAndProxy(b2World* world, b2Shape* shape)
{
	int32_t shapeIndex = shape->object.index;

	// Destroy any contacts associated with the shape
	b2ContactList* b2GetContactList()
	int32_t contactKey = body->contactList;
	while (contactKey != B2_NULL_INDEX)
	{
		int32_t contactId = contactKey >> 1;
		int32_t edgeIndex = contactKey & 1;

		b2Contact* contact = b2GetContactFromRawId(world, contactId);
		contactKey = contact->edges[edgeIndex].nextKey;

		if (contact->shapeIndexA == shapeIndex || contact->shapeIndexB == shapeIndex)
		{
			// careful this can wake bodies and orphan pointers
			b2DestroyContact(world, contact);
		}
	}

	// refresh body pointer in case destroying a contact waked the body and orphaned the pointer
	body = b2GetBodyFromRawId(world, shape->bodyId);

	if (body->isEnabled)
	{
		// Must recreate proxy due to changed filter bits that exist in the dynamic tree
		b2DestroyShapeProxy(shape, &world->broadPhase);
		b2CreateShapeProxy(shape, &world->broadPhase, body->type, b2MakeTransform(body));
	}
	else
	{
		B2_ASSERT(shape->proxyKey == B2_NULL_INDEX);
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
	int32_t shapeIndex = shape->object.index;

	b2Body* body = b2GetBodyFromRawId(world, shape->bodyId);

	// Destroy any contacts associated with the shape
	int32_t contactKey = body->contactList;
	while (contactKey != B2_NULL_INDEX)
	{
		int32_t contactId = contactKey >> 1;
		int32_t edgeIndex = contactKey & 1;

		b2Contact* contact = b2GetContactFromRawId(world, contactId);
		contactKey = contact->edges[edgeIndex].nextKey;

		if (contact->shapeIndexA == shapeIndex || contact->shapeIndexB == shapeIndex)
		{
			b2DestroyContact(world, contact);
		}
	}

	// refresh body pointer in case destroying a contact waked the body and orphaned the pointer
	body = b2GetBodyFromRawId(world, shape->bodyId);

	if (body->isEnabled)
	{
		// Must recreate proxy due to changed filter bits that exist in the dynamic tree
		b2DestroyShapeProxy(shape, &world->broadPhase);
		b2CreateShapeProxy(shape, &world->broadPhase, body->type, b2MakeTransform(body));
	}
	else
	{
		B2_ASSERT(shape->proxyKey == B2_NULL_INDEX);
	}
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

const b2Circle b2Shape_GetCircle(b2ShapeId shapeId)
{
	b2World* world = b2GetWorld(shapeId.world0);
	b2Shape* shape = b2GetShape(world, shapeId);
	B2_ASSERT(shape->type == b2_circleShape);
	return shape->circle;
}

const b2Segment b2Shape_GetSegment(b2ShapeId shapeId)
{
	b2World* world = b2GetWorld(shapeId.world0);
	b2Shape* shape = b2GetShape(world, shapeId);
	B2_ASSERT(shape->type == b2_segmentShape);
	return shape->segment;
}

const b2SmoothSegment b2Shape_GetSmoothSegment(b2ShapeId shapeId)
{
	b2World* world = b2GetWorld(shapeId.world0);
	b2Shape* shape = b2GetShape(world, shapeId);
	B2_ASSERT(shape->type == b2_smoothSegmentShape);
	return shape->smoothSegment;
}

const b2Capsule b2Shape_GetCapsule(b2ShapeId shapeId)
{
	b2World* world = b2GetWorld(shapeId.world0);
	b2Shape* shape = b2GetShape(world, shapeId);
	B2_ASSERT(shape->type == b2_capsuleShape);
	return shape->capsule;
}

const b2Polygon b2Shape_GetPolygon(b2ShapeId shapeId)
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

	b2ResetContactsAndProxy(world, shape);
	b2WakeBody(world, shape->bodyId);
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

	b2ResetContactsAndProxy(world, shape);
	b2WakeBody(world, shape->bodyId);
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

	b2ResetContactsAndProxy(world, shape);
	b2WakeBody(world, shape->bodyId);
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

	b2ResetContactsAndProxy(world, shape);
	b2WakeBody(world, shape->bodyId);
}

b2ChainId b2Shape_GetParentChain(b2ShapeId shapeId)
{
	b2World* world = b2GetWorld(shapeId.world0);
	b2Shape* shape = b2GetShape(world, shapeId);
	if (shape->type == b2_smoothSegmentShape)
	{
		int32_t chainIndex = shape->smoothSegment.chainIndex;
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

	int32_t count = chainShape->count;

	for (int32_t i = 0; i < count; ++i)
	{
		int32_t shapeIndex = chainShape->shapeIndices[i];
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

	int32_t count = chainShape->count;

	for (int32_t i = 0; i < count; ++i)
	{
		int32_t shapeIndex = chainShape->shapeIndices[i];
		B2_ASSERT(0 <= shapeIndex && shapeIndex < world->shapePool.count);
		b2Shape* shape = world->shapes + shapeIndex;
		shape->restitution = restitution;
	}
}

int32_t b2Shape_GetContactCapacity(b2ShapeId shapeId)
{
	b2World* world = b2GetWorldLocked(shapeId.world0);
	if (world == NULL)
	{
		return 0;
	}

	b2Shape* shape = b2GetShape(world, shapeId);
	b2Body* body = b2GetBodyFromRawId(world, shape->bodyId);

	// Conservative and fast
	return body->contactCount;
}

int32_t b2Shape_GetContactData(b2ShapeId shapeId, b2ContactData* contactData, int32_t capacity)
{
	b2World* world = b2GetWorldLocked(shapeId.world0);
	if (world == NULL)
	{
		return 0;
	}

	b2Shape* shape = b2GetShape(world, shapeId);

	b2Body* body = b2GetBodyFromRawId(world, shape->bodyId);
	int32_t contactKey = body->contactList;
	int32_t index = 0;
	while (contactKey != B2_NULL_INDEX && index < capacity)
	{
		int32_t contactId = contactKey >> 1;
		int32_t edgeIndex = contactKey & 1;

		b2Contact* contact = b2GetContactFromRawId(world, contactId);

		// Does contact involve this shape and is it touching?
		if ((contact->shapeIndexA == shapeId.index1 - 1 || contact->shapeIndexB == shapeId.index1 - 1) &&
			(contact->flags & b2_contactTouchingFlag) != 0)
		{
			b2Shape* shapeA = world->shapes + contact->shapeIndexA;
			b2Shape* shapeB = world->shapes + contact->shapeIndexB;

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
