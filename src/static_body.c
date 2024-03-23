// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "static_body.h"
#include "world.h"

// needed for dll export
#include "box2d/box2d.h"
#include "box2d/id.h"

b2StaticBodyDef b2DefaultStaticBodyDef()
{
	return (b2StaticBodyDef){0};
}

b2StaticBodyId b2CreateStaticBody(b2WorldId worldId, const b2StaticBodyDef* def)
{
	b2World* world = b2GetWorldFromId(worldId);
	B2_ASSERT(world->locked == false);

	if (world->locked)
	{
		return b2_nullStaticBodyId;
	}

	b2Body* body = b2AddBody(&world->blockAllocator, &set->bodies);
	b2BodyState* state = NULL;
	if (setIndex == b2_awakeSet)
	{
		state = b2AddBodyState(&world->blockAllocator, &set->states);
		B2_ASSERT(((uintptr_t)state & 0x1F) == 0);
	}

	int bodyId = b2AllocId(&world->bodyIdPool);

	B2_ASSERT(0 <= def->type && def->type < b2_bodyTypeCount);
	B2_ASSERT(b2Vec2_IsValid(def->position));
	B2_ASSERT(b2IsValid(def->angle));
	B2_ASSERT(b2Vec2_IsValid(def->linearVelocity));
	B2_ASSERT(b2IsValid(def->angularVelocity));
	B2_ASSERT(b2IsValid(def->linearDamping) && def->linearDamping >= 0.0f);
	B2_ASSERT(b2IsValid(def->angularDamping) && def->angularDamping >= 0.0f);
	B2_ASSERT(b2IsValid(def->gravityScale));

	*body = (b2Body){0};
	body->bodyId = bodyId;
	body->type = def->type;
	body->origin = def->position;
	body->rotation = b2MakeRot(def->angle);
	body->position = def->position;
	body->rotation0 = body->rotation;
	body->position0 = body->position;
	body->localCenter = b2Vec2_zero;
	body->force = b2Vec2_zero;
	body->torque = 0.0f;
	body->shapeList = B2_NULL_INDEX;
	body->shapeCount = 0;
	body->chainList = B2_NULL_INDEX;
	body->jointList = B2_NULL_INDEX;
	body->jointCount = 0;
	body->contactList = B2_NULL_INDEX;
	body->contactCount = 0;
	body->mass = 0.0f;
	body->invMass = 0.0f;
	body->I = 0.0f;
	body->invI = 0.0f;
	body->minExtent = b2_huge;
	body->maxExtent = 0.0f;
	body->linearDamping = def->linearDamping;
	body->angularDamping = def->angularDamping;
	body->gravityScale = def->gravityScale;
	body->sleepTime = 0.0f;
	body->userData = def->userData;
	body->world = world->worldId;
	body->enableSleep = def->enableSleep;
	body->fixedRotation = def->fixedRotation;
	body->isBullet = def->isBullet;
	body->isEnabled = def->isEnabled;
	body->isMarked = false;
	body->enlargeAABB = false;
	body->isFast = false;
	body->isSpeedCapped = false;
	body->islandId = B2_NULL_INDEX;
	body->islandPrev = B2_NULL_INDEX;
	body->islandNext = B2_NULL_INDEX;

	if (state != NULL)
	{
		*state = (b2BodyState){0};
		state->linearVelocity = def->linearVelocity;
		state->angularVelocity = def->angularVelocity;
		state->deltaRotation = b2Rot_identity;
	}

	// All disabled bodies are packed together in a single array where islands
	// have no meaning. Static bodies are never in islands. Sleeping solver sets
	// should have an islands so they can be quickly made awake.
	if (setIndex != b2_disabledSet && setIndex != b2_staticSet)
	{
		b2CreateIslandForBody(world, setIndex, body);
	}

	if (bodyId == b2Array(world->bodyLookupArray).count)
	{
		body->revision = 0;
		b2BodyLookup lookup = {setIndex, set->bodies.count - 1, 0};
		b2Array_Push(world->bodyLookupArray, lookup);
	}
	else
	{
		b2CheckIndex(world->bodyLookupArray, bodyId);
		b2BodyLookup* lookup = world->bodyLookupArray + bodyId;
		B2_ASSERT(lookup->setIndex == B2_NULL_INDEX && lookup->bodyIndex == B2_NULL_INDEX);
		lookup->setIndex = setIndex;
		lookup->bodyIndex = set->bodies.count - 1;
		lookup->revision += 1;
		body->revision = lookup->revision;
	}

	b2ValidateWorld(world);

	b2BodyId id = {bodyId + 1, world->worldId, body->revision};
	return id;
}

bool b2IsBodyAwake(b2World* world, int bodyId)
{
	b2CheckIndex(world->bodyLookupArray, bodyId);
	b2BodyLookup lookup = world->bodyLookupArray[bodyId];
	return lookup.setIndex == b2_awakeSet;
}

bool b2WakeBody(b2World* world, int bodyId)
{
	b2CheckIndex(world->bodyLookupArray, bodyId);
	b2BodyLookup lookup = world->bodyLookupArray[bodyId];
	B2_ASSERT(lookup.setIndex != b2_disabledSet);

	if (lookup.setIndex >= b2_firstSleepingSet)
	{
		b2WakeSolverSet(world, lookup.setIndex);
		return true;
	}

	return false;
}

static void b2DestroyBodyInternal(b2World* world, b2Body* body)
{
	// User must destroy joints before destroying bodies
	B2_ASSERT(body->jointList == B2_NULL_INDEX && body->jointCount == 0);

	b2DestroyBodyContacts(world, body);

	// Delete the attached shapes and their broad-phase proxies.
	int shapeIndex = body->shapeList;
	while (shapeIndex != B2_NULL_INDEX)
	{
		b2Shape* shape = world->shapes + shapeIndex;
		shapeIndex = shape->nextShapeIndex;

		b2DestroyShapeProxy(shape, &world->broadPhase);
		b2FreeObject(&world->shapePool, &shape->object);
	}

	// Delete the attached chains. The associated shapes have already been deleted above.
	int chainIndex = body->chainList;
	while (chainIndex != B2_NULL_INDEX)
	{
		b2ChainShape* chain = world->chains + chainIndex;
		chainIndex = chain->nextIndex;

		b2Free(chain->shapeIndices, chain->count * sizeof(int));
		chain->shapeIndices = NULL;
		b2FreeObject(&world->chainPool, &chain->object);
	}

	b2RemoveBodyFromIsland(world, body);

	// Remove body from solver set that owns it
	int bodyId = body->bodyId;
	b2CheckIndex(world->bodyLookupArray, bodyId);
	b2BodyLookup* lookup = world->bodyLookupArray + bodyId;
	b2CheckIndex(world->solverSetArray, lookup->setIndex);
	b2SolverSet* set = world->solverSetArray + lookup->setIndex;
	int movedIndex = b2RemoveBody(&set->bodies, lookup->bodyIndex);
	if (movedIndex != B2_NULL_INDEX)
	{
		// Fix lookup on moved body
		b2Body* movedBody = set->bodies.data + lookup->bodyIndex;
		int movedId = movedBody->bodyId;
		b2BodyLookup* movedLookup = world->bodyLookupArray + movedId;
		B2_ASSERT(movedLookup->bodyIndex == movedIndex);
		movedLookup->bodyIndex = lookup->bodyIndex;
	}

	// Only awake bodies have a body state
	if (lookup->setIndex == b2_awakeSet)
	{
		int result = b2RemoveBodyState(&set->states, lookup->bodyIndex);
		B2_MAYBE_UNUSED(result);
		B2_ASSERT(result == movedIndex);
	}

	// Free lookup and id (preserve lookup revision)
	lookup->setIndex = B2_NULL_INDEX;
	lookup->bodyIndex = B2_NULL_INDEX;
	b2FreeId(&world->bodyIdPool, bodyId);

	b2ValidateWorld(world);
}

void b2DestroyBody(b2BodyId bodyId)
{
	b2World* world = b2GetWorldLocked(bodyId.world0);
	if (world == NULL)
	{
		return;
	}

	b2Body* body = b2GetBody(world, bodyId);
	b2DestroyBodyInternal(world, body);
}

void b2DestroyBodyAndJoints(b2BodyId bodyId)
{
	b2World* world = b2GetWorldLocked(bodyId.world0);
	if (world == NULL)
	{
		return;
	}

	// Wake body before destroying, this way we don't need to wake
	// bodies attached to joints and contacts when they are destroyed
	B2_ASSERT(b2Body_IsValid(bodyId));
	int rawBodyId = bodyId.index1 - 1;
	b2WakeBody(world, rawBodyId);

	b2Body* body = b2GetBodyFromRawId(world, rawBodyId);

	// Destroy the attached joints
	int edgeKey = body->jointList;
	while (edgeKey != B2_NULL_INDEX)
	{
		int jointKey = edgeKey >> 1;
		int edgeIndex = edgeKey & 1;

		b2Joint* joint = b2GetJoint(world, jointKey);
		edgeKey = joint->edges[edgeIndex].nextKey;

		// Careful because this modifies the list being traversed
		b2DestroyJointInternal(world, joint, true);
	}

	b2DestroyBodyInternal(world, body);
}

int b2Body_GetContactCapacity(b2BodyId bodyId)
{
	b2World* world = b2GetWorldLocked(bodyId.world0);
	if (world == NULL)
	{
		return 0;
	}

	b2Body* body = b2GetBody(world, bodyId);

	// Conservative and fast
	return body->contactCount;
}

int b2Body_GetContactData(b2BodyId bodyId, b2ContactData* contactData, int capacity)
{
	b2World* world = b2GetWorldLocked(bodyId.world0);
	if (world == NULL)
	{
		return 0;
	}

	b2Body* body = b2GetBody(world, bodyId);

	int contactKey = body->contactList;
	int index = 0;
	while (contactKey != B2_NULL_INDEX && index < capacity)
	{
		int edgeIndex = contactKey & 1;

		int contactId = contactKey >> 1;
		b2Contact* contact = b2GetContactFromRawId(world, contactId);

		// Is contact touching?
		if (contact->flags & b2_contactTouchingFlag)
		{
			b2Shape* shapeA = world->shapes + contact->shapeIndexA;
			b2Shape* shapeB = world->shapes + contact->shapeIndexB;

			contactData[index].shapeIdA = (b2ShapeId){shapeA->object.index + 1, bodyId.world0, shapeA->object.revision};
			contactData[index].shapeIdB = (b2ShapeId){shapeB->object.index + 1, bodyId.world0, shapeB->object.revision};
			contactData[index].manifold = contact->manifold;
			index += 1;
		}

		contactKey = contact->edges[edgeIndex].nextKey;
	}

	B2_ASSERT(index < capacity);

	return index;
}

b2AABB b2Body_ComputeAABB(b2BodyId bodyId)
{
	b2World* world = b2GetWorldLocked(bodyId.world0);
	if (world == NULL)
	{
		return (b2AABB){0};
	}

	b2Body* body = b2GetBody(world, bodyId);
	if (body->shapeList == B2_NULL_INDEX)
	{
		return (b2AABB){body->origin, body->origin};
	}

	b2Shape* shape = world->shapes + body->shapeList;
	b2AABB aabb = shape->aabb;
	while (shape->nextShapeIndex != B2_NULL_INDEX)
	{
		shape = world->shapes + shape->nextShapeIndex;
		aabb = b2AABB_Union(aabb, shape->aabb);
	}

	return aabb;
}

void b2UpdateBodyMassData(b2World* world, b2Body* body)
{
	// Compute mass data from shapes. Each shape has its own density.
	body->mass = 0.0f;
	body->invMass = 0.0f;
	body->I = 0.0f;
	body->invI = 0.0f;
	body->localCenter = b2Vec2_zero;
	body->minExtent = b2_huge;
	body->maxExtent = 0.0f;

	// Static and kinematic bodies have zero mass.
	if (body->type == b2_staticBody || body->type == b2_kinematicBody)
	{
		body->position = body->origin;
		return;
	}

	B2_ASSERT(body->type == b2_dynamicBody);

	// Accumulate mass over all shapes.
	b2Vec2 localCenter = b2Vec2_zero;
	int shapeIndex = body->shapeList;
	while (shapeIndex != B2_NULL_INDEX)
	{
		const b2Shape* s = world->shapes + shapeIndex;
		shapeIndex = s->nextShapeIndex;

		if (s->density == 0.0f)
		{
			continue;
		}

		b2MassData massData = b2ComputeShapeMass(s);
		body->mass += massData.mass;
		localCenter = b2MulAdd(localCenter, massData.mass, massData.center);
		body->I += massData.I;

		b2ShapeExtent extent = b2ComputeShapeExtent(s);
		body->minExtent = B2_MIN(body->minExtent, extent.minExtent);
		body->maxExtent = B2_MAX(body->maxExtent, extent.maxExtent);
	}

	// Compute center of mass.
	if (body->mass > 0.0f)
	{
		body->invMass = 1.0f / body->mass;
		localCenter = b2MulSV(body->invMass, localCenter);
	}

	if (body->I > 0.0f && body->fixedRotation == false)
	{
		// Center the inertia about the center of mass.
		body->I -= body->mass * b2Dot(localCenter, localCenter);
		B2_ASSERT(body->I > 0.0f);
		body->invI = 1.0f / body->I;
	}
	else
	{
		body->I = 0.0f;
		body->invI = 0.0f;
	}

	// Move center of mass.
	b2Vec2 oldCenter = body->position;
	body->localCenter = localCenter;
	body->position = b2Add(b2RotateVector(body->rotation, body->localCenter), body->origin);

	// Update center of mass velocity
	b2BodyState* state = b2GetBodyState(world, body->bodyId);
	if (state != NULL)
	{
		b2Vec2 deltaLinear = b2CrossSV(state->angularVelocity, b2Sub(body->position, oldCenter));
		state->linearVelocity = b2Add(state->linearVelocity, deltaLinear);
	}
}

static b2ShapeId b2CreateShape(b2BodyId bodyId, const b2ShapeDef* def, const void* geometry, b2ShapeType shapeType)
{
	b2World* world = b2GetWorld(bodyId.world0);
	B2_ASSERT(world->locked == false);
	if (world->locked)
	{
		return b2_nullShapeId;
	}

	b2Body* body = b2GetBody(world, bodyId);

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

	shape->bodyId = body->bodyId;
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

	if (body->isEnabled)
	{
		b2CreateShapeProxy(shape, &world->broadPhase, body->type, b2MakeTransform(body));
	}

	// Add to shape linked list
	shape->nextShapeIndex = body->shapeList;
	body->shapeList = shape->object.index;
	body->shapeCount += 1;

	if (shape->density > 0.0f)
	{
		b2UpdateBodyMassData(world, body);
	}

	b2ValidateWorld(world);

	b2ShapeId id = {shape->object.index + 1, bodyId.world0, shape->object.revision};
	return id;
}

b2ShapeId b2CreateCircleShape(b2BodyId bodyId, const b2ShapeDef* def, const b2Circle* circle)
{
	return b2CreateShape(bodyId, def, circle, b2_circleShape);
}

b2ShapeId b2CreateCapsuleShape(b2BodyId bodyId, const b2ShapeDef* def, const b2Capsule* capsule)
{
	float lengthSqr = b2DistanceSquared(capsule->point1, capsule->point2);
	if (lengthSqr <= b2_linearSlop * b2_linearSlop)
	{
		B2_ASSERT(false);
		return (b2ShapeId){0};
	}

	return b2CreateShape(bodyId, def, capsule, b2_capsuleShape);
}

b2ShapeId b2CreatePolygonShape(b2BodyId bodyId, const b2ShapeDef* def, const b2Polygon* polygon)
{
	return b2CreateShape(bodyId, def, polygon, b2_polygonShape);
}

b2ShapeId b2CreateSegmentShape(b2BodyId bodyId, const b2ShapeDef* def, const b2Segment* segment)
{
	float lengthSqr = b2DistanceSquared(segment->point1, segment->point2);
	if (lengthSqr <= b2_linearSlop * b2_linearSlop)
	{
		B2_ASSERT(false);
		return b2_nullShapeId;
	}

	return b2CreateShape(bodyId, def, segment, b2_segmentShape);
}

// Destroy a shape on a body. This doesn't need to be called when destroying a body.
static void b2DestroyShapeInternal(b2World* world, b2Shape* shape)
{
	int shapeIndex = shape->object.index;
	B2_ASSERT(shapeIndex == shape->object.next);
	int bodyId = shape->bodyId;

	b2CheckIndex(world->bodyLookupArray, bodyId);
	b2BodyLookup lookup = world->bodyLookupArray[bodyId];
	b2CheckIndex(world->solverSetArray, lookup.setIndex);
	b2SolverSet* set = world->solverSetArray + lookup.setIndex;
	B2_ASSERT(0 <= lookup.bodyIndex && lookup.bodyIndex < set->bodies.count);
	b2Body* body = set->bodies.data + lookup.bodyIndex;

	// Remove the shape from the body's singly linked list.
	int* indexPtr = &body->shapeList;
	bool found = false;
	while (*indexPtr != B2_NULL_INDEX)
	{
		if (*indexPtr == shape->object.index)
		{
			*indexPtr = shape->nextShapeIndex;
			found = true;
			break;
		}

		indexPtr = &(world->shapes[*indexPtr].nextShapeIndex);
	}

	B2_ASSERT(found);
	if (found == false)
	{
		return;
	}

	body->shapeCount -= 1;

	const float density = shape->density;

	// Destroy any contacts associated with the shape
	int contactKey = body->contactList;
	while (contactKey != B2_NULL_INDEX)
	{
		int contactId = contactKey >> 1;
		int edgeIndex = contactKey & 1;

		b2Contact* contact = b2GetContactFromRawId(world, contactId);
		contactKey = contact->edges[edgeIndex].nextKey;

		if (contact->shapeIndexA == shapeIndex || contact->shapeIndexB == shapeIndex)
		{
			b2DestroyContact(world, contact);
		}
	}

	if (lookup.setIndex != b2_disabledSet)
	{
		b2DestroyShapeProxy(shape, &world->broadPhase);
	}

	b2FreeObject(&world->shapePool, &shape->object);

	// Reset the mass data
	if (density > 0.0f)
	{
		b2UpdateBodyMassData(world, body);
	}
}

// Destroy a shape on a body. This doesn't need to be called when destroying a body.
void b2DestroyShape(b2ShapeId shapeId)
{
	b2World* world = b2GetWorld(shapeId.world0);
	B2_ASSERT(world->locked == false);
	if (world->locked)
	{
		return;
	}

	b2Shape* shape = b2GetShape(world, shapeId);
	b2DestroyShapeInternal(world, shape);
}

b2ChainId b2CreateChain(b2BodyId bodyId, const b2ChainDef* def)
{
	B2_ASSERT(b2IsValid(def->friction) && def->friction >= 0.0f);
	B2_ASSERT(b2IsValid(def->restitution) && def->restitution >= 0.0f);
	B2_ASSERT(def->count >= 4);

	b2World* world = b2GetWorld(bodyId.world0);
	B2_ASSERT(world->locked == false);
	if (world->locked)
	{
		return b2_nullChainId;
	}

	b2Body* body = b2GetBody(world, bodyId);

	b2ChainShape* chainShape = (b2ChainShape*)b2AllocObject(&world->chainPool);
	world->chains = (b2ChainShape*)world->chainPool.memory;

	int chainIndex = chainShape->object.index;
	chainShape->bodyId = body->bodyId;
	chainShape->nextIndex = body->chainList;
	body->chainList = chainShape->object.index;

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

			b2ShapeId shapeId = b2CreateShape(bodyId, &shapeDef, &smoothSegment, b2_smoothSegmentShape);
			chainShape->shapeIndices[i] = shapeId.index1 - 1;
		}

		{
			smoothSegment.ghost1 = points[n - 3];
			smoothSegment.segment.point1 = points[n - 2];
			smoothSegment.segment.point2 = points[n - 1];
			smoothSegment.ghost2 = points[0];
			smoothSegment.chainIndex = chainIndex;
			b2ShapeId shapeId = b2CreateShape(bodyId, &shapeDef, &smoothSegment, b2_smoothSegmentShape);
			chainShape->shapeIndices[n - 2] = shapeId.index1 - 1;
		}

		{
			smoothSegment.ghost1 = points[n - 2];
			smoothSegment.segment.point1 = points[n - 1];
			smoothSegment.segment.point2 = points[0];
			smoothSegment.ghost2 = points[1];
			smoothSegment.chainIndex = chainIndex;
			b2ShapeId shapeId = b2CreateShape(bodyId, &shapeDef, &smoothSegment, b2_smoothSegmentShape);
			chainShape->shapeIndices[n - 1] = shapeId.index1 - 1;
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

			b2ShapeId shapeId = b2CreateShape(bodyId, &shapeDef, &smoothSegment, b2_smoothSegmentShape);
			chainShape->shapeIndices[i] = shapeId.index1 - 1;
		}
	}

	b2ChainId id = {chainShape->object.index + 1, bodyId.world0, chainShape->object.revision};
	return id;
}

void b2DestroyChain(b2ChainId chainId)
{
	b2World* world = b2GetWorld(chainId.world0);
	B2_ASSERT(world->locked == false);
	if (world->locked)
	{
		return;
	}

	B2_ASSERT(1 <= chainId.index1 && chainId.index1 <= world->chainPool.count);

	b2ChainShape* chain = world->chains + (chainId.index1 - 1);
	B2_ASSERT(chain->object.revision == chainId.revision);

	// Remove the chain from the body's singly linked list.
	b2Body* body = b2GetBodyFromRawId(world, chain->bodyId);
	int* indexPtr = &body->chainList;
	bool found = false;
	while (*indexPtr != B2_NULL_INDEX)
	{
		if (*indexPtr == chain->object.index)
		{
			*indexPtr = chain->nextIndex;
			found = true;
			break;
		}

		indexPtr = &(world->chains[*indexPtr].nextIndex);
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
		b2DestroyShapeInternal(world, shape);
	}

	b2Free(chain->shapeIndices, count * sizeof(int));
	b2FreeObject(&world->chainPool, &chain->object);
}

b2Vec2 b2Body_GetPosition(b2BodyId bodyId)
{
	b2World* world = b2GetWorld(bodyId.world0);
	b2Body* body = b2GetBody(world, bodyId);
	return body->origin;
}

b2Rot b2Body_GetRotation(b2BodyId bodyId)
{
	b2World* world = b2GetWorld(bodyId.world0);
	b2Body* body = b2GetBody(world, bodyId);
	return body->rotation;
}

float b2Body_GetAngle(b2BodyId bodyId)
{
	b2World* world = b2GetWorld(bodyId.world0);
	b2Body* body = b2GetBody(world, bodyId);
	return b2Rot_GetAngle(body->rotation);
}

b2Transform b2Body_GetTransform(b2BodyId bodyId)
{
	b2World* world = b2GetWorld(bodyId.world0);
	b2Body* body = b2GetBody(world, bodyId);
	return b2MakeTransform(body);
}

b2Vec2 b2Body_GetLocalPoint(b2BodyId bodyId, b2Vec2 worldPoint)
{
	b2World* world = b2GetWorld(bodyId.world0);
	b2Body* body = b2GetBody(world, bodyId);
	return b2InvTransformPoint(b2MakeTransform(body), worldPoint);
}

b2Vec2 b2Body_GetWorldPoint(b2BodyId bodyId, b2Vec2 localPoint)
{
	b2World* world = b2GetWorld(bodyId.world0);
	b2Body* body = b2GetBody(world, bodyId);
	return b2TransformPoint(b2MakeTransform(body), localPoint);
}

b2Vec2 b2Body_GetLocalVector(b2BodyId bodyId, b2Vec2 worldVector)
{
	b2World* world = b2GetWorld(bodyId.world0);
	b2Body* body = b2GetBody(world, bodyId);
	return b2InvRotateVector(body->rotation, worldVector);
}

b2Vec2 b2Body_GetWorldVector(b2BodyId bodyId, b2Vec2 localVector)
{
	B2_ASSERT(b2Body_IsValid(bodyId));
	b2World* world = b2GetWorld(bodyId.world0);
	b2Body* body = b2GetBody(world, bodyId);
	return b2RotateVector(body->rotation, localVector);
}

void b2Body_SetTransform(b2BodyId bodyId, b2Vec2 position, float angle)
{
	B2_ASSERT(b2Body_IsValid(bodyId));
	b2World* world = b2GetWorld(bodyId.world0);
	B2_ASSERT(world->locked == false);

	b2Body* body = b2GetBody(world, bodyId);

	body->origin = position;

	body->rotation = b2MakeRot(angle);
	body->position = b2Add(b2RotateVector(body->rotation, body->localCenter), body->origin);

	body->rotation0 = body->rotation;
	body->position0 = body->position;

	b2BroadPhase* broadPhase = &world->broadPhase;

	b2Transform transform = b2MakeTransform(body);
	float margin = body->type == b2_staticBody ? b2_speculativeDistance : b2_aabbMargin;

	int shapeIndex = body->shapeList;
	while (shapeIndex != B2_NULL_INDEX)
	{
		b2Shape* shape = world->shapes + shapeIndex;
		b2AABB aabb = b2ComputeShapeAABB(shape, transform);
		aabb.lowerBound.x -= b2_speculativeDistance;
		aabb.lowerBound.y -= b2_speculativeDistance;
		aabb.upperBound.x += b2_speculativeDistance;
		aabb.upperBound.y += b2_speculativeDistance;
		shape->aabb = aabb;

		if (b2AABB_Contains(shape->fatAABB, aabb) == false)
		{
			b2AABB fatAABB;
			fatAABB.lowerBound.x = aabb.lowerBound.x - margin;
			fatAABB.lowerBound.y = aabb.lowerBound.y - margin;
			fatAABB.upperBound.x = aabb.upperBound.x + margin;
			fatAABB.upperBound.y = aabb.upperBound.y + margin;
			shape->fatAABB = fatAABB;

			b2BroadPhase_MoveProxy(broadPhase, shape->proxyKey, fatAABB);
		}

		shapeIndex = shape->nextShapeIndex;
	}
}

b2Vec2 b2Body_GetLinearVelocity(b2BodyId bodyId)
{
	B2_ASSERT(b2Body_IsValid(bodyId));
	b2World* world = b2GetWorld(bodyId.world0);
	b2BodyState* state = b2GetBodyState(world, bodyId.index1 - 1);
	if (state != NULL)
	{
		return state->linearVelocity;
	}
	return b2Vec2_zero;
}

float b2Body_GetAngularVelocity(b2BodyId bodyId)
{
	B2_ASSERT(b2Body_IsValid(bodyId));
	b2World* world = b2GetWorld(bodyId.world0);
	b2BodyState* state = b2GetBodyState(world, bodyId.index1 - 1);
	if (state != NULL)
	{
		return state->angularVelocity;
	}
	return 0.0;
}

void b2Body_SetLinearVelocity(b2BodyId bodyId, b2Vec2 linearVelocity)
{
	B2_ASSERT(b2Body_IsValid(bodyId));
	b2World* world = b2GetWorld(bodyId.world0);
	b2BodyState* state = b2GetBodyState(world, bodyId.index1 - 1);
	if (state == NULL)
	{
		return;
	}

	state->linearVelocity = linearVelocity;
	if (b2LengthSquared(linearVelocity) > 0.0f)
	{
		b2WakeBody(world, bodyId.index1 - 1);
	}
}

void b2Body_SetAngularVelocity(b2BodyId bodyId, float angularVelocity)
{
	B2_ASSERT(b2Body_IsValid(bodyId));
	b2World* world = b2GetWorld(bodyId.world0);
	b2BodyState* state = b2GetBodyState(world, bodyId.index1 - 1);
	if (state == NULL)
	{
		return;
	}

	state->angularVelocity = angularVelocity;
	if (angularVelocity != 0.0f)
	{
		b2WakeBody(world, bodyId.index1 - 1);
	}
}

void b2Body_ApplyForce(b2BodyId bodyId, b2Vec2 force, b2Vec2 point, bool wake)
{
	B2_ASSERT(b2Body_IsValid(bodyId));
	b2World* world = b2GetWorld(bodyId.world0);
	b2Body* body = b2GetBodyFromRawId(world, bodyId.index1 - 1);
	if (body->type == b2_staticBody || body->isEnabled == false)
	{
		return;
	}

	if (wake)
	{
		bool wasWaked = b2WakeBody(world, bodyId.index1 - 1);
		if (wasWaked)
		{
			// bodies move when woken
			body = b2GetBody(world, bodyId);
		}
	}

	if (b2IsBodyAwake(world, bodyId.index1 - 1))
	{
		body->force = b2Add(body->force, force);
		body->torque += b2Cross(b2Sub(point, body->position), force);
	}
}

void b2Body_ApplyForceToCenter(b2BodyId bodyId, b2Vec2 force, bool wake)
{
	B2_ASSERT(b2Body_IsValid(bodyId));
	b2World* world = b2GetWorld(bodyId.world0);
	b2Body* body = b2GetBody(world, bodyId);
	if (body->type == b2_staticBody || body->isEnabled == false)
	{
		return;
	}

	if (wake)
	{
		bool wasWaked = b2WakeBody(world, body->bodyId);
		if (wasWaked)
		{
			// bodies move when woken
			body = b2GetBody(world, bodyId);
		}
	}

	if (b2IsBodyAwake(world, body->bodyId))
	{
		body->force = b2Add(body->force, force);
	}
}

void b2Body_ApplyTorque(b2BodyId bodyId, float torque, bool wake)
{
	B2_ASSERT(b2Body_IsValid(bodyId));
	b2World* world = b2GetWorld(bodyId.world0);
	b2Body* body = b2GetBody(world, bodyId);
	if (body->type == b2_staticBody || body->isEnabled == false)
	{
		return;
	}

	if (wake)
	{
		bool wasWaked = b2WakeBody(world, body->bodyId);
		if (wasWaked)
		{
			// bodies move when woken
			body = b2GetBody(world, bodyId);
		}
	}

	if (b2IsBodyAwake(world, body->bodyId))
	{
		body->torque += torque;
	}
}

void b2Body_ApplyLinearImpulse(b2BodyId bodyId, b2Vec2 impulse, b2Vec2 point, bool wake)
{
	B2_ASSERT(b2Body_IsValid(bodyId));
	b2World* world = b2GetWorld(bodyId.world0);
	int id = bodyId.index1 - 1;
	b2CheckIndex(world->bodyLookupArray, id);
	b2BodyLookup lookup = world->bodyLookupArray[id];
	B2_ASSERT(lookup.revision == bodyId.revision);

	if (lookup.setIndex == b2_staticSet || lookup.setIndex == b2_disabledSet)
	{
		return;
	}

	if (wake)
	{
		b2WakeBody(world, id);
	}

	b2BodyState* state = b2GetBodyState(world, id);
	if (state != NULL)
	{
		b2Body* body = b2GetBodyFromRawId(world, id);
		state->linearVelocity = b2MulAdd(state->linearVelocity, body->invMass, impulse);
		state->angularVelocity += body->invI * b2Cross(b2Sub(point, body->position), impulse);
	}
}

void b2Body_ApplyLinearImpulseToCenter(b2BodyId bodyId, b2Vec2 impulse, bool wake)
{
	B2_ASSERT(b2Body_IsValid(bodyId));
	b2World* world = b2GetWorld(bodyId.world0);
	int id = bodyId.index1 - 1;
	b2CheckIndex(world->bodyLookupArray, id);
	b2BodyLookup lookup = world->bodyLookupArray[id];
	B2_ASSERT(lookup.revision == bodyId.revision);

	if (lookup.setIndex == b2_staticSet || lookup.setIndex == b2_disabledSet)
	{
		return;
	}

	if (wake)
	{
		b2WakeBody(world, id);
	}

	b2BodyState* state = b2GetBodyState(world, id);
	if (state != NULL)
	{
		b2Body* body = b2GetBodyFromRawId(world, id);
		state->linearVelocity = b2MulAdd(state->linearVelocity, body->invMass, impulse);
	}
}

void b2Body_ApplyAngularImpulse(b2BodyId bodyId, float impulse, bool wake)
{
	B2_ASSERT(b2Body_IsValid(bodyId));
	b2World* world = b2GetWorld(bodyId.world0);
	int id = bodyId.index1 - 1;
	b2CheckIndex(world->bodyLookupArray, id);
	b2BodyLookup lookup = world->bodyLookupArray[id];
	B2_ASSERT(lookup.revision == bodyId.revision);

	if (lookup.setIndex == b2_staticSet || lookup.setIndex == b2_disabledSet)
	{
		return;
	}

	if (wake)
	{
		b2WakeBody(world, id);
	}

	b2BodyState* state = b2GetBodyState(world, id);
	if (state != NULL)
	{
		b2Body* body = b2GetBodyFromRawId(world, id);
		state->angularVelocity += body->invI * impulse;
	}
}

b2BodyType b2Body_GetType(b2BodyId bodyId)
{
	B2_ASSERT(b2Body_IsValid(bodyId));
	b2World* world = b2GetWorld(bodyId.world0);
	b2Body* body = b2GetBody(world, bodyId);
	return body->type;
}

// Changing the body type is quite complex. So we basically recreate the body.
// #todo check joints
void b2Body_SetType(b2BodyId bodyId, b2BodyType type)
{
	B2_ASSERT(b2Body_IsValid(bodyId));
	b2World* world = b2GetWorld(bodyId.world0);
	b2Body* body = b2GetBody(world, bodyId);
	if (body->type == type)
	{
		return;
	}

	// this is slow
	if (body->isEnabled == true)
	{
		b2DisableBody(world, body);

		body->type = type;

		b2EnableBody(world, body);
	}
	else
	{
		body->type = type;
	}

	// Body type affects the mass
	b2UpdateBodyMassData(world, body);
}

void b2Body_SetUserData(b2BodyId bodyId, void* userData)
{
	B2_ASSERT(b2Body_IsValid(bodyId));
	b2World* world = b2GetWorld(bodyId.world0);
	b2Body* body = b2GetBody(world, bodyId);
	body->userData = userData;
}

void* b2Body_GetUserData(b2BodyId bodyId)
{
	B2_ASSERT(b2Body_IsValid(bodyId));
	b2World* world = b2GetWorld(bodyId.world0);
	b2Body* body = b2GetBody(world, bodyId);
	return body->userData;
}

float b2Body_GetMass(b2BodyId bodyId)
{
	B2_ASSERT(b2Body_IsValid(bodyId));
	b2World* world = b2GetWorld(bodyId.world0);
	b2Body* body = b2GetBody(world, bodyId);
	return body->mass;
}

float b2Body_GetInertiaTensor(b2BodyId bodyId)
{
	B2_ASSERT(b2Body_IsValid(bodyId));
	b2World* world = b2GetWorld(bodyId.world0);
	b2Body* body = b2GetBody(world, bodyId);
	return body->I;
}

b2Vec2 b2Body_GetLocalCenterOfMass(b2BodyId bodyId)
{
	B2_ASSERT(b2Body_IsValid(bodyId));
	b2World* world = b2GetWorld(bodyId.world0);
	b2Body* body = b2GetBody(world, bodyId);
	return body->localCenter;
}

b2Vec2 b2Body_GetWorldCenterOfMass(b2BodyId bodyId)
{
	B2_ASSERT(b2Body_IsValid(bodyId));
	b2World* world = b2GetWorld(bodyId.world0);
	b2Body* body = b2GetBody(world, bodyId);
	return body->position;
}

void b2Body_SetMassData(b2BodyId bodyId, b2MassData massData)
{
	B2_ASSERT(b2Body_IsValid(bodyId));
	B2_ASSERT(b2IsValid(massData.mass) && massData.mass >= 0.0f);
	B2_ASSERT(b2IsValid(massData.I) && massData.I >= 0.0f);
	B2_ASSERT(b2Vec2_IsValid(massData.center));

	b2World* world = b2GetWorldLocked(bodyId.world0);
	if (world == NULL)
	{
		return;
	}

	b2Body* body = b2GetBody(world, bodyId);
	body->mass = massData.mass;
	body->I = massData.I;
	body->localCenter = massData.center;

	b2Vec2 p = b2Add(b2RotateVector(body->rotation, massData.center), body->origin);
	body->position = p;
	body->position0 = p;

	body->invMass = body->mass > 0.0f ? 1.0f / body->mass : 0.0f;
	body->invI = body->I > 0.0f ? 1.0f / body->I : 0.0f;
}

b2MassData b2Body_GetMassData(b2BodyId bodyId)
{
	B2_ASSERT(b2Body_IsValid(bodyId));
	b2World* world = b2GetWorld(bodyId.world0);
	b2Body* body = b2GetBody(world, bodyId);
	b2MassData massData = {body->mass, body->localCenter, body->I};
	return massData;
}

void b2Body_ResetMassData(b2BodyId bodyId)
{
	B2_ASSERT(b2Body_IsValid(bodyId));
	b2World* world = b2GetWorldLocked(bodyId.world0);
	if (world == NULL)
	{
		return;
	}

	b2Body* body = b2GetBody(world, bodyId);

	b2UpdateBodyMassData(world, body);
}

void b2Body_SetLinearDamping(b2BodyId bodyId, float linearDamping)
{
	B2_ASSERT(b2Body_IsValid(bodyId));
	B2_ASSERT(b2IsValid(linearDamping) && linearDamping >= 0.0f);

	b2World* world = b2GetWorldLocked(bodyId.world0);
	if (world == NULL)
	{
		return;
	}

	b2Body* body = b2GetBody(world, bodyId);
	body->linearDamping = linearDamping;
}

float b2Body_GetLinearDamping(b2BodyId bodyId)
{
	B2_ASSERT(b2Body_IsValid(bodyId));
	b2World* world = b2GetWorld(bodyId.world0);
	b2Body* body = b2GetBody(world, bodyId);
	return body->linearDamping;
}

void b2Body_SetAngularDamping(b2BodyId bodyId, float angularDamping)
{
	B2_ASSERT(b2Body_IsValid(bodyId));
	B2_ASSERT(b2IsValid(angularDamping) && angularDamping >= 0.0f);

	b2World* world = b2GetWorldLocked(bodyId.world0);
	if (world == NULL)
	{
		return;
	}

	b2Body* body = b2GetBody(world, bodyId);
	body->angularDamping = angularDamping;
}

float b2Body_GetAngularDamping(b2BodyId bodyId)
{
	B2_ASSERT(b2Body_IsValid(bodyId));
	b2World* world = b2GetWorld(bodyId.world0);
	b2Body* body = b2GetBody(world, bodyId);
	return body->angularDamping;
}

void b2Body_SetGravityScale(b2BodyId bodyId, float gravityScale)
{
	B2_ASSERT(b2Body_IsValid(bodyId));
	B2_ASSERT(b2IsValid(gravityScale));

	b2World* world = b2GetWorldLocked(bodyId.world0);
	if (world == NULL)
	{
		return;
	}

	b2Body* body = b2GetBody(world, bodyId);
	body->gravityScale = gravityScale;
}

float b2Body_GetGravityScale(b2BodyId bodyId)
{
	B2_ASSERT(b2Body_IsValid(bodyId));
	b2World* world = b2GetWorld(bodyId.world0);
	b2Body* body = b2GetBody(world, bodyId);
	return body->gravityScale;
}

bool b2Body_IsAwake(b2BodyId bodyId)
{
	b2World* world = b2GetWorld(bodyId.world0);
	B2_ASSERT(b2Body_IsValid(bodyId));
	return b2IsBodyAwake(world, bodyId.index1 - 1);
}

void b2Body_Wake(b2BodyId bodyId)
{
	B2_ASSERT(b2Body_IsValid(bodyId));
	b2World* world = b2GetWorldLocked(bodyId.world0);
	if (world == NULL)
	{
		return;
	}

	b2WakeBody(world, bodyId.index1 - 1);
}

bool b2Body_IsEnabled(b2BodyId bodyId)
{
	b2World* world = b2GetWorld(bodyId.world0);
	b2Body* body = b2GetBody(world, bodyId);
	return body->isEnabled;
}

bool b2Body_IsSleepEnabled(b2BodyId bodyId)
{
	b2World* world = b2GetWorld(bodyId.world0);
	b2Body* body = b2GetBody(world, bodyId);
	return body->enableSleep;
}

void b2Body_EnableSleep(b2BodyId bodyId, bool enableSleep)
{
	b2World* world = b2GetWorldLocked(bodyId.world0);
	if (world == NULL)
	{
		return;
	}

	b2Body* body = b2GetBody(world, bodyId);
	body->enableSleep = enableSleep;

	if (enableSleep == false)
	{
		b2WakeBody(world, bodyId.index1 - 1);
	}
}

void b2Body_Disable(b2BodyId bodyId)
{
	b2World* world = b2GetWorldLocked(bodyId.world0);
	if (world == NULL)
	{
		return;
	}

	b2Body* body = b2GetBody(world, bodyId);
	if (body->isEnabled == true)
	{
		b2DisableBody(world, body);
		body->isEnabled = false;
	}
}

void b2Body_Enable(b2BodyId bodyId)
{
	b2World* world = b2GetWorldLocked(bodyId.world0);
	if (world == NULL)
	{
		return;
	}

	b2Body* body = b2GetBody(world, bodyId);
	if (body->isEnabled == false)
	{
		b2EnableBody(world, body);
		body->isEnabled = true;
	}
}

void b2Body_SetFixedRotation(b2BodyId bodyId, bool flag)
{
	b2World* world = b2GetWorldLocked(bodyId.world0);
	if (world == NULL)
	{
		return;
	}

	b2Body* body = b2GetBody(world, bodyId);
	if (body->fixedRotation != flag)
	{
		body->fixedRotation = flag;

		b2BodyState* state = b2GetBodyState(world, bodyId.index1 - 1);
		if (state != NULL)
		{
			state->angularVelocity = 0.0f;
		}
		b2UpdateBodyMassData(world, body);
	}
}

bool b2Body_IsFixedRotation(b2BodyId bodyId)
{
	b2World* world = b2GetWorld(bodyId.world0);
	b2Body* body = b2GetBody(world, bodyId);
	return body->fixedRotation;
}

void b2Body_SetBullet(b2BodyId bodyId, bool flag)
{
	b2World* world = b2GetWorldLocked(bodyId.world0);
	if (world == NULL)
	{
		return;
	}

	b2Body* body = b2GetBody(world, bodyId);
	body->isBullet = flag;
}

bool b2Body_IsBullet(b2BodyId bodyId)
{
	b2World* world = b2GetWorld(bodyId.world0);
	b2Body* body = b2GetBody(world, bodyId);
	return body->isBullet;
}

int b2Body_GetShapeCount(b2BodyId bodyId)
{
	b2World* world = b2GetWorld(bodyId.world0);
	b2Body* body = b2GetBody(world, bodyId);
	return body->shapeCount;
}

void b2Body_GetShapes(b2BodyId bodyId, b2ShapeId* shapeArray, int capacity)
{
	b2World* world = b2GetWorld(bodyId.world0);
	b2Body* body = b2GetBody(world, bodyId);
	int shapeIndex = body->shapeList;
	int shapeCount = 0;
	while (shapeIndex != B2_NULL_INDEX && shapeCount < capacity)
	{
		b2Shape* shape = world->shapes + shapeIndex;
		B2_ASSERT(b2IsValidObject(&shape->object));

		b2ShapeId id = {shape->object.index + 1, bodyId.world0, shape->object.revision};
		shapeArray[shapeCount] = id;
		shapeCount += 1;
	
		shapeIndex = shape->nextShapeIndex;
	}
}

int b2Body_GetJointCount(b2BodyId bodyId)
{
	b2World* world = b2GetWorld(bodyId.world0);
	b2Body* body = b2GetBody(world, bodyId);
	return body->jointCount;
}

void b2Body_GetJoints(b2BodyId bodyId, b2JointId* jointArray, int capacity)
{
	b2World* world = b2GetWorld(bodyId.world0);
	b2Body* body = b2GetBody(world, bodyId);
	int jointKey = body->jointList;

	int jointCount = 0;
	while (jointKey != B2_NULL_INDEX && jointCount < capacity)
	{
		int jointId = jointKey >> 1;
		int edgeIndex = jointKey & 1;

		b2Joint* joint = b2GetJoint(world, jointId);

		b2JointId id = {jointId + 1, bodyId.world0, joint->revision};
		jointArray[jointCount] = id;
		jointCount += 1;
	
		jointKey = joint->edges[edgeIndex].nextKey;
	}
}

bool b2ShouldBodiesCollide(b2World* world, b2Body* bodyA, b2Body* bodyB)
{
	int jointKey;
	int otherBodyId;
	if (bodyA->jointCount < bodyB->jointCount)
	{
		jointKey = bodyA->jointList;
		otherBodyId = bodyB->bodyId;
	}
	else
	{
		jointKey = bodyB->jointList;
		otherBodyId = bodyA->bodyId;
	}

	while (jointKey != B2_NULL_INDEX)
	{
		int jointId = jointKey >> 1;
		int edgeIndex = jointKey & 1;
		int otherEdgeIndex = edgeIndex ^ 1;

		b2Joint* joint = b2GetJoint(world, jointId);
		if (joint->collideConnected == false && joint->edges[otherEdgeIndex].bodyId == otherBodyId)
		{
			return false;
		}

		jointKey = joint->edges[edgeIndex].nextKey;
	}

	return true;
}

#if 0
void b2Body_Dump(b2Body* b)
{
	int bodyIndex = body->islandIndex;

	// %.9g is sufficient to save and load the same value using text
	// FLT_DECIMAL_DIG == 9

	b2Dump("{\n");
	b2Dump("  b2BodyDef bd;\n");
	b2Dump("  bd.type = b2BodyType(%d);\n", body->type);
	b2Dump("  bd.position.Set(%.9g, %.9g);\n", m_xf.p.x, m_xf.p.y);
	b2Dump("  bd.angle = %.9g;\n", m_sweep.a);
	b2Dump("  bd.linearVelocity.Set(%.9g, %.9g);\n", m_linearVelocity.x, m_linearVelocity.y);
	b2Dump("  bd.angularVelocity = %.9g;\n", m_angularVelocity);
	b2Dump("  bd.linearDamping = %.9g;\n", m_linearDamping);
	b2Dump("  bd.angularDamping = %.9g;\n", m_angularDamping);
	b2Dump("  bd.allowSleep = bool(%d);\n", m_flags & e_autoSleepFlag);
	b2Dump("  bd.awake = bool(%d);\n", m_flags & e_awakeFlag);
	b2Dump("  bd.fixedRotation = bool(%d);\n", m_flags & e_fixedRotationFlag);
	b2Dump("  bd.bullet = bool(%d);\n", m_flags & e_bulletFlag);
	b2Dump("  bd.enabled = bool(%d);\n", m_flags & e_enabledFlag);
	b2Dump("  bd.gravityScale = %.9g;\n", m_gravityScale);
	b2Dump("  bodies[%d] = m_world->CreateBody(&bd);\n", m_islandIndex);
	b2Dump("\n");
	for (b2Shape* shape = m_shapeList; shape; shape = shape->m_next)
	{
		b2Dump("  {\n");
		shape->Dump(bodyIndex);
		b2Dump("  }\n");
	}
	b2Dump("}\n");
}
#endif
