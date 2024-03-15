// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "body.h"

#include "aabb.h"
#include "allocate.h"
#include "array.h"
#include "contact.h"
#include "core.h"
#include "id_pool.h"
#include "island.h"
#include "joint.h"
#include "shape.h"
#include "solver_set.h"
#include "util.h"
#include "world.h"

// needed for dll export
#include "box2d/box2d.h"
#include "box2d/event_types.h"
#include "box2d/id.h"

b2Body* b2GetBodyFromRawId(b2World* world, int bodyKey)
{
	b2CheckIndex(world->bodyLookupArray, bodyKey);
	b2BodyLookup lookup = world->bodyLookupArray[bodyKey];
	b2CheckIndex(world->solverSetArray, lookup.setIndex);
	b2SolverSet* set = world->solverSetArray + lookup.setIndex;
	B2_ASSERT(0 <= lookup.bodyIndex && lookup.bodyIndex <= set->bodies.count);
	b2Body* body = set->bodies.data + lookup.bodyIndex;
	return body;
}

// Get a validated body from a world using an id.
// todo remove this function and instead use B2_ASSERT(b2Body_IsValid(bodyId))
b2Body* b2GetBody(b2World* world, b2BodyId bodyId)
{
	B2_ASSERT(b2Body_IsValid(bodyId));

	// id index starts at one so that zero can represent null
	return b2GetBodyFromRawId(world, bodyId.index1 - 1);
}

// Create a b2BodyId from a key.
b2BodyId b2MakeBodyId(b2World* world, int bodyKey)
{
	B2_ASSERT(0 <= bodyKey && bodyKey < b2Array(world->bodyLookupArray).count);
	b2BodyLookup lookup = world->bodyLookupArray[bodyKey];
	B2_ASSERT(0 <= lookup.setIndex && lookup.setIndex < b2Array(world->solverSetArray).count);
	return (b2BodyId){bodyKey + 1, world->worldId, lookup.revision};
}

b2BodyState* b2GetBodyState(b2World* world, int bodyId)
{
	b2CheckIndex(world->bodyLookupArray, bodyId);
	b2BodyLookup lookup = world->bodyLookupArray[bodyId];
	if (lookup.setIndex == b2_awakeSet)
	{
		b2SolverSet* set = world->solverSetArray + b2_awakeSet;
		B2_ASSERT(0 <= lookup.bodyIndex && lookup.bodyIndex < set->states.count);
		return set->states.data + lookup.bodyIndex;
	}

	return NULL;
}

static void b2CreateIslandForBody(b2World* world, int setIndex, b2Body* body)
{
	B2_ASSERT(body->islandId == B2_NULL_INDEX);
	B2_ASSERT(body->islandPrev == B2_NULL_INDEX);
	B2_ASSERT(body->islandNext == B2_NULL_INDEX);

	B2_ASSERT(body->type != b2_staticBody && body->isEnabled == true);
	B2_ASSERT(setIndex != b2_staticSet && setIndex != b2_disabledSet);

	b2Island* island = b2CreateIsland(world, setIndex);

	body->islandId = island->islandId;
	island->headBody = body->bodyId;
	island->tailBody = body->bodyId;
	island->bodyCount = 1;
}

static void b2RemoveBodyFromIsland(b2World* world, b2Body* body)
{
	if (body->islandId == B2_NULL_INDEX)
	{
		B2_ASSERT(body->islandPrev == B2_NULL_INDEX);
		B2_ASSERT(body->islandNext == B2_NULL_INDEX);
		return;
	}

	b2Island* island = b2GetIsland(world, body->islandId);

	// Fix the island's linked list of bodies
	if (body->islandPrev != B2_NULL_INDEX)
	{
		b2Body* prevBody = b2GetBodyFromRawId(world, body->islandPrev);
		prevBody->islandNext = body->islandNext;
	}

	if (body->islandNext != B2_NULL_INDEX)
	{
		b2Body* nextBody = b2GetBodyFromRawId(world, body->islandNext);
		nextBody->islandPrev = body->islandPrev;
	}

	B2_ASSERT(island->bodyCount > 0);
	island->bodyCount -= 1;
	bool islandDestroyed = false;

	if (island->headBody == body->bodyId)
	{
		island->headBody = body->islandNext;

		if (island->headBody == B2_NULL_INDEX)
		{
			// Destroy empty island
			B2_ASSERT(island->tailBody == body->bodyId);
			B2_ASSERT(island->bodyCount == 0);
			B2_ASSERT(island->contactCount == 0);
			B2_ASSERT(island->jointCount == 0);

			// Free the island
			b2DestroyIsland(world, island->islandId);
			islandDestroyed = true;
		}
	}
	else if (island->tailBody == body->bodyId)
	{
		island->tailBody = body->islandPrev;
	}

	if (islandDestroyed == false)
	{
		b2WakeIsland(world, island);
		b2ValidateIsland(world, island, true);
	}

	body->islandId = B2_NULL_INDEX;
	body->islandPrev = B2_NULL_INDEX;
	body->islandNext = B2_NULL_INDEX;
}

static void b2DestroyBodyContacts(b2World* world, b2Body* body)
{
	// Destroy the attached contacts
	int32_t edgeKey = body->contactList;
	while (edgeKey != B2_NULL_INDEX)
	{
		int32_t contactKey = edgeKey >> 1;
		int32_t edgeIndex = edgeKey & 1;

		b2Contact* contact = b2GetContactFromRawId(world, contactKey);
		edgeKey = contact->edges[edgeIndex].nextKey;
		b2DestroyContact(world, contact);
	}
}

static void b2EnableBody(b2World* world, b2Body* body)
{
// todo
#if 0
	// Add shapes to broad-phase
	int32_t shapeIndex = body->shapeList;
	while (shapeIndex != B2_NULL_INDEX)
	{
		b2Shape* shape = world->shapes + shapeIndex;
		shapeIndex = shape->nextShapeIndex;

		b2CreateShapeProxy(shape, &world->broadPhase, body->type, b2MakeTransform(body));
	}

	b2CreateIslandForBody(world, body, true);

	int32_t jointKey = body->jointList;
	while (jointKey != B2_NULL_INDEX)
	{
		int32_t jointIndex = jointKey >> 1;
		int32_t edgeIndex = jointKey & 1;
		b2Joint* joint = world->joints + jointIndex;
		B2_ASSERT(joint->islandIndex == B2_NULL_INDEX);
		b2Body* bodyA = b2GetBodyFromRawId(world, joint->edges[0].bodyKey);
		b2Body* bodyB = b2GetBodyFromRawId(world, joint->edges[1].bodyKey);
		if (bodyA->type == b2_dynamicBody || bodyB->type == b2_dynamicBody)
		{
			b2AddJointToGraph(world, joint);
			b2LinkJoint(world, joint);
		}
		jointKey = joint->edges[edgeIndex].nextKey;
	}
#endif
}

static void b2DisableBody(b2World* world, b2Body* body)
{
// todo
#if 0
	b2DestroyBodyContacts(world, body);
	b2RemoveBodyFromIsland(world, body);

	// Remove shapes from broad-phase
	int32_t shapeIndex = body->shapeList;
	while (shapeIndex != B2_NULL_INDEX)
	{
		b2Shape* shape = world->shapes + shapeIndex;
		shapeIndex = shape->nextShapeIndex;

		b2DestroyShapeProxy(shape, &world->broadPhase);
	}

	int32_t jointKey = body->jointList;
	while (jointKey != B2_NULL_INDEX)
	{
		int32_t jointIndex = jointKey >> 1;
		int32_t edgeIndex = jointKey & 1;
		b2Joint* joint = world->joints + jointIndex;
		if (joint->colorIndex != B2_NULL_INDEX)
		{
			b2RemoveJointFromGraph(world, joint);
		}

		if (joint->islandIndex != B2_NULL_INDEX)
		{
			b2UnlinkJoint(world, joint);
		}
		jointKey = joint->edges[edgeIndex].nextKey;
	}
#endif
}

b2BodyId b2CreateBody(b2WorldId worldId, const b2BodyDef* def)
{
	b2World* world = b2GetWorldFromId(worldId);
	B2_ASSERT(world->locked == false);

	if (world->locked)
	{
		return b2_nullBodyId;
	}

	bool isAwake = (def->isAwake || def->enableSleep == false) && def->isEnabled && def->type != b2_staticBody;

	// Determine the body set
	int setIndex;
	if (def->isEnabled == false)
	{
		setIndex = b2_disabledSet;
	}
	else if (def->type == b2_staticBody)
	{
		setIndex = b2_staticSet;
	}
	else if (isAwake == true )
	{
		setIndex = b2_awakeSet;
	}
	else
	{
		// new set for a sleeping body in its own island
		setIndex = b2AllocId(&world->solverSetIdPool);
		if (setIndex == b2Array(world->solverSetArray).count)
		{
			b2Array_Push(world->solverSetArray, (b2SolverSet){0});
		}
	}

	B2_ASSERT(0 <= setIndex && setIndex < b2Array(world->solverSetArray).count);
	
	b2SolverSet* set = world->solverSetArray + setIndex;
	b2Body* body = b2AddBody(&world->blockAllocator, &set->bodies);
	b2BodyState* state = NULL;
	if (setIndex == b2_awakeSet)
	{
		state = b2AddBodyState(&world->blockAllocator, &set->states);
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
	}

	// All disabled bodies are packed together in a single array where islands
	// have no meaning. Static bodies are never in islands. Sleeping solver sets
	// should have an islands so they can be quickly made awake.
	if (setIndex != b2_disabledSet && setIndex != b2_staticSet)
	{
		b2CreateIslandForBody(world, setIndex, body);
	}

	uint16_t revision = 0;
	if (bodyId == b2Array(world->bodyLookupArray).count)
	{
		b2BodyLookup lookup = {setIndex, set->bodies.count - 1, revision};
		b2Array_Push(world->bodyLookupArray, (b2BodyLookup){0});
	}
	else
	{
		b2CheckIndex(world->bodyLookupArray, bodyId);
		b2BodyLookup* lookup = world->bodyLookupArray + bodyId;
		B2_ASSERT(lookup->setIndex == B2_NULL_INDEX && lookup->bodyIndex == B2_NULL_INDEX);
		lookup->setIndex = setIndex;
		lookup->bodyIndex = set->bodies.count - 1;
		lookup->revision += 1;
		revision = lookup->revision;
	}

	b2BodyId id = {bodyId + 1, world->worldId, revision};
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
	int32_t shapeIndex = body->shapeList;
	while (shapeIndex != B2_NULL_INDEX)
	{
		b2Shape* shape = world->shapes + shapeIndex;
		shapeIndex = shape->nextShapeIndex;

		b2DestroyShapeProxy(shape, &world->broadPhase);
		b2FreeObject(&world->shapePool, &shape->object);
	}

	// Delete the attached chains. The associated shapes have already been deleted above.
	int32_t chainIndex = body->chainList;
	while (chainIndex != B2_NULL_INDEX)
	{
		b2ChainShape* chain = world->chains + chainIndex;
		chainIndex = chain->nextIndex;

		b2Free(chain->shapeIndices, chain->count * sizeof(int32_t));
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
	int movedIndex = b2RemoveBody(&world->blockAllocator, &set->bodies, lookup->bodyIndex);
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
		int result = b2RemoveBodyState(&world->blockAllocator, &set->states, lookup->bodyIndex);
		B2_MAYBE_UNUSED(result);
		B2_ASSERT(result == movedIndex);
	}

	// Free lookup and id (preserve lookup revision)
	lookup->setIndex = B2_NULL_INDEX;
	lookup->bodyIndex = B2_NULL_INDEX;
	b2FreeId(&world->bodyIdPool, bodyId);
}

void b2DestroyBody(b2BodyId bodyId)
{
	b2World* world = b2GetWorldFromIndexLocked(bodyId.world0);
	if (world == NULL)
	{
		return;
	}

	b2Body* body = b2GetBody(world, bodyId);
	b2DestroyBodyInternal(world, body);
}

void b2DestroyBodyAndJoints(b2BodyId bodyId)
{
	b2World* world = b2GetWorldFromIndexLocked(bodyId.world0);
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
	int32_t edgeKey = body->jointList;
	while (edgeKey != B2_NULL_INDEX)
	{
		int32_t jointKey = edgeKey >> 1;
		int32_t edgeIndex = edgeKey & 1;

		b2Joint* joint = b2GetJointFromKey(world, jointKey);
		edgeKey = joint->edges[edgeIndex].nextKey;

		// Careful because this modifies the list being traversed
		b2DestroyJointInternal(world, joint, true);
	}

	b2DestroyBodyInternal(world, body);
}

int32_t b2Body_GetContactCapacity(b2BodyId bodyId)
{
	b2World* world = b2GetWorldFromIndexLocked(bodyId.world0);
	if (world == NULL)
	{
		return 0;
	}

	b2Body* body = b2GetBody(world, bodyId);

	// Conservative and fast
	return body->contactCount;
}

int32_t b2Body_GetContactData(b2BodyId bodyId, b2ContactData* contactData, int32_t capacity)
{
	b2World* world = b2GetWorldFromIndexLocked(bodyId.world0);
	if (world == NULL)
	{
		return 0;
	}

	b2Body* body = b2GetBody(world, bodyId);

	int contactKey = body->contactList;
	int index = 0;
	while (contactKey != B2_NULL_INDEX && index < capacity)
	{
		int32_t edgeIndex = contactKey & 1;

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
	b2World* world = b2GetWorldFromIndexLocked(bodyId.world0);
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
	int32_t shapeIndex = body->shapeList;
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
	b2World* world = b2GetWorldFromIndex(bodyId.world0);
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
	int32_t shapeIndex = shape->object.index;
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
	b2World* world = b2GetWorldFromIndex(shapeId.world0);
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

	b2World* world = b2GetWorldFromIndex(bodyId.world0);
	B2_ASSERT(world->locked == false);
	if (world->locked)
	{
		return b2_nullChainId;
	}

	b2Body* body = b2GetBody(world, bodyId);

	b2ChainShape* chainShape = (b2ChainShape*)b2AllocObject(&world->chainPool);
	world->chains = (b2ChainShape*)world->chainPool.memory;

	int32_t chainIndex = chainShape->object.index;
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

	int32_t n = def->count;
	const b2Vec2* points = def->points;

	if (def->isLoop)
	{
		chainShape->count = n;
		chainShape->shapeIndices = b2Alloc(n * sizeof(int32_t));

		b2SmoothSegment smoothSegment;

		int32_t prevIndex = n - 1;
		for (int32_t i = 0; i < n - 2; ++i)
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
		chainShape->shapeIndices = b2Alloc(n * sizeof(int32_t));

		b2SmoothSegment smoothSegment;

		for (int32_t i = 0; i < n - 3; ++i)
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
	b2World* world = b2GetWorldFromIndex(chainId.world0);
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

	int32_t count = chain->count;
	for (int32_t i = 0; i < count; ++i)
	{
		int32_t shapeIndex = chain->shapeIndices[i];
		B2_ASSERT(0 <= shapeIndex && shapeIndex < world->shapePool.count);
		b2Shape* shape = world->shapes + shapeIndex;
		b2DestroyShapeInternal(world, shape);
	}

	b2Free(chain->shapeIndices, count * sizeof(int32_t));
	b2FreeObject(&world->chainPool, &chain->object);
}

b2Vec2 b2Body_GetPosition(b2BodyId bodyId)
{
	b2World* world = b2GetWorldFromIndex(bodyId.world0);
	b2Body* body = b2GetBody(world, bodyId);
	return body->origin;
}

b2Rot b2Body_GetRotation(b2BodyId bodyId)
{
	b2World* world = b2GetWorldFromIndex(bodyId.world0);
	b2Body* body = b2GetBody(world, bodyId);
	return body->rotation;
}

float b2Body_GetAngle(b2BodyId bodyId)
{
	b2World* world = b2GetWorldFromIndex(bodyId.world0);
	b2Body* body = b2GetBody(world, bodyId);
	return b2Rot_GetAngle(body->rotation);
}

b2Transform b2Body_GetTransform(b2BodyId bodyId)
{
	b2World* world = b2GetWorldFromIndex(bodyId.world0);
	b2Body* body = b2GetBody(world, bodyId);
	return b2MakeTransform(body);
}

b2Vec2 b2Body_GetLocalPoint(b2BodyId bodyId, b2Vec2 worldPoint)
{
	b2World* world = b2GetWorldFromIndex(bodyId.world0);
	b2Body* body = b2GetBody(world, bodyId);
	return b2InvTransformPoint(b2MakeTransform(body), worldPoint);
}

b2Vec2 b2Body_GetWorldPoint(b2BodyId bodyId, b2Vec2 localPoint)
{
	b2World* world = b2GetWorldFromIndex(bodyId.world0);
	b2Body* body = b2GetBody(world, bodyId);
	return b2TransformPoint(b2MakeTransform(body), localPoint);
}

b2Vec2 b2Body_GetLocalVector(b2BodyId bodyId, b2Vec2 worldVector)
{
	b2World* world = b2GetWorldFromIndex(bodyId.world0);
	b2Body* body = b2GetBody(world, bodyId);
	return b2InvRotateVector(body->rotation, worldVector);
}

b2Vec2 b2Body_GetWorldVector(b2BodyId bodyId, b2Vec2 localVector)
{
	B2_ASSERT(b2Body_IsValid(bodyId));
	b2World* world = b2GetWorldFromIndex(bodyId.world0);
	b2Body* body = b2GetBody(world, bodyId);
	return b2RotateVector(body->rotation, localVector);
}

void b2Body_SetTransform(b2BodyId bodyId, b2Vec2 position, float angle)
{
	B2_ASSERT(b2Body_IsValid(bodyId));
	b2World* world = b2GetWorldFromIndex(bodyId.world0);
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

	int32_t shapeIndex = body->shapeList;
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
	b2World* world = b2GetWorldFromIndex(bodyId.world0);
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
	b2World* world = b2GetWorldFromIndex(bodyId.world0);
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
	b2World* world = b2GetWorldFromIndex(bodyId.world0);
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
	b2World* world = b2GetWorldFromIndex(bodyId.world0);
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
	b2World* world = b2GetWorldFromIndex(bodyId.world0);
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
	b2World* world = b2GetWorldFromIndex(bodyId.world0);
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
	b2World* world = b2GetWorldFromIndex(bodyId.world0);
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
	b2World* world = b2GetWorldFromIndex(bodyId.world0);
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
	b2World* world = b2GetWorldFromIndex(bodyId.world0);
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
	b2World* world = b2GetWorldFromIndex(bodyId.world0);
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
	b2World* world = b2GetWorldFromIndex(bodyId.world0);
	b2Body* body = b2GetBody(world, bodyId);
	return body->type;
}

// Changing the body type is quite complex. So we basically recreate the body.
// #todo check joints
void b2Body_SetType(b2BodyId bodyId, b2BodyType type)
{
	B2_ASSERT(b2Body_IsValid(bodyId));
	b2World* world = b2GetWorldFromIndex(bodyId.world0);
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
	b2World* world = b2GetWorldFromIndex(bodyId.world0);
	b2Body* body = b2GetBody(world, bodyId);
	body->userData = userData;
}

void* b2Body_GetUserData(b2BodyId bodyId)
{
	B2_ASSERT(b2Body_IsValid(bodyId));
	b2World* world = b2GetWorldFromIndex(bodyId.world0);
	b2Body* body = b2GetBody(world, bodyId);
	return body->userData;
}

float b2Body_GetMass(b2BodyId bodyId)
{
	B2_ASSERT(b2Body_IsValid(bodyId));
	b2World* world = b2GetWorldFromIndex(bodyId.world0);
	b2Body* body = b2GetBody(world, bodyId);
	return body->mass;
}

float b2Body_GetInertiaTensor(b2BodyId bodyId)
{
	B2_ASSERT(b2Body_IsValid(bodyId));
	b2World* world = b2GetWorldFromIndex(bodyId.world0);
	b2Body* body = b2GetBody(world, bodyId);
	return body->I;
}

b2Vec2 b2Body_GetLocalCenterOfMass(b2BodyId bodyId)
{
	B2_ASSERT(b2Body_IsValid(bodyId));
	b2World* world = b2GetWorldFromIndex(bodyId.world0);
	b2Body* body = b2GetBody(world, bodyId);
	return body->localCenter;
}

b2Vec2 b2Body_GetWorldCenterOfMass(b2BodyId bodyId)
{
	B2_ASSERT(b2Body_IsValid(bodyId));
	b2World* world = b2GetWorldFromIndex(bodyId.world0);
	b2Body* body = b2GetBody(world, bodyId);
	return body->position;
}

void b2Body_SetMassData(b2BodyId bodyId, b2MassData massData)
{
	B2_ASSERT(b2Body_IsValid(bodyId));
	B2_ASSERT(b2IsValid(massData.mass) && massData.mass >= 0.0f);
	B2_ASSERT(b2IsValid(massData.I) && massData.I >= 0.0f);
	B2_ASSERT(b2Vec2_IsValid(massData.center));

	b2World* world = b2GetWorldFromIndexLocked(bodyId.world0);
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
	b2World* world = b2GetWorldFromIndex(bodyId.world0);
	b2Body* body = b2GetBody(world, bodyId);
	b2MassData massData = {body->mass, body->localCenter, body->I};
	return massData;
}

void b2Body_ResetMassData(b2BodyId bodyId)
{
	B2_ASSERT(b2Body_IsValid(bodyId));
	b2World* world = b2GetWorldFromIndexLocked(bodyId.world0);
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

	b2World* world = b2GetWorldFromIndexLocked(bodyId.world0);
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
	b2World* world = b2GetWorldFromIndex(bodyId.world0);
	b2Body* body = b2GetBody(world, bodyId);
	return body->linearDamping;
}

void b2Body_SetAngularDamping(b2BodyId bodyId, float angularDamping)
{
	B2_ASSERT(b2Body_IsValid(bodyId));
	B2_ASSERT(b2IsValid(angularDamping) && angularDamping >= 0.0f);

	b2World* world = b2GetWorldFromIndexLocked(bodyId.world0);
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
	b2World* world = b2GetWorldFromIndex(bodyId.world0);
	b2Body* body = b2GetBody(world, bodyId);
	return body->angularDamping;
}

void b2Body_SetGravityScale(b2BodyId bodyId, float gravityScale)
{
	B2_ASSERT(b2Body_IsValid(bodyId));
	B2_ASSERT(b2IsValid(gravityScale));

	b2World* world = b2GetWorldFromIndexLocked(bodyId.world0);
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
	b2World* world = b2GetWorldFromIndex(bodyId.world0);
	b2Body* body = b2GetBody(world, bodyId);
	return body->gravityScale;
}

bool b2Body_IsAwake(b2BodyId bodyId)
{
	b2World* world = b2GetWorldFromIndex(bodyId.world0);
	B2_ASSERT(b2Body_IsValid(bodyId));
	return b2IsBodyAwake(world, bodyId.index1 - 1);
}

void b2Body_Wake(b2BodyId bodyId)
{
	B2_ASSERT(b2Body_IsValid(bodyId));
	b2World* world = b2GetWorldFromIndexLocked(bodyId.world0);
	if (world == NULL)
	{
		return;
	}

	b2WakeBody(world, bodyId.index1 - 1);
}

bool b2Body_IsEnabled(b2BodyId bodyId)
{
	b2World* world = b2GetWorldFromIndex(bodyId.world0);
	b2Body* body = b2GetBody(world, bodyId);
	return body->isEnabled;
}

bool b2Body_IsSleepEnabled(b2BodyId bodyId)
{
	b2World* world = b2GetWorldFromIndex(bodyId.world0);
	b2Body* body = b2GetBody(world, bodyId);
	return body->enableSleep;
}

void b2Body_EnableSleep(b2BodyId bodyId, bool enableSleep)
{
	b2World* world = b2GetWorldFromIndexLocked(bodyId.world0);
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
	b2World* world = b2GetWorldFromIndexLocked(bodyId.world0);
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
	b2World* world = b2GetWorldFromIndexLocked(bodyId.world0);
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
	b2World* world = b2GetWorldFromIndexLocked(bodyId.world0);
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
	b2World* world = b2GetWorldFromIndex(bodyId.world0);
	b2Body* body = b2GetBody(world, bodyId);
	return body->fixedRotation;
}

void b2Body_SetBullet(b2BodyId bodyId, bool flag)
{
	b2World* world = b2GetWorldFromIndexLocked(bodyId.world0);
	if (world == NULL)
	{
		return;
	}

	b2Body* body = b2GetBody(world, bodyId);
	body->isBullet = flag;
}

bool b2Body_IsBullet(b2BodyId bodyId)
{
	b2World* world = b2GetWorldFromIndex(bodyId.world0);
	b2Body* body = b2GetBody(world, bodyId);
	return body->isBullet;
}

b2ShapeId b2Body_GetFirstShape(b2BodyId bodyId)
{
	b2World* world = b2GetWorldFromIndex(bodyId.world0);
	b2Body* body = b2GetBody(world, bodyId);

	if (body->shapeList == B2_NULL_INDEX)
	{
		return (b2ShapeId){0};
	}

	b2Shape* shape = world->shapes + body->shapeList;
	b2ShapeId id = {shape->object.index + 1, bodyId.world0, shape->object.revision};
	return id;
}

b2ShapeId b2Body_GetNextShape(b2ShapeId shapeId)
{
	b2World* world = b2GetWorldFromIndex(shapeId.world0);
	b2Shape* shape = b2GetShape(world, shapeId);

	if (shape->nextShapeIndex == B2_NULL_INDEX)
	{
		return (b2ShapeId){0};
	}

	shape = world->shapes + shape->nextShapeIndex;
	b2ShapeId id = {shape->object.index + 1, shapeId.world0, shape->object.revision};
	return id;
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
		int32_t jointIndex = jointKey >> 1;
		int32_t edgeIndex = jointKey & 1;
		int32_t otherEdgeIndex = edgeIndex ^ 1;

		b2Joint* joint = b2GetJointFromKey(world, jointKey);
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
	int32_t bodyIndex = body->islandIndex;

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
