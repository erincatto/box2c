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

b2Body* b2GetBody(b2World* world, int bodyId)
{
	b2CheckIndex(world->bodyLookupArray, bodyId);
	b2BodyLookup lookup = world->bodyLookupArray[bodyId];
	b2CheckIndex(world->solverSetArray, lookup.setIndex);
	b2SolverSet* set = world->solverSetArray + lookup.setIndex;
	B2_ASSERT(0 <= lookup.bodyIndex && lookup.bodyIndex <= set->bodies.count);
	b2Body* body = set->bodies.data + lookup.bodyIndex;
	return body;
}

// Get a validated body from a world using an id.
// todo remove this function and instead use B2_ASSERT(b2Body_IsValid(bodyId))
b2Body* b2GetBodyFullId(b2World* world, b2BodyId bodyId)
{
	B2_ASSERT(b2Body_IsValid(bodyId));

	// id index starts at one so that zero can represent null
	return b2GetBody(world, bodyId.index1 - 1);
}

b2Transform b2GetBodyTransform(b2World* world, int bodyId)
{
	b2CheckIndex(world->bodyLookupArray, bodyId);
	b2BodyLookup lookup = world->bodyLookupArray[bodyId];
	b2CheckIndex(world->solverSetArray, lookup.setIndex);
	b2SolverSet* set = world->solverSetArray + lookup.setIndex;
	B2_ASSERT(0 <= lookup.bodyIndex && lookup.bodyIndex <= set->bodies.count);
	b2Body* body = set->bodies.data + lookup.bodyIndex;
	return b2MakeTransform(body);
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
	B2_ASSERT(setIndex != b2_disabledSet);

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
		b2Body* prevBody = b2GetBody(world, body->islandPrev);
		prevBody->islandNext = body->islandNext;
	}

	if (body->islandNext != B2_NULL_INDEX)
	{
		b2Body* nextBody = b2GetBody(world, body->islandNext);
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

static void b2DestroyBodyContacts(b2World* world, b2Body* body, bool wakeBodies)
{
	// Destroy the attached contacts
	int edgeKey = body->contactList.headContactKey;
	while (edgeKey != B2_NULL_INDEX)
	{
		int contactId = edgeKey >> 1;
		int edgeIndex = edgeKey & 1;

		b2Contact* contact = b2GetContactFromRawId(world, contactId);
		edgeKey = contact->edges[edgeIndex].nextKey;
		b2DestroyContact(world, contact, wakeBodies);
	}
}

static void b2EnableBody(b2World* world, b2Body* body)
{
// todo
#if 0
	// Add shapes to broad-phase
	int shapeIndex = body->shapeList;
	while (shapeIndex != B2_NULL_INDEX)
	{
		b2Shape* shape = world->shapes + shapeIndex;
		shapeIndex = shape->nextShapeId;

		b2CreateShapeProxy(shape, &world->broadPhase, body->type, b2MakeTransform(body));
	}

	b2CreateIslandForBody(world, body, true);

	int headJointKey = body->jointList;
	while (headJointKey != B2_NULL_INDEX)
	{
		int jointIndex = headJointKey >> 1;
		int edgeIndex = headJointKey & 1;
		b2Joint* joint = world->joints + jointIndex;
		B2_ASSERT(joint->islandIndex == B2_NULL_INDEX);
		b2Body* bodyA = b2GetBody(world, joint->edges[0].bodyKey);
		b2Body* bodyB = b2GetBody(world, joint->edges[1].bodyKey);
		if (bodyA->type == b2_dynamicBody || bodyB->type == b2_dynamicBody)
		{
			b2AddJointToGraph(world, joint);
			b2LinkJoint(world, joint);
		}
		headJointKey = joint->edges[edgeIndex].nextKey;
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
	int shapeIndex = body->shapeList;
	while (shapeIndex != B2_NULL_INDEX)
	{
		b2Shape* shape = world->shapes + shapeIndex;
		shapeIndex = shape->nextShapeId;

		b2DestroyShapeProxy(shape, &world->broadPhase);
	}

	int headJointKey = body->jointList;
	while (headJointKey != B2_NULL_INDEX)
	{
		int jointIndex = headJointKey >> 1;
		int edgeIndex = headJointKey & 1;
		b2Joint* joint = world->joints + jointIndex;
		if (joint->colorIndex != B2_NULL_INDEX)
		{
			b2RemoveJointFromGraph(world, joint);
		}

		if (joint->islandIndex != B2_NULL_INDEX)
		{
			b2UnlinkJoint(world, joint);
		}
		headJointKey = joint->edges[edgeIndex].nextKey;
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

	bool isAwake = (def->isAwake || def->enableSleep == false) && def->isEnabled;

	// Determine the body set
	int setIndex;
	if (def->isEnabled == false)
	{
		setIndex = b2_disabledSet;
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
		B2_ASSERT(((uintptr_t)state & 0x1F) == 0);
	}

	int bodyId = b2AllocId(&world->bodyIdPool);

	B2_ASSERT(b2Vec2_IsValid(def->position));
	B2_ASSERT(b2IsValid(def->angle));
	B2_ASSERT(b2Vec2_IsValid(def->linearVelocity));
	B2_ASSERT(b2IsValid(def->angularVelocity));
	B2_ASSERT(b2IsValid(def->linearDamping) && def->linearDamping >= 0.0f);
	B2_ASSERT(b2IsValid(def->angularDamping) && def->angularDamping >= 0.0f);
	B2_ASSERT(b2IsValid(def->gravityScale));

	*body = (b2Body){0};
	body->bodyId = bodyId;
	body->origin = def->position;
	body->rotation = b2MakeRot(def->angle);
	body->position = def->position;
	body->rotation0 = body->rotation;
	body->position0 = body->position;
	body->localCenter = b2Vec2_zero;
	body->force = b2Vec2_zero;
	body->torque = 0.0f;
	body->shapeList.headShapeId = B2_NULL_INDEX;
	body->shapeList.shapeCount = 0;
	body->chainList.headChainId = B2_NULL_INDEX;
	body->contactList.headContactKey = B2_NULL_INDEX;
	body->contactList.contactCount = 0;
	body->headJointKey = B2_NULL_INDEX;
	body->jointCount = 0;
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
	body->isKinematic = def->isKinematic;
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
	if (setIndex != b2_disabledSet)
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

void b2DestroyBody(b2BodyId bodyId)
{
	b2World* world = b2GetWorldLocked(bodyId.world0);
	if (world == NULL)
	{
		return;
	}

	// Wake body before destroying, this way we don't need to wake
	// bodies attached to joints and contacts when they are destroyed
	b2CheckBodyId(bodyId);
	int id = bodyId.index1 - 1;
	b2WakeBody(world, id);

	bool wakeBodies = false;
	b2Body* body = b2GetBody(world, id);

	// Destroy the attached joints
	int edgeKey = body->headJointKey;
	while (edgeKey != B2_NULL_INDEX)
	{
		int headJointKey = edgeKey >> 1;
		int edgeIndex = edgeKey & 1;

		b2Joint* joint = b2GetJoint(world, headJointKey);
		edgeKey = joint->edges[edgeIndex].nextKey;

		// careful because this modifies the list being traversed
		b2DestroyJointInternal(world, joint, wakeBodies);
	}

	// destroy all contacts attached to this body, but don't wake others.
	b2DestroyBodyContacts(world, body, wakeBodies);

	// Delete the attached shapes and their broad-phase proxies.
	int shapeId = body->shapeList.headShapeId;
	while (shapeId != B2_NULL_INDEX)
	{
		b2Shape* shape = world->shapes + shapeId;
		shapeId = shape->nextShapeId;

		b2DestroyShapeProxy(shape, &world->broadPhase);
		b2FreeObject(&world->shapePool, &shape->object);
	}

	// Delete the attached chains. The associated shapes have already been deleted above.
	int chainId = body->chainList.headChainId;
	while (chainId != B2_NULL_INDEX)
	{
		b2ChainShape* chain = world->chains + chainId;
		chainId = chain->nextIndex;

		b2Free(chain->shapeIndices, chain->count * sizeof(int));
		chain->shapeIndices = NULL;
		b2FreeObject(&world->chainPool, &chain->object);
	}

	b2RemoveBodyFromIsland(world, body);

	// Remove body from solver set that owns it
	b2CheckIndex(world->bodyLookupArray, id);
	b2BodyLookup* lookup = world->bodyLookupArray + id;
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
	b2FreeId(&world->bodyIdPool, id);

	b2ValidateWorld(world);
}

int b2Body_GetContactCapacity(b2BodyId bodyId)
{
	b2World* world = b2GetWorldLocked(bodyId.world0);
	if (world == NULL)
	{
		return 0;
	}

	b2Body* body = b2GetBodyFullId(world, bodyId);

	// Conservative and fast
	return body->contactList.contactCount;
}

int b2Body_GetContactData(b2BodyId bodyId, b2ContactData* contactData, int capacity)
{
	b2World* world = b2GetWorldLocked(bodyId.world0);
	if (world == NULL)
	{
		return 0;
	}

	b2Body* body = b2GetBodyFullId(world, bodyId);

	int contactKey = body->contactList.headContactKey;
	int index = 0;
	while (contactKey != B2_NULL_INDEX && index < capacity)
	{
		int edgeIndex = contactKey & 1;

		int contactId = contactKey >> 1;
		b2Contact* contact = b2GetContactFromRawId(world, contactId);

		// Is contact touching?
		if (contact->flags & b2_contactTouchingFlag)
		{
			b2Shape* shapeA = world->shapes + contact->shapeIdA;
			b2Shape* shapeB = world->shapes + contact->shapeIdB;

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

	b2Body* body = b2GetBodyFullId(world, bodyId);
	if (body->shapeList.headShapeId == B2_NULL_INDEX)
	{
		return (b2AABB){body->origin, body->origin};
	}

	b2Shape* shape = world->shapes + body->shapeList.headShapeId;
	b2AABB aabb = shape->aabb;
	while (shape->nextShapeId != B2_NULL_INDEX)
	{
		shape = world->shapes + shape->nextShapeId;
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
	if (body->isKinematic)
	{
		body->position = body->origin;
		return;
	}

	// Accumulate mass over all shapes.
	b2Vec2 localCenter = b2Vec2_zero;
	int shapeId = body->shapeList.headShapeId;
	while (shapeId != B2_NULL_INDEX)
	{
		const b2Shape* s = world->shapes + shapeId;
		shapeId = s->nextShapeId;

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

b2Vec2 b2Body_GetPosition(b2BodyId bodyId)
{
	b2World* world = b2GetWorld(bodyId.world0);
	b2Body* body = b2GetBodyFullId(world, bodyId);
	return body->origin;
}

b2Rot b2Body_GetRotation(b2BodyId bodyId)
{
	b2World* world = b2GetWorld(bodyId.world0);
	b2Body* body = b2GetBodyFullId(world, bodyId);
	return body->rotation;
}

float b2Body_GetAngle(b2BodyId bodyId)
{
	b2World* world = b2GetWorld(bodyId.world0);
	b2Body* body = b2GetBodyFullId(world, bodyId);
	return b2Rot_GetAngle(body->rotation);
}

b2Transform b2Body_GetTransform(b2BodyId bodyId)
{
	b2World* world = b2GetWorld(bodyId.world0);
	b2Body* body = b2GetBodyFullId(world, bodyId);
	return b2MakeTransform(body);
}

b2Vec2 b2Body_GetLocalPoint(b2BodyId bodyId, b2Vec2 worldPoint)
{
	b2World* world = b2GetWorld(bodyId.world0);
	b2Body* body = b2GetBodyFullId(world, bodyId);
	return b2InvTransformPoint(b2MakeTransform(body), worldPoint);
}

b2Vec2 b2Body_GetWorldPoint(b2BodyId bodyId, b2Vec2 localPoint)
{
	b2World* world = b2GetWorld(bodyId.world0);
	b2Body* body = b2GetBodyFullId(world, bodyId);
	return b2TransformPoint(b2MakeTransform(body), localPoint);
}

b2Vec2 b2Body_GetLocalVector(b2BodyId bodyId, b2Vec2 worldVector)
{
	b2World* world = b2GetWorld(bodyId.world0);
	b2Body* body = b2GetBodyFullId(world, bodyId);
	return b2InvRotateVector(body->rotation, worldVector);
}

b2Vec2 b2Body_GetWorldVector(b2BodyId bodyId, b2Vec2 localVector)
{
	B2_ASSERT(b2Body_IsValid(bodyId));
	b2World* world = b2GetWorld(bodyId.world0);
	b2Body* body = b2GetBodyFullId(world, bodyId);
	return b2RotateVector(body->rotation, localVector);
}

void b2Body_SetTransform(b2BodyId bodyId, b2Vec2 position, float angle)
{
	B2_ASSERT(b2Body_IsValid(bodyId));
	b2World* world = b2GetWorld(bodyId.world0);
	B2_ASSERT(world->locked == false);

	b2Body* body = b2GetBodyFullId(world, bodyId);

	body->origin = position;

	body->rotation = b2MakeRot(angle);
	body->position = b2Add(b2RotateVector(body->rotation, body->localCenter), body->origin);

	body->rotation0 = body->rotation;
	body->position0 = body->position;

	b2BroadPhase* broadPhase = &world->broadPhase;

	b2Transform transform = b2MakeTransform(body);
	float margin = b2_aabbMargin;

	int shapeId = body->shapeList.headShapeId;
	while (shapeId != B2_NULL_INDEX)
	{
		b2Shape* shape = world->shapes + shapeId;
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

		shapeId = shape->nextShapeId;
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

	int id = bodyId.index1 - 1;
	b2CheckIndex(world->bodyLookupArray, id);
	b2BodyLookup* lookup = world->bodyLookupArray + id;
	B2_ASSERT(lookup->revision == bodyId.revision);

	if (wake && lookup->setIndex >= b2_firstSleepingSet)
	{
		// this will not invalidate lookup pointer
		b2WakeBody(world, id);
	}

	if (lookup->setIndex == b2_awakeSet)
	{
		b2Body* body = b2GetBody(world, id);
		body->force = b2Add(body->force, force);
		body->torque += b2Cross(b2Sub(point, body->position), force);
	}
}

void b2Body_ApplyForceToCenter(b2BodyId bodyId, b2Vec2 force, bool wake)
{
	B2_ASSERT(b2Body_IsValid(bodyId));
	b2World* world = b2GetWorld(bodyId.world0);

	int id = bodyId.index1 - 1;
	b2CheckIndex(world->bodyLookupArray, id);
	b2BodyLookup* lookup = world->bodyLookupArray + id;
	B2_ASSERT(lookup->revision == bodyId.revision);

	if (wake && lookup->setIndex >= b2_firstSleepingSet)
	{
		// this will not invalidate lookup pointer
		b2WakeBody(world, id);
	}

	if (lookup->setIndex == b2_awakeSet)
	{
		b2Body* body = b2GetBody(world, id);
		body->force = b2Add(body->force, force);
	}
}

void b2Body_ApplyTorque(b2BodyId bodyId, float torque, bool wake)
{
	B2_ASSERT(b2Body_IsValid(bodyId));
	b2World* world = b2GetWorld(bodyId.world0);

	int id = bodyId.index1 - 1;
	b2CheckIndex(world->bodyLookupArray, id);
	b2BodyLookup* lookup = world->bodyLookupArray + id;
	B2_ASSERT(lookup->revision == bodyId.revision);

	if (wake && lookup->setIndex >= b2_firstSleepingSet)
	{
		// this will not invalidate lookup pointer
		b2WakeBody(world, id);
	}

	if (lookup->setIndex == b2_awakeSet)
	{
		b2Body* body = b2GetBody(world, id);
		body->torque += torque;
	}
}

void b2Body_ApplyLinearImpulse(b2BodyId bodyId, b2Vec2 impulse, b2Vec2 point, bool wake)
{
	B2_ASSERT(b2Body_IsValid(bodyId));
	b2World* world = b2GetWorld(bodyId.world0);

	int id = bodyId.index1 - 1;
	b2CheckIndex(world->bodyLookupArray, id);
	b2BodyLookup* lookup = world->bodyLookupArray + id;
	B2_ASSERT(lookup->revision == bodyId.revision);

	if (wake && lookup->setIndex >= b2_firstSleepingSet)
	{
		// this will not invalidate lookup pointer
		b2WakeBody(world, id);
	}

	if (lookup->setIndex == b2_awakeSet)
	{
		int localIndex = lookup->bodyIndex;
		b2SolverSet* set = world->solverSetArray + b2_awakeSet;
		B2_ASSERT(0 <= localIndex && lookup->bodyIndex < set->states.count);
		b2BodyState* state = set->states.data + lookup->bodyIndex;
		b2Body* body = set->bodies.data + lookup->bodyIndex;
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
	b2BodyLookup* lookup = world->bodyLookupArray + id;
	B2_ASSERT(lookup->revision == bodyId.revision);

	if (wake && lookup->setIndex >= b2_firstSleepingSet)
	{
		// this will not invalidate lookup pointer
		b2WakeBody(world, id);
	}

	if (lookup->setIndex == b2_awakeSet)
	{
		int localIndex = lookup->bodyIndex;
		b2SolverSet* set = world->solverSetArray + b2_awakeSet;
		B2_ASSERT(0 <= localIndex && lookup->bodyIndex < set->states.count);
		b2BodyState* state = set->states.data + lookup->bodyIndex;
		b2Body* body = set->bodies.data + lookup->bodyIndex;
		state->linearVelocity = b2MulAdd(state->linearVelocity, body->invMass, impulse);
	}
}

void b2Body_ApplyAngularImpulse(b2BodyId bodyId, float impulse, bool wake)
{
	B2_ASSERT(b2Body_IsValid(bodyId));
	b2World* world = b2GetWorld(bodyId.world0);

	int id = bodyId.index1 - 1;
	b2CheckIndex(world->bodyLookupArray, id);
	b2BodyLookup* lookup = world->bodyLookupArray + id;
	B2_ASSERT(lookup->revision == bodyId.revision);

	if (wake && lookup->setIndex >= b2_firstSleepingSet)
	{
		// this will not invalidate lookup pointer
		b2WakeBody(world, id);
	}

	if (lookup->setIndex == b2_awakeSet)
	{
		int localIndex = lookup->bodyIndex;
		b2SolverSet* set = world->solverSetArray + b2_awakeSet;
		B2_ASSERT(0 <= localIndex && lookup->bodyIndex < set->states.count);
		b2BodyState* state = set->states.data + lookup->bodyIndex;
		b2Body* body = set->bodies.data + lookup->bodyIndex;
		state->angularVelocity += body->invI * impulse;
	}
}

bool b2Body_IsKinematic(b2BodyId bodyId)
{
	b2World* world = b2GetWorld(bodyId.world0);
	b2Body* body = b2GetBodyFullId(world, bodyId);
	return body->isKinematic;
}

// a kinematic body only collides with dynamic bodies
// a dynamic body collides with everything
// so I have to reset all contacts
void b2Body_SetKinematic(b2BodyId bodyId, bool kinematic)
{
	b2World* world = b2GetWorld(bodyId.world0);
	b2Body* body = b2GetBodyFullId(world, bodyId);
	if (body->isKinematic == kinematic)
	{
		return;
	}

	body->isKinematic = kinematic;

	// destroy all contacts, even the valid ones
	b2DestroyBodyContacts(world, body, false);

	// touch the broad-phase proxies to ensure the correct contacts get created
	int shapeId = body->shapeList.headShapeId;
	while (shapeId != B2_NULL_INDEX)
	{
		b2Shape* shape = world->shapes + shapeId;
		b2BufferMove(&world->broadPhase, shape->proxyKey);
		shapeId = shape->nextShapeId;
	}

	// Body type affects the mass
	b2UpdateBodyMassData(world, body);
}

void b2Body_SetUserData(b2BodyId bodyId, void* userData)
{
	B2_ASSERT(b2Body_IsValid(bodyId));
	b2World* world = b2GetWorld(bodyId.world0);
	b2Body* body = b2GetBodyFullId(world, bodyId);
	body->userData = userData;
}

void* b2Body_GetUserData(b2BodyId bodyId)
{
	B2_ASSERT(b2Body_IsValid(bodyId));
	b2World* world = b2GetWorld(bodyId.world0);
	b2Body* body = b2GetBodyFullId(world, bodyId);
	return body->userData;
}

float b2Body_GetMass(b2BodyId bodyId)
{
	B2_ASSERT(b2Body_IsValid(bodyId));
	b2World* world = b2GetWorld(bodyId.world0);
	b2Body* body = b2GetBodyFullId(world, bodyId);
	return body->mass;
}

float b2Body_GetInertiaTensor(b2BodyId bodyId)
{
	B2_ASSERT(b2Body_IsValid(bodyId));
	b2World* world = b2GetWorld(bodyId.world0);
	b2Body* body = b2GetBodyFullId(world, bodyId);
	return body->I;
}

b2Vec2 b2Body_GetLocalCenterOfMass(b2BodyId bodyId)
{
	B2_ASSERT(b2Body_IsValid(bodyId));
	b2World* world = b2GetWorld(bodyId.world0);
	b2Body* body = b2GetBodyFullId(world, bodyId);
	return body->localCenter;
}

b2Vec2 b2Body_GetWorldCenterOfMass(b2BodyId bodyId)
{
	B2_ASSERT(b2Body_IsValid(bodyId));
	b2World* world = b2GetWorld(bodyId.world0);
	b2Body* body = b2GetBodyFullId(world, bodyId);
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

	b2Body* body = b2GetBodyFullId(world, bodyId);
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
	b2Body* body = b2GetBodyFullId(world, bodyId);
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

	b2Body* body = b2GetBodyFullId(world, bodyId);

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

	b2Body* body = b2GetBodyFullId(world, bodyId);
	body->linearDamping = linearDamping;
}

float b2Body_GetLinearDamping(b2BodyId bodyId)
{
	B2_ASSERT(b2Body_IsValid(bodyId));
	b2World* world = b2GetWorld(bodyId.world0);
	b2Body* body = b2GetBodyFullId(world, bodyId);
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

	b2Body* body = b2GetBodyFullId(world, bodyId);
	body->angularDamping = angularDamping;
}

float b2Body_GetAngularDamping(b2BodyId bodyId)
{
	B2_ASSERT(b2Body_IsValid(bodyId));
	b2World* world = b2GetWorld(bodyId.world0);
	b2Body* body = b2GetBodyFullId(world, bodyId);
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

	b2Body* body = b2GetBodyFullId(world, bodyId);
	body->gravityScale = gravityScale;
}

float b2Body_GetGravityScale(b2BodyId bodyId)
{
	B2_ASSERT(b2Body_IsValid(bodyId));
	b2World* world = b2GetWorld(bodyId.world0);
	b2Body* body = b2GetBodyFullId(world, bodyId);
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
	b2Body* body = b2GetBodyFullId(world, bodyId);
	return body->isEnabled;
}

bool b2Body_IsSleepEnabled(b2BodyId bodyId)
{
	b2World* world = b2GetWorld(bodyId.world0);
	b2Body* body = b2GetBodyFullId(world, bodyId);
	return body->enableSleep;
}

void b2Body_EnableSleep(b2BodyId bodyId, bool enableSleep)
{
	b2World* world = b2GetWorldLocked(bodyId.world0);
	if (world == NULL)
	{
		return;
	}

	b2Body* body = b2GetBodyFullId(world, bodyId);
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

	b2Body* body = b2GetBodyFullId(world, bodyId);
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

	b2Body* body = b2GetBodyFullId(world, bodyId);
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

	b2Body* body = b2GetBodyFullId(world, bodyId);
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
	b2Body* body = b2GetBodyFullId(world, bodyId);
	return body->fixedRotation;
}

void b2Body_SetBullet(b2BodyId bodyId, bool flag)
{
	b2World* world = b2GetWorldLocked(bodyId.world0);
	if (world == NULL)
	{
		return;
	}

	b2Body* body = b2GetBodyFullId(world, bodyId);
	body->isBullet = flag;
}

bool b2Body_IsBullet(b2BodyId bodyId)
{
	b2World* world = b2GetWorld(bodyId.world0);
	b2Body* body = b2GetBodyFullId(world, bodyId);
	return body->isBullet;
}

int b2Body_GetShapeCount(b2BodyId bodyId)
{
	b2World* world = b2GetWorld(bodyId.world0);
	b2Body* body = b2GetBodyFullId(world, bodyId);
	return body->shapeList.shapeCount;
}

void b2Body_GetShapes(b2BodyId bodyId, b2ShapeId* shapeArray, int capacity)
{
	b2World* world = b2GetWorld(bodyId.world0);
	b2Body* body = b2GetBodyFullId(world, bodyId);
	int shapeId = body->shapeList.headShapeId;
	int shapeCount = 0;
	while (shapeId != B2_NULL_INDEX && shapeCount < capacity)
	{
		b2Shape* shape = world->shapes + shapeId;
		B2_ASSERT(b2IsValidObject(&shape->object));

		b2ShapeId id = {shape->object.index + 1, bodyId.world0, shape->object.revision};
		shapeArray[shapeCount] = id;
		shapeCount += 1;
	
		shapeId = shape->nextShapeId;
	}
}

int b2Body_GetJointCount(b2BodyId bodyId)
{
	b2World* world = b2GetWorld(bodyId.world0);
	b2Body* body = b2GetBodyFullId(world, bodyId);
	return body->jointCount;
}

void b2Body_GetJoints(b2BodyId bodyId, b2JointId* jointArray, int capacity)
{
	b2World* world = b2GetWorld(bodyId.world0);
	b2Body* body = b2GetBodyFullId(world, bodyId);
	int headJointKey = body->headJointKey;

	int jointCount = 0;
	while (headJointKey != B2_NULL_INDEX && jointCount < capacity)
	{
		int jointId = headJointKey >> 1;
		int edgeIndex = headJointKey & 1;

		b2Joint* joint = b2GetJoint(world, jointId);

		b2JointId id = {jointId + 1, bodyId.world0, joint->revision};
		jointArray[jointCount] = id;
		jointCount += 1;
	
		headJointKey = joint->edges[edgeIndex].nextKey;
	}
}

bool b2ShouldBodiesCollide(b2World* world, b2Body* bodyA, b2Body* bodyB)
{
	if (bodyA->isKinematic && bodyB->isKinematic)
	{
		return false;
	}

	int headJointKey;
	int otherBodyId;
	if (bodyA->jointCount < bodyB->jointCount)
	{
		headJointKey = bodyA->headJointKey;
		otherBodyId = bodyB->bodyId;
	}
	else
	{
		headJointKey = bodyB->headJointKey;
		otherBodyId = bodyA->bodyId;
	}

	while (headJointKey != B2_NULL_INDEX)
	{
		int jointId = headJointKey >> 1;
		int edgeIndex = headJointKey & 1;
		int otherEdgeIndex = edgeIndex ^ 1;

		b2Joint* joint = b2GetJoint(world, jointId);
		if (joint->collideConnected == false && joint->edges[otherEdgeIndex].bodyId == otherBodyId)
		{
			return false;
		}

		headJointKey = joint->edges[edgeIndex].nextKey;
	}

	return true;
}
