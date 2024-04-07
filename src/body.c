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
	b2CheckIndex(world->bodyArray, bodyId);
	return world->bodyArray + bodyId;
}

// Get a validated body from a world using an id.
// todo remove this function and instead use B2_ASSERT(b2Body_IsValid(bodyId))
b2Body* b2GetBodyFullId(b2World* world, b2BodyId bodyId)
{
	B2_ASSERT(b2Body_IsValid(bodyId));

	// id index starts at one so that zero can represent null
	return b2GetBody(world, bodyId.index1 - 1);
}

b2Transform b2GetBodyTransformQuick(b2World* world, b2Body* body)
{
	b2CheckIndex(world->solverSetArray, body->setIndex);
	b2SolverSet* set = world->solverSetArray + body->setIndex;
	B2_ASSERT(0 <= body->localIndex && body->localIndex <= set->sims.count);
	b2BodySim* bodySim = set->sims.data + body->localIndex;
	return bodySim->transform;
}

b2Transform b2GetBodyTransform(b2World* world, int bodyId)
{
	b2CheckIndex(world->bodyArray, bodyId);
	b2Body* body = world->bodyArray + bodyId;
	return b2GetBodyTransformQuick(world, body);
}

// Create a b2BodyId from a raw id.
b2BodyId b2MakeBodyId(b2World* world, int bodyId)
{
	B2_ASSERT(0 <= bodyId && bodyId < b2Array(world->bodyArray).count);
	b2Body* body = world->bodyArray + bodyId;
	return (b2BodyId){bodyId + 1, world->worldId, body->revision};
}

b2BodySim* b2GetBodySim(b2World* world, b2Body* body)
{
	b2CheckIndex(world->solverSetArray, body->setIndex);
	b2SolverSet* set = world->solverSetArray + body->setIndex;
	B2_ASSERT(0 <= body->localIndex && body->localIndex < set->sims.count);
	return set->sims.data + body->localIndex;
}

b2BodyState* b2GetBodyState(b2World* world, b2Body* body)
{
	b2CheckIndex(world->solverSetArray, body->setIndex);
	if (body->setIndex == b2_awakeSet)
	{
		b2SolverSet* set = world->solverSetArray + b2_awakeSet;
		B2_ASSERT(0 <= body->localIndex && body->localIndex < set->states.count);
		return set->states.data + body->localIndex;
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

	// Fix the island's linked list of sims
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
		b2WakeSolverSet(world, body->setIndex);
	}

	body->islandId = B2_NULL_INDEX;
	body->islandPrev = B2_NULL_INDEX;
	body->islandNext = B2_NULL_INDEX;
}

static void b2DestroyBodyContacts(b2World* world, b2Body* body, bool wakeBodies)
{
	// Destroy the attached contacts
	int edgeKey = body->headContactKey;
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
	B2_ASSERT(b2Vec2_IsValid(def->position));
	B2_ASSERT(b2IsValid(def->angle));
	B2_ASSERT(b2Vec2_IsValid(def->linearVelocity));
	B2_ASSERT(b2IsValid(def->angularVelocity));
	B2_ASSERT(b2IsValid(def->linearDamping) && def->linearDamping >= 0.0f);
	B2_ASSERT(b2IsValid(def->angularDamping) && def->angularDamping >= 0.0f);
	B2_ASSERT(b2IsValid(def->gravityScale));

	b2World* world = b2GetWorldFromId(worldId);
	B2_ASSERT(world->locked == false);

	if (world->locked)
	{
		return b2_nullBodyId;
	}

	bool isAwake = (def->isAwake || def->enableSleep == false) && def->isEnabled;

	// determine the solver set
	int setIndex;
	if (def->isEnabled == false)
	{
		// any body type can be disabled
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
	
	int bodyId = b2AllocId(&world->bodyIdPool);

	b2SolverSet* set = world->solverSetArray + setIndex;
	b2BodySim* bodySim = b2AddBodySim(&world->blockAllocator, &set->sims);
	*bodySim = (b2BodySim){0};
	bodySim->transform.p = def->position;
	bodySim->transform.q = b2MakeRot(def->angle);
	bodySim->center = def->position;
	bodySim->rotation0 = bodySim->transform.q;
	bodySim->center0 = bodySim->center;
	bodySim->localCenter = b2Vec2_zero;
	bodySim->force = b2Vec2_zero;
	bodySim->torque = 0.0f;
	bodySim->mass = 0.0f;
	bodySim->invMass = 0.0f;
	bodySim->I = 0.0f;
	bodySim->invI = 0.0f;
	bodySim->minExtent = b2_huge;
	bodySim->maxExtent = 0.0f;
	bodySim->linearDamping = def->linearDamping;
	bodySim->angularDamping = def->angularDamping;
	bodySim->gravityScale = def->gravityScale;
	bodySim->sleepTime = 0.0f;
	bodySim->bodyId = bodyId;
	bodySim->type = def->type;
	bodySim->enableSleep = def->enableSleep;
	bodySim->fixedRotation = def->fixedRotation;
	bodySim->isBullet = def->isBullet;
	bodySim->isMarked = false;
	bodySim->enlargeAABB = false;
	bodySim->isFast = false;
	bodySim->isSpeedCapped = false;

	if (setIndex == b2_awakeSet)
	{
		b2BodyState* bodyState = b2AddBodyState(&world->blockAllocator, &set->states);
		B2_ASSERT(((uintptr_t)bodyState & 0x1F) == 0);

		*bodyState = (b2BodyState){0};
		bodyState->linearVelocity = def->linearVelocity;
		bodyState->angularVelocity = def->angularVelocity;
		bodyState->deltaRotation = b2Rot_identity;
	}

	if (bodyId == b2Array(world->bodyArray).count)
	{
		b2Array_Push(world->bodyArray, (b2Body){0});
	}
	else
	{
		B2_ASSERT(world->bodyArray[bodyId].setIndex == B2_NULL_INDEX);
		B2_ASSERT(world->bodyArray[bodyId].localIndex == B2_NULL_INDEX);
	}

	b2CheckIndex(world->bodyArray, bodyId);
	b2Body* body = world->bodyArray + bodyId;
	body->userData = def->userData;
	body->setIndex = setIndex;
	body->localIndex = set->sims.count - 1;
	body->revision += 1;
	body->headShapeId = B2_NULL_INDEX;
	body->shapeCount = 0;
	body->headChainId = B2_NULL_INDEX;
	body->headContactKey = B2_NULL_INDEX;
	body->contactCount = 0;
	body->headJointKey = B2_NULL_INDEX;
	body->jointCount = 0;
	body->islandId = B2_NULL_INDEX;
	body->islandPrev = B2_NULL_INDEX;
	body->islandNext = B2_NULL_INDEX;
	body->bodyId = bodyId;
	body->worldId = world->worldId;

	// dynamic and kinematic bodies that are enabled need a island
	if (setIndex >= b2_awakeSet)
	{
		b2CreateIslandForBody(world, setIndex, body);
	}

	b2ValidateWorld(world);

	b2BodyId id = {bodyId + 1, world->worldId, body->revision};
	return id;
}

bool b2IsBodyAwake(b2World* world, b2Body* body)
{
	return body->setIndex == b2_awakeSet;
}

bool b2WakeBody(b2World* world, b2Body* body)
{
	if (body->setIndex >= b2_firstSleepingSet)
	{
		b2WakeSolverSet(world, body->setIndex);
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
	// sims attached to joints and contacts when they are destroyed
	b2Body* body = b2GetBodyFullId(world, bodyId);
	b2WakeBody(world, body);

	bool wakeBodies = false;

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
	int shapeId = body->headShapeId;
	while (shapeId != B2_NULL_INDEX)
	{
		b2Shape* shape = world->shapes + shapeId;
		shapeId = shape->nextShapeId;

		b2DestroyShapeProxy(shape, &world->broadPhase);
		b2FreeObject(&world->shapePool, &shape->object);
	}

	// Delete the attached chains. The associated shapes have already been deleted above.
	int chainId = body->headChainId;
	while (chainId != B2_NULL_INDEX)
	{
		b2ChainShape* chain = world->chains + chainId;
		chainId = chain->nextIndex;

		b2Free(chain->shapeIndices, chain->count * sizeof(int));
		chain->shapeIndices = NULL;
		b2FreeObject(&world->chainPool, &chain->object);
	}

	b2RemoveBodyFromIsland(world, body);

	// Remove body sim from solver set that owns it
	b2CheckIndex(world->solverSetArray, body->setIndex);
	b2SolverSet* set = world->solverSetArray + body->setIndex;
	int movedIndex = b2RemoveBodySim(&set->sims, body->localIndex);
	if (movedIndex != B2_NULL_INDEX)
	{
		// Fix moved body index
		b2BodySim* movedSim = set->sims.data + body->localIndex;
		int movedId = movedSim->bodyId;
		b2Body* movedBody = world->bodyArray + movedId;
		B2_ASSERT(movedBody->localIndex == movedIndex);
		movedBody->localIndex = body->localIndex;
	}

	// Remove body state from awake set
	if (body->setIndex == b2_awakeSet)
	{
		int result = b2RemoveBodyState(&set->states, body->localIndex);
		B2_MAYBE_UNUSED(result);
		B2_ASSERT(result == movedIndex);
	}

	// Free body and id (preserve body revision)
	body->setIndex = B2_NULL_INDEX;
	body->localIndex = B2_NULL_INDEX;
	b2FreeId(&world->bodyIdPool, body->bodyId);

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
	return body->contactCount;
}

int b2Body_GetContactData(b2BodyId bodyId, b2ContactData* contactData, int capacity)
{
	b2World* world = b2GetWorldLocked(bodyId.world0);
	if (world == NULL)
	{
		return 0;
	}

	b2Body* body = b2GetBodyFullId(world, bodyId);

	int contactKey = body->headContactKey;
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
	if (body->headShapeId == B2_NULL_INDEX)
	{
		b2Transform transform = b2GetBodyTransform(world, body->bodyId);
		return (b2AABB){transform.p, transform.p};
	}

	b2Shape* shape = world->shapes + body->headShapeId;
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
	b2BodySim* bodySim = b2GetBodySim(world, body);

	// Compute mass data from shapes. Each shape has its own density.
	bodySim->mass = 0.0f;
	bodySim->invMass = 0.0f;
	bodySim->I = 0.0f;
	bodySim->invI = 0.0f;
	bodySim->localCenter = b2Vec2_zero;
	bodySim->minExtent = b2_huge;
	bodySim->maxExtent = 0.0f;

	// Static and kinematic sims have zero mass.
	if (bodySim->type != b2_dynamicBody)
	{
		bodySim->center = bodySim->transform.p;
		return;
	}

	// Accumulate mass over all shapes.
	b2Vec2 localCenter = b2Vec2_zero;
	int shapeId = body->headShapeId;
	while (shapeId != B2_NULL_INDEX)
	{
		const b2Shape* s = world->shapes + shapeId;
		shapeId = s->nextShapeId;

		if (s->density == 0.0f)
		{
			continue;
		}

		b2MassData massData = b2ComputeShapeMass(s);
		bodySim->mass += massData.mass;
		localCenter = b2MulAdd(localCenter, massData.mass, massData.center);
		bodySim->I += massData.I;

		b2ShapeExtent extent = b2ComputeShapeExtent(s);
		bodySim->minExtent = B2_MIN(bodySim->minExtent, extent.minExtent);
		bodySim->maxExtent = B2_MAX(bodySim->maxExtent, extent.maxExtent);
	}

	// Compute center of mass.
	if (bodySim->mass > 0.0f)
	{
		bodySim->invMass = 1.0f / bodySim->mass;
		localCenter = b2MulSV(bodySim->invMass, localCenter);
	}

	if (bodySim->I > 0.0f && bodySim->fixedRotation == false)
	{
		// Center the inertia about the center of mass.
		bodySim->I -= bodySim->mass * b2Dot(localCenter, localCenter);
		B2_ASSERT(bodySim->I > 0.0f);
		bodySim->invI = 1.0f / bodySim->I;
	}
	else
	{
		bodySim->I = 0.0f;
		bodySim->invI = 0.0f;
	}

	// Move center of mass.
	b2Vec2 oldCenter = bodySim->center;
	bodySim->localCenter = localCenter;
	bodySim->transform.p = b2TransformPoint(bodySim->transform, bodySim->localCenter);

	// Update center of mass velocity
	b2BodyState* state = b2GetBodyState(world, body);
	if (state != NULL)
	{
		b2Vec2 deltaLinear = b2CrossSV(state->angularVelocity, b2Sub(bodySim->center, oldCenter));
		state->linearVelocity = b2Add(state->linearVelocity, deltaLinear);
	}
}

b2Vec2 b2Body_GetPosition(b2BodyId bodyId)
{
	b2World* world = b2GetWorld(bodyId.world0);
	b2Body* body = b2GetBodyFullId(world, bodyId);
	b2Transform transform = b2GetBodyTransformQuick(world, body);
	return transform.p;
}

b2Rot b2Body_GetRotation(b2BodyId bodyId)
{
	b2World* world = b2GetWorld(bodyId.world0);
	b2Body* body = b2GetBodyFullId(world, bodyId);
	b2Transform transform = b2GetBodyTransformQuick(world, body);
	return transform.q;
}

float b2Body_GetAngle(b2BodyId bodyId)
{
	b2World* world = b2GetWorld(bodyId.world0);
	b2Body* body = b2GetBodyFullId(world, bodyId);
	b2Transform transform = b2GetBodyTransformQuick(world, body);
	return b2Rot_GetAngle(transform.q);
}

b2Transform b2Body_GetTransform(b2BodyId bodyId)
{
	b2World* world = b2GetWorld(bodyId.world0);
	b2Body* body = b2GetBodyFullId(world, bodyId);
	return b2GetBodyTransformQuick(world, body);
}

b2Vec2 b2Body_GetLocalPoint(b2BodyId bodyId, b2Vec2 worldPoint)
{
	b2World* world = b2GetWorld(bodyId.world0);
	b2Body* body = b2GetBodyFullId(world, bodyId);
	b2Transform transform = b2GetBodyTransformQuick(world, body);
	return b2InvTransformPoint(transform, worldPoint);
}

b2Vec2 b2Body_GetWorldPoint(b2BodyId bodyId, b2Vec2 localPoint)
{
	b2World* world = b2GetWorld(bodyId.world0);
	b2Body* body = b2GetBodyFullId(world, bodyId);
	b2Transform transform = b2GetBodyTransformQuick(world, body);
	return b2TransformPoint(transform, localPoint);
}

b2Vec2 b2Body_GetLocalVector(b2BodyId bodyId, b2Vec2 worldVector)
{
	b2World* world = b2GetWorld(bodyId.world0);
	b2Body* body = b2GetBodyFullId(world, bodyId);
	b2Transform transform = b2GetBodyTransformQuick(world, body);
	return b2InvRotateVector(transform.q, worldVector);
}

b2Vec2 b2Body_GetWorldVector(b2BodyId bodyId, b2Vec2 localVector)
{
	b2World* world = b2GetWorld(bodyId.world0);
	b2Body* body = b2GetBodyFullId(world, bodyId);
	b2Transform transform = b2GetBodyTransformQuick(world, body);
	return b2RotateVector(transform.q, localVector);
}

void b2Body_SetTransform(b2BodyId bodyId, b2Vec2 position, float angle)
{
	B2_ASSERT(b2Body_IsValid(bodyId));
	b2World* world = b2GetWorld(bodyId.world0);
	B2_ASSERT(world->locked == false);

	b2Body* body = b2GetBodyFullId(world, bodyId);
	b2BodySim* bodySim = b2GetBodySim(world, body);

	bodySim->transform.p = position;
	bodySim->transform.q = b2MakeRot(angle);
	bodySim->center = b2TransformPoint(bodySim->transform, bodySim->localCenter);

	bodySim->rotation0 = bodySim->transform.q;
	bodySim->center0 = bodySim->center;

	b2BroadPhase* broadPhase = &world->broadPhase;

	b2Transform transform = bodySim->transform;
	float margin = b2_aabbMargin;

	int shapeId = body->headShapeId;
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

			// They body could be disabled
			if (shape->proxyKey != B2_NULL_INDEX)
			{
				b2BroadPhase_MoveProxy(broadPhase, shape->proxyKey, fatAABB);
			}
		}

		shapeId = shape->nextShapeId;
	}
}

b2Vec2 b2Body_GetLinearVelocity(b2BodyId bodyId)
{
	b2World* world = b2GetWorld(bodyId.world0);
	b2Body* body = b2GetBodyFullId(world, bodyId);
	b2BodyState* state = b2GetBodyState(world, body);
	if (state != NULL)
	{
		return state->linearVelocity;
	}
	return b2Vec2_zero;
}

float b2Body_GetAngularVelocity(b2BodyId bodyId)
{
	b2World* world = b2GetWorld(bodyId.world0);
	b2Body* body = b2GetBodyFullId(world, bodyId);
	b2BodyState* state = b2GetBodyState(world, body);
	if (state != NULL)
	{
		return state->angularVelocity;
	}
	return 0.0;
}

void b2Body_SetLinearVelocity(b2BodyId bodyId, b2Vec2 linearVelocity)
{
	b2World* world = b2GetWorld(bodyId.world0);
	b2Body* body = b2GetBodyFullId(world, bodyId);
	b2BodyState* state = b2GetBodyState(world, body);
	if (state == NULL)
	{
		return;
	}

	state->linearVelocity = linearVelocity;
	if (b2LengthSquared(linearVelocity) > 0.0f)
	{
		b2WakeBody(world, body);
	}
}

void b2Body_SetAngularVelocity(b2BodyId bodyId, float angularVelocity)
{
	b2World* world = b2GetWorld(bodyId.world0);
	b2Body* body = b2GetBodyFullId(world, bodyId);
	b2BodyState* state = b2GetBodyState(world, body);
	if (state == NULL)
	{
		return;
	}

	state->angularVelocity = angularVelocity;
	if (angularVelocity != 0.0f)
	{
		b2WakeBody(world, body);
	}
}

void b2Body_ApplyForce(b2BodyId bodyId, b2Vec2 force, b2Vec2 point, bool wake)
{
	b2World* world = b2GetWorld(bodyId.world0);
	b2Body* body = b2GetBodyFullId(world, bodyId);

	if (wake && body->setIndex >= b2_firstSleepingSet)
	{
		b2WakeBody(world, body);
	}

	if (body->setIndex == b2_awakeSet)
	{
		b2BodySim* bodySim = b2GetBodySim(world, body);
		bodySim->force = b2Add(bodySim->force, force);
		bodySim->torque += b2Cross(b2Sub(point, bodySim->center), force);
	}
}

void b2Body_ApplyForceToCenter(b2BodyId bodyId, b2Vec2 force, bool wake)
{
	b2World* world = b2GetWorld(bodyId.world0);
	b2Body* body = b2GetBodyFullId(world, bodyId);

	if (wake && body->setIndex >= b2_firstSleepingSet)
	{
		b2WakeBody(world, body);
	}

	if (body->setIndex == b2_awakeSet)
	{
		b2BodySim* bodySim = b2GetBodySim(world, body);
		bodySim->force = b2Add(bodySim->force, force);
	}
}

void b2Body_ApplyTorque(b2BodyId bodyId, float torque, bool wake)
{
	b2World* world = b2GetWorld(bodyId.world0);
	b2Body* body = b2GetBodyFullId(world, bodyId);

	if (wake && body->setIndex >= b2_firstSleepingSet)
	{
		b2WakeBody(world, body);
	}

	if (body->setIndex == b2_awakeSet)
	{
		b2BodySim* bodySim = b2GetBodySim(world, body);
		bodySim->torque += torque;
	}
}

void b2Body_ApplyLinearImpulse(b2BodyId bodyId, b2Vec2 impulse, b2Vec2 point, bool wake)
{
	b2World* world = b2GetWorld(bodyId.world0);
	b2Body* body = b2GetBodyFullId(world, bodyId);

	if (wake && body->setIndex >= b2_firstSleepingSet)
	{
		b2WakeBody(world, body);
	}

	if (body->setIndex == b2_awakeSet)
	{
		int localIndex = body->localIndex;
		b2SolverSet* set = world->solverSetArray + b2_awakeSet;
		B2_ASSERT(0 <= localIndex && localIndex < set->states.count);
		b2BodyState* state = set->states.data + localIndex;
		b2BodySim* bodySim = set->sims.data + localIndex;
		state->linearVelocity = b2MulAdd(state->linearVelocity, bodySim->invMass, impulse);
		state->angularVelocity += bodySim->invI * b2Cross(b2Sub(point, bodySim->center), impulse);
	}
}

void b2Body_ApplyLinearImpulseToCenter(b2BodyId bodyId, b2Vec2 impulse, bool wake)
{
	b2World* world = b2GetWorld(bodyId.world0);
	b2Body* body = b2GetBodyFullId(world, bodyId);

	if (wake && body->setIndex >= b2_firstSleepingSet)
	{
		b2WakeBody(world, body);
	}

	if (body->setIndex == b2_awakeSet)
	{
		int localIndex = body->localIndex;
		b2SolverSet* set = world->solverSetArray + b2_awakeSet;
		B2_ASSERT(0 <= localIndex && localIndex < set->states.count);
		b2BodyState* state = set->states.data + localIndex;
		b2BodySim* bodySim = set->sims.data + localIndex;
		state->linearVelocity = b2MulAdd(state->linearVelocity, bodySim->invMass, impulse);
	}
}

void b2Body_ApplyAngularImpulse(b2BodyId bodyId, float impulse, bool wake)
{
	B2_ASSERT(b2Body_IsValid(bodyId));
	b2World* world = b2GetWorld(bodyId.world0);

	int id = bodyId.index1 - 1;
	b2CheckIndex(world->bodyArray, id);
	b2Body* body = world->bodyArray + id;
	B2_ASSERT(body->revision == bodyId.revision);

	if (wake && body->setIndex >= b2_firstSleepingSet)
	{
		// this will not invalidate lookup pointer
		b2WakeBody(world, body);
	}

	if (body->setIndex == b2_awakeSet)
	{
		int localIndex = body->localIndex;
		b2SolverSet* set = world->solverSetArray + b2_awakeSet;
		B2_ASSERT(0 <= localIndex && localIndex < set->states.count);
		b2BodyState* state = set->states.data + localIndex;
		b2BodySim* sim = set->sims.data + localIndex;
		state->angularVelocity += sim->invI * impulse;
	}
}

b2BodyType b2Body_GetType(b2BodyId bodyId)
{
	b2World* world = b2GetWorld(bodyId.world0);
	b2Body* body = b2GetBodyFullId(world, bodyId);
	if (body->setIndex == b2_staticSet)
	{
		return b2_staticBody;
	}

	b2BodySim* bodySim = b2GetBodySim(world, body);
	return bodySim->type;
}

// a kinematic body only collides with dynamic sims
// a dynamic body collides with everything
// so I have to reset all contacts
void b2Body_SetType(b2BodyId bodyId, b2BodyType type)
{
	b2World* world = b2GetWorld(bodyId.world0);
	b2Body* body = b2GetBodyFullId(world, bodyId);

	{
		// scope for pointer safety
		b2BodySim* bodySim = b2GetBodySim(world, body);
		b2BodyType originalType = bodySim->type;

		if (originalType == type)
		{
			return;
		}

		bodySim->type = type;

		if (originalType == b2_staticBody)
		{
			// todo move body to awake set
		}
		else if (type == b2_staticBody)
		{
			// todo move body to static set
		}
	}

	b2WakeBody(world, body);

	// destroy all contacts, even the valid ones
	b2DestroyBodyContacts(world, body, false);

	// touch the broad-phase proxies to ensure the correct contacts get created
	int shapeId = body->headShapeId;
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
	b2World* world = b2GetWorld(bodyId.world0);
	b2Body* body = b2GetBodyFullId(world, bodyId);
	body->userData = userData;
}

void* b2Body_GetUserData(b2BodyId bodyId)
{
	b2World* world = b2GetWorld(bodyId.world0);
	b2Body* body = b2GetBodyFullId(world, bodyId);
	return body->userData;
}

float b2Body_GetMass(b2BodyId bodyId)
{
	b2World* world = b2GetWorld(bodyId.world0);
	b2Body* body = b2GetBodyFullId(world, bodyId);
	b2BodySim* bodySim = b2GetBodySim(world, body);
	return bodySim->mass;
}

float b2Body_GetInertiaTensor(b2BodyId bodyId)
{
	b2World* world = b2GetWorld(bodyId.world0);
	b2Body* body = b2GetBodyFullId(world, bodyId);
	b2BodySim* bodySim = b2GetBodySim(world, body);
	return bodySim->I;
}

b2Vec2 b2Body_GetLocalCenterOfMass(b2BodyId bodyId)
{
	b2World* world = b2GetWorld(bodyId.world0);
	b2Body* body = b2GetBodyFullId(world, bodyId);
	b2BodySim* bodySim = b2GetBodySim(world, body);
	return bodySim->localCenter;
}

b2Vec2 b2Body_GetWorldCenterOfMass(b2BodyId bodyId)
{
	b2World* world = b2GetWorld(bodyId.world0);
	b2Body* body = b2GetBodyFullId(world, bodyId);
	b2BodySim* bodySim = b2GetBodySim(world, body);
	return bodySim->center;
}

void b2Body_SetMassData(b2BodyId bodyId, b2MassData massData)
{
	B2_ASSERT(b2IsValid(massData.mass) && massData.mass >= 0.0f);
	B2_ASSERT(b2IsValid(massData.I) && massData.I >= 0.0f);
	B2_ASSERT(b2Vec2_IsValid(massData.center));

	b2World* world = b2GetWorldLocked(bodyId.world0);
	if (world == NULL)
	{
		return;
	}

	b2Body* body = b2GetBodyFullId(world, bodyId);
	b2BodySim* bodySim = b2GetBodySim(world, body);

	bodySim->mass = massData.mass;
	bodySim->I = massData.I;
	bodySim->localCenter = massData.center;

	b2Vec2 center = b2TransformPoint(bodySim->transform, massData.center);
	bodySim->center = center;
	bodySim->center0 = center;

	bodySim->invMass = bodySim->mass > 0.0f ? 1.0f / bodySim->mass : 0.0f;
	bodySim->invI = bodySim->I > 0.0f ? 1.0f / bodySim->I : 0.0f;
}

b2MassData b2Body_GetMassData(b2BodyId bodyId)
{
	b2World* world = b2GetWorld(bodyId.world0);
	b2Body* body = b2GetBodyFullId(world, bodyId);
	b2BodySim* bodySim = b2GetBodySim(world, body);
	b2MassData massData = {bodySim->mass, bodySim->localCenter, bodySim->I};
	return massData;
}

void b2Body_ResetMassData(b2BodyId bodyId)
{
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
	B2_ASSERT(b2IsValid(linearDamping) && linearDamping >= 0.0f);

	b2World* world = b2GetWorldLocked(bodyId.world0);
	if (world == NULL)
	{
		return;
	}

	b2Body* body = b2GetBodyFullId(world, bodyId);
	b2BodySim* bodySim = b2GetBodySim(world, body);
	bodySim->linearDamping = linearDamping;
}

float b2Body_GetLinearDamping(b2BodyId bodyId)
{
	b2World* world = b2GetWorld(bodyId.world0);
	b2Body* body = b2GetBodyFullId(world, bodyId);
	b2BodySim* bodySim = b2GetBodySim(world, body);
	return bodySim->linearDamping;
}

void b2Body_SetAngularDamping(b2BodyId bodyId, float angularDamping)
{
	B2_ASSERT(b2IsValid(angularDamping) && angularDamping >= 0.0f);

	b2World* world = b2GetWorldLocked(bodyId.world0);
	if (world == NULL)
	{
		return;
	}

	b2Body* body = b2GetBodyFullId(world, bodyId);
	b2BodySim* bodySim = b2GetBodySim(world, body);
	bodySim->angularDamping = angularDamping;
}

float b2Body_GetAngularDamping(b2BodyId bodyId)
{
	b2World* world = b2GetWorld(bodyId.world0);
	b2Body* body = b2GetBodyFullId(world, bodyId);
	b2BodySim* bodySim = b2GetBodySim(world, body);
	return bodySim->angularDamping;
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
	b2BodySim* bodySim = b2GetBodySim(world, body);
	bodySim->gravityScale = gravityScale;
}

float b2Body_GetGravityScale(b2BodyId bodyId)
{
	B2_ASSERT(b2Body_IsValid(bodyId));
	b2World* world = b2GetWorld(bodyId.world0);
	b2Body* body = b2GetBodyFullId(world, bodyId);
	b2BodySim* bodySim = b2GetBodySim(world, body);
	return bodySim->gravityScale;
}

bool b2Body_IsAwake(b2BodyId bodyId)
{
	b2World* world = b2GetWorld(bodyId.world0);
	b2Body* body = b2GetBodyFullId(world, bodyId);
	return body->setIndex == b2_awakeSet;
}

void b2Body_Wake(b2BodyId bodyId)
{
	b2World* world = b2GetWorldLocked(bodyId.world0);
	if (world == NULL)
	{
		return;
	}

	b2Body* body = b2GetBodyFullId(world, bodyId);
	b2WakeBody(world, body);
}

bool b2Body_IsEnabled(b2BodyId bodyId)
{
	b2World* world = b2GetWorld(bodyId.world0);
	b2Body* body = b2GetBodyFullId(world, bodyId);
	return body->setIndex != b2_disabledSet;
}

bool b2Body_IsSleepEnabled(b2BodyId bodyId)
{
	b2World* world = b2GetWorld(bodyId.world0);
	b2Body* body = b2GetBodyFullId(world, bodyId);
	b2BodySim* bodySim = b2GetBodySim(world, body);
	return bodySim->enableSleep;
}

void b2Body_EnableSleep(b2BodyId bodyId, bool enableSleep)
{
	b2World* world = b2GetWorldLocked(bodyId.world0);
	if (world == NULL)
	{
		return;
	}

	b2Body* body = b2GetBodyFullId(world, bodyId);
	b2BodySim* bodySim = b2GetBodySim(world, body);
	bodySim->enableSleep = enableSleep;

	if (enableSleep == false)
	{
		b2WakeBody(world, body);
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
	if (body->setIndex != b2_disabledSet)
	{
		b2DisableBody(world, body);
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
	if (body->setIndex == b2_disabledSet)
	{
		b2EnableBody(world, body);
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
	b2BodySim* bodySim = b2GetBodySim(world, body);

	if (bodySim->fixedRotation != flag)
	{
		bodySim->fixedRotation = flag;

		b2BodyState* state = b2GetBodyState(world, body);
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
	b2BodySim* bodySim = b2GetBodySim(world, body);
	return bodySim->fixedRotation;
}

void b2Body_SetBullet(b2BodyId bodyId, bool flag)
{
	b2World* world = b2GetWorldLocked(bodyId.world0);
	if (world == NULL)
	{
		return;
	}

	b2Body* body = b2GetBodyFullId(world, bodyId);
	b2BodySim* bodySim = b2GetBodySim(world, body);
	bodySim->isBullet = flag;
}

bool b2Body_IsBullet(b2BodyId bodyId)
{
	b2World* world = b2GetWorld(bodyId.world0);
	b2Body* body = b2GetBodyFullId(world, bodyId);
	b2BodySim* bodySim = b2GetBodySim(world, body);
	return bodySim->isBullet;
}

int b2Body_GetShapeCount(b2BodyId bodyId)
{
	b2World* world = b2GetWorld(bodyId.world0);
	b2Body* body = b2GetBodyFullId(world, bodyId);
	return body->shapeCount;
}

int b2Body_GetShapes(b2BodyId bodyId, b2ShapeId* shapeArray, int capacity)
{
	b2World* world = b2GetWorld(bodyId.world0);
	b2Body* body = b2GetBodyFullId(world, bodyId);
	int shapeId = body->headShapeId;
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

	return shapeCount;
}

int b2Body_GetJointCount(b2BodyId bodyId)
{
	b2World* world = b2GetWorld(bodyId.world0);
	b2Body* body = b2GetBodyFullId(world, bodyId);
	return body->jointCount;
}

int b2Body_GetJoints(b2BodyId bodyId, b2JointId* jointArray, int capacity)
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

	return jointCount;
}

bool b2ShouldBodiesCollide(b2World* world, b2Body* bodyA, b2Body* bodyB)
{
	// todo cache misses
	b2BodySim* bodySimA = b2GetBodySim(world, bodyA);
	b2BodySim* bodySimB = b2GetBodySim(world, bodyB);

	if (bodySimA->type != b2_dynamicBody && bodySimB->type != b2_dynamicBody)
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
