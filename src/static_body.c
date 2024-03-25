// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "allocate.h"
#include "contact.h"
#include "shape.h"
#include "static_body.h"
#include "world.h"

// needed for dll export
#include "box2d/box2d.h"
#include "box2d/id.h"
#include "box2d/math_functions.h"

b2StaticBodyDef b2DefaultStaticBodyDef()
{
	return (b2StaticBodyDef){0};
}

b2StaticBody* b2GetStaticBodyFullId(b2World* world, b2StaticBodyId bodyId)
{
	int id = bodyId.index1 - 1;
	b2CheckIndex(world->staticBodyArray, id);
	b2StaticBody* body = world->staticBodyArray + id;
	B2_ASSERT(body->bodyId == id);
	B2_ASSERT(body->revision == bodyId.revision);
	return body;
}

b2StaticBody* b2GetStaticBody(b2World* world, int bodyId)
{
	b2CheckIndex(world->staticBodyArray, bodyId);
	b2StaticBody* body = world->staticBodyArray + bodyId;
	B2_ASSERT(body->bodyId == bodyId);
	return body;
}

b2Transform b2GetStaticBodyTransform(b2World* world, int bodyId)
{
	b2StaticBody* body = b2GetStaticBody(world, bodyId);
	return body->transform;
}

b2StaticBodyId b2CreateStaticBody(b2WorldId worldId, const b2StaticBodyDef* def)
{
	b2World* world = b2GetWorldFromId(worldId);
	B2_ASSERT(world->locked == false);

	if (world->locked)
	{
		return b2_nullStaticBodyId;
	}

	int bodyId = b2AllocId(&world->staticBodyIdPool);
	if (bodyId == b2Array(world->staticBodyArray).count)
	{
		b2Array_Push(world->staticBodyArray, (b2StaticBody){0});
	}
	else
	{
		B2_ASSERT(world->staticBodyArray[bodyId].bodyId == B2_NULL_INDEX);
	}
	
	b2StaticBody* body = world->staticBodyArray + bodyId;
	body->revision += 1;
	
	B2_ASSERT(b2Vec2_IsValid(def->position));
	B2_ASSERT(b2IsValid(def->angle));

	*body = (b2StaticBody){0};
	body->transform.p = def->position;
	body->transform.q = b2MakeRot(def->angle);
	body->shapeList.headShapeId = B2_NULL_INDEX;
	body->shapeList.shapeCount = 0;
	body->chainList.headChainId = B2_NULL_INDEX;
	body->contactList.headContactKey = B2_NULL_INDEX;
	body->contactList.contactCount = 0;
	body->userData = def->userData;
	body->worldId = world->worldId;

	b2StaticBodyId id = {bodyId + 1, world->worldId, body->revision};
	return id;
}

void b2DestroyStaticBody(b2StaticBodyId staticBodyId)
{
	b2World* world = b2GetWorldLocked(staticBodyId.world0);
	if (world == NULL)
	{
		return;
	}

	int id = staticBodyId.index1 - 1;
	b2CheckIndex(world->staticBodyArray, id);
	b2StaticBody* staticBody = world->staticBodyArray + id;
	B2_ASSERT(staticBody->bodyId == id);
	B2_ASSERT(staticBody->revision == staticBodyId.revision);

	bool wakeBodies = true;

	// Destroy the attached contacts
	int edgeKey = staticBody->contactList.headContactKey;
	while (edgeKey != B2_NULL_INDEX)
	{
		int contactId = edgeKey >> 1;
		int edgeIndex = edgeKey & 1;

		b2Contact* contact = b2GetContactFromRawId(world, contactId);
		edgeKey = contact->edges[edgeIndex].nextKey;
		b2DestroyContact(world, contact, wakeBodies);
	}

	// Destroy the attached shapes and their broad-phase proxies.
	int shapeId = staticBody->shapeList.headShapeId;
	while (shapeId != B2_NULL_INDEX)
	{
		b2Shape* shape = world->shapes + shapeId;
		shapeId = shape->nextShapeId;

		b2DestroyShapeProxy(shape, &world->broadPhase);
		b2FreeObject(&world->shapePool, &shape->object);
	}

	// Delete the attached chains. The associated shapes have already been deleted above.
	int chainId = staticBody->chainList.headChainId;
	while (chainId != B2_NULL_INDEX)
	{
		b2ChainShape* chain = world->chains + chainId;
		chainId = chain->nextIndex;

		b2Free(chain->shapeIndices, chain->count * sizeof(int));
		chain->shapeIndices = NULL;
		b2FreeObject(&world->chainPool, &chain->object);
	}

	// return to free list
	staticBody->bodyId = B2_NULL_INDEX;
	b2FreeId(&world->staticBodyIdPool, id);

	b2ValidateWorld(world);
}

b2Transform b2StaticBody_GetTransform(b2StaticBodyId bodyId)
{
	b2World* world = b2GetWorld(bodyId.world0);
	b2StaticBody* body = b2GetStaticBodyFullId(world, bodyId);
	return body->transform;
}

void b2StaticBody_SetUserData(b2StaticBodyId bodyId, void* userData)
{
	b2World* world = b2GetWorld(bodyId.world0);
	b2StaticBody* body = b2GetStaticBodyFullId(world, bodyId);
	body->userData = userData;
}

void* b2StaticBody_GetUserData(b2StaticBodyId bodyId)
{
	b2World* world = b2GetWorld(bodyId.world0);
	b2StaticBody* body = b2GetStaticBodyFullId(world, bodyId);
	return body->userData;
}

int b2StaticBody_GetShapeCount(b2StaticBodyId bodyId)
{
	b2World* world = b2GetWorld(bodyId.world0);
	b2StaticBody* body = b2GetStaticBodyFullId(world, bodyId);
	return body->shapeList.shapeCount;
}

void b2StaticBody_GetShapes(b2StaticBodyId bodyId, b2ShapeId* shapeArray, int capacity)
{
	b2World* world = b2GetWorld(bodyId.world0);
	b2StaticBody* body = b2GetStaticBodyFullId(world, bodyId);
	int shapeId = body->shapeList.headShapeId;
	int shapeCount = 0;
	while (shapeId != B2_NULL_INDEX && shapeCount < capacity)
	{
		b2Shape* shape = world->shapes + shapeId;
		b2CheckValidObject(&shape->object);

		b2ShapeId id = {shapeId + 1, bodyId.world0, shape->object.revision};
		shapeArray[shapeCount] = id;
		shapeCount += 1;
	
		shapeId = shape->nextShapeId;
	}
}
