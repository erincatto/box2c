// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "box2d/distance.h"
#include "box2d/manifold.h"
#include "box2d/timer.h"

#include "array.h"
#include "block_allocator.h"
#include "body.h"
#include "contact.h"
#include "shape.h"
#include "world.h"

#include <assert.h>
#include <float.h>
#include <math.h>

// Friction mixing law. The idea is to allow either fixture to drive the friction to zero.
// For example, anything slides on ice.
static inline float b2MixFriction(float friction1, float friction2)
{
	return sqrtf(friction1 * friction2);
}

// Restitution mixing law. The idea is allow for anything to bounce off an inelastic surface.
// For example, a superball bounces on anything.
static inline float b2MixRestitution(float restitution1, float restitution2)
{
	return restitution1 > restitution2 ? restitution1 : restitution2;
}

typedef b2Manifold b2ManifoldFcn(const b2Shape* shapeA, int32_t childIndexA, b2Transform xfA, const b2Shape* shapeB,
								 b2Transform xfB, b2DistanceCache* cache);

struct b2ContactRegister
{
	b2ManifoldFcn* fcn;
	bool primary;
};

static struct b2ContactRegister s_registers[b2_shapeTypeCount][b2_shapeTypeCount];
static bool s_initialized = false;

static b2Manifold b2CircleManifold(const b2Shape* shapeA, int32_t childIndexA, b2Transform xfA, const b2Shape* shapeB,
								   b2Transform xfB, b2DistanceCache* cache)
{
	B2_MAYBE_UNUSED(childIndexA);
	B2_MAYBE_UNUSED(cache);
	return b2CollideCircles(&shapeA->circle, xfA, &shapeB->circle, xfB);
}

static b2Manifold b2PolygonAndCircleManifold(const b2Shape* shapeA, int32_t childIndexA, b2Transform xfA,
											 const b2Shape* shapeB, b2Transform xfB, b2DistanceCache* cache)
{
	B2_MAYBE_UNUSED(childIndexA);
	B2_MAYBE_UNUSED(cache);
	return b2CollidePolygonAndCircle(&shapeA->polygon, xfA, &shapeB->circle, xfB);
}

static b2Manifold b2PolygonManifold(const b2Shape* shapeA, int32_t childIndexA, b2Transform xfA, const b2Shape* shapeB,
									b2Transform xfB, b2DistanceCache* cache)
{
	B2_MAYBE_UNUSED(childIndexA);
	return b2CollidePolygons(&shapeA->polygon, xfA, &shapeB->polygon, xfB, cache);
}

static void b2AddType(b2ManifoldFcn* fcn, enum b2ShapeType type1, enum b2ShapeType type2)
{
	assert(0 <= type1 && type1 < b2_shapeTypeCount);
	assert(0 <= type2 && type2 < b2_shapeTypeCount);

	s_registers[type1][type2].fcn = fcn;
	s_registers[type1][type2].primary = true;

	if (type1 != type2)
	{
		s_registers[type2][type1].fcn = fcn;
		s_registers[type2][type1].primary = false;
	}
}

void b2InitializeContactRegisters()
{
	if (s_initialized == false)
	{
		b2AddType(b2CircleManifold, b2_circleShape, b2_circleShape);
		b2AddType(b2PolygonAndCircleManifold, b2_polygonShape, b2_circleShape);
		b2AddType(b2PolygonManifold, b2_polygonShape, b2_polygonShape);
		s_initialized = true;
	}
}

void b2CreateContact(b2World* world, b2Shape* shapeA, int32_t childA, b2Shape* shapeB, int32_t childB)
{
	b2ShapeType type1 = shapeA->type;
	b2ShapeType type2 = shapeB->type;

	assert(0 <= type1 && type1 < b2_shapeTypeCount);
	assert(0 <= type2 && type2 < b2_shapeTypeCount);

	if (s_registers[type1][type2].fcn == NULL)
	{
		return;
	}

	if (s_registers[type1][type2].primary == false)
	{
		// flip order
		b2CreateContact(world, shapeB, childB, shapeA, childA);
		return;
	}

	b2Contact* c = (b2Contact*)b2AllocObject(&world->contactPool);
	world->contacts = (b2Contact*)world->contactPool.memory;

	c->flags = b2_contactEnabledFlag;

	if (shapeA->isSensor || shapeB->isSensor)
	{
		c->flags |= b2_contactSensorFlag;
	}

	c->shapeIndexA = shapeA->object.index;
	c->shapeIndexB = shapeB->object.index;
	c->childA = childA;
	c->childB = childB;
	c->cache = b2_emptyDistanceCache;
	c->manifold = b2_emptyManifold;
	c->islandIndex = B2_NULL_INDEX;
	c->awakeIndex = B2_NULL_INDEX;
	c->friction = b2MixFriction(shapeA->friction, shapeB->friction);
	c->restitution = b2MixRestitution(shapeA->restitution, shapeB->restitution);
	c->tangentSpeed = 0.0f;

	// Connect to body A
	{
		b2Body* bodyA = world->bodies + shapeA->bodyIndex;
		c->edges[0].bodyIndex = shapeA->bodyIndex;
		c->edges[0].prevKey = B2_NULL_INDEX;
		c->edges[0].nextKey = bodyA->contactList;

		int32_t keyA = (c->object.index << 1) | 0;
		if (bodyA->contactList != B2_NULL_INDEX)
		{
			b2Contact* contactA = world->contacts + (bodyA->contactList >> 1);
			b2ContactEdge* edgeA = contactA->edges + (bodyA->contactList & 1);
			edgeA->prevKey = keyA;
		}
		bodyA->contactList = keyA;
		bodyA->contactCount += 1;
	}

	// Connect to body B
	{
		b2Body* bodyB = world->bodies + shapeB->bodyIndex;
		c->edges[1].bodyIndex = shapeB->bodyIndex;
		c->edges[1].prevKey = B2_NULL_INDEX;
		c->edges[1].nextKey = bodyB->contactList;

		int32_t keyB = (c->object.index << 1) | 1;
		if (bodyB->contactList != B2_NULL_INDEX)
		{
			b2Contact* contactB = world->contacts + (bodyB->contactList >> 1);
			b2ContactEdge* edgeB = contactB->edges + (bodyB->contactList & 1);
			edgeB->prevKey = keyB;
		}
		bodyB->contactList = keyB;
		bodyB->contactCount += 1;
	}

	// TODO_ERIN add to island?
}

void b2DestroyContact(b2World* world, b2Contact* contact)
{
	b2ContactEdge* edgeA = contact->edges + 0;
	b2ContactEdge* edgeB = contact->edges + 1;

	b2Body* bodyA = world->bodies + edgeA->bodyIndex;
	b2Body* bodyB = world->bodies + edgeB->bodyIndex;

	// if (contactListener && contact->IsTouching())
	//{
	//	contactListener->EndContact(contact);
	// }

	// Remove from body A
	if (edgeA->prevKey != B2_NULL_INDEX)
	{
		b2Contact* prevContact = world->contacts + (edgeA->prevKey >> 1);
		b2ContactEdge* prevEdge = prevContact->edges + (edgeA->prevKey & 1);
		prevEdge->nextKey = edgeA->nextKey;
	}

	if (edgeA->nextKey != B2_NULL_INDEX)
	{
		b2Contact* nextContact = world->contacts + (edgeA->nextKey >> 1);
		b2ContactEdge* nextEdge = nextContact->edges + (edgeA->nextKey & 1);
		nextEdge->prevKey = edgeA->prevKey;
	}

	int32_t edgeKeyA = (contact->object.index << 1) | 0;
	if (bodyA->contactList == edgeKeyA)
	{
		bodyA->contactList = edgeA->nextKey;
	}

	bodyA->contactCount -= 1;

	// Remove from body B
	if (edgeB->prevKey != B2_NULL_INDEX)
	{
		b2Contact* prevContact = world->contacts + (edgeB->prevKey >> 1);
		b2ContactEdge* prevEdge = prevContact->edges + (edgeB->prevKey & 1);
		prevEdge->nextKey = edgeB->nextKey;
	}

	if (edgeB->nextKey != B2_NULL_INDEX)
	{
		b2Contact* nextContact = world->contacts + (edgeB->nextKey >> 1);
		b2ContactEdge* nextEdge = nextContact->edges + (edgeB->nextKey & 1);
		nextEdge->prevKey = edgeB->prevKey;
	}

	int32_t edgeKeyB = (contact->object.index << 1) | 1;
	if (bodyB->contactList == edgeKeyB)
	{
		bodyB->contactList = edgeB->nextKey;
	}

	bodyB->contactCount -= 1;

	// TODO_ERIN remove from island?

	b2FreeObject(&world->contactPool, &contact->object);
}

bool b2ShouldCollide(b2Filter filterA, b2Filter filterB)
{
	if (filterA.groupIndex == filterB.groupIndex && filterA.groupIndex != 0)
	{
		return filterA.groupIndex > 0;
	}

	bool collide = (filterA.maskBits & filterB.categoryBits) != 0 && (filterA.categoryBits & filterB.maskBits) != 0;
	return collide;
}

static bool b2TestShapeOverlap(const b2Shape* shapeA, int32_t childA, b2Transform xfA, const b2Shape* shapeB,
							   int32_t childB, b2Transform xfB)
{
	b2DistanceInput input;
	input.proxyA = b2Shape_MakeDistanceProxy(shapeA, childA);
	input.proxyB = b2Shape_MakeDistanceProxy(shapeB, childB);
	input.transformA = xfA;
	input.transformB = xfB;
	input.useRadii = true;

	b2DistanceCache cache = {0};
	b2DistanceOutput output = b2ShapeDistance(&cache, &input);

	return output.distance < 10.0f * FLT_EPSILON;
}

// Update the contact manifold and touching status.
// Note: do not assume the fixture AABBs are overlapping or are valid.
void b2Contact_Update(b2World* world, b2Contact* contact, b2Shape* shapeA, b2Body* bodyA, b2Shape* shapeB,
					  b2Body* bodyB)
{
	b2Manifold oldManifold = contact->manifold;

	assert(shapeA->object.index == contact->shapeIndexA);
	assert(shapeB->object.index == contact->shapeIndexB);

	b2ShapeId shapeIdA = {shapeA->object.index, world->index, shapeA->object.revision};
	b2ShapeId shapeIdB = {shapeB->object.index, world->index, shapeB->object.revision};

	// Re-enable this contact.
	contact->flags |= b2_contactEnabledFlag;

	bool touching = false;
	contact->manifold.pointCount = 0;

	// bool wasTouching = (contact->flags & b2_contactTouchingFlag) == b2_contactTouchingFlag;

	bool sensorA = shapeA->isSensor;
	bool sensorB = shapeB->isSensor;
	bool sensor = sensorA || sensorB;

	int32_t childA = contact->childA;
	int32_t childB = contact->childB;

	// Is this contact a sensor?
	if (sensor)
	{
		touching = b2TestShapeOverlap(shapeA, childA, bodyA->transform, shapeB, childB, bodyB->transform);

		// Sensors don't generate manifolds.
	}
	else
	{
		// Compute TOI
		b2ManifoldFcn* fcn = s_registers[shapeA->type][shapeB->type].fcn;

		contact->manifold = fcn(shapeA, childA, bodyA->transform, shapeB, bodyB->transform, &contact->cache);

		touching = contact->manifold.pointCount > 0;

		// Match old contact ids to new contact ids and copy the
		// stored impulses to warm start the solver.
		for (int32_t i = 0; i < contact->manifold.pointCount; ++i)
		{
			b2ManifoldPoint* mp2 = contact->manifold.points + i;
			mp2->normalImpulse = 0.0f;
			mp2->tangentImpulse = 0.0f;
			mp2->persisted = false;
			uint16_t id2 = mp2->id;

			for (int32_t j = 0; j < oldManifold.pointCount; ++j)
			{
				b2ManifoldPoint* mp1 = oldManifold.points + j;

				if (mp1->id == id2)
				{
					mp2->normalImpulse = mp1->normalImpulse;
					mp2->tangentImpulse = mp1->tangentImpulse;
					mp2->persisted = true;
					break;
				}
			}

			// For debugging ids
			// if (mp2->persisted == false && contact->manifold.pointCount == oldManifold.pointCount)
			//{
			//	i += 0;
			//}
		}

		if (touching && world->preSolveFcn)
		{
			// TODO_ERIN this call assumes thread safety
			bool collide = world->preSolveFcn(shapeIdA, shapeIdB, &contact->manifold, world->preSolveContext);
			if (collide == false)
			{
				// disable contact
				contact->flags &= ~b2_contactEnabledFlag;
			}
		}
	}

	if (touching)
	{
		contact->flags |= b2_contactTouchingFlag;
	}
	else
	{
		contact->flags &= ~b2_contactTouchingFlag;
	}

	// if (wasTouching == false && touching == true && world->callbacks.beginContactFcn)
	//{
	//	world->callbacks.beginContactFcn(shapeIdA, shapeIdB);
	// }

	// if (wasTouching == true && touching == false && world->callbacks.endContactFcn)
	//{
	//	world->callbacks.endContactFcn(shapeIdA, shapeIdB);
	// }
}
