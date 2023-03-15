// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "box2d/distance.h"
#include "box2d/manifold.h"

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
		b2CreateContact(world, shapeB, childB, shapeA, childA);
		return;
	}

	b2Contact* c = (b2Contact*)b2AllocBlock(world->blockAllocator, sizeof(b2Contact));

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
	c->islandId = 0;
	c->friction = b2MixFriction(shapeA->friction, shapeB->friction);
	c->restitution = b2MixRestitution(shapeA->restitution, shapeB->restitution);
	c->tangentSpeed = 0.0f;

	// Insert into the world
	c->prev = NULL;
	c->next = world->contacts;
	if (world->contacts != NULL)
	{
		world->contacts->prev = c;
	}
	world->contacts = c;
	world->contactCount += 1;

	// Connect to island graph
	c->edgeA = (b2ContactEdge){shapeB->bodyIndex, c, NULL, NULL};
	c->edgeB = (b2ContactEdge){shapeA->bodyIndex, c, NULL, NULL};

	b2Body* bodyA = world->bodies + shapeA->bodyIndex;
	b2Body* bodyB = world->bodies + shapeB->bodyIndex;

	// Connect to body A
	c->edgeA.contact = c;

	c->edgeA.prev = NULL;
	c->edgeA.next = bodyA->contacts;
	if (bodyA->contacts != NULL)
	{
		bodyA->contacts->prev = &c->edgeA;
	}
	bodyA->contacts = &c->edgeA;
	bodyA->contactCount += 1;

	// Connect to body B
	c->edgeB.contact = c;

	c->edgeB.prev = NULL;
	c->edgeB.next = bodyB->contacts;
	if (bodyB->contacts != NULL)
	{
		bodyB->contacts->prev = &c->edgeB;
	}
	bodyB->contacts = &c->edgeB;
	bodyB->contactCount += 1;
}

void b2DestroyContact(b2World* world, b2Contact* contact)
{
	b2Body* bodyA = world->bodies + contact->edgeB.otherBodyIndex;
	b2Body* bodyB = world->bodies + contact->edgeA.otherBodyIndex;

	// if (contactListener && contact->IsTouching())
	//{
	//	contactListener->EndContact(contact);
	// }

	// Remove from the world.
	if (contact->prev)
	{
		contact->prev->next = contact->next;
	}

	if (contact->next)
	{
		contact->next->prev = contact->prev;
	}

	if (contact == world->contacts)
	{
		world->contacts = contact->next;
	}

	// Remove from body A
	if (contact->edgeA.prev)
	{
		contact->edgeA.prev->next = contact->edgeA.next;
	}

	if (contact->edgeA.next)
	{
		contact->edgeA.next->prev = contact->edgeA.prev;
	}

	if (&contact->edgeA == bodyA->contacts)
	{
		bodyA->contacts = contact->edgeA.next;
	}

	bodyA->contactCount -= 1;

	// Remove from body B
	if (contact->edgeB.prev)
	{
		contact->edgeB.prev->next = contact->edgeB.next;
	}

	if (contact->edgeB.next)
	{
		contact->edgeB.next->prev = contact->edgeB.prev;
	}

	if (&contact->edgeB == bodyB->contacts)
	{
		bodyB->contacts = contact->edgeB.next;
	}

	b2FreeBlock(world->blockAllocator, contact, sizeof(b2Contact));

	world->contactCount -= 1;
	assert(world->contactCount >= 0);
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
			world->preSolveFcn(shapeIdA, shapeIdB, &contact->manifold, world->preSolveContext);
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
