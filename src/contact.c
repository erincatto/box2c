// SPDX-FileCopyrightText: 2022 Erin Catto
// SPDX-License-Identifier: MIT

#include "box2d/manifold.h"

#include "block_allocator.h"
#include "body.h"
#include "contact.h"
#include "shape.h"
#include "world.h"

#include <assert.h>
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

typedef b2Manifold b2ManifoldFcn(const b2Shape* shapeA, int32_t childIndexA, b2Transform xfA, const b2Shape* shapeB, b2Transform xfB);

struct b2ContactRegister
{
	b2ManifoldFcn* fcn;
	bool primary;
};

static struct b2ContactRegister s_registers[b2_shapeTypeCount][b2_shapeTypeCount];
static bool s_initialized = false;

b2Manifold b2PolygonManifold(const b2Shape* shapeA, int32_t childIndexA, b2Transform xfA, const b2Shape* shapeB, b2Transform xfB)
{
	B2_MAYBE_UNUSED(childIndexA);
	return b2CollidePolygons(&shapeA->polygon, xfA, &shapeB->polygon, xfB);
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

static void b2InitializeRegisters()
{
	b2AddType(b2PolygonManifold, b2_polygonShape, b2_polygonShape);
}

void b2Contact_Create(b2World* world, b2Shape* shapeA, int32_t childA, b2Shape* shapeB, int32_t childB)
{
	if (s_initialized == false)
	{
		b2InitializeRegisters();
		s_initialized = true;
	}

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
		b2Contact_Create(world, shapeB, childB, shapeA, childA);
		return;
	}

	b2Contact* c = (b2Contact*)b2AllocBlock(world->blockAllocator, sizeof(b2Contact));

	c->flags = b2_contactEnabledFlag;
	c->shapeIndexA = shapeA->object.index;
	c->shapeIndexB = shapeB->object.index;
	c->childA = childA;
	c->childB = childB;

	c->manifold = b2EmptyManifold();
	c->childA = childA;
	c->childB = childB;

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
	c->edgeA = (b2ContactEdge){shapeB->object.index, c, NULL, NULL};
	c->edgeB = (b2ContactEdge){shapeA->object.index, c, NULL, NULL};

	// Connect to body A
	c->edgeA.contact = c;
	c->edgeA.otherShapeIndex = c->shapeIndexB;

	c->edgeA.prev = NULL;
	c->edgeA.next = shapeA->contacts;
	if (shapeA->contacts != NULL)
	{
		shapeA->contacts->prev = &c->edgeA;
	}
	shapeA->contacts = &c->edgeA;
	shapeA->contactCount += 1;

	// Connect to body B
	c->edgeB.contact = c;
	c->edgeB.otherShapeIndex = c->shapeIndexA;

	c->edgeB.prev = NULL;
	c->edgeB.next = shapeB->contacts;
	if (shapeB->contacts != NULL)
	{
		shapeB->contacts->prev = &c->edgeB;
	}
	shapeB->contacts = &c->edgeB;
	shapeB->contactCount += 1;
}

void b2Contact_Destroy(b2World* world, b2Contact* c)
{
	b2Shape* shapeA = world->shapes + c->shapeIndexA;
	b2Shape* shapeB = world->shapes + c->shapeIndexB;

	if (c->manifold.pointCount > 0 &&
		shapeA->isSensor == false &&
		shapeB->isSensor == false)
	{
		b2Body_SetAwake(world->bodies + shapeA->bodyIndex, true);
		b2Body_SetAwake(world->bodies + shapeB->bodyIndex, true);
	}

	//if (contactListener && c->IsTouching())
	//{
	//	contactListener->EndContact(c);
	//}

	// Remove from the world.
	if (c->prev)
	{
		c->prev->next = c->next;
	}

	if (c->next)
	{
		c->next->prev = c->prev;
	}

	if (c == world->contacts)
	{
		world->contacts = c->next;
	}

	// Remove from body A
	if (c->edgeA.prev)
	{
		c->edgeA.prev->next = c->edgeA.next;
	}

	if (c->edgeA.next)
	{
		c->edgeA.next->prev = c->edgeA.prev;
	}

	if (&c->edgeA == shapeA->contacts)
	{
		shapeA->contacts = c->edgeA.next;
	}

	shapeA->contactCount -= 1;

	// Remove from body B
	if (c->edgeB.prev)
	{
		c->edgeB.prev->next = c->edgeB.next;
	}

	if (c->edgeB.next)
	{
		c->edgeB.next->prev = c->edgeB.prev;
	}

	if (&c->edgeB == shapeB->contacts)
	{
		shapeB->contacts = c->edgeB.next;
	}

	b2FreeBlock(world->blockAllocator, c, sizeof(b2Contact));

	world->contactCount -= 1;
	assert(world->contactCount >= 0);
}

#if 0
// Update the contact manifold and touching status.
// Note: do not assume the fixture AABBs are overlapping or are valid.
void b2Contact::Update(b2ContactListener* listener)
{
	b2Manifold oldManifold = manifold;

	// Re-enable this contact.
	flags |= e_enabledFlag;

	bool touching = false;
	manifold.pointCount = 0;

	bool wasTouching = (flags & e_touchingFlag) == e_touchingFlag;

	bool sensorA = fixtureA->IsSensor();
	bool sensorB = fixtureB->IsSensor();
	bool sensor = sensorA || sensorB;

	b2Body* bodyA = fixtureA->GetBody();
	b2Body* bodyB = fixtureB->GetBody();
	const b2Shape* shapeA = fixtureA->GetShape();
	const b2Shape* shapeB = fixtureB->GetShape();

	bool noStatic = bodyA->type != b2_staticBody && bodyB->type != b2_staticBody;

	// Is this contact a sensor?
	if (sensor)
	{
		const b2Transform& xfA = bodyA->GetTransform();
		const b2Transform& xfB = bodyB->GetTransform();
		touching = b2TestOverlap(shapeA, indexA, shapeB, indexB, xfA, xfB);

		// Sensors don't generate manifolds.
	}
	else
	{
		// Compute TOI
		b2TOIInput input;
		input.proxyA.Set(shapeA, indexA);
		input.proxyB.Set(shapeB, indexB);
		input.sweepA = bodyA->GetSweep();
		input.sweepB = bodyB->GetSweep();
		input.tMax = 1.0f;

		b2TOIOutput output;
		b2TimeOfImpact(&output, &input);

		if (output.state != b2TOIOutput::e_separated || noStatic)
		{
			b2Transform xfA, xfB;
			input.sweepA.GetTransform(&xfA, output.t);
			input.sweepB.GetTransform(&xfB, output.t);

			Evaluate(&manifold, xfA, xfB);

			touching = manifold.pointCount > 0;

			// Match old contact ids to new contact ids and copy the
			// stored impulses to warm start the solver.
			for (int32 i = 0; i < manifold.pointCount; ++i)
			{
				b2ManifoldPoint* mp2 = manifold.points + i;
				mp2->normalImpulse = 0.0f;
				mp2->tangentImpulse = 0.0f;
				mp2->persisted = false;
				b2ContactID id2 = mp2->id;

				for (int32 j = 0; j < oldManifold.pointCount; ++j)
				{
					b2ManifoldPoint* mp1 = oldManifold.points + j;

					if (mp1->id.key == id2.key)
					{
						mp2->normalImpulse = mp1->normalImpulse;
						mp2->tangentImpulse = mp1->tangentImpulse;
						mp2->persisted = true;
						break;
					}
				}

				// For debugging ids
				//if (mp2->persisted == false && manifold.pointCount == oldManifold.pointCount)
				//{
				//	i += 0;
				//}
			}
		}
	}

	if (touching)
	{
		flags |= e_touchingFlag;
	}
	else
	{
		flags &= ~e_touchingFlag;
	}

	if (wasTouching == false && touching == true && listener)
	{
		listener->BeginContact(this);
	}

	if (wasTouching == true && touching == false && listener)
	{
		listener->EndContact(this);
	}

	if (sensor == false && touching && listener)
	{
		listener->PreSolve(this, &oldManifold);
	}
}
#endif
