// SPDX-FileCopyrightText: 2022 Erin Catto
// SPDX-License-Identifier: MIT

#include "block_allocator.h"
#include "contact.h"
#include "shape.h"




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

static void b2Contact_AddType(b2ManifoldFcn* fcn, enum b2ShapeType type1, enum b2ShapeType type2)
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

static void b2Contact_InitializeRegisters()
{
	AddType(b2PolygonManifold, b2_polygonShape, b2_polygonShape);
}


b2Contact* b2Contact::Create(b2Fixture* fixtureA, int32 indexA, b2Fixture* fixtureB, int32 indexB, b2BlockAllocator* allocator)
{
	if (s_initialized == false)
	{
		InitializeRegisters();
		s_initialized = true;
	}

	b2Shape::Type type1 = fixtureA->GetType();
	b2Shape::Type type2 = fixtureB->GetType();

	b2Assert(0 <= type1 && type1 < b2Shape::e_typeCount);
	b2Assert(0 <= type2 && type2 < b2Shape::e_typeCount);
	
	b2ContactCreateFcn* createFcn = s_registers[type1][type2].createFcn;
	if (createFcn)
	{
		if (s_registers[type1][type2].primary)
		{
			return createFcn(fixtureA, indexA, fixtureB, indexB, allocator);
		}
		else
		{
			return createFcn(fixtureB, indexB, fixtureA, indexA, allocator);
		}
	}
	else
	{
		return nullptr;
	}
}

void b2Contact::Destroy(b2Contact* contact, b2BlockAllocator* allocator)
{
	b2Assert(s_initialized == true);

	b2Fixture* fixtureA = contact->m_fixtureA;
	b2Fixture* fixtureB = contact->m_fixtureB;

	if (contact->m_manifold.pointCount > 0 &&
		fixtureA->IsSensor() == false &&
		fixtureB->IsSensor() == false)
	{
		fixtureA->GetBody()->SetAwake(true);
		fixtureB->GetBody()->SetAwake(true);
	}

	b2Shape::Type typeA = fixtureA->GetType();
	b2Shape::Type typeB = fixtureB->GetType();

	b2Assert(0 <= typeA && typeA < b2Shape::e_typeCount);
	b2Assert(0 <= typeB && typeB < b2Shape::e_typeCount);

	b2ContactDestroyFcn* destroyFcn = s_registers[typeA][typeB].destroyFcn;
	destroyFcn(contact, allocator);
}

b2Contact_(b2Fixture* fA, int32 indexA, b2Fixture* fB, int32 indexB)
{
	m_flags = e_enabledFlag;

	m_fixtureA = fA;
	m_fixtureB = fB;

	m_indexA = indexA;
	m_indexB = indexB;

	m_manifold.pointCount = 0;

	m_prev = nullptr;
	m_next = nullptr;

	m_nodeA.contact = nullptr;
	m_nodeA.prev = nullptr;
	m_nodeA.next = nullptr;
	m_nodeA.other = nullptr;

	m_nodeB.contact = nullptr;
	m_nodeB.prev = nullptr;
	m_nodeB.next = nullptr;
	m_nodeB.other = nullptr;

	m_toiCount = 0;

	m_friction = b2MixFriction(m_fixtureA->m_friction, m_fixtureB->m_friction);
	m_restitution = b2MixRestitution(m_fixtureA->m_restitution, m_fixtureB->m_restitution);
	m_restitutionThreshold = b2MixRestitutionThreshold(m_fixtureA->m_restitutionThreshold, m_fixtureB->m_restitutionThreshold);

	m_tangentSpeed = 0.0f;
}

// Update the contact manifold and touching status.
// Note: do not assume the fixture AABBs are overlapping or are valid.
void b2Contact::Update(b2ContactListener* listener)
{
	b2Manifold oldManifold = m_manifold;

	// Re-enable this contact.
	m_flags |= e_enabledFlag;

	bool touching = false;
	m_manifold.pointCount = 0;

	bool wasTouching = (m_flags & e_touchingFlag) == e_touchingFlag;

	bool sensorA = m_fixtureA->IsSensor();
	bool sensorB = m_fixtureB->IsSensor();
	bool sensor = sensorA || sensorB;

	b2Body* bodyA = m_fixtureA->GetBody();
	b2Body* bodyB = m_fixtureB->GetBody();
	const b2Shape* shapeA = m_fixtureA->GetShape();
	const b2Shape* shapeB = m_fixtureB->GetShape();

	bool noStatic = bodyA->m_type != b2_staticBody && bodyB->m_type != b2_staticBody;

	// Is this contact a sensor?
	if (sensor)
	{
		const b2Transform& xfA = bodyA->GetTransform();
		const b2Transform& xfB = bodyB->GetTransform();
		touching = b2TestOverlap(shapeA, m_indexA, shapeB, m_indexB, xfA, xfB);

		// Sensors don't generate manifolds.
	}
	else
	{
		// Compute TOI
		b2TOIInput input;
		input.proxyA.Set(shapeA, m_indexA);
		input.proxyB.Set(shapeB, m_indexB);
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

			Evaluate(&m_manifold, xfA, xfB);

			touching = m_manifold.pointCount > 0;

			// Match old contact ids to new contact ids and copy the
			// stored impulses to warm start the solver.
			for (int32 i = 0; i < m_manifold.pointCount; ++i)
			{
				b2ManifoldPoint* mp2 = m_manifold.points + i;
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
				//if (mp2->persisted == false && m_manifold.pointCount == oldManifold.pointCount)
				//{
				//	i += 0;
				//}
			}
		}
	}

	if (touching)
	{
		m_flags |= e_touchingFlag;
	}
	else
	{
		m_flags &= ~e_touchingFlag;
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
