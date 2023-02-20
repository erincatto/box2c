// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "box2d/id.h"

#include "array.h"
#include "block_allocator.h"
#include "body.h"
#include "contact.h"
#include "world.h"
#include "shape.h"

#include <assert.h>


b2BodyId b2World_CreateBody(b2WorldId worldId, const b2BodyDef* def)
{
	b2World* world = b2GetWorldFromId(worldId);
	assert(world->locked == false);

	if (world->locked)
	{
		return b2_nullBodyId;
	}

	b2Body* b = (b2Body*)b2AllocObject(&world->bodyPool);
	world->bodies = (b2Body*)world->bodyPool.memory;

	assert(0 <= def->type && def->type < b2_bodyTypeCount);
	assert(b2Vec2_IsValid(def->position));
	assert(b2IsValid(def->angle));
	assert(b2Vec2_IsValid(def->linearVelocity));
	assert(b2IsValid(def->angularVelocity));
	assert(b2IsValid(def->linearDamping) && def->linearDamping >= 0.0f);
	assert(b2IsValid(def->angularDamping) && def->angularDamping >= 0.0f);
	assert(b2IsValid(def->gravityScale) && def->gravityScale >= 0.0f);

	b->type = def->type;
	b->islandIndex = B2_NULL_INDEX;
	b->transform.p = def->position;
	b->transform.q = b2MakeRot(def->angle);
	b->position = def->position;
	b->angle = def->angle;
	b->localCenter = b2Vec2_zero;
	b->speculativePosition = def->position;
	b->speculativeAngle = def->angle;
	b->linearVelocity = def->linearVelocity;
	b->angularVelocity = def->angularVelocity;
	b->force = b2Vec2_zero;
	b->torque = 0.0f;
	b->shapeIndex = B2_NULL_INDEX;
	b->jointIndex = B2_NULL_INDEX;
	b->mass = 0.0f;
	b->invMass = 0.0f;
	b->I = 0.0f;
	b->invI = 0.0f;
	b->linearDamping = def->linearDamping;
	b->angularDamping = def->angularDamping;
	b->gravityScale = def->gravityScale;
	b->sleepTime = 0.0f;
	b->userData = def->userData;
	b->world = worldId.index;
	b->islandId = 0;
	b->isAwake = b->type == b2_staticBody ? false : def->isAwake;
	b->enableSleep = def->enableSleep;
	b->fixedRotation = def->fixedRotation;
	b->isEnabled = def->isEnabled;

	if (b->isAwake)
	{
		b->awakeIndex = b2Array(world->awakeBodies).count;
		b2Array_Push(world->awakeBodies, b->object.index);
	}
	else
	{
		b->awakeIndex = B2_NULL_INDEX;
	}

	b2BodyId id = {b->object.index, worldId.index, b->object.revision};
	return id;
}

void b2World_DestroyBody(b2BodyId bodyId)
{
	b2World* world = b2GetWorldFromIndex(bodyId.world);
	assert(world->locked == false);

	if (world->locked)
	{
		return;
	}

	assert(0 <= bodyId.index && bodyId.index < world->bodyPool.capacity);

	b2Body* body = world->bodies + bodyId.index;

#if 0
	// Delete the attached joints.
	b2JointEdge* je = b->m_jointList;
	while (je)
	{
		b2JointEdge* je0 = je;
		je = je->next;

		if (m_destructionListener)
		{
			m_destructionListener->SayGoodbye(je0->joint);
		}

		DestroyJoint(je0->joint);

		b->m_jointList = je;
	}
	b->m_jointList = nullptr;
#endif

	// Delete the attached fixtures. This destroys broad-phase proxies.
	int32_t shapeIndex = body->shapeIndex;
	while (shapeIndex != B2_NULL_INDEX)
	{
		b2Shape* shape = world->shapes + shapeIndex;
		shapeIndex = shape->nextShapeIndex;

		// Delete the attached contacts
		b2ContactEdge* ce = shape->contacts;
		while (ce)
		{
			b2Contact* contact = ce->contact;

			// TODO_ERIN EndContact callback?

			b2Shape* otherShape = world->shapes + ce->otherShapeIndex;
			if (contact->manifold.pointCount > 0 && shape->isSensor == false && otherShape->isSensor == false)
			{
				b2Body* otherBody = world->bodies + otherShape->bodyIndex;
				b2SetAwake(world, otherBody, true);
			}

			b2ContactEdge* ce0 = ce;
			ce = ce->next;

			b2DestroyContact(world, ce0->contact);
		}
		shape->contacts = NULL;

		// if (m_destructionListener)
		//{
		//	m_destructionListener->SayGoodbye(f0);
		// }

		// The broad-phase proxies only exist if the body is enabled
		if (body->isEnabled)
		{
			b2Shape_DestroyProxies(shape, &world->broadPhase);
		}

		b2FreeBlock(world->blockAllocator, shape->proxies, shape->proxyCount * sizeof(b2ShapeProxy));

		// TODO_ERIN destroy contacts

		b2FreeObject(&world->shapePool, &shape->object);
		world->shapes = (b2Shape*)world->shapePool.memory;
	}

	// Remove awake body.
	// Must do this after destroying contacts because that might wake this body.
	if (body->awakeIndex != B2_NULL_INDEX)
	{
		assert(body->isAwake);
		assert(0 <= body->awakeIndex && body->awakeIndex < b2Array(world->awakeBodies).count);
		assert(world->awakeBodies[body->awakeIndex] == body->object.index);

		// Elements of the awake list cannot be moved around, so nullify this element
		world->awakeBodies[body->awakeIndex] = B2_NULL_INDEX;
	}

	b2FreeObject(&world->bodyPool, &body->object);
}

static void b2ComputeMass(b2World* w, b2Body* b)
{
	// Compute mass data from shapes. Each shape has its own density.
	b->mass = 0.0f;
	b->invMass = 0.0f;
	b->I = 0.0f;
	b->invI = 0.0f;
	b->localCenter = b2Vec2_zero;

	// Static and kinematic bodies have zero mass.
	if (b->type == b2_staticBody || b->type == b2_kinematicBody)
	{
		b->position = b->transform.p;
		return;
	}

	assert(b->type == b2_dynamicBody);

	// Accumulate mass over all shapes.
	b2Vec2 localCenter = b2Vec2_zero;
	int32_t shapeIndex = b->shapeIndex;
	while (shapeIndex != B2_NULL_INDEX)
	{
		const b2Shape* s = w->shapes + shapeIndex;
		shapeIndex = s->nextShapeIndex;

		if (s->density == 0.0f)
		{
			continue;
		}

		b2MassData massData = b2Shape_ComputeMass(s);

		b->mass += massData.mass;
		localCenter = b2MulAdd(localCenter, massData.mass, massData.center);
		b->I += massData.I;
	}

	// Compute center of mass.
	if (b->mass > 0.0f)
	{
		b->invMass = 1.0f / b->mass;
		localCenter = b2MulSV(b->invMass, localCenter);
	}

	if (b->I > 0.0f && b->fixedRotation == false)
	{
		// Center the inertia about the center of mass.
		b->I -= b->mass * b2Dot(localCenter, localCenter);
		assert(b->I > 0.0f);
		b->invI = 1.0f / b->I;
	}
	else
	{
		b->I = 0.0f;
		b->invI = 0.0f;
	}

	// Move center of mass.
	b2Vec2 oldCenter = b->position;
	b->localCenter = localCenter;
	b->position = b2TransformPoint(b->transform, b->localCenter);

	// Update center of mass velocity.
	b2Vec2 deltaLinear = b2CrossSV(b->angularVelocity, b2Sub(b->position, oldCenter));
	b->linearVelocity = b2Add(b->linearVelocity, deltaLinear);
}

b2ShapeId b2Body_CreatePolygon(b2BodyId bodyId, const b2ShapeDef* def, const struct b2Polygon* polygon)
{
	b2World* w = b2GetWorldFromIndex(bodyId.world);
	assert(w->locked == false);
	if (w->locked)
	{
		return b2_nullShapeId;
	}

	assert(0 <= bodyId.index && bodyId.index < w->bodyPool.capacity);
	
	b2Body* body = w->bodies + bodyId.index;

	b2Shape* shape = (b2Shape*)b2AllocObject(&w->shapePool);
	w->shapes = (b2Shape*)w->shapePool.memory;

	assert(b2IsValid(def->density) && def->density >= 0.0f);
	assert(b2IsValid(def->friction) && def->friction >= 0.0f);
	assert(b2IsValid(def->restitution) && def->restitution >= 0.0f);

	shape->bodyIndex = body->object.index;
	shape->type = b2_polygonShape;
	shape->density = def->density;
	shape->friction = def->friction;
	shape->restitution = def->restitution;
	shape->filter = def->filter;
	shape->userData = def->userData;
	shape->isSensor = def->isSensor;
	shape->polygon = *polygon;
	shape->contacts = NULL;
	shape->contactCount = 0;
	shape->reportContacts = false;

	shape->proxyCount = 1;
	shape->proxies = (b2ShapeProxy*)b2AllocBlock(w->blockAllocator, sizeof(b2ShapeProxy));
	shape->proxies[0].aabb = (b2AABB){b2Vec2_zero, b2Vec2_zero};
	shape->proxies[0].childIndex = 0;
	shape->proxies[0].proxyKey = B2_NULL_INDEX;
	shape->proxies[0].shapeIndex = shape->object.index;

	if (body->isEnabled)
	{
		b2Shape_CreateProxies(shape, &w->broadPhase, body->type, body->transform);
	}

	// Add to shape linked list
	shape->nextShapeIndex = body->shapeIndex;
	body->shapeIndex = shape->object.index;

	if (shape->density)
	{
		b2ComputeMass(w, body);
	}

	w->newContacts = true;

	b2ShapeId id = {shape->object.index, bodyId.world, shape->object.revision};
	return id;
}

// Destroy a shape on a body. This doesn't need to be called when destroying a body.
void b2Body_DestroyShape(b2ShapeId shapeId)
{
	b2World* world = b2GetWorldFromIndex(shapeId.world);
	assert(world->locked == false);
	if (world->locked)
	{
		return;
	}

	assert(0 <= shapeId.index && shapeId.index < world->shapePool.count);

	b2Shape* shape = world->shapes + shapeId.index;
	assert(shape->object.index == shape->object.next);
	assert(shape->object.revision == shapeId.revision);
	assert(0 <= shape->bodyIndex && shape->bodyIndex < world->bodyPool.capacity);

	b2Body* body = world->bodies + shape->bodyIndex;

	// Remove the shape from the body's singly linked list.
	int32_t* shapeIndex = &body->shapeIndex;
	bool found = false;
	while (*shapeIndex != B2_NULL_INDEX)
	{
		if (*shapeIndex == shapeId.index)
		{
			*shapeIndex = shape->nextShapeIndex;
			found = true;
			break;
		}

		shapeIndex = &(world->shapes[*shapeIndex].nextShapeIndex);
	}

	assert(found);
	if (found == false)
	{
		return;
	}

	const float density = shape->density;

	// Destroy any contacts associated with the fixture.
	//b2ContactEdge* edge = m_contactList;
	//while (edge)
	//{
	//	b2Contact* c = edge->contact;
	//	edge = edge->next;

	//	b2Fixture* fixtureA = c->GetFixtureA();
	//	b2Fixture* fixtureB = c->GetFixtureB();

	//	if (fixture == fixtureA || fixture == fixtureB)
	//	{
	//		// This destroys the contact and removes it from
	//		// this body's contact list.
	//		m_world->m_contactManager.Destroy(c);
	//	}
	//}

	if (body->isEnabled)
	{
		b2Shape_DestroyProxies(shape, &world->broadPhase);
	}

	b2FreeBlock(world->blockAllocator, shape->proxies, shape->proxyCount * sizeof(b2ShapeProxy));

	b2FreeObject(&world->shapePool, &shape->object);

	// Reset the mass data
	if (density > 0.0f)
	{
		b2ComputeMass(world, body);
	}
}

b2Vec2 b2Body_GetPosition(b2BodyId bodyId)
{
	b2World* world = b2GetWorldFromIndex(bodyId.world);
	assert(0 <= bodyId.index && bodyId.index < world->bodyPool.capacity);
	return world->bodies[bodyId.index].transform.p;
}

float b2Body_GetAngle(b2BodyId bodyId)
{
	b2World* world = b2GetWorldFromIndex(bodyId.world);
	assert(0 <= bodyId.index && bodyId.index < world->bodyPool.capacity);
	return world->bodies[bodyId.index].angle;
}

b2Vec2 b2Body_GetLocalPoint(b2BodyId bodyId, b2Vec2 globalPoint)
{
	b2World* world = b2GetWorldFromIndex(bodyId.world);
	assert(0 <= bodyId.index && bodyId.index < world->bodyPool.capacity);
	return b2InvTransformPoint(world->bodies[bodyId.index].transform, globalPoint);
}

b2BodyType b2Body_GetType(b2BodyId bodyId)
{
	b2World* world = b2GetWorldFromIndex(bodyId.world);
	assert(0 <= bodyId.index && bodyId.index < world->bodyPool.capacity);
	return world->bodies[bodyId.index].type;
}

float b2Body_GetMass(b2BodyId bodyId)
{
	b2World* world = b2GetWorldFromIndex(bodyId.world);
	assert(0 <= bodyId.index && bodyId.index < world->bodyPool.capacity);
	return world->bodies[bodyId.index].mass;
}

void b2Body_SetAwake(b2BodyId bodyId, bool awake)
{
	b2World* world = b2GetWorldFromIndex(bodyId.world);
	assert(0 <= bodyId.index && bodyId.index < world->bodyPool.capacity);
	b2SetAwake(world, world->bodies + bodyId.index, awake);
}

// This should NOT be called from island code which is using awake array double buffering.
void b2SetAwake(b2World* world, b2Body* body, bool flag)
{
	if (body->type == b2_staticBody)
	{
		return;
	}

	if (body->isAwake == flag)
	{
		return;
	}

	body->isAwake = flag;

	if (flag)
	{
		body->sleepTime = 0.0f;
		assert(body->awakeIndex == B2_NULL_INDEX);
		body->awakeIndex = b2Array(world->awakeBodies).count;
		b2Array_Push(world->awakeBodies, body->object.index);
	}
	else
	{
		body->sleepTime = 0.0f;
		body->linearVelocity = b2Vec2_zero;
		body->angularVelocity = 0.0f;
		body->force = b2Vec2_zero;
		body->torque = 0.0f;

		assert(0 <= body->awakeIndex && body->awakeIndex < b2Array(world->awakeBodies).count);
		world->awakeBodies[body->awakeIndex] = B2_NULL_INDEX;
		body->awakeIndex = B2_NULL_INDEX;
	}
}

#if 0
void b2Body::SetType(b2BodyType type)
{
	b2Assert(m_world->IsLocked() == false);
	if (m_world->IsLocked() == true)
	{
		return;
	}

	if (m_type == type)
	{
		return;
	}

	m_type = type;

	ResetMassData();

	if (m_type == b2_staticBody)
	{
		m_linearVelocity.SetZero();
		m_angularVelocity = 0.0f;
		m_sweep.a0 = m_sweep.a;
		m_sweep.c0 = m_sweep.c;
		m_flags &= ~e_awakeFlag;
		SynchronizeFixtures();
	}

	SetAwake(true);

	m_force.SetZero();
	m_torque = 0.0f;

	// Delete the attached contacts.
	b2ContactEdge* ce = m_contactList;
	while (ce)
	{
		b2ContactEdge* ce0 = ce;
		ce = ce->next;
		m_world->m_contactManager.Destroy(ce0->contact);
	}
	m_contactList = nullptr;

	// Touch the proxies so that new contacts will be created (when appropriate)
	b2BroadPhase* broadPhase = &m_world->m_contactManager.m_broadPhase;
	for (b2Fixture* f = m_fixtureList; f; f = f->m_next)
	{
		int32 proxyCount = f->m_proxyCount;
		for (int32 i = 0; i < proxyCount; ++i)
		{
			broadPhase->TouchProxy(f->m_proxies[i].proxyId);
		}
	}
}

b2Fixture* b2Body::CreateFixture(const b2FixtureDef* def)
{
	b2Assert(m_world->IsLocked() == false);
	if (m_world->IsLocked() == true)
	{
		return nullptr;
	}

	b2BlockAllocator* allocator = &m_world->m_blockAllocator;

	void* memory = allocator->Allocate(sizeof(b2Fixture));
	b2Fixture* fixture = new (memory) b2Fixture;
	fixture->Create(allocator, this, def);

	if (m_flags & e_enabledFlag)
	{
		b2BroadPhase* broadPhase = &m_world->m_contactManager.m_broadPhase;
		fixture->CreateProxies(broadPhase, m_xf);
	}

	fixture->m_next = m_fixtureList;
	m_fixtureList = fixture;
	++m_fixtureCount;

	fixture->m_body = this;

	// Adjust mass properties if needed.
	if (fixture->m_density > 0.0f)
	{
		ResetMassData();
	}

	// Let the world know we have a new fixture. This will cause new contacts
	// to be created at the beginning of the next time step.
	m_world->m_newContacts = true;

	return fixture;
}

b2Fixture* b2Body::CreateFixture(const b2Shape* shape, float density)
{
	b2FixtureDef def;
	def.shape = shape;
	def.density = density;

	return CreateFixture(&def);
}



void b2Body::ResetMassData()
{
	// Compute mass data from shapes. Each shape has its own density.
	m_mass = 0.0f;
	m_invMass = 0.0f;
	m_I = 0.0f;
	m_invI = 0.0f;
	m_sweep.localCenter.SetZero();

	// Static and kinematic bodies have zero mass.
	if (m_type == b2_staticBody || m_type == b2_kinematicBody)
	{
		m_sweep.c0 = m_xf.p;
		m_sweep.c = m_xf.p;
		m_sweep.a0 = m_sweep.a;
		return;
	}

	b2Assert(m_type == b2_dynamicBody);

	// Accumulate mass over all fixtures.
	b2Vec2 localCenter = b2Vec2_zero;
	for (b2Fixture* f = m_fixtureList; f; f = f->m_next)
	{
		if (f->m_density == 0.0f)
		{
			continue;
		}

		b2MassData massData;
		f->GetMassData(&massData);
		m_mass += massData.mass;
		localCenter += massData.mass * massData.center;
		m_I += massData.I;
	}

	// Compute center of mass.
	if (m_mass > 0.0f)
	{
		m_invMass = 1.0f / m_mass;
		localCenter *= m_invMass;
	}

	if (m_I > 0.0f && (m_flags & e_fixedRotationFlag) == 0)
	{
		// Center the inertia about the center of mass.
		m_I -= m_mass * b2Dot(localCenter, localCenter);
		b2Assert(m_I > 0.0f);
		m_invI = 1.0f / m_I;

	}
	else
	{
		m_I = 0.0f;
		m_invI = 0.0f;
	}

	// Move center of mass.
	b2Vec2 oldCenter = m_sweep.c;
	m_sweep.localCenter = localCenter;
	m_sweep.c0 = m_sweep.c = b2Mul(m_xf, m_sweep.localCenter);

	// Update center of mass velocity.
	m_linearVelocity += b2Cross(m_angularVelocity, m_sweep.c - oldCenter);
}

void b2Body::SetMassData(const b2MassData* massData)
{
	b2Assert(m_world->IsLocked() == false);
	if (m_world->IsLocked() == true)
	{
		return;
	}

	if (m_type != b2_dynamicBody)
	{
		return;
	}

	m_invMass = 0.0f;
	m_I = 0.0f;
	m_invI = 0.0f;

	m_mass = massData->mass;
	if (m_mass <= 0.0f)
	{
		m_mass = 1.0f;
	}

	m_invMass = 1.0f / m_mass;

	if (massData->I > 0.0f && (m_flags & b2Body::e_fixedRotationFlag) == 0)
	{
		m_I = massData->I - m_mass * b2Dot(massData->center, massData->center);
		b2Assert(m_I > 0.0f);
		m_invI = 1.0f / m_I;
	}

	// Move center of mass.
	b2Vec2 oldCenter = m_sweep.c;
	m_sweep.localCenter =  massData->center;
	m_sweep.c0 = m_sweep.c = b2Mul(m_xf, m_sweep.localCenter);

	// Update center of mass velocity.
	m_linearVelocity += b2Cross(m_angularVelocity, m_sweep.c - oldCenter);
}

bool b2Body::ShouldCollide(const b2Body* other) const
{
	// At least one body should be dynamic.
	if (m_type != b2_dynamicBody && other->m_type != b2_dynamicBody)
	{
		return false;
	}

	// Does a joint prevent collision?
	for (b2JointEdge* jn = m_jointList; jn; jn = jn->next)
	{
		if (jn->other == other)
		{
			if (jn->joint->m_collideConnected == false)
			{
				return false;
			}
		}
	}

	return true;
}

void b2Body::SetTransform(const b2Vec2& position, float angle)
{
	b2Assert(m_world->IsLocked() == false);
	if (m_world->IsLocked() == true)
	{
		return;
	}

	m_xf.q.Set(angle);
	m_xf.p = position;

	m_sweep.c = b2Mul(m_xf, m_sweep.localCenter);
	m_sweep.a = angle;

	m_sweep.c0 = m_sweep.c;
	m_sweep.a0 = angle;

	b2BroadPhase* broadPhase = &m_world->m_contactManager.m_broadPhase;
	for (b2Fixture* f = m_fixtureList; f; f = f->m_next)
	{
		f->Synchronize(broadPhase, m_xf, m_xf);
	}

	// Check for new contacts the next step
	m_world->m_newContacts = true;
}

void b2Body::SynchronizeFixtures()
{
	b2BroadPhase* broadPhase = &m_world->m_contactManager.m_broadPhase;

	if (m_flags & b2Body::e_awakeFlag)
	{
		b2Transform xf1;
		xf1.q.Set(m_sweep.a0);
		xf1.p = m_sweep.c0 - b2Mul(xf1.q, m_sweep.localCenter);

		for (b2Fixture* f = m_fixtureList; f; f = f->m_next)
		{
			f->Synchronize(broadPhase, xf1, m_xf);
		}
	}
	else
	{
		for (b2Fixture* f = m_fixtureList; f; f = f->m_next)
		{
			f->Synchronize(broadPhase, m_xf, m_xf);
		}
	}
}

void b2Body::SetEnabled(bool flag)
{
	b2Assert(m_world->IsLocked() == false);

	if (flag == IsEnabled())
	{
		return;
	}

	if (flag)
	{
		m_flags |= e_enabledFlag;

		// Create all proxies.
		b2BroadPhase* broadPhase = &m_world->m_contactManager.m_broadPhase;
		for (b2Fixture* f = m_fixtureList; f; f = f->m_next)
		{
			f->CreateProxies(broadPhase, m_xf);
		}

		// Contacts are created at the beginning of the next
		m_world->m_newContacts = true;
	}
	else
	{
		m_flags &= ~e_enabledFlag;

		// Destroy all proxies.
		b2BroadPhase* broadPhase = &m_world->m_contactManager.m_broadPhase;
		for (b2Fixture* f = m_fixtureList; f; f = f->m_next)
		{
			f->DestroyProxies(broadPhase);
		}

		// Destroy the attached contacts.
		b2ContactEdge* ce = m_contactList;
		while (ce)
		{
			b2ContactEdge* ce0 = ce;
			ce = ce->next;
			m_world->m_contactManager.Destroy(ce0->contact);
		}
		m_contactList = nullptr;
	}
}

void b2Body::SetFixedRotation(bool flag)
{
	bool status = (m_flags & e_fixedRotationFlag) == e_fixedRotationFlag;
	if (status == flag)
	{
		return;
	}

	if (flag)
	{
		m_flags |= e_fixedRotationFlag;
	}
	else
	{
		m_flags &= ~e_fixedRotationFlag;
	}

	m_angularVelocity = 0.0f;

	ResetMassData();
}

void b2Body_Dump(b2Body* b)
{
	int32_t bodyIndex = b->islandIndex;

	// %.9g is sufficient to save and load the same value using text
	// FLT_DECIMAL_DIG == 9

	b2Dump("{\n");
	b2Dump("  b2BodyDef bd;\n");
	b2Dump("  bd.type = b2BodyType(%d);\n", b->type);
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
	for (b2Fixture* f = m_fixtureList; f; f = f->m_next)
	{
		b2Dump("  {\n");
		f->Dump(bodyIndex);
		b2Dump("  }\n");
	}
	b2Dump("}\n");
}
#endif
