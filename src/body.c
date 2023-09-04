// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "box2d/aabb.h"
#include "box2d/id.h"

#include "array.h"
#include "block_allocator.h"
#include "body.h"
#include "contact.h"
#include "core.h"
#include "graph.h"
#include "island.h"
#include "joint.h"
#include "world.h"
#include "shape.h"

b2BodyId b2World_CreateBody(b2WorldId worldId, const b2BodyDef* def)
{
	b2World* world = b2GetWorldFromId(worldId);
	B2_ASSERT(world->locked == false);

	if (world->locked)
	{
		return b2_nullBodyId;
	}

	b2Body* b = (b2Body*)b2AllocObject(&world->bodyPool);
	world->bodies = (b2Body*)world->bodyPool.memory;

	B2_ASSERT(0 <= def->type && def->type < b2_bodyTypeCount);
	B2_ASSERT(b2IsValidVec2(def->position));
	B2_ASSERT(b2IsValid(def->angle));
	B2_ASSERT(b2IsValidVec2(def->linearVelocity));
	B2_ASSERT(b2IsValid(def->angularVelocity));
	B2_ASSERT(b2IsValid(def->linearDamping) && def->linearDamping >= 0.0f);
	B2_ASSERT(b2IsValid(def->angularDamping) && def->angularDamping >= 0.0f);
	B2_ASSERT(b2IsValid(def->gravityScale) && def->gravityScale >= 0.0f);

	b->type = def->type;
	b->transform.p = def->position;
	b->transform.q = b2MakeRot(def->angle);
	b->position0 = def->position;
	b->position = def->position;
	b->angle0 = def->angle;
	b->angle = def->angle;
	b->localCenter = b2Vec2_zero;
	b->linearVelocity = def->linearVelocity;
	b->angularVelocity = def->angularVelocity;
	b->deltaPosition = b2Vec2_zero;
	b->deltaAngle = 0.0f;
	b->deltaPositionIter = b2Vec2_zero;
	b->deltaAngleIter = 0.0f;
	b->force = b2Vec2_zero;
	b->torque = 0.0f;
	b->shapeList = B2_NULL_INDEX;
	b->jointList = B2_NULL_INDEX;
	b->jointCount = 0;
	b->contactList = B2_NULL_INDEX;
	b->contactCount = 0;
	b->mass = 0.0f;
	b->invMass = 0.0f;
	b->I = 0.0f;
	b->invI = 0.0f;
	b->minExtent = b2_huge;
	b->linearDamping = def->linearDamping;
	b->angularDamping = def->angularDamping;
	b->gravityScale = def->gravityScale;
	b->sleepTime = 0.0f;
	b->userData = def->userData;
	b->world = worldId.index;
	b->islandIndex = 0;
	b->enableSleep = def->enableSleep;
	b->fixedRotation = def->fixedRotation;
	b->isEnabled = def->isEnabled;
	b->isMarked = false;
	b->enlargeAABB = false;
	b->isFast = false;
	b->islandIndex = B2_NULL_INDEX;
	b->islandPrev = B2_NULL_INDEX;
	b->islandNext = B2_NULL_INDEX;

	if (b->type != b2_staticBody)
	{
		// Every new body gets a new island. Islands get merged during simulation.
		b2Island* island = (b2Island*)b2AllocObject(&world->islandPool);
		world->islands = (b2Island*)world->islandPool.memory;
		b2CreateIsland(island);
		island->world = world;

		b->islandIndex = island->object.index;
		island->headBody = b->object.index;
		island->tailBody = b->object.index;
		island->bodyCount = 1;

		if (def->isAwake)
		{
			island->awakeIndex = b2Array(world->awakeIslandArray).count;
			b2Array_Push(world->awakeIslandArray, island->object.index);
		}
	}

	b2BodyId id = {b->object.index, worldId.index, b->object.revision};
	return id;
}

void b2World_DestroyBody(b2BodyId bodyId)
{
	b2World* world = b2GetWorldFromIndex(bodyId.world);
	B2_ASSERT(world->locked == false);

	if (world->locked)
	{
		return;
	}

	B2_ASSERT(0 <= bodyId.index && bodyId.index < world->bodyPool.capacity);

	b2Body* body = world->bodies + bodyId.index;

	// User must destroy joints before destroying bodies
	B2_ASSERT(body->jointList == B2_NULL_INDEX && body->jointCount == 0);

	// Destroy the attached contacts
	int32_t edgeKey = body->contactList;
	while (edgeKey != B2_NULL_INDEX)
	{
		int32_t contactIndex = edgeKey >> 1;
		int32_t edgeIndex = edgeKey & 1;

		int32_t twinKey = edgeKey ^ 1;
		int32_t twinIndex = twinKey & 1;

		b2Contact* contact = world->contacts + contactIndex;

		if (contact->colorIndex != B2_NULL_INDEX)
		{
			b2RemoveContactFromGraph(world, contact);
		}

		b2ContactEdge* twin = contact->edges + twinIndex;

		// Remove contact from other body's doubly linked list
		if (twin->prevKey != B2_NULL_INDEX)
		{
			b2Contact* prevContact = world->contacts + (twin->prevKey >> 1);
			b2ContactEdge* prevEdge = prevContact->edges + (twin->prevKey & 1);
			prevEdge->nextKey = twin->nextKey;
		}

		if (twin->nextKey != B2_NULL_INDEX)
		{
			b2Contact* nextContact = world->contacts + (twin->nextKey >> 1);
			b2ContactEdge* nextEdge = nextContact->edges + (twin->nextKey & 1);
			nextEdge->prevKey = twin->prevKey;
		}

		// Check other body's list head
		b2Body* other = world->bodies + twin->bodyIndex;
		if (other->contactList == twinKey)
		{
			other->contactList = twin->nextKey;
		}

		B2_ASSERT(other->contactCount > 0);
		other->contactCount -= 1;

		// Disconnect contact from island graph
		if (contact->islandIndex != B2_NULL_INDEX)
		{
			b2UnlinkContact(world, contact);
		}

		// Remove from awake contact array
		int32_t awakeIndex = world->contactAwakeIndexArray[contactIndex];
		if (awakeIndex != B2_NULL_INDEX)
		{
			B2_ASSERT(0 <= awakeIndex && awakeIndex < b2Array(world->awakeContactArray).count);
			world->awakeContactArray[awakeIndex] = B2_NULL_INDEX;
			world->contactAwakeIndexArray[contactIndex] = B2_NULL_INDEX;
		}

		// Remove pair from set
		uint64_t pairKey = B2_SHAPE_PAIR_KEY(contact->shapeIndexA, contact->shapeIndexB);
		b2RemoveKey(&world->broadPhase.pairSet, pairKey);

		b2ContactEdge* edge = contact->edges + edgeIndex;
		edgeKey = edge->nextKey;

		// Free contact
		b2FreeObject(&world->contactPool, &contact->object);
	}

	// Delete the attached shapes. This destroys broad-phase proxies.
	int32_t shapeIndex = body->shapeList;
	while (shapeIndex != B2_NULL_INDEX)
	{
		b2Shape* shape = world->shapes + shapeIndex;
		shapeIndex = shape->nextShapeIndex;

		// The broad-phase proxies only exist if the body is enabled
		if (body->isEnabled)
		{
			b2Shape_DestroyProxy(shape, &world->broadPhase);
		}

		b2FreeObject(&world->shapePool, &shape->object);
	}

	// Remove from island
	if (body->islandIndex != B2_NULL_INDEX)
	{
		b2Island* island = world->islands + body->islandIndex;

		// Fix the island's linked list of bodies
		if (body->islandPrev != B2_NULL_INDEX)
		{
			world->bodies[body->islandPrev].islandNext = body->islandNext;
		}

		if (body->islandNext != B2_NULL_INDEX)
		{
			world->bodies[body->islandNext].islandPrev = body->islandPrev;
		}

		B2_ASSERT(island->bodyCount > 0);
		island->bodyCount -= 1;
		bool islandDestroyed = false;

		if (island->headBody == body->object.index)
		{
			island->headBody = body->islandNext;

			if (island->headBody == B2_NULL_INDEX)
			{
				// Destroy empty island
				B2_ASSERT(island->tailBody == body->object.index);
				B2_ASSERT(island->bodyCount == 0);
				B2_ASSERT(island->contactCount == 0);
				B2_ASSERT(island->jointCount == 0);

				// Remove from awake islands array
				if (island->awakeIndex != B2_NULL_INDEX)
				{
					int32_t islandCount = b2Array(world->awakeIslandArray).count;
					B2_ASSERT(islandCount > 0);
					b2Array_RemoveSwap(world->awakeIslandArray, island->awakeIndex);
					if (island->awakeIndex < islandCount - 1)
					{
						// Fix awake index on swapped island
						int32_t swappedIslandIndex = world->awakeIslandArray[island->awakeIndex];
						world->islands[swappedIslandIndex].awakeIndex = island->awakeIndex;
					}
				}

				// Free the island
				b2DestroyIsland(island);
				b2FreeObject(&world->islandPool, &island->object);
				islandDestroyed = true;
			}
		}
		else if (island->tailBody == body->object.index)
		{
			island->tailBody = body->islandPrev;
		}

		if (islandDestroyed == false)
		{
			b2WakeIsland(island);
			b2ValidateIsland(island);
		}
	}

	// Free body
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
	b->minExtent = b2_huge;
	b->maxExtent = 0.0f;

	// Static and kinematic bodies have zero mass.
	if (b->type == b2_staticBody || b->type == b2_kinematicBody)
	{
		b->position = b->transform.p;
		return;
	}

	B2_ASSERT(b->type == b2_dynamicBody);

	// Accumulate mass over all shapes.
	b2Vec2 localCenter = b2Vec2_zero;
	int32_t shapeIndex = b->shapeList;
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

		b->minExtent = B2_MIN(b->minExtent, massData.minExtent);
		b->maxExtent = B2_MAX(b->maxExtent, massData.maxExtent);
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
		B2_ASSERT(b->I > 0.0f);
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

static b2ShapeId b2CreateShape(b2BodyId bodyId, const b2ShapeDef* def, const void* geometry, b2ShapeType shapeType)
{
	b2World* w = b2GetWorldFromIndex(bodyId.world);
	B2_ASSERT(w->locked == false);
	if (w->locked)
	{
		return b2_nullShapeId;
	}

	B2_ASSERT(0 <= bodyId.index && bodyId.index < w->bodyPool.capacity);

	b2Body* body = w->bodies + bodyId.index;

	b2Shape* shape = (b2Shape*)b2AllocObject(&w->shapePool);
	w->shapes = (b2Shape*)w->shapePool.memory;

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

	default:
		B2_ASSERT(false);
		break;
	}

	shape->bodyIndex = body->object.index;
	shape->type = shapeType;
	shape->density = def->density;
	shape->friction = def->friction;
	shape->restitution = def->restitution;
	shape->filter = def->filter;
	shape->userData = def->userData;
	shape->isSensor = def->isSensor;
	shape->enlargedAABB = false;
	shape->reportContacts = false;
	shape->isFast = false;

	if (body->isEnabled)
	{
		b2Shape_CreateProxy(shape, &w->broadPhase, body->type, body->transform);
	}

	// Add to shape linked list
	shape->nextShapeIndex = body->shapeList;
	body->shapeList = shape->object.index;

	if (shape->density)
	{
		b2ComputeMass(w, body);
	}

	b2ShapeId id = {shape->object.index, bodyId.world, shape->object.revision};
	return id;
}

b2ShapeId b2Body_CreateCircle(b2BodyId bodyId, const b2ShapeDef* def, const b2Circle* circle)
{
	return b2CreateShape(bodyId, def, circle, b2_circleShape);
}

b2ShapeId b2Body_CreatePolygon(b2BodyId bodyId, const b2ShapeDef* def, const b2Polygon* polygon)
{
	return b2CreateShape(bodyId, def, polygon, b2_polygonShape);
}

b2ShapeId b2Body_CreateSegment(b2BodyId bodyId, const b2ShapeDef* def, const b2Segment* segment)
{
	float lengthSqr = b2DistanceSquared(segment->point1, segment->point2);
	if (lengthSqr <= b2_linearSlop * b2_linearSlop)
	{
		B2_ASSERT(false);
		return b2_nullShapeId;
	}

	//b2Vec2 axis = b2Normalize(b2Sub(segment->point2, segment->point1));

	//b2Polygon polygon;
	//polygon.vertices[0] = segment->point1;
	//polygon.vertices[1] = segment->point2;
	//polygon.count = 2;
	//polygon.radius = 0.0f;
	//
	//polygon.normals[0] = b2RightPerp(axis);
	//polygon.normals[1] = b2Neg(polygon.normals[0]);

	return b2CreateShape(bodyId, def, segment, b2_segmentShape);
}

b2ShapeId b2Body_CreateCapsule(b2BodyId bodyId, const b2ShapeDef* def, const b2Capsule* capsule)
{
	float lengthSqr = b2DistanceSquared(capsule->point1, capsule->point2);
	if (lengthSqr <= b2_linearSlop * b2_linearSlop)
	{
		B2_ASSERT(false);
		return b2_nullShapeId;
	}

	//b2Vec2 axis = b2Normalize(b2Sub(segment->point2, segment->point1));

	//b2Polygon polygon;
	//polygon.vertices[0] = segment->point1;
	//polygon.vertices[1] = segment->point2;
	//polygon.count = 2;
	//polygon.radius = 0.0f;
	//
	//polygon.normals[0] = b2RightPerp(axis);
	//polygon.normals[1] = b2Neg(polygon.normals[0]);

	return b2CreateShape(bodyId, def, capsule, b2_capsuleShape);
}

// Destroy a shape on a body. This doesn't need to be called when destroying a body.
void b2Body_DestroyShape(b2ShapeId shapeId)
{
	b2World* world = b2GetWorldFromIndex(shapeId.world);
	B2_ASSERT(world->locked == false);
	if (world->locked)
	{
		return;
	}

	B2_ASSERT(0 <= shapeId.index && shapeId.index < world->shapePool.count);

	b2Shape* shape = world->shapes + shapeId.index;
	B2_ASSERT(shape->object.index == shape->object.next);
	B2_ASSERT(shape->object.revision == shapeId.revision);
	B2_ASSERT(0 <= shape->bodyIndex && shape->bodyIndex < world->bodyPool.capacity);

	b2Body* body = world->bodies + shape->bodyIndex;

	// Remove the shape from the body's singly linked list.
	int32_t* shapeIndex = &body->shapeList;
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

	B2_ASSERT(found);
	if (found == false)
	{
		return;
	}

	const float density = shape->density;

	// TODO_ERIN
	B2_ASSERT(false);
	// Destroy any contacts associated with the shape.
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
		b2Shape_DestroyProxy(shape, &world->broadPhase);
	}

	b2FreeObject(&world->shapePool, &shape->object);

	// Reset the mass data
	if (density > 0.0f)
	{
		b2ComputeMass(world, body);
	}
}

bool b2IsBodyAwake(b2World* world, b2Body* body)
{
	if (body->islandIndex != B2_NULL_INDEX)
	{
		b2Island* island = world->islands + body->islandIndex;
		return island->awakeIndex != B2_NULL_INDEX;
	}

	B2_ASSERT(body->type == b2_staticBody);
	return false;
}

b2Vec2 b2Body_GetPosition(b2BodyId bodyId)
{
	b2World* world = b2GetWorldFromIndex(bodyId.world);
	B2_ASSERT(0 <= bodyId.index && bodyId.index < world->bodyPool.capacity);
	return world->bodies[bodyId.index].transform.p;
}

float b2Body_GetAngle(b2BodyId bodyId)
{
	b2World* world = b2GetWorldFromIndex(bodyId.world);
	B2_ASSERT(0 <= bodyId.index && bodyId.index < world->bodyPool.capacity);
	return world->bodies[bodyId.index].angle;
}

b2Vec2 b2Body_GetLocalPoint(b2BodyId bodyId, b2Vec2 globalPoint)
{
	b2World* world = b2GetWorldFromIndex(bodyId.world);
	B2_ASSERT(0 <= bodyId.index && bodyId.index < world->bodyPool.capacity);
	return b2InvTransformPoint(world->bodies[bodyId.index].transform, globalPoint);
}

void b2Body_SetTransform(b2BodyId bodyId, b2Vec2 position, float angle)
{
	b2World* world = b2GetWorldFromIndex(bodyId.world);
	B2_ASSERT(world->locked == false);
	B2_ASSERT(0 <= bodyId.index && bodyId.index < world->bodyPool.capacity);

	b2Body* body = world->bodies + bodyId.index;

	body->transform.p = position;
	body->transform.q = b2MakeRot(angle);

	body->position = b2TransformPoint(body->transform, body->localCenter);
	body->angle = angle;

	body->position0 = body->position;
	body->angle0 = body->angle;

	b2BroadPhase* broadPhase = &world->broadPhase;

	const b2Vec2 aabbMargin = {b2_aabbMargin, b2_aabbMargin};
	int32_t shapeIndex = body->shapeList;
	while (shapeIndex != B2_NULL_INDEX)
	{
		b2Shape* shape = world->shapes + shapeIndex;
		shape->aabb = b2Shape_ComputeAABB(shape, body->transform);

		if (b2AABB_Contains(shape->fatAABB, shape->aabb) == false)
		{
			shape->fatAABB.lowerBound = b2Sub(shape->aabb.lowerBound, aabbMargin);
			shape->fatAABB.upperBound = b2Add(shape->aabb.upperBound, aabbMargin);
			b2BroadPhase_MoveProxy(broadPhase, shape->proxyKey, shape->fatAABB);
		}

		shapeIndex = shape->nextShapeIndex;
	}
}

void b2Body_SetLinearVelocity(b2BodyId bodyId, b2Vec2 linearVelocity)
{
	b2World* world = b2GetWorldFromIndex(bodyId.world);
	B2_ASSERT(0 <= bodyId.index && bodyId.index < world->bodyPool.capacity);
	world->bodies[bodyId.index].linearVelocity = linearVelocity;
}

void b2Body_SetAngularVelocity(b2BodyId bodyId, float angularVelocity)
{
	b2World* world = b2GetWorldFromIndex(bodyId.world);
	B2_ASSERT(0 <= bodyId.index && bodyId.index < world->bodyPool.capacity);
	world->bodies[bodyId.index].angularVelocity = angularVelocity;
}

b2BodyType b2Body_GetType(b2BodyId bodyId)
{
	b2World* world = b2GetWorldFromIndex(bodyId.world);
	B2_ASSERT(0 <= bodyId.index && bodyId.index < world->bodyPool.capacity);
	return world->bodies[bodyId.index].type;
}

float b2Body_GetMass(b2BodyId bodyId)
{
	b2World* world = b2GetWorldFromIndex(bodyId.world);
	B2_ASSERT(0 <= bodyId.index && bodyId.index < world->bodyPool.capacity);
	return world->bodies[bodyId.index].mass;
}

void b2Body_Wake(b2BodyId bodyId)
{
	b2World* world = b2GetWorldFromIndex(bodyId.world);
	B2_ASSERT(0 <= bodyId.index && bodyId.index < world->bodyPool.capacity);
	b2Body* body = world->bodies + bodyId.index;
	if (body->type == b2_staticBody)
	{
		return;
	}

	int32_t islandIndex = body->islandIndex;
	B2_ASSERT(0 <= islandIndex && islandIndex < world->islandPool.capacity);

	b2WakeIsland(world->islands + islandIndex);
}

bool b2ShouldBodiesCollide(b2World* world, b2Body* bodyA, b2Body* bodyB)
{
	int32_t jointKey;
	int32_t otherBodyIndex;
	if (bodyA->jointCount < bodyB->jointCount)
	{
		jointKey = bodyA->jointList;
		otherBodyIndex = bodyB->object.index;
	}
	else
	{
		jointKey = bodyB->jointList;
		otherBodyIndex = bodyA->object.index;
	}

	while (jointKey != B2_NULL_INDEX)
	{
		int32_t jointIndex = jointKey >> 1;
		int32_t edgeIndex = jointKey & 1;
		int32_t otherEdgeIndex = edgeIndex ^ 1;

		b2Joint* joint = world->joints + jointIndex;
		if (joint->edges[otherEdgeIndex].bodyIndex == otherBodyIndex)
		{
			return false;
		}

		jointKey = joint->edges[edgeIndex].nextKey;
	}

	return true;
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
