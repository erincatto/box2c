// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

// #include "contact_solver.h"
// #include "island.h"

#include "box2d/allocate.h"
#include "box2d/box2d.h"
#include "box2d/constants.h"
#include "box2d/debug_draw.h"
#include "box2d/timer.h"

#include "array.h"
#include "body.h"
#include "contact.h"
#include "island.h"
#include "pool.h"
#include "shape.h"
#include "solver_data.h"
#include "world.h"

#include <assert.h>
#include <string.h>

b2World g_worlds[b2_maxWorlds];

b2World* b2GetWorldFromId(b2WorldId id)
{
	assert(0 <= id.index && id.index < b2_maxWorlds);
	b2World* world = g_worlds + id.index;
	assert(id.revision == world->revision);
	return world;
}

b2World* b2GetWorldFromIndex(int16_t index)
{
	assert(0 <= index && index < b2_maxWorlds);
	b2World* world = g_worlds + index;
	assert(world->blockAllocator != NULL);
	return world;
}

static void b2AddPair(void* userDataA, void* userDataB, void* context)
{
	b2World* world = (b2World*)context;

	b2ShapeProxy* proxyA = (b2ShapeProxy*)userDataA;
	b2ShapeProxy* proxyB = (b2ShapeProxy*)userDataB;

	int32_t shapeIndexA = proxyA->shapeIndex;
	int32_t shapeIndexB = proxyB->shapeIndex;

	b2Shape* shapeA = world->shapes + shapeIndexA;
	b2Shape* shapeB = world->shapes + shapeIndexB;

	// Are the fixtures on the same body?
	if (shapeA->bodyIndex == shapeB->bodyIndex)
	{
		return;
	}

	// Search contacts on shape with the fewest contacts.
	b2ContactEdge* edge;
	int32_t otherShapeIndex;
	if (shapeA->contactCount < shapeB->contactCount)
	{
		edge = shapeA->contacts;
		otherShapeIndex = shapeIndexB;
	}
	else
	{
		edge = shapeB->contacts;
		otherShapeIndex = shapeIndexA;
	}

	int32_t childA = proxyA->childIndex;
	int32_t childB = proxyB->childIndex;

	while (edge)
	{
		if (edge->otherShapeIndex == otherShapeIndex)
		{
			int32_t sA = edge->contact->shapeIndexA;
			int32_t sB = edge->contact->shapeIndexB;
			int32_t cA = edge->contact->childA;
			int32_t cB = edge->contact->childB;

			if (sA == shapeIndexA && sB == shapeIndexB && cA == childA && cB == childB)
			{
				// A contact already exists.
				return;
			}

			if (sA == shapeIndexB && sB == shapeIndexB && cA == childB && cB == childA)
			{
				// A contact already exists.
				return;
			}
		}

		edge = edge->next;
	}

	// b2Body* bodyA = world->bodies + shapeA->bodyIndex;
	// b2Body* bodyB = world->bodies + shapeB->bodyIndex;

	// Does a joint override collision? Is at least one body dynamic?
	// if (b2ShouldBodiesCollide(bodyA, bodyB) == false)
	//{
	//	return;
	//}

	// Check user filtering.
	// if (m_contactFilter && m_contactFilter->ShouldCollide(fixtureA, fixtureB) == false)
	//{
	//	return;
	//}

	b2CreateContact(world, shapeA, childA, shapeB, childB);
}

b2WorldId b2CreateWorld(const b2WorldDef* def)
{
	b2WorldId id = b2_nullWorldId;
	for (int16_t i = 0; i < b2_maxWorlds; ++i)
	{
		if (g_worlds[i].blockAllocator == NULL)
		{
			id.index = i;
			break;
		}
	}

	if (id.index == b2_nullWorldId.index)
	{
		return id;
	}

	b2InitializeContactRegisters();

	b2World* world = g_worlds + id.index;
	world->index = id.index;

	world->blockAllocator = b2CreateBlockAllocator();

	world->stackAllocator.allocation = 0;
	world->stackAllocator.maxAllocation = 0;
	world->stackAllocator.entryCount = 0;
	world->stackAllocator.index = 0;

	b2BroadPhase_Create(&world->broadPhase, b2AddPair, world);

	// pools
	world->bodyPool = b2CreatePool(sizeof(b2Body), B2_MAX(def->bodyCapacity, 1));
	world->bodies = (b2Body*)world->bodyPool.memory;

	world->shapePool = b2CreatePool(sizeof(b2Shape), B2_MAX(def->shapeCapacity, 1));
	world->shapes = (b2Shape*)world->shapePool.memory;

	world->contacts = NULL;
	world->contactCount = 0;

	world->awakeBodies = b2CreateArray(sizeof(int32_t), def->bodyCapacity);
	world->seedBodies = b2CreateArray(sizeof(int32_t), def->bodyCapacity);

	world->awakeContacts = b2CreateArray(sizeof(b2Contact*), def->shapeCapacity);

	// Globals start at 0. It should be fine for this to roll over.
	world->revision += 1;

	world->gravity = def->gravity;
	world->restitutionThreshold = def->restitutionThreshold;
	world->inv_dt0 = 0.0f;
	world->enableSleep = true;
	world->newContacts = false;
	world->locked = false;
	world->warmStarting = true;

	b2Profile profile = {0};
	world->profile = profile;
	world->callbacks = b2DefaultWorldCallbacks();

	id.revision = world->revision;
	return id;
}

void b2DestroyWorld(b2WorldId id)
{
	b2World* world = b2GetWorldFromId(id);

	b2DestroyArray(world->awakeBodies);
	b2DestroyArray(world->seedBodies);

	b2DestroyArray(world->awakeContacts);

	b2DestroyPool(&world->shapePool);
	world->shapes = NULL;

	b2DestroyPool(&world->bodyPool);
	world->bodies = NULL;

	b2BroadPhase_Destroy(&world->broadPhase);

	b2DestroyBlockAllocator(world->blockAllocator);
	world->blockAllocator = NULL;
}

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
	b->islandFlag = false;
	b->isAwake = b->type == b2_staticBody ? false : def->isAwake;
	b->enableSleep = def->enableSleep;
	b->fixedRotation = def->fixedRotation;
	b->isEnabled = def->isEnabled;

	if (b->isAwake)
	{
		b->awakeIndex = b2Array(world->awakeBodies).count;
		b2Array_Push(world->awakeBodies, b->object.index);
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

			// Remove awake contact
			int32_t awakeIndex = contact->awakeIndex;
			if (awakeIndex != B2_NULL_INDEX)
			{
				assert(0 <= awakeIndex && awakeIndex < b2Array(world->awakeContacts).count);
				world->awakeContacts[awakeIndex] = NULL;
				contact->awakeIndex = B2_NULL_INDEX;
			}

			// TODO_ERIN EndContact callback?

			b2Shape* otherShape = world->shapes + ce->otherShapeIndex;
			if (contact->manifold.pointCount > 0 && shape->isSensor == false && otherShape->isSensor == false)
			{
				b2Body* otherBody = world->bodies + otherShape->bodyIndex;
				b2Body_SetAwake(world, otherBody, true);
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
	world->bodies = (b2Body*)world->bodyPool.memory;
}

static void b2Collide(b2World* world)
{
	b2ArrayHeader* h1 = (b2ArrayHeader*)world->awakeContacts - 1;
	B2_MAYBE_UNUSED(h1);

	// Validate awake contacts
	{
		int32_t N = b2Array(world->awakeContacts).count;
		int32_t awakeCount = 0;
		for (int32_t i = 0; i < N; ++i)
		{
			b2Contact* contact = world->awakeContacts[i];
			if (contact != NULL)
			{
				++awakeCount;
				assert(contact->awakeIndex == i);
			}
		}
		assert(awakeCount <= world->contactCount);
	}

	// Update awake contacts
	int32_t count = b2Array(world->awakeContacts).count;
	for (int32_t i = 0; i < count; ++i)
	{
		b2Contact* contact = world->awakeContacts[i];
		if (contact == NULL)
		{
			// This means the contact was destroyed
			continue;
		}

		// Must clear awake index here because the awake array will be rebuilt
		// in the island builder.
		contact->awakeIndex = B2_NULL_INDEX;
		world->awakeContacts[i] = NULL;

		b2Shape* shapeA = world->shapes + contact->shapeIndexA;
		b2Shape* shapeB = world->shapes + contact->shapeIndexB;
		b2Body* bodyA = world->bodies + shapeA->bodyIndex;
		b2Body* bodyB = world->bodies + shapeB->bodyIndex;

		// Is this contact flagged for filtering?
		if (contact->flags & b2_contactFilterFlag)
		{
			// Should these bodies collide?
			if (b2ShouldBodiesCollide(bodyA, bodyB) == false)
			{
				b2Contact* cNuke = contact;
				contact = cNuke->next;
				b2DestroyContact(world, cNuke);
				continue;
			}

			if (b2ShouldCollide(shapeA->filter, shapeB->filter))
			{
				b2Contact* cNuke = contact;
				contact = cNuke->next;
				b2DestroyContact(world, cNuke);
				continue;
			}
			// Check user filtering.
			b2ShapeId shapeIdA = {shapeA->object.index, world->index, shapeA->object.revision};
			b2ShapeId shapeIdB = {shapeA->object.index, world->index, shapeA->object.revision};
			if (world->callbacks.shouldCollideFcn && world->callbacks.shouldCollideFcn(shapeIdA, shapeIdB) == false)
			{
				b2Contact* cNuke = contact;
				contact = cNuke->next;
				b2DestroyContact(world, cNuke);
				continue;
			}

			// Clear the filtering flag.
			contact->flags &= ~b2_contactFilterFlag;
		}

		bool awakeA = bodyA->isAwake && bodyA->type != b2_staticBody;
		bool awakeB = bodyB->isAwake && bodyB->type != b2_staticBody;

		// At least one body must be awake and it must be dynamic or kinematic.
		if (awakeA == false && awakeB == false)
		{
			contact = contact->next;
			continue;
		}

		int32_t proxyKeyA = shapeA->proxies[contact->childA].proxyKey;
		int32_t proxyKeyB = shapeB->proxies[contact->childB].proxyKey;
		bool overlap = b2BroadPhase_TestOverlap(&world->broadPhase, proxyKeyA, proxyKeyB);

		// Here we destroy contacts that cease to overlap in the broad-phase.
		if (overlap == false)
		{
			b2Contact* cNuke = contact;
			contact = cNuke->next;
			b2DestroyContact(world, cNuke);
			continue;
		}

		// The contact persists.
		b2Contact_Update(world, contact, shapeA, bodyA, shapeB, bodyB);
		contact = contact->next;
	}

	// Clear awake contacts
	b2Array_Clear(world->awakeContacts);
}

// Find islands, integrate and solve constraints, solve position constraints
static void b2Solve(b2World* world, const b2TimeStep* step)
{
	world->profile.solveInit = 0.0f;
	world->profile.solveVelocity = 0.0f;
	world->profile.solvePosition = 0.0f;

	int32_t bodyCapacity = world->bodyPool.capacity;
	int32_t bodyCount = world->bodyPool.count;

	// Size the island for the worst case.
	b2Island island = b2CreateIsland(bodyCount, world->contactCount, 0, world);

	// Clear all the island flags.
	for (int32_t i = 0; i < bodyCapacity; ++i)
	{
		b2Body* b = world->bodies + i;
		b->islandFlag = false;
	}

	for (b2Contact* c = world->contacts; c; c = c->next)
	{
		c->flags &= ~b2_contactIslandFlag;
	}

	// for (b2Joint* j = m_jointList; j; j = j->m_next)
	//{
	//	j->m_islandFlag = false;
	// }

	b2ArrayHeader* header = (b2ArrayHeader*)world->awakeBodies - 1;
	B2_MAYBE_UNUSED(header);

	// Validate awake bodies
	{
		int32_t N = b2Array(world->awakeBodies).count;
		int32_t awakeCount = 0;
		for (int32_t i = 0; i < N; ++i)
		{
			int32_t bodyIndex = world->awakeBodies[i];
			if (bodyIndex != B2_NULL_INDEX)
			{
				++awakeCount;
				assert(bodyIndex < world->bodyPool.capacity);
				b2Body* body = world->bodies + bodyIndex;
				assert(body->isAwake);
				assert(body->awakeIndex == i);
			}
		}
		assert(awakeCount <= world->bodyPool.count);
	}

	// Swap awake body buffer
	{
		int32_t* temp = world->awakeBodies;
		world->awakeBodies = world->seedBodies;
		world->seedBodies = temp;
	}

	const int32_t* seedBuffer = world->seedBodies;
	b2Array_Clear(world->awakeBodies);

	int32_t seedCount = b2Array(seedBuffer).count;

	// Build and simulate all awake islands.
	b2Body** stack = (b2Body**)b2AllocateStackItem(&world->stackAllocator, seedCount * sizeof(b2Body*));

	for (int32_t i = 0; i < seedCount; ++i)
	{
		int32_t seedIndex = seedBuffer[i];
		if (seedIndex == B2_NULL_INDEX)
		{
			// body was destroyed, manually put to sleep, or disabled
			continue;
		}

		b2Body* seed = world->bodies + seedIndex;
		assert(seed->object.next == seedIndex);
		assert(seed->isEnabled);
		assert(seed->type != b2_staticBody);

		if (seed->islandFlag)
		{
			continue;
		}

		assert(seed->isAwake);

		// Reset island and stack.
		b2ClearIsland(&island);
		int32_t stackCount = 0;
		stack[stackCount++] = seed;
		seed->islandFlag = true;

		// Perform a depth first search (DFS) on the constraint graph.
		while (stackCount > 0)
		{
			// Grab the next body off the stack and add it to the island.
			b2Body* b = stack[--stackCount];
			assert(b->isEnabled == true);
			b2Island_AddBody(&island, b);

			// To keep islands as small as possible, we don't
			// propagate islands across static bodies.
			if (b->type == b2_staticBody)
			{
				continue;
			}

			// The awake body array is being rebuilt so the awake index is no longer valid
			b->awakeIndex = B2_NULL_INDEX;

			// Make sure the body is awake (without resetting sleep timer).
			b->isAwake = true;

			// Search all contacts connected to this body.
			int32_t shapeIndex = b->shapeIndex;
			while (shapeIndex != B2_NULL_INDEX)
			{
				b2Shape* shape = world->shapes + shapeIndex;
				assert(shape->object.index == shapeIndex);
				shapeIndex = shape->nextShapeIndex;

				for (b2ContactEdge* ce = shape->contacts; ce; ce = ce->next)
				{
					b2Contact* contact = ce->contact;

					// Has this contact already been added to an island?
					if (contact->flags & b2_contactIslandFlag)
					{
						assert(0 <= contact->awakeIndex && contact->awakeIndex < b2Array(world->awakeContacts).count);
						assert(world->awakeContacts[contact->awakeIndex] == contact);
						continue;
					}

					// This contact is awake even if not touching. This means some contacts will
					// be computed that fell asleep the previous time step. The contact may have
					// been added for the other body already.
					if (contact->awakeIndex == B2_NULL_INDEX)
					{
						contact->awakeIndex = b2Array(world->awakeContacts).count;
						b2Array_Push(world->awakeContacts, contact);
					}

					if (shape->isSensor)
					{
						continue;
					}

					// Is this contact solid and touching?
					if ((contact->flags & b2_contactEnabledFlag) == 0 || (contact->flags & b2_contactTouchingFlag) == 0)
					{
						continue;
					}

					// Skip sensors.
					b2Shape* otherShape = world->shapes + ce->otherShapeIndex;
					if (otherShape->isSensor)
					{
						continue;
					}

					b2Island_AddContact(&island, contact);
					contact->flags |= b2_contactIslandFlag;

					b2Body* otherBody = world->bodies + otherShape->bodyIndex;

					// Was the other body already added to this island?
					if (otherBody->islandFlag)
					{
						continue;
					}

					assert(stackCount < bodyCount);
					stack[stackCount++] = otherBody;
					otherBody->islandFlag = true;
				}
			}

#if 0
			// Search all joints connect to this body.
			for (b2JointEdge* je = b->m_jointList; je; je = je->next)
			{
				if (je->joint->m_islandFlag == true)
				{
					continue;
				}

				b2Body* other = je->other;

				// Don't simulate joints connected to disabled bodies.
				if (other->IsEnabled() == false)
				{
					continue;
				}

				island.Add(je->joint);
				je->joint->m_islandFlag = true;

				if (other->m_flags & b2Body::e_islandFlag)
				{
					continue;
				}

				assert(stackCount < stackSize);
				stack[stackCount++] = other;
				other->m_flags |= b2Body::e_islandFlag;
			}
#endif
		}

		b2Profile profile;
		b2SolveIsland(&island, &profile, step, world->gravity, world->enableSleep);

		world->profile.solveInit += profile.solveInit;
		world->profile.solveVelocity += profile.solveVelocity;
		world->profile.solvePosition += profile.solvePosition;

		// Post solve cleanup.
		for (int32_t j = 0; j < island.bodyCount; ++j)
		{
			// Allow static bodies to participate in other islands.
			b2Body* b = island.bodies[j];
			if (b->type == b2_staticBody)
			{
				b->islandFlag = false;
			}
		}
	}

	b2FreeStackItem(&world->stackAllocator, stack);

	b2Timer timer;

	// Synchronize fixtures, check for out of range bodies.
	for (int32_t i = 0; i < bodyCapacity; ++i)
	{
		b2Body* b = world->bodies + i;
		if (b->object.next != i)
		{
			// body is in free list
			continue;
		}

		// If a body was not in an island then it did not move.
		if (b->islandFlag == false)
		{
			continue;
		}

		if (b->type == b2_staticBody)
		{
			continue;
		}

		// Update shapes (for broad-phase).
		int32_t shapeIndex = b->shapeIndex;
		while (shapeIndex != B2_NULL_INDEX)
		{
			b2Shape* shape = world->shapes + shapeIndex;
			for (int32_t j = 0; j < shape->proxyCount; ++j)
			{
				b2ShapeProxy* proxy = shape->proxies + j;

				proxy->aabb = b2Shape_ComputeAABB(shape, b->transform, proxy->childIndex);

				b2BroadPhase_MoveProxy(&world->broadPhase, proxy->proxyKey, proxy->aabb);
			}

			shapeIndex = shape->nextShapeIndex;
		}
	}

	// Look for new contacts.
	b2BroadPhase_UpdatePairs(&world->broadPhase);
	world->profile.broadphase = b2GetMilliseconds(&timer);

	b2DestroyIsland(&island);
}

void b2World_Step(b2WorldId worldId, float timeStep, int32_t velocityIterations, int32_t positionIterations)
{
	b2World* world = b2GetWorldFromId(worldId);
	assert(world->locked == false);
	if (world->locked)
	{
		return;
	}

	b2Timer stepTimer;

	// If new fixtures were added, we need to find the new contacts.
	if (world->newContacts)
	{
		b2BroadPhase_UpdatePairs(&world->broadPhase);
		world->newContacts = false;
	}

	// TODO_ERIN atomic
	world->locked = true;

	b2TimeStep step;
	step.dt = timeStep;
	step.velocityIterations = velocityIterations;
	step.positionIterations = positionIterations;
	if (timeStep > 0.0f)
	{
		step.inv_dt = 1.0f / timeStep;
	}
	else
	{
		step.inv_dt = 0.0f;
	}

	step.dtRatio = world->inv_dt0 * timeStep;

	step.warmStarting = world->warmStarting;

	// Update contacts. This is where some contacts are destroyed.
	{
		b2Timer timer;
		b2Collide(world);
		world->profile.collide = b2GetMillisecondsAndReset(&timer);
	}

	// Integrate velocities, solve velocity constraints, and integrate positions.
	if (step.dt > 0.0f)
	{
		b2Timer timer;
		b2Solve(world, &step);
		world->profile.solve = b2GetMillisecondsAndReset(&timer);
	}

	if (step.dt > 0.0f)
	{
		world->inv_dt0 = step.inv_dt;
	}

	// TODO_ERIN clear forces in island solver on last sub-step
	// if (m_clearForces)
	//{
	int32_t count = world->bodyPool.capacity;
	for (int32_t i = 0; i < count; ++i)
	{
		world->bodies[i].force = b2Vec2_zero;
		world->bodies[i].torque = 0.0f;
	}
	//}

	world->locked = false;

	world->profile.step = b2GetMilliseconds(&stepTimer);
}

static void b2DrawShape(b2DebugDraw* draw, b2Shape* shape, b2Transform xf, b2Color color)
{
	switch (shape->type)
	{
		case b2_circleShape:
		{
			b2Circle* circle = &shape->circle;

			b2Vec2 center = b2TransformPoint(xf, circle->point);
			float radius = circle->radius;
			b2Vec2 axis = b2RotateVector(xf.q, (b2Vec2){1.0f, 0.0f});

			draw->DrawSolidCircle(center, radius, axis, color, draw->context);
		}
		break;

			// case b2_segmentShape:
			//{
			// b2EdgeShape* edge = (b2EdgeShape*)shape->GetShape();
			// b2Vec2 v1 = b2Mul(xf, edge->m_vertex1);
			// b2Vec2 v2 = b2Mul(xf, edge->m_vertex2);
			// m_debugDraw->DrawSegment(v1, v2, color);

			// if (edge->m_oneSided == false)
			//{
			//	m_debugDraw->DrawPoint(v1, 4.0f, color);
			//	m_debugDraw->DrawPoint(v2, 4.0f, color);
			// }
			// }
			// break;

			// case b2Shape::e_chain:
			//{
			// b2ChainShape* chain = (b2ChainShape*)shape->GetShape();
			// int32 count = chain->m_count;
			// const b2Vec2* vertices = chain->m_vertices;

			// b2Vec2 v1 = b2Mul(xf, vertices[0]);
			// for (int32 i = 1; i < count; ++i)
			//{
			//	b2Vec2 v2 = b2Mul(xf, vertices[i]);
			//	m_debugDraw->DrawSegment(v1, v2, color);
			//	v1 = v2;
			// }
			// }
			// break;

		case b2_polygonShape:
		{
			b2Polygon* poly = &shape->polygon;
			int32_t count = poly->count;
			assert(count <= b2_maxPolygonVertices);
			b2Vec2 vertices[b2_maxPolygonVertices];

			for (int32_t i = 0; i < count; ++i)
			{
				vertices[i] = b2TransformPoint(xf, poly->vertices[i]);
			}

			draw->DrawSolidPolygon(vertices, count, color, draw->context);
		}
		break;

		default:
			break;
	}
}

void b2World_Draw(b2WorldId worldId, b2DebugDraw* draw)
{
	b2World* world = b2GetWorldFromId(worldId);
	assert(world->locked == false);
	if (world->locked)
	{
		return;
	}

	if (draw->drawShapes)
	{
		int32_t count = world->bodyPool.capacity;
		for (int32_t i = 0; i < count; ++i)
		{
			b2Body* b = world->bodies + i;
			if (b->object.next != i)
			{
				continue;
			}

			b2Transform xf = b->transform;
			int32_t shapeIndex = b->shapeIndex;
			while (shapeIndex != B2_NULL_INDEX)
			{
				b2Shape* shape = world->shapes + shapeIndex;
				if (b->type == b2_dynamicBody && b->mass == 0.0f)
				{
					// Bad body
					b2DrawShape(draw, shape, xf, (b2Color){1.0f, 0.0f, 0.0f, 1.0f});
				}
				else if (b->isEnabled == false)
				{
					b2DrawShape(draw, shape, xf, (b2Color){0.5f, 0.5f, 0.3f, 1.0f});
				}
				else if (b->type == b2_staticBody)
				{
					b2DrawShape(draw, shape, xf, (b2Color){0.5f, 0.9f, 0.5f, 1.0f});
				}
				else if (b->type == b2_kinematicBody)
				{
					b2DrawShape(draw, shape, xf, (b2Color){0.5f, 0.5f, 0.9f, 1.0f});
				}
				else if (b->isAwake == false)
				{
					b2DrawShape(draw, shape, xf, (b2Color){0.6f, 0.6f, 0.6f, 1.0f});
				}
				else
				{
					b2DrawShape(draw, shape, xf, (b2Color){0.9f, 0.7f, 0.7f, 1.0f});
				}

				shapeIndex = shape->nextShapeIndex;
			}
		}
	}

	if (draw->drawJoints)
	{
		// for (b2Joint* j = m_jointList; j; j = j->GetNext())
		//{
		// j->Draw(m_debugDraw);
		// }
	}

	// if (debugDraw->drawPi & b2Draw::e_pairBit)
	//{
	//		b2Color color(0.3f, 0.9f, 0.9f);
	//		for (b2Contact* c = m_contactManager.m_contactList; c; c = c->GetNext())
	//		{
	//		b2Shape* fixtureA = c->GetFixtureA();
	//		b2Shape* fixtureB = c->GetFixtureB();
	//		int32 indexA = c->GetChildIndexA();
	//		int32 indexB = c->GetChildIndexB();
	//		b2Vec2 cA = fixtureA->GetAABB(indexA).GetCenter();
	//		b2Vec2 cB = fixtureB->GetAABB(indexB).GetCenter();

	//		m_debugDraw->DrawSegment(cA, cB, color);
	//		}
	//}

	if (draw->drawAABBs)
	{
		b2Color color = {0.9f, 0.3f, 0.9f, 1.0f};
		b2BroadPhase* bp = &world->broadPhase;

		int32_t count = world->bodyPool.capacity;
		for (int32_t i = 0; i < count; ++i)
		{
			b2Body* b = world->bodies + i;
			if (b->object.next != i)
			{
				continue;
			}

			int32_t shapeIndex = b->shapeIndex;
			while (shapeIndex != B2_NULL_INDEX)
			{
				b2Shape* shape = world->shapes + shapeIndex;
				for (int32_t j = 0; j < shape->proxyCount; ++j)
				{
					b2ShapeProxy* proxy = shape->proxies + j;
					b2AABB aabb = b2BroadPhase_GetFatAABB(bp, proxy->proxyKey);

					b2Vec2 vs[4] = {{aabb.lowerBound.x, aabb.lowerBound.y},
									{aabb.upperBound.x, aabb.lowerBound.y},
									{aabb.upperBound.x, aabb.upperBound.y},
									{aabb.lowerBound.x, aabb.upperBound.y}};

					draw->DrawPolygon(vs, 4, color, draw->context);
				}

				shapeIndex = shape->nextShapeIndex;
			}
		}

		// for (b2Shape* f = b->GetFixtureList(); f; f = f->GetNext())
		//{
		//	for (int32 i = 0; i < f->m_proxyCount; ++i)
		//	{
		//		b2FixtureProxy* proxy = f->m_proxies + i;
		//		b2AABB aabb = bp->GetFatAABB(proxy->proxyId);
		//		b2Vec2 vs[4];
		//		vs[0].Set(aabb.lowerBound.x, aabb.lowerBound.y);
		//		vs[1].Set(aabb.upperBound.x, aabb.lowerBound.y);
		//		vs[2].Set(aabb.upperBound.x, aabb.upperBound.y);
		//		vs[3].Set(aabb.lowerBound.x, aabb.upperBound.y);

		//		m_debugDraw->DrawPolygon(vs, 4, color);
		//	}
		//}
	}

// if (flags & b2Draw::e_centerOfMassBit)
//{
//		for (b2Body* b = m_bodyList; b; b = b->GetNext())
//		{
//		b2Transform xf = b->GetTransform();
//		xf.p = b->GetWorldCenter();
//		m_debugDraw->DrawTransform(xf);
//		}
// }
}

void b2World_EnableSleeping(b2WorldId worldId, bool flag)
{
	b2World* world = b2GetWorldFromId(worldId);
	assert(world->locked == false);
	if (world->locked)
	{
		return;
	}

	if (flag == world->enableSleep)
	{
		return;
	}

	world->enableSleep = flag;
	if (flag == false)
	{
		int32_t count = world->bodyPool.capacity;
		for (int32_t i = 0; i < count; ++i)
		{
			b2Body* b = world->bodies + i;
			if (b->object.next != i)
			{
				continue;
			}

			b2Body_SetAwake(world, b, true);
		}
	}
}

#if 0
b2Joint* b2World::CreateJoint(const b2JointDef* def)
{
	assert(IsLocked() == false);
	if (IsLocked())
	{
		return nullptr;
	}

	b2Joint* j = b2Joint::Create(def, &m_blockAllocator);

	// Connect to the world list.
	j->m_prev = nullptr;
	j->m_next = m_jointList;
	if (m_jointList)
	{
		m_jointList->m_prev = j;
	}
	m_jointList = j;
	++m_jointCount;

	// Connect to the bodies' doubly linked lists.
	j->m_edgeA.joint = j;
	j->m_edgeA.other = j->m_bodyB;
	j->m_edgeA.prev = nullptr;
	j->m_edgeA.next = j->m_bodyA->m_jointList;
	if (j->m_bodyA->m_jointList) j->m_bodyA->m_jointList->prev = &j->m_edgeA;
	j->m_bodyA->m_jointList = &j->m_edgeA;

	j->m_edgeB.joint = j;
	j->m_edgeB.other = j->m_bodyA;
	j->m_edgeB.prev = nullptr;
	j->m_edgeB.next = j->m_bodyB->m_jointList;
	if (j->m_bodyB->m_jointList) j->m_bodyB->m_jointList->prev = &j->m_edgeB;
	j->m_bodyB->m_jointList = &j->m_edgeB;

	b2Body* bodyA = def->bodyA;
	b2Body* bodyB = def->bodyB;

	// If the joint prevents collisions, then flag any contacts for filtering.
	if (def->collideConnected == false)
	{
		b2ContactEdge* edge = bodyB->GetContactList();
		while (edge)
		{
			if (edge->other == bodyA)
			{
				// Flag the contact for filtering at the next time step (where either
				// body is awake).
				edge->contact->FlagForFiltering();
			}

			edge = edge->next;
		}
	}

	// Note: creating a joint doesn't wake the bodies.

	return j;
}

void b2World::DestroyJoint(b2Joint* j)
{
	assert(IsLocked() == false);
	if (IsLocked())
	{
		return;
	}

	bool collideConnected = j->m_collideConnected;

	// Remove from the doubly linked list.
	if (j->m_prev)
	{
		j->m_prev->m_next = j->m_next;
	}

	if (j->m_next)
	{
		j->m_next->m_prev = j->m_prev;
	}

	if (j == m_jointList)
	{
		m_jointList = j->m_next;
	}

	// Disconnect from island graph.
	b2Body* bodyA = j->m_bodyA;
	b2Body* bodyB = j->m_bodyB;

	// Wake up connected bodies.
	bodyA->SetAwake(true);
	bodyB->SetAwake(true);

	// Remove from body 1.
	if (j->m_edgeA.prev)
	{
		j->m_edgeA.prev->next = j->m_edgeA.next;
	}

	if (j->m_edgeA.next)
	{
		j->m_edgeA.next->prev = j->m_edgeA.prev;
	}

	if (&j->m_edgeA == bodyA->m_jointList)
	{
		bodyA->m_jointList = j->m_edgeA.next;
	}

	j->m_edgeA.prev = nullptr;
	j->m_edgeA.next = nullptr;

	// Remove from body 2
	if (j->m_edgeB.prev)
	{
		j->m_edgeB.prev->next = j->m_edgeB.next;
	}

	if (j->m_edgeB.next)
	{
		j->m_edgeB.next->prev = j->m_edgeB.prev;
	}

	if (&j->m_edgeB == bodyB->m_jointList)
	{
		bodyB->m_jointList = j->m_edgeB.next;
	}

	j->m_edgeB.prev = nullptr;
	j->m_edgeB.next = nullptr;

	b2Joint::Destroy(j, &m_blockAllocator);

	assert(m_jointCount > 0);
	--m_jointCount;

	// If the joint prevents collisions, then flag any contacts for filtering.
	if (collideConnected == false)
	{
		b2ContactEdge* edge = bodyB->GetContactList();
		while (edge)
		{
			if (edge->other == bodyA)
			{
				// Flag the contact for filtering at the next time step (where either
				// body is awake).
				edge->contact->FlagForFiltering();
			}

			edge = edge->next;
		}
	}
}


// Find TOI contacts and solve them.
void b2World::SolveTOI(const b2TimeStep& step)
{
	b2Island island(2 * b2_maxTOIContacts, b2_maxTOIContacts, 0, &m_stackAllocator, m_contactManager.m_contactListener);

	if (m_stepComplete)
	{
		for (b2Body* b = m_bodyList; b; b = b->m_next)
		{
			b->m_flags &= ~b2Body::e_islandFlag;
			b->m_sweep.alpha0 = 0.0f;
		}

		for (b2Contact* c = m_contactManager.m_contactList; c; c = c->m_next)
		{
			// Invalidate TOI
			c->m_flags &= ~(b2Contact::e_toiFlag | b2Contact::e_islandFlag);
			c->m_toiCount = 0;
			c->m_toi = 1.0f;
		}
	}

	// Find TOI events and solve them.
	for (;;)
	{
		// Find the first TOI.
		b2Contact* minContact = nullptr;
		float minAlpha = 1.0f;

		for (b2Contact* c = m_contactManager.m_contactList; c; c = c->m_next)
		{
			// Is this contact disabled?
			if (c->IsEnabled() == false)
			{
				continue;
			}

			// Prevent excessive sub-stepping.
			if (c->m_toiCount > b2_maxSubSteps)
			{
				continue;
			}

			float alpha = 1.0f;
			if (c->m_flags & b2Contact::e_toiFlag)
			{
				// This contact has a valid cached TOI.
				alpha = c->m_toi;
			}
			else
			{
				b2Shape* fA = c->GetFixtureA();
				b2Shape* fB = c->GetFixtureB();

				// Is there a sensor?
				if (fA->IsSensor() || fB->IsSensor())
				{
					continue;
				}

				b2Body* bA = fA->GetBody();
				b2Body* bB = fB->GetBody();

				b2BodyType typeA = bA->m_type;
				b2BodyType typeB = bB->m_type;
				assert(typeA == b2_dynamicBody || typeB == b2_dynamicBody);

				bool activeA = bA->IsAwake() && typeA != b2_staticBody;
				bool activeB = bB->IsAwake() && typeB != b2_staticBody;

				// Is at least one body active (awake and dynamic or kinematic)?
				if (activeA == false && activeB == false)
				{
					continue;
				}

				bool collideA = bA->IsBullet() || typeA != b2_dynamicBody;
				bool collideB = bB->IsBullet() || typeB != b2_dynamicBody;

				// Are these two non-bullet dynamic bodies?
				if (collideA == false && collideB == false)
				{
					continue;
				}

				// Compute the TOI for this contact.
				// Put the sweeps onto the same time interval.
				float alpha0 = bA->m_sweep.alpha0;

				if (bA->m_sweep.alpha0 < bB->m_sweep.alpha0)
				{
					alpha0 = bB->m_sweep.alpha0;
					bA->m_sweep.Advance(alpha0);
				}
				else if (bB->m_sweep.alpha0 < bA->m_sweep.alpha0)
				{
					alpha0 = bA->m_sweep.alpha0;
					bB->m_sweep.Advance(alpha0);
				}

				assert(alpha0 < 1.0f);

				int32 indexA = c->GetChildIndexA();
				int32 indexB = c->GetChildIndexB();

				// Compute the time of impact in interval [0, minTOI]
				b2TOIInput input;
				input.proxyA.Set(fA->GetShape(), indexA);
				input.proxyB.Set(fB->GetShape(), indexB);
				input.sweepA = bA->m_sweep;
				input.sweepB = bB->m_sweep;
				input.tMax = 1.0f;

				b2TOIOutput output;
				b2TimeOfImpact(&output, &input);

				// Beta is the fraction of the remaining portion of the .
				float beta = output.t;
				if (output.state == b2TOIOutput::e_touching)
				{
					alpha = b2Min(alpha0 + (1.0f - alpha0) * beta, 1.0f);
				}
				else
				{
					alpha = 1.0f;
				}

				c->m_toi = alpha;
				c->m_flags |= b2Contact::e_toiFlag;
			}

			if (alpha < minAlpha)
			{
				// This is the minimum TOI found so far.
				minContact = c;
				minAlpha = alpha;
			}
		}

		if (minContact == nullptr || 1.0f - 10.0f * b2_epsilon < minAlpha)
		{
			// No more TOI events. Done!
			m_stepComplete = true;
			break;
		}

		// Advance the bodies to the TOI.
		b2Shape* fA = minContact->GetFixtureA();
		b2Shape* fB = minContact->GetFixtureB();
		b2Body* bA = fA->GetBody();
		b2Body* bB = fB->GetBody();

		b2Sweep backup1 = bA->m_sweep;
		b2Sweep backup2 = bB->m_sweep;

		bA->Advance(minAlpha);
		bB->Advance(minAlpha);

		// The TOI contact likely has some new contact points.
		minContact->Update(m_contactManager.m_contactListener);
		minContact->m_flags &= ~b2Contact::e_toiFlag;
		++minContact->m_toiCount;

		// Is the contact solid?
		if (minContact->IsEnabled() == false || minContact->IsTouching() == false)
		{
			// Restore the sweeps.
			minContact->SetEnabled(false);
			bA->m_sweep = backup1;
			bB->m_sweep = backup2;
			bA->SynchronizeTransform();
			bB->SynchronizeTransform();
			continue;
		}

		bA->SetAwake(true);
		bB->SetAwake(true);

		// Build the island
		island.Clear();
		island.Add(bA);
		island.Add(bB);
		island.Add(minContact);

		bA->m_flags |= b2Body::e_islandFlag;
		bB->m_flags |= b2Body::e_islandFlag;
		minContact->m_flags |= b2Contact::e_islandFlag;

		// Get contacts on bodyA and bodyB.
		b2Body* bodies[2] = {bA, bB};
		for (int32 i = 0; i < 2; ++i)
		{
			b2Body* body = bodies[i];
			if (body->m_type == b2_dynamicBody)
			{
				for (b2ContactEdge* ce = body->m_contactList; ce; ce = ce->next)
				{
					if (island.m_bodyCount == island.m_bodyCapacity)
					{
						break;
					}

					if (island.m_contactCount == island.m_contactCapacity)
					{
						break;
					}

					b2Contact* contact = ce->contact;

					// Has this contact already been added to the island?
					if (contact->m_flags & b2Contact::e_islandFlag)
					{
						continue;
					}

					// Only add static, kinematic, or bullet bodies.
					b2Body* other = ce->other;
					if (other->m_type == b2_dynamicBody &&
						body->IsBullet() == false && other->IsBullet() == false)
					{
						continue;
					}

					// Skip sensors.
					bool sensorA = contact->m_fixtureA->m_isSensor;
					bool sensorB = contact->m_fixtureB->m_isSensor;
					if (sensorA || sensorB)
					{
						continue;
					}

					// Tentatively advance the body to the TOI.
					b2Sweep backup = other->m_sweep;
					if ((other->m_flags & b2Body::e_islandFlag) == 0)
					{
						other->Advance(minAlpha);
					}

					// Update the contact points
					contact->Update(m_contactManager.m_contactListener);

					// Was the contact disabled by the user?
					if (contact->IsEnabled() == false)
					{
						other->m_sweep = backup;
						other->SynchronizeTransform();
						continue;
					}

					// Are there contact points?
					if (contact->IsTouching() == false)
					{
						other->m_sweep = backup;
						other->SynchronizeTransform();
						continue;
					}

					// Add the contact to the island
					contact->m_flags |= b2Contact::e_islandFlag;
					island.Add(contact);

					// Has the other body already been added to the island?
					if (other->m_flags & b2Body::e_islandFlag)
					{
						continue;
					}
					
					// Add the other body to the island.
					other->m_flags |= b2Body::e_islandFlag;

					if (other->m_type != b2_staticBody)
					{
						other->SetAwake(true);
					}

					island.Add(other);
				}
			}
		}

		b2TimeStep subStep;
		subStep.dt = (1.0f - minAlpha) * step.dt;
		subStep.inv_dt = 1.0f / subStep.dt;
		subStep.dtRatio = 1.0f;
		subStep.positionIterations = 20;
		subStep.velocityIterations = step.velocityIterations;
		subStep.warmStarting = false;
		island.SolveTOI(subStep, bA->m_islandIndex, bB->m_islandIndex);

		// Reset island flags and synchronize broad-phase proxies.
		for (int32 i = 0; i < island.m_bodyCount; ++i)
		{
			b2Body* body = island.m_bodies[i];
			body->m_flags &= ~b2Body::e_islandFlag;

			if (body->m_type != b2_dynamicBody)
			{
				continue;
			}

			body->SynchronizeFixtures();

			// Invalidate all contact TOIs on this displaced body.
			for (b2ContactEdge* ce = body->m_contactList; ce; ce = ce->next)
			{
				ce->contact->m_flags &= ~(b2Contact::e_toiFlag | b2Contact::e_islandFlag);
			}
		}

		// Commit shape proxy movements to the broad-phase so that new contacts are created.
		// Also, some contacts can be destroyed.
		m_contactManager.FindNewContacts();

		if (m_subStepping)
		{
			m_stepComplete = false;
			break;
		}
	}
}

struct b2WorldQueryWrapper
{
	bool QueryCallback(int32 proxyId)
	{
		b2FixtureProxy* proxy = (b2FixtureProxy*)broadPhase->GetUserData(proxyId);
		return callback->ReportFixture(proxy->shape);
	}

	const b2BroadPhase* broadPhase;
	b2QueryCallback* callback;
};

void b2World::QueryAABB(b2QueryCallback* callback, const b2AABB& aabb) const
{
	b2WorldQueryWrapper wrapper;
	wrapper.broadPhase = &m_contactManager.m_broadPhase;
	wrapper.callback = callback;
	m_contactManager.m_broadPhase.Query(&wrapper, aabb);
}

struct b2WorldRayCastWrapper
{
	float RayCastCallback(const b2RayCastInput& input, int32 proxyId)
	{
		void* userData = broadPhase->GetUserData(proxyId);
		b2FixtureProxy* proxy = (b2FixtureProxy*)userData;
		b2Shape* shape = proxy->shape;
		int32 index = proxy->childIndex;
		b2RayCastOutput output;
		bool hit = shape->RayCast(&output, input, index);

		if (hit)
		{
			float fraction = output.fraction;
			b2Vec2 point = (1.0f - fraction) * input.p1 + fraction * input.p2;
			return callback->ReportFixture(shape, point, output.normal, fraction);
		}

		return input.maxFraction;
	}

	const b2BroadPhase* broadPhase;
	b2RayCastCallback* callback;
};

void b2World::RayCast(b2RayCastCallback* callback, const b2Vec2& point1, const b2Vec2& point2) const
{
	b2WorldRayCastWrapper wrapper;
	wrapper.broadPhase = &m_contactManager.m_broadPhase;
	wrapper.callback = callback;
	b2RayCastInput input;
	input.maxFraction = 1.0f;
	input.p1 = point1;
	input.p2 = point2;
	m_contactManager.m_broadPhase.RayCast(&wrapper, input);
}



int32 b2World::GetProxyCount() const
{
	return m_contactManager.m_broadPhase.GetProxyCount();
}

int32 b2World::GetTreeHeight() const
{
	return m_contactManager.m_broadPhase.GetTreeHeight();
}

int32 b2World::GetTreeBalance() const
{
	return m_contactManager.m_broadPhase.GetTreeBalance();
}

float b2World::GetTreeQuality() const
{
	return m_contactManager.m_broadPhase.GetTreeQuality();
}

void b2World::ShiftOrigin(const b2Vec2& newOrigin)
{
	assert(m_locked == false);
	if (m_locked)
	{
		return;
	}

	for (b2Body* b = m_bodyList; b; b = b->m_next)
	{
		b->m_xf.p -= newOrigin;
		b->m_sweep.c0 -= newOrigin;
		b->m_sweep.c -= newOrigin;
	}

	for (b2Joint* j = m_jointList; j; j = j->m_next)
	{
		j->ShiftOrigin(newOrigin);
	}

	m_contactManager.m_broadPhase.ShiftOrigin(newOrigin);
}

void b2World::Dump()
{
	if (m_locked)
	{
		return;
	}

	b2OpenDump("box2d_dump.inl");

	b2Dump("b2Vec2 g(%.9g, %.9g);\n", m_gravity.x, m_gravity.y);
	b2Dump("m_world->SetGravity(g);\n");

	b2Dump("b2Body** bodies = (b2Body**)b2Alloc(%d * sizeof(b2Body*));\n", m_bodyCount);
	b2Dump("b2Joint** joints = (b2Joint**)b2Alloc(%d * sizeof(b2Joint*));\n", m_jointCount);

	int32 i = 0;
	for (b2Body* b = m_bodyList; b; b = b->m_next)
	{
		b->m_islandIndex = i;
		b->Dump();
		++i;
	}

	i = 0;
	for (b2Joint* j = m_jointList; j; j = j->m_next)
	{
		j->m_index = i;
		++i;
	}

	// First pass on joints, skip gear joints.
	for (b2Joint* j = m_jointList; j; j = j->m_next)
	{
		if (j->m_type == e_gearJoint)
		{
			continue;
		}

		b2Dump("{\n");
		j->Dump();
		b2Dump("}\n");
	}

	// Second pass on joints, only gear joints.
	for (b2Joint* j = m_jointList; j; j = j->m_next)
	{
		if (j->m_type != e_gearJoint)
		{
			continue;
		}

		b2Dump("{\n");
		j->Dump();
		b2Dump("}\n");
	}

	b2Dump("b2Free(joints);\n");
	b2Dump("b2Free(bodies);\n");
	b2Dump("joints = nullptr;\n");
	b2Dump("bodies = nullptr;\n");

	b2CloseDump();
}
#endif
