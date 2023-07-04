// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "box2d/allocate.h"
#include "box2d/box2d.h"
#include "box2d/constants.h"
#include "box2d/debug_draw.h"
#include "box2d/timer.h"

#include "array.h"
#include "block_allocator.h"
#include "body.h"
#include "contact.h"
#include "island.h"
#include "joint.h"
#include "pool.h"
#include "shape.h"
#include "solver_data.h"
#include "stack_allocator.h"
#include "thread.h"
#include "world.h"

#include <assert.h>
#include <string.h>

#include "atomic.inl"

#define B2_VALIDATE 1

b2World g_worlds[b2_maxWorlds];
bool g_parallel = true;

// Per thread task storage
typedef struct b2TaskContext
{
	// These bits align with the awake contact array and signal change in contact status
	// that affects the island graph.
	bool* contactBitArray;
} b2TaskContext;

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

static void b2DefaultAddTaskFcn(b2TaskCallback* task, int32_t count, int32_t minRange, void* taskContext, void* userContext)
{
	B2_MAYBE_UNUSED(minRange);
	B2_MAYBE_UNUSED(userContext);
	task(0, count, 0, taskContext);
}

static void b2DefaultFinishTasksFcn(void* userContext)
{
	B2_MAYBE_UNUSED(userContext);
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

	if (b2ShouldCollide(shapeA->filter, shapeB->filter) == false)
	{
		return;
	}

	int32_t bodyIndexA = shapeA->bodyIndex;
	int32_t bodyIndexB = shapeB->bodyIndex;

	b2Body* bodyA = world->bodies + bodyIndexA;
	b2Body* bodyB = world->bodies + bodyIndexB;

	// Search contacts on body with the fewest contacts.
	// TODO_ERIN use hash table
	int32_t edgeKey;
	int32_t secondaryBodyIndex;
	if (bodyA->contactCount < bodyB->contactCount)
	{
		edgeKey = bodyA->contactList;
		secondaryBodyIndex = bodyIndexB;
	}
	else
	{
		edgeKey = bodyB->contactList;
		secondaryBodyIndex = bodyIndexA;
	}

	int32_t childA = proxyA->childIndex;
	int32_t childB = proxyB->childIndex;

	while (edgeKey != B2_NULL_INDEX)
	{
		int32_t contactIndex = edgeKey >> 1;
		int32_t edgeIndex = edgeKey & 1;
		int32_t twinIndex = edgeIndex ^ 1;

		b2Contact* contact = world->contacts + contactIndex;
		
		b2ContactEdge* edge = contact->edges + edgeIndex;
		b2ContactEdge* twin = contact->edges + twinIndex;

		if (twin->bodyIndex == secondaryBodyIndex)
		{
			int32_t sA = contact->shapeIndexA;
			int32_t sB = contact->shapeIndexB;
			int32_t cA = contact->childA;
			int32_t cB = contact->childB;

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

		edgeKey = edge->nextKey;
	}

	// Does a joint override collision? Is at least one body dynamic?
	if (b2ShouldBodiesCollide(world, bodyA, bodyB) == false)
	{
		return;
	}

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

	b2World empty = {0};
	b2World* world = g_worlds + id.index;
	*world = empty;

	world->index = id.index;

	world->blockAllocator = b2CreateBlockAllocator();
	world->stackAllocator = b2CreateStackAllocator(def->stackAllocatorCapacity);

	b2BroadPhase_Create(&world->broadPhase, b2AddPair, world);

	// pools
	world->bodyPool = b2CreatePool(sizeof(b2Body), B2_MAX(def->bodyCapacity, 1));
	world->bodies = (b2Body*)world->bodyPool.memory;

	world->shapePool = b2CreatePool(sizeof(b2Shape), B2_MAX(def->shapeCapacity, 1));
	world->shapes = (b2Shape*)world->shapePool.memory;

	world->contactPool = b2CreatePool(sizeof(b2Contact), B2_MAX(def->contactCapacity, 1));
	world->contacts = (b2Contact*)world->contactPool.memory;

	world->jointPool = b2CreatePool(sizeof(b2Joint), B2_MAX(def->jointCapacity, 1));
	world->joints = (b2Joint*)world->jointPool.memory;

	world->islandPool = b2CreatePool(sizeof(b2PersistentIsland), B2_MAX(def->bodyCapacity, 1));
	world->islands = (b2PersistentIsland*)world->islandPool.memory;

	world->awakeIslandArray = b2CreateArray(sizeof(int32_t), B2_MAX(def->bodyCapacity, 1));
	world->awakeContactArray = b2CreateArray(sizeof(int32_t), B2_MAX(def->contactCapacity, 1));

	world->stepId = 0;

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

	id.revision = world->revision;

	if (def->workerCount > 0 && def->enqueueTask != NULL && def->finishTasks != NULL)
	{
		world->workerCount = B2_MIN(def->workerCount, b2_maxWorkers);
		world->enqueueTask = def->enqueueTask;
		world->finishTasks = def->finishTasks;
		world->userTaskContext = def->userTaskContext;
	}
	else
	{
		world->workerCount = 1;
		world->enqueueTask = b2DefaultAddTaskFcn;
		world->finishTasks = b2DefaultFinishTasksFcn;
		world->userTaskContext = NULL;
	}
	
	world->taskContextArray = b2CreateArray(sizeof(b2TaskContext), world->workerCount);
	for (uint32_t i = 0; i < world->workerCount; ++i)
	{
		world->taskContextArray[i].contactBitArray = b2CreateArray(sizeof(bool), def->contactCapacity);
	}

	return id;
}

void b2DestroyWorld(b2WorldId id)
{
	b2World* world = b2GetWorldFromId(id);

	for (uint32_t i = 0; i < world->workerCount; ++i)
	{
		b2DestroyArray(world->taskContextArray[i].contactBitArray);
	}
	b2DestroyArray(world->taskContextArray);

	b2DestroyArray(world->awakeContactArray);
	b2DestroyArray(world->awakeIslandArray);
	b2DestroyPool(&world->islandPool);
	b2DestroyPool(&world->jointPool);
	b2DestroyPool(&world->contactPool);
	b2DestroyPool(&world->shapePool);
	b2DestroyPool(&world->bodyPool);

	b2BroadPhase_Destroy(&world->broadPhase);

	b2DestroyBlockAllocator(world->blockAllocator);
	b2DestroyStackAllocator(world->stackAllocator);

	memset(world, 0, sizeof(b2World));
}

static void b2AddAwakeIsland(b2World* world, b2PersistentIsland* island)
{
	assert(island->awakeIndex == B2_NULL_INDEX);
	island->awakeIndex = b2Array(world->awakeIslandArray).count;
	b2Array_Push(world->awakeIslandArray, island->object.index);
}

static void b2RemoveAwakeIsland(b2World* world, b2PersistentIsland* island)
{
	int32_t awakeIndex = island->awakeIndex;
	assert(awakeIndex != B2_NULL_INDEX && awakeIndex < b2Array(world->awakeIslandArray).count);
	world->awakeIslandArray[awakeIndex] = B2_NULL_INDEX;
}

// Locked version
static void b2CollideTask(int32_t startIndex, int32_t endIndex, uint32_t threadIndex, void* context)
{
	b2TracyCZoneNC(collide_task, "Collide Task", b2_colorDodgerBlue1, true);

	b2World* world = context;
	assert(threadIndex < world->workerCount);
	b2TaskContext* taskContext = world->taskContextArray + threadIndex;
	b2Shape* shapes = world->shapes;
	b2Body* bodies = world->bodies;
	b2Contact* contacts = world->contacts;

	assert(startIndex < endIndex);
	assert(endIndex <= b2Array(world->awakeContactArray).count);

	for (int32_t awakeIndex = startIndex; awakeIndex < endIndex; ++awakeIndex)
	{
		int32_t contactIndex = world->awakeContactArray[awakeIndex];
		if (contactIndex == B2_NULL_INDEX)
		{
			// Contact was destroyed or put to sleep
			continue;
		}

		assert(0 <= contactIndex && contactIndex < world->contactPool.capacity);

		b2Contact* contact = contacts + contactIndex;

		assert(contact->awakeIndex == awakeIndex);
		assert(contact->object.index == contactIndex && contact->object.index == contact->object.next);

		b2Shape* shapeA = shapes + contact->shapeIndexA;
		b2Shape* shapeB = shapes + contact->shapeIndexB;
		b2Body* bodyA = bodies + shapeA->bodyIndex;
		b2Body* bodyB = bodies + shapeB->bodyIndex;

		int32_t proxyKeyA = shapeA->proxies[contact->childA].proxyKey;
		int32_t proxyKeyB = shapeB->proxies[contact->childB].proxyKey;

		// Do proxies still overlap?
		// TODO_ERIN if we keep fat bounding boxes on shapes we don't need to dive into the broadphase here
		bool overlap = b2BroadPhase_TestOverlap(&world->broadPhase, proxyKeyA, proxyKeyB);
		if (overlap == false)
		{
			contact->flags |= b2_contactDisjoint;
			taskContext->contactBitArray[awakeIndex] = true;
		}
		else
		{
			bool wasTouching = (contact->flags & b2_contactTouchingFlag);

			// Update contact respecting shape/body order (A,B)
			b2Contact_Update(world, contact, shapeA, bodyA, shapeB, bodyB);

			// State changes that affect island connectivity
			if ((contact->flags & b2_contactTouchingFlag) != 0 && wasTouching == false)
			{
				contact->flags |= b2_contactStartedTouching;
				taskContext->contactBitArray[awakeIndex] = true;
			}
			else if ((contact->flags & b2_contactTouchingFlag) == 0 && wasTouching == true)
			{
				contact->flags |= b2_contactStoppedTouching;
				taskContext->contactBitArray[awakeIndex] = true;
			}
		}
	}

	b2TracyCZoneEnd(collide_task);
}

static void b2Collide(b2World* world)
{
	world->contactPointCount = 0;

	int32_t awakeContactCount = b2Array(world->awakeContactArray).count;

	if (awakeContactCount == 0)
	{
		return;
	}

	b2TracyCZoneNC(collide, "Collide", b2_colorDarkOrchid, true);

	for (uint32_t i = 0; i < world->workerCount; ++i)
	{
		memset(world->taskContextArray[i].contactBitArray, 0, awakeContactCount * sizeof(bool));
	}

	if (g_parallel)
	{
		int32_t minRange = 8;
		world->enqueueTask(&b2CollideTask, awakeContactCount, minRange, world, world->userTaskContext);
		world->finishTasks(world->userTaskContext);
	}
	else
	{
		b2CollideTask(0, awakeContactCount, 0, world);
	}

	// Serially update contact state
	b2TracyCZoneNC(contact_state, "Contact State", b2_colorCoral, true);

	// Bitwise OR all contact bits
	bool* bitArray = world->taskContextArray[0].contactBitArray;
	for (uint32_t i = 1; i < world->workerCount; ++i)
	{
		bool* threadBits = world->taskContextArray[i].contactBitArray;
		for (int32_t j = 0; j < awakeContactCount; ++j)
		{
			bitArray[j] |= threadBits[j];
		}
	}

	// Process contact state changes
	for (int32_t i = 0; i < awakeContactCount; ++i)
	{
		if (bitArray[i] == false)
		{
			continue;
		}

		int32_t index = world->awakeContactArray[i];
		b2Contact* contact = world->contacts + index;

		if (contact->flags & b2_contactDisjoint)
		{
			b2DestroyContact(world, contact);
		}
		else if (contact->flags & b2_contactStartedTouching)
		{
			// TODO_ERIN
			b2LinkContact(world, contact);
			contact->flags |= ~b2_contactStartedTouching;
		}
		else
		{
			assert(contact->flags & b2_contactStoppedTouching);

			b2UnlinkContact(world, contact);
			contact->flags |= ~b2_contactStoppedTouching;
		}
	}

	b2TracyCZoneEnd(contact_state);

	b2TracyCZoneEnd(collide);
}


static void b2IslandParallelForTask(int32_t startIndex, int32_t endIndex, uint32_t threadIndex, void* taskContext)
{
	B2_MAYBE_UNUSED(startIndex);
	B2_MAYBE_UNUSED(endIndex);
	B2_MAYBE_UNUSED(threadIndex);

	b2TracyCZoneNC(island_task, "Island Task", b2_colorYellow, true);

	b2World* world = taskContext;

	assert(startIndex < endIndex);
	assert(startIndex < b2Array(world->awakeIslandArray).count);
	assert(endIndex <= b2Array(world->awakeIslandArray).count);

	for (int32_t i = startIndex; i < endIndex; ++i)
	{
		int32_t index = world->awakeIslandArray[i];
		b2SolveIsland(world->islands + index);
	}

	b2TracyCZoneEnd(island_task);
}

// Solve with union-find islands
static void b2Solve(b2World* world, b2StepContext* context)
{
	b2TracyCZoneNC(solve, "Solve", b2_colorMistyRose, true);
	b2TracyCZoneNC(prepare_islands, "Prepare Islands", b2_colorDarkSalmon, true);

	b2Timer timer = b2CreateTimer();

	world->stepId += 1;

	b2MergeAwakeIslands(world);

	int32_t count = b2Array(world->awakeIslandArray).count;

	b2PersistentIsland** islands = b2AllocateStackItem(world->stackAllocator, count * sizeof(b2PersistentIsland*), "island array");
	for (int32_t i = 0; i < count; ++i)
	{
		islands[i] = world->islands + world->awakeIslandArray[i];
	}

	b2SortIslands(islands, count);

	// Now create the island solvers
	for (int32_t i = 0; i < count; ++i)
	{
		b2PrepareIsland(islands[i], context);
	}

	b2TracyCZoneEnd(prepare_islands);

	world->profile.buildIslands = b2GetMillisecondsAndReset(&timer);

	b2TracyCZoneNC(island_solver, "Island Solver", b2_colorSeaGreen, true);

	if (g_parallel)
	{
		int32_t minRange = 1;
		world->enqueueTask(&b2IslandParallelForTask, count, minRange, world, world->userTaskContext);
	}
	else
	{
		b2IslandParallelForTask(0, count, 0, world);
	}

	world->finishTasks(world->userTaskContext);

	b2TracyCZoneEnd(island_solver);

	world->profile.solveIslands = b2GetMillisecondsAndReset(&timer);

	b2TracyCZoneNC(broad_phase, "Broadphase", b2_colorPurple, true);

	// Complete islands (reverse order for stack allocator)
	// This rebuilds the awake island array and awake contact array
	b2Array_Clear(world->awakeIslandArray);
	b2Array_Clear(world->awakeContactArray);

	for (int32_t i = count - 1; i >= 0; --i)
	{
		b2PersistentIsland* island = world->islands + index;
		b2CompleteIsland(island);
	}

	b2FreeStackItem(world->stackAllocator, islands);

	// Look for new contacts
	b2BroadPhase_UpdatePairs(&world->broadPhase);

	world->profile.broadphase = b2GetMilliseconds(&timer);

	b2TracyCZoneEnd(broad_phase);

	b2TracyCZoneEnd(solve);
}

void b2World_Step(b2WorldId worldId, float timeStep, int32_t velocityIterations, int32_t positionIterations)
{
	b2TracyCZoneNC(world_step, "Step", b2_colorChartreuse, true);

	b2World* world = b2GetWorldFromId(worldId);
	assert(world->locked == false);
	if (world->locked)
	{
		return;
	}

	world->profile = b2_emptyProfile;

	b2Timer stepTimer = b2CreateTimer();

	// If new shapes were added, we need to find the new contacts.
	if (world->newContacts)
	{
		b2BroadPhase_UpdatePairs(&world->broadPhase);
		world->newContacts = false;
	}

	// TODO_ERIN atomic
	world->locked = true;

	b2StepContext context = {0};
	context.dt = timeStep;
	context.velocityIterations = velocityIterations;
	context.positionIterations = positionIterations;
	if (timeStep > 0.0f)
	{
		context.inv_dt = 1.0f / timeStep;
	}
	else
	{
		context.inv_dt = 0.0f;
	}

	context.dtRatio = world->inv_dt0 * timeStep;
	context.restitutionThreshold = world->restitutionThreshold;
	context.warmStarting = world->warmStarting;
	context.bodies = world->bodies;
	context.bodyCapacity = world->bodyPool.capacity;

	// Update contacts
	{
		b2Timer timer = b2CreateTimer();
		b2Collide(world);
		world->profile.collide = b2GetMilliseconds(&timer);
	}

	// Integrate velocities, solve velocity constraints, and integrate positions.
	if (context.dt > 0.0f)
	{
		b2Timer timer = b2CreateTimer();
		b2Solve(world, &context);
		world->profile.solve = b2GetMilliseconds(&timer);
	}

	if (context.dt > 0.0f)
	{
		world->inv_dt0 = context.inv_dt;
	}

	world->locked = false;

	world->profile.step = b2GetMilliseconds(&stepTimer);

	assert(b2GetStackAllocation(world->stackAllocator) == 0);

	b2TracyCZoneEnd(world_step);
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

			b2Color fillColor = {0.5f * color.r, 0.5f * color.g, 0.5f * color.b, 0.5f};

			if (poly->radius > 0.0f)
			{
				draw->DrawRoundedPolygon(vertices, count, poly->radius, fillColor, color, draw->context);
			}
			else
			{
				draw->DrawSolidPolygon(vertices, count, color, draw->context);
			}
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

			bool isAwake = false;
			if (b->islandIndex != B2_NULL_INDEX)
			{
				isAwake = world->islands[b->islandIndex].awakeIndex != B2_NULL_INDEX;
			}

			b2Transform xf = b->transform;
			int32_t shapeIndex = b->shapeList;
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
				else if (isAwake)
				{
					b2DrawShape(draw, shape, xf, (b2Color){0.9f, 0.7f, 0.7f, 1.0f});
				}
				else
				{
					b2DrawShape(draw, shape, xf, (b2Color){0.6f, 0.6f, 0.6f, 1.0f});
				}

				shapeIndex = shape->nextShapeIndex;
			}
		}
	}

	if (draw->drawJoints)
	{
		int32_t count = world->jointPool.capacity;
		for (int32_t i = 0; i < count; ++i)
		{
			b2Joint* joint = world->joints + i;
			if (joint->object.next != i)
			{
				continue;
			}

			b2DrawJoint(draw, world, joint);
		}
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

			int32_t shapeIndex = b->shapeList;
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
		int32_t count = world->islandPool.capacity;
		for (int32_t i = 0; i < count; ++i)
		{
			b2PersistentIsland* island = world->islands + i;
			if (island->object.next != i)
			{
				continue;
			}

			b2WakeIsland(island);
		}
	}
}

b2Profile b2World_GetProfile(b2WorldId worldId)
{
	b2World* world = b2GetWorldFromId(worldId);
	return world->profile;
}

b2Statistics b2World_GetStatistics(b2WorldId worldId)
{
	b2World* world = b2GetWorldFromId(worldId);
	b2Statistics s = {0};
	s.islandCount = world->islandPool.count;
	s.bodyCount = world->bodyPool.count;
	s.contactCount = world->contactPool.count;
	s.jointCount = world->jointPool.count;

	b2DynamicTree* tree = world->broadPhase.trees + b2_dynamicBody;
	s.proxyCount = tree->nodeCount;
	s.treeHeight = b2DynamicTree_GetHeight(tree);
	s.contactPointCount = world->contactPointCount;
	s.maxStackAllocation = b2GetMaxStackAllocation(world->stackAllocator);
	return s;
}

#if 0

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

typedef struct WorldQueryContext
{
	b2World* world;
	b2QueryCallbackFcn* fcn;
	void* userContext;
} WorldQueryContext;

static bool TreeQueryCallback(int32_t proxyId, void* userData, void* context)
{
	B2_MAYBE_UNUSED(proxyId);

	b2ShapeProxy* proxy = (b2ShapeProxy*)userData;
	WorldQueryContext* worldContext = (WorldQueryContext*)context;
	b2World* world = worldContext->world;

	assert(0 <= proxy->shapeIndex && proxy->shapeIndex < world->shapePool.capacity);

	b2Shape* shape = world->shapes + proxy->shapeIndex;
	assert(shape->object.index == shape->object.next);

	b2ShapeId shapeId = {shape->object.index, world->index, shape->object.revision};
	bool result = worldContext->fcn(shapeId, worldContext->userContext);
	return result;
}

void b2World_QueryAABB(b2WorldId worldId, b2AABB aabb, b2QueryCallbackFcn* fcn, void* context)
{
	b2World* world = b2GetWorldFromId(worldId);
	assert(world->locked == false);
	if (world->locked)
	{
		return;
	}

	WorldQueryContext worldContext = {world, fcn, context};

	for (int32_t i = 0; i < b2_bodyTypeCount; ++i)
	{
		b2DynamicTree_Query(world->broadPhase.trees + i, aabb, TreeQueryCallback, &worldContext);
	}
}

bool b2IsBodyIdValid(b2World* world, b2BodyId id)
{
	if (id.world != world->index)
	{
		return false;
	}

	if (id.index >= world->bodyPool.capacity)
	{
		return false;
	}

	b2Body* body = world->bodies + id.index;
	if (body->object.index != body->object.next)
	{
		return false;
	}

	if (body->object.revision != id.revision)
	{
		return false;
	}

	return true;
}

void b2World_SetPreSolveCallback(b2WorldId worldId, b2PreSolveFcn* fcn, void* context)
{
	b2World* world = b2GetWorldFromId(worldId);
	world->preSolveFcn = fcn;
	world->preSolveContext = context;
}

void b2World_SetPostSolveCallback(b2WorldId worldId, b2PostSolveFcn* fcn, void* context)
{
	b2World* world = b2GetWorldFromId(worldId);
	world->postSolveFcn = fcn;
	world->postSolveContext = context;
}
