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
#include "island2.h"
#include "joint.h"
#include "pool.h"
#include "shape.h"
#include "solver_data.h"
#include "stack_allocator.h"
#include "thread.h"
#include "world.h"

#include <assert.h>
#include <string.h>

b2World g_worlds[b2_maxWorlds];
bool g_parallel = true;

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
	task(0, count, taskContext);
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

	// Search contacts on shape with the fewest contacts.
	b2ContactEdge* edge;
	int32_t otherBodyIndex;
	if (bodyA->contactCount < bodyB->contactCount)
	{
		edge = bodyA->contacts;
		otherBodyIndex = bodyIndexB;
	}
	else
	{
		edge = bodyB->contacts;
		otherBodyIndex = bodyIndexA;
	}

	int32_t childA = proxyA->childIndex;
	int32_t childB = proxyB->childIndex;

	while (edge)
	{
		if (edge->otherBodyIndex == otherBodyIndex)
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
	world->islandBuilder = b2CreateIslandBuilder(def->bodyCapacity);

	// pools
	world->bodyPool = b2CreatePool(sizeof(b2Body), B2_MAX(def->bodyCapacity, 1));
	world->bodies = (b2Body*)world->bodyPool.memory;

	world->jointPool = b2CreatePool(sizeof(b2Joint), B2_MAX(def->jointCapacity, 1));
	world->joints = (b2Joint*)world->jointPool.memory;

	world->shapePool = b2CreatePool(sizeof(b2Shape), B2_MAX(def->shapeCapacity, 1));
	world->shapes = (b2Shape*)world->shapePool.memory;

	world->contacts = b2CreateArray(sizeof(b2Contact*), B2_MAX(def->contactCapacity, 1));

	world->invalidContactMutex = b2CreateMutex("Invalid Contact");
	world->invalidContacts = b2CreateArray(sizeof(b2Contact*), 16);

	world->awakeBodies = b2CreateArray(sizeof(int32_t), def->bodyCapacity);
	world->seedBodies = b2CreateArray(sizeof(int32_t), def->bodyCapacity);

	// Globals start at 0. It should be fine for this to roll over.
	world->revision += 1;

	world->islandId = 0;

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

	// Make ground body
	b2BodyDef groundDef = b2DefaultBodyDef();
	b2BodyId groundId = b2World_CreateBody(id, &groundDef);
	world->groundBodyIndex = groundId.index;

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

	return id;
}

void b2DestroyWorld(b2WorldId id)
{
	b2World* world = b2GetWorldFromId(id);

	b2DestroyArray(world->awakeBodies);
	world->awakeBodies = NULL;

	b2DestroyArray(world->seedBodies);
	world->seedBodies = NULL;

	b2DestroyMutex(world->invalidContactMutex);
	world->invalidContactMutex = NULL;
	b2DestroyArray(world->invalidContacts);
	world->invalidContacts = NULL;

	b2DestroyPool(&world->shapePool);
	world->shapes = NULL;

	b2DestroyPool(&world->jointPool);
	world->joints = NULL;

	b2DestroyPool(&world->bodyPool);
	world->bodies = NULL;

	b2BroadPhase_Destroy(&world->broadPhase);
	b2DestroyIslandBuilder(&world->islandBuilder);

	b2DestroyBlockAllocator(world->blockAllocator);
	world->blockAllocator = NULL;

	b2DestroyStackAllocator(world->stackAllocator);
	world->stackAllocator = NULL;
}

static void b2CollideTask(int32_t startIndex, int32_t endIndex, void* taskContext)
{
	b2TracyCZoneC(collide_task, b2_colorYellow, true);

	b2World* world = taskContext;

	// Loop awake bodies
	const int32_t* awakeBodies = world->awakeBodies;

	assert(startIndex < endIndex);
	assert(endIndex <= b2Array(awakeBodies).count);

	for (int32_t i = startIndex; i < endIndex; ++i)
	{
		int32_t bodyIndex = awakeBodies[i];
		if (bodyIndex == B2_NULL_INDEX)
		{
			// Body was destroyed or put to sleep
			continue;
		}

		assert(0 <= bodyIndex && bodyIndex < world->bodyPool.capacity);

		b2Body* body = world->bodies + bodyIndex;

		assert(body->object.index == bodyIndex && body->object.index == body->object.next);
		assert(body->type != b2_staticBody);

		b2ContactEdge* ce = body->contacts;
		while (ce != NULL)
		{
			b2Body* otherBody = world->bodies + ce->otherBodyIndex;
			if (otherBody->awakeIndex != B2_NULL_INDEX && bodyIndex > ce->otherBodyIndex)
			{
				// avoid double evaluation
				ce = ce->next;
				continue;
			}

			b2Contact* contact = ce->contact;

			b2Shape* shapeA = world->shapes + contact->shapeIndexA;
			b2Shape* shapeB = world->shapes + contact->shapeIndexB;
			b2Body* bodyA = world->bodies + shapeA->bodyIndex;
			b2Body* bodyB = world->bodies + shapeB->bodyIndex;

			int32_t proxyKeyA = shapeA->proxies[contact->childA].proxyKey;
			int32_t proxyKeyB = shapeB->proxies[contact->childB].proxyKey;

			// Do proxies still overlap?
			// TODO_ERIN if we keep fat aabbs on shapes we don't need to dive into the broadphase here
			bool overlap = b2BroadPhase_TestOverlap(&world->broadPhase, proxyKeyA, proxyKeyB);
			if (overlap == false)
			{
				ce = ce->next;

				// Safely make array of invalid contacts to be destroyed serially
				// TODO_ERIN use CAS
				b2LockMutex(world->invalidContactMutex);
				b2Array_Push(world->invalidContacts, contact);
				b2UnlockMutex(world->invalidContactMutex);

				continue;
			}

			// Update contact respecting shape/body order (A,B)
			b2Contact_Update(world, contact, shapeA, bodyA, shapeB, bodyB);

			ce = ce->next;
		}
	}

	b2TracyCZoneEnd(collide_task);
}

static void b2Collide(b2World* world)
{
	b2TracyCZoneC(collide, b2_colorDarkOrchid, true);

	const int32_t* awakeBodies = world->awakeBodies;
	int32_t count = b2Array(awakeBodies).count;

	if (count == 0)
	{
		b2TracyCZoneEnd(collide);
		return;
	}

	if (g_parallel)
	{
		int32_t minRange = 16;
		world->enqueueTask(&b2CollideTask, count, minRange, world, world->userTaskContext);
		world->finishTasks(world->userTaskContext);
	}
	else
	{
		b2CollideTask(0, count, world);
	}

	// Serially destroy contacts
	b2TracyCZoneNC(destroy_contacts, "Destroy Contact", b2_colorCoral, true);
	int32_t invalidContactCount = b2Array(world->invalidContacts).count;
	for (int32_t i = 0; i < invalidContactCount; ++i)
	{
		b2Contact* contact = world->invalidContacts[i];
		b2DestroyContact(world, contact);
	}
	b2Array_Clear(world->invalidContacts);
	b2TracyCZoneEnd(destroy_contacts);

	b2TracyCZoneEnd(collide);
}

#define B2_ISLAND_PARALLEL_FOR 1

#if B2_ISLAND_PARALLEL_FOR == 0
static void b2IslandTask(int32_t startIndex, int32_t endIndex, void* taskContext)
{
	B2_MAYBE_UNUSED(startIndex);
	B2_MAYBE_UNUSED(endIndex);

	b2TracyCZoneNC(island_task, "Island Task", b2_colorYellow, true);

	b2Island* island = taskContext;
	b2SolveIsland(island);

	b2TracyCZoneEnd(island_task);
}
#else
static void b2IslandParallelForTask(int32_t startIndex, int32_t endIndex, void* taskContext)
{
	B2_MAYBE_UNUSED(startIndex);
	B2_MAYBE_UNUSED(endIndex);

	b2TracyCZoneNC(island_task, "Island Task", b2_colorYellow, true);

	b2Island** islands = taskContext;

	assert(startIndex < endIndex);

	for (int32_t i = startIndex; i < endIndex; ++i)
	{
		b2SolveIsland(islands[i]);
	}

	b2TracyCZoneEnd(island_task);
}
#endif

// Find islands, integrate equations of motion and solve constraints.
// Also reports contact results and updates sleep.
static void b2Solve(b2World* world, const b2TimeStep* step)
{
	b2TracyCZoneC(solve, b2_colorMistyRose, true);
	b2TracyCZoneNC(island_builder, "Island Builder", b2_colorDarkSalmon, true);

	b2Timer timer = b2CreateTimer();

	world->contactPointCount = 0;

	// Island buffers
	const int32_t bodyCapacity = world->bodyPool.count;
	const int32_t jointCapacity = world->jointPool.count;
	const int32_t contactCapacity = b2Array(world->contacts).count;

	// Body island indices could be stored in b2Body, but they are only used in this scope. Also
	// this is safer because static bodies can have multiple island indices within this scope.
	int32_t* islandIndices = b2AllocateStackItem(world->stackAllocator, world->bodyPool.capacity * sizeof(int32_t));
	b2Island** islands = b2AllocateStackItem(world->stackAllocator, world->bodyPool.capacity * sizeof(b2Island*));

	b2Body** bodies = b2AllocateStackItem(world->stackAllocator, bodyCapacity * sizeof(b2Body*));
	b2Joint** joints = b2AllocateStackItem(world->stackAllocator, jointCapacity * sizeof(b2Joint*));
	b2Contact** contacts = b2AllocateStackItem(world->stackAllocator, contactCapacity * sizeof(b2Contact*));

#if defined(_DEBUG)
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
#endif

	// Swap awake body buffer
	{
		int32_t* temp = world->awakeBodies;
		world->awakeBodies = world->seedBodies;
		world->seedBodies = temp;
	}

	// The awake bodies are the seeds for the island depth first search
	const int32_t* seedBuffer = world->seedBodies;
	b2Array_Clear(world->awakeBodies);

	int32_t seedCount = b2Array(seedBuffer).count;

	uint64_t baseId = world->islandId;
	uint64_t islandId = baseId;

	// Build and simulate all awake islands.
	int32_t* stack = (int32_t*)b2AllocateStackItem(world->stackAllocator, bodyCapacity * sizeof(int32_t));
	b2Island* islandList = NULL;

#if B2_ISLAND_PARALLEL_FOR == 1
	int32_t islandCount = 0;
#endif

	// Each island is found as a depth first search starting from a seed body
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

		if (seed->islandId > baseId)
		{
			// The body is already in an island
			continue;
		}

		assert(seed->isAwake);

		// Reset stack, bump island id
		int32_t stackCount = 0;
		stack[stackCount++] = seedIndex;
		++islandId;

		// Add seed to island
		seed->islandId = islandId;
		islandIndices[seedIndex] = 0;
		bodies[0] = seed;

		int32_t bodyCount = 1;
		int32_t jointCount = 0;
		int32_t contactCount = 0;

		// Perform a depth first search (DFS) on the constraint graph.
		while (stackCount > 0)
		{
			// Grab the next body off the stack and add it to the island.
			int32_t bodyIndex = stack[--stackCount];
			b2Body* b = world->bodies + bodyIndex;
			assert(b->type != b2_staticBody);

			// The awake body array is being rebuilt so the awake index is no longer valid
			b->awakeIndex = B2_NULL_INDEX;

			// Make sure the body is awake (without resetting sleep timer).
			b->isAwake = true;

			// Search all contacts connected to this body.
			for (b2ContactEdge* ce = b->contacts; ce; ce = ce->next)
			{
				b2Contact* contact = ce->contact;

				// Has this contact already been added to this island?
				if (contact->islandId == seed->islandId)
				{
					continue;
				}

				// Skip sensors
				if (contact->flags & b2_contactSensorFlag)
				{
					continue;
				}

				// Is this contact solid and touching?
				if ((contact->flags & b2_contactEnabledFlag) == 0 || (contact->flags & b2_contactTouchingFlag) == 0)
				{
					continue;
				}

				int32_t otherBodyIndex = ce->otherBodyIndex;
				b2Body* otherBody = world->bodies + otherBodyIndex;

				// Maybe add other body to island
				if (otherBody->islandId != islandId)
				{
					otherBody->islandId = islandId;
					islandIndices[otherBodyIndex] = bodyCount;
					bodies[bodyCount++] = otherBody;
					assert(otherBody->isEnabled == true);

					if (otherBody->type != b2_staticBody)
					{
						assert(stackCount < bodyCapacity);
						stack[stackCount++] = otherBodyIndex;
					}
				}

				world->contactPointCount += contact->manifold.pointCount;

				// Add contact to island
				contact->islandId = islandId;
				if (ce == &contact->edgeA)
				{
					contact->islandIndexA = islandIndices[bodyIndex];
					contact->islandIndexB = islandIndices[otherBodyIndex];
				}
				else
				{
					contact->islandIndexA = islandIndices[otherBodyIndex];
					contact->islandIndexB = islandIndices[bodyIndex];
				}

				assert(0 <= contact->islandIndexA && contact->islandIndexA < world->bodyPool.capacity);
				assert(0 <= contact->islandIndexB && contact->islandIndexB < world->bodyPool.capacity);

				contacts[contactCount++] = contact;
			}

			// Search all joints connect to this body.
			int32_t jointIndex = b->jointIndex;
			while (jointIndex != B2_NULL_INDEX)
			{
				b2Joint* joint = world->joints + jointIndex;
				assert(joint->object.index == jointIndex);

				bool isBodyA;
				int32_t otherBodyIndex;
				if (joint->edgeA.bodyIndex == b->object.index)
				{
					jointIndex = joint->edgeA.nextJointIndex;
					otherBodyIndex = joint->edgeB.bodyIndex;
					isBodyA = true;
				}
				else
				{
					assert(joint->edgeB.bodyIndex == b->object.index);
					jointIndex = joint->edgeB.nextJointIndex;
					otherBodyIndex = joint->edgeA.bodyIndex;
					isBodyA = false;
				}

				// Has this joint already been added to this island?
				if (joint->islandId == islandId)
				{
					continue;
				}

				b2Body* otherBody = world->bodies + otherBodyIndex;

				// Don't simulate joints connected to disabled bodies.
				if (otherBody->isEnabled == false)
				{
					continue;
				}

				// Maybe add other body to island
				if (otherBody->islandId != islandId)
				{
					otherBody->islandId = islandId;
					islandIndices[otherBodyIndex] = bodyCount;
					bodies[bodyCount++] = otherBody;
					assert(otherBody->isEnabled == true);

					if (otherBody->type != b2_staticBody)
					{
						assert(stackCount < bodyCapacity);
						stack[stackCount++] = otherBodyIndex;
					}
				}

				// Add joint to island
				joint->islandId = islandId;
				if (isBodyA)
				{
					joint->islandIndexA = islandIndices[bodyIndex];
					joint->islandIndexB = islandIndices[otherBodyIndex];
				}
				else
				{
					joint->islandIndexA = islandIndices[otherBodyIndex];
					joint->islandIndexB = islandIndices[bodyIndex];
				}

				assert(0 <= joint->islandIndexA && joint->islandIndexA < world->bodyPool.capacity);
				assert(0 <= joint->islandIndexB && joint->islandIndexB < world->bodyPool.capacity);

				joints[jointCount++] = joint;
			}
		}

		// Create island and add to linked list
		b2Island* island = b2CreateIsland(bodies, bodyCount, contacts, contactCount, joints, jointCount, world, step);
		island->nextIsland = islandList;
		islandList = island;

#if B2_ISLAND_PARALLEL_FOR == 0
		if (g_parallel)
		{
			world->enqueueTask(&b2IslandTask, 1, 1, island, world->userTaskContext);
		}
		else
		{
			b2IslandTask(0, 1, island);
		}
#else
		assert(islandCount < world->bodyPool.capacity);
		islands[islandCount++] = island;
#endif
	}

	b2TracyCZoneEnd(island_builder);

	world->profile.buildIslands = b2GetMillisecondsAndReset(&timer);

	b2TracyCZoneNC(island_solver, "Island Solver", b2_colorSeaGreen, true);

#if B2_ISLAND_PARALLEL_FOR == 1
	if (g_parallel)
	{
		world->enqueueTask(&b2IslandParallelForTask, islandCount, 1, islands, world->userTaskContext);
	}
	else
	{
		b2IslandParallelForTask(0, islandCount, islands);
	}
#endif

	world->finishTasks(world->userTaskContext);

	b2TracyCZoneEnd(island_solver);

	world->profile.solveIslands = b2GetMillisecondsAndReset(&timer);

	b2TracyCZoneNC(broad_phase, "Broadphase", b2_colorPurple, true);

	// Complete and destroy islands in reverse order
	b2Island* island = islandList;
	while (island)
	{
		b2CompleteIsland(island);
		b2Island* next = island->nextIsland;
		b2DestroyIsland(island);
		island = next;
	}

	b2FreeStackItem(world->stackAllocator, stack);
	b2FreeStackItem(world->stackAllocator, contacts);
	b2FreeStackItem(world->stackAllocator, joints);
	b2FreeStackItem(world->stackAllocator, bodies);
	b2FreeStackItem(world->stackAllocator, islands);
	b2FreeStackItem(world->stackAllocator, islandIndices);

	// Look for new contacts
	b2BroadPhase_UpdatePairs(&world->broadPhase);

	// Store new island id
	world->islandId = islandId;

	world->profile.broadphase = b2GetMilliseconds(&timer);

	b2TracyCZoneEnd(broad_phase);

	b2TracyCZoneEnd(solve);
}

// Solve with union-find islands
static void b2Solve2(b2World* world, const b2TimeStep* step)
{
	b2TracyCZoneC(solve, b2_colorMistyRose, true);
	b2TracyCZoneNC(island_builder, "Island Builder", b2_colorDarkSalmon, true);

	b2Timer timer = b2CreateTimer();

	world->contactPointCount = 0;

	b2IslandBuilder* builder = &world->islandBuilder;

	const int32_t jointCount = world->jointPool.count;
	const int32_t contactCount = b2Array(world->contacts).count;

	b2StackAllocator* allocator = world->stackAllocator;

	b2InitializeIslands(builder, contactCount, jointCount, allocator);

	b2Body* bodies = world->bodies;
	b2Joint* joints = world->joints;
	b2Contact** contacts = world->contacts;

	// TODO_ERIN inefficient
	for (int32_t i = 0; i < jointCount; ++i)
	{
		b2Joint* joint = joints + i;
		if (joint->object.next == joint->object.index)
		{
			int32_t indexA = joint->edgeA.bodyIndex;
			int32_t indexB = joint->edgeB.bodyIndex;
			b2Body* bodyA = bodies + indexA;
			b2Body* bodyB = bodies + indexB;

			assert(bodyA->object.index == bodyA->object.next);
			assert(bodyB->object.index == bodyB->object.next);

			if (bodyA->awakeIndex != B2_NULL_INDEX || bodyB->awakeIndex != B2_NULL_INDEX)
			{
				b2LinkJoint(builder, i, indexA, indexB);
			}
		}
	}

	// TODO_ERIN inefficient
	for (int32_t i = 0; i < contactCount; ++i)
	{
		b2Contact* contact = contacts[i];

		int32_t indexA = contact->edgeB.otherBodyIndex;
		int32_t indexB = contact->edgeA.otherBodyIndex;
		b2Body* bodyA = bodies + indexA;
		b2Body* bodyB = bodies + indexB;

		assert(bodyA->object.index == bodyA->object.next);
		assert(bodyB->object.index == bodyB->object.next);

		if (bodyA->awakeIndex != B2_NULL_INDEX || bodyB->awakeIndex != B2_NULL_INDEX)
		{
			b2LinkContact(builder, i, indexA, indexB);
		}
	}

	// Swap awake body buffer
	{
		int32_t* temp = world->awakeBodies;
		world->awakeBodies = world->seedBodies;
		world->seedBodies = temp;
	}

	// The awake bodies are the seeds for the island depth first search
	const int32_t* seedBuffer = world->seedBodies;
	b2Array_Clear(world->awakeBodies);

	int32_t seedCount = b2Array(seedBuffer).count;

	b2FinalizeIslands(builder, seedBuffer, seedCount, contactCount, allocator);

	int32_t islandCount = builder->islandCount;

	b2Island2** islands = b2AllocateStackItem(allocator, islandCount * sizeof(b2Island2*));
	for (int32_t i = 0; i < islandCount; ++i)
	{
		islands[i] = b2CreateIsland2(builder, i, world, step);
	}

	b2TracyCZoneEnd(island_builder);

	world->profile.buildIslands = b2GetMillisecondsAndReset(&timer);

	b2TracyCZoneNC(island_solver, "Island Solver", b2_colorSeaGreen, true);

#if B2_ISLAND_PARALLEL_FOR == 1
	if (g_parallel)
	{
		world->enqueueTask(&b2IslandParallelForTask, islandCount, 1, islands, world->userTaskContext);
	}
	else
	{
		b2IslandParallelForTask(0, islandCount, islands);
	}
#endif

	world->finishTasks(world->userTaskContext);

	b2TracyCZoneEnd(island_solver);

	world->profile.solveIslands = b2GetMillisecondsAndReset(&timer);

	b2TracyCZoneNC(broad_phase, "Broadphase", b2_colorPurple, true);

	// Complete and destroy islands
	for (int32_t i = 0; i < islandCount; ++i)
	{
		b2Island2* island = islands[i];
		b2CompleteIsland2(island);
		b2DestroyIsland2(island);
	}

	b2FreeStackItem(world->stackAllocator, islands);
	b2DestroyIslands(builder, allocator);

	// Look for new contacts
	b2BroadPhase_UpdatePairs(&world->broadPhase);

	world->profile.broadphase = b2GetMilliseconds(&timer);

	b2TracyCZoneEnd(broad_phase);

	b2TracyCZoneEnd(solve);
}

void b2World_Step(b2WorldId worldId, float timeStep, int32_t velocityIterations, int32_t positionIterations)
{
	b2TracyCZoneC(step_ctx, b2_colorChartreuse, true);

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
	step.restitutionThreshold = world->restitutionThreshold;
	step.warmStarting = world->warmStarting;

	// Update contacts. This is where some contacts are destroyed.
	{
		b2Timer timer = b2CreateTimer();
		b2Collide(world);
		world->profile.collide = b2GetMilliseconds(&timer);
	}

	// Integrate velocities, solve velocity constraints, and integrate positions.
	if (step.dt > 0.0f)
	{
		b2Timer timer = b2CreateTimer();
		b2Solve2(world, &step);
		world->profile.solve = b2GetMilliseconds(&timer);
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

	assert(b2GetStackAllocation(world->stackAllocator) == 0);

	b2TracyCZoneEnd(step_ctx);
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

			b2SetAwake(world, b, true);
		}
	}
}

b2Profile* b2World_GetProfile(b2WorldId worldId)
{
	b2World* world = b2GetWorldFromId(worldId);
	return &world->profile;
}

b2Statistics b2World_GetStatistics(b2WorldId worldId)
{
	b2World* world = b2GetWorldFromId(worldId);
	b2Statistics s = {0};
	s.bodyCount = world->bodyPool.count;
	s.contactCount = b2Array(world->contacts).count;
	s.jointCount = world->jointPool.count;

	b2DynamicTree* tree = world->broadPhase.trees + b2_dynamicBody;
	s.proxyCount = tree->nodeCount;
	s.treeHeight = b2DynamicTree_GetHeight(tree);
	s.contactPointCount = world->contactPointCount;
	s.maxStackAllocation = b2GetMaxStackAllocation(world->stackAllocator);
	return s;
}

b2BodyId b2World_GetGroundBodyId(b2WorldId worldId)
{
	b2World* world = b2GetWorldFromId(worldId);
	b2Body* body = world->bodies + world->groundBodyIndex;
	b2BodyId bodyId = {world->groundBodyIndex, world->index, body->object.revision};
	return bodyId;
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
