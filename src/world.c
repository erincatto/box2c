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

typedef struct b2TaskContext
{
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

	world->awakeContactCapacity = world->contactPool.capacity;
	world->awakeContacts = b2Alloc(world->awakeContactCapacity * sizeof(int32_t));
	world->awakeContactCount = 0;

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

	world->contactBitArray = b2CreateArray(sizeof(bool), world->contactPool.capacity);
	
	world->taskContextArray = b2CreateArray(sizeof(b2TaskContext), world->workerCount);
	for (int32_t i = 0; i < world->workerCount; ++i)
	{
		world->taskContextArray[i].contactBitArray = b2CreateArray(sizeof(bool), world->contactPool.capacity);
	}

	return id;
}

void b2DestroyWorld(b2WorldId id)
{
	b2World* world = b2GetWorldFromId(id);

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

// Locked version
static void b2CollideTask(int32_t startIndex, int32_t endIndex, void* taskContext)
{
	b2TracyCZoneNC(collide_task, "Collide Task", b2_colorDodgerBlue1, true);

	b2World* world = taskContext;
	b2Shape* shapes = world->shapes;
	b2Body* bodies = world->bodies;
	b2Contact* contacts = world->contacts;

	const int32_t* awakeContacts = world->awakeContacts;

	assert(startIndex < endIndex);
	assert(endIndex <= world->awakeContactCount);

	for (int32_t awakeIndex = startIndex; awakeIndex < endIndex; ++awakeIndex)
	{
		int32_t contactIndex = awakeContacts[awakeIndex];
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
		// TODO_ERIN if we keep fat aabbs on shapes we don't need to dive into the broadphase here
		bool overlap = b2BroadPhase_TestOverlap(&world->broadPhase, proxyKeyA, proxyKeyB);
		if (overlap == false)
		{
			// Safely add to array of invalid contacts to be destroyed serially.
			// (relaxed)
			long index = atomic_fetch_add_long(&world->invalidContactCount, 1);
			assert(0 <= index && index < b2Array(world->contactArray).count);
			world->invalidContacts[index] = contact;
		}
		else
		{
			// Update contact respecting shape/body order (A,B)
			b2Contact_Update(world, contact, shapeA, bodyA, shapeB, bodyB);

			if (contact->flags & b2_contactTouchingFlag)
			{
				atomic_fetch_add_long(&world->contactPointCount, contact->manifold.pointCount);

				// Add to array of active contacts
				// (relaxed)
				long activeContactIndex = atomic_fetch_add_long(&world->activeContactCount, 1);
				assert(0 <= activeContactIndex && activeContactIndex < b2Array(world->contactArray).count);
#if B2_VALIDATE == 1
				int32_t contactCount = b2Array(world->contactArray).count;
				B2_MAYBE_UNUSED(contactCount);
				for (long j = 0; j < activeContactIndex; ++j)
				{
					assert(world->activeContacts[j] != contact);
				}
#endif
				world->activeContacts[activeContactIndex] = contact;

				// Is the other body asleep?
				// https://en.wikipedia.org/wiki/Double-checked_locking
				if (otherBody->type != b2_staticBody && otherAwakeIndex == B2_NULL_INDEX)
				{
					b2LockMutex(world->awakeMutex);

					// There are two values that need to be consistent
					// We need to allocate an awake index while incrementing the awake count
					// step 1: long index = word->awakeCount++;
					// And this needs to be stored in the body
					// step 2: otherBody->awakeIndex = index;
					// However between steps 1 and 2, another thread could reach step 2
					// This leaves awakeCount too large and world->awakeBodies can have duplicates
					// We can tag otherBody->awakeIndex with -2 so other threads will not attempt
					// to wake it. However other threads may need to acquire a valid awake index
					// in order to link the body in the union-find.

					// Is this the thread to wake it?
					// (relaxed)
					otherAwakeIndex = atomic_load_long(&otherBody->awakeIndex);
					if (otherAwakeIndex == B2_NULL_INDEX)
					{
						// Wake the body
						otherBody->sleepTime = 0.0f;

						// (relaxed)
						otherAwakeIndex = atomic_fetch_add_long(&world->awakeCount, 1);

						assert(otherAwakeIndex < world->bodyCapacity);

						world->awakeBodies[otherAwakeIndex] = otherBody->object.index;

						// (release)
						atomic_store_long(&otherBody->awakeIndex, otherAwakeIndex);
					}
					else
					{
						otherAwakeIndex += 0;
					}

					b2UnlockMutex(world->awakeMutex);
				}

				assert(otherBody->type == b2_staticBody || awakeIndex < otherAwakeIndex);

			}
		}
	}

	b2TracyCZoneEnd(collide_task);
}

#if 0
// Lockfree version (has race)
static void b2CollideTask(int32_t startIndex, int32_t endIndex, void* taskContext)
{
	b2TracyCZoneNC(collide_task, "Collide Task", b2_colorDodgerBlue1, true);

	b2World* world = taskContext;
	b2Shape* shapes = world->shapes;
	b2Body* bodies = world->bodies;

	// Loop awake bodies
	const int32_t* awakeBodies = world->awakeBodies;

	assert(startIndex < endIndex);
	assert(endIndex <= world->awakeCount);

	long baseCount = atomic_load_long(&world->baseAwakeCount);
	int32_t basedStartIndex = startIndex + baseCount;
	int32_t basedEndIndex = endIndex + baseCount;

	for (int32_t i = basedStartIndex; i < basedEndIndex; ++i)
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
		for (; ce != NULL; ce = ce->next)
		{
			b2Body* otherBody = world->bodies + ce->otherBodyIndex;
			long otherAwakeIndex = atomic_load_long(&otherBody->awakeIndex);

			if (otherAwakeIndex != B2_NULL_INDEX && i < otherAwakeIndex)
			{
				// avoid double evaluation
				// as sleeping bodies are woken up they are added to the end of the awake body array
				// and they have already been collided with the lower index body that woke them up
				continue;
			}

			b2Contact* contact = ce->contact;

			b2Shape* shapeA = shapes + contact->shapeIndexA;
			b2Shape* shapeB = shapes + contact->shapeIndexB;
			b2Body* bodyA = bodies + shapeA->bodyIndex;
			b2Body* bodyB = bodies + shapeB->bodyIndex;

			int32_t proxyKeyA = shapeA->proxies[contact->childA].proxyKey;
			int32_t proxyKeyB = shapeB->proxies[contact->childB].proxyKey;

			// Do proxies still overlap?
			// TODO_ERIN if we keep fat aabbs on shapes we don't need to dive into the broadphase here
			bool overlap = b2BroadPhase_TestOverlap(&world->broadPhase, proxyKeyA, proxyKeyB);
			if (overlap == false)
			{
				// Safely add to array of invalid contacts to be destroyed serially. Relaxed.
				int32_t index = atomic_fetch_add_long(&world->invalidContactCount, 1);
				assert(0 <= index && index < world->contactCapacity);
				world->invalidContacts[index] = contact;
			}
			else
			{
				// Update contact respecting shape/body order (A,B)
				b2Contact_Update(world, contact, shapeA, bodyA, shapeB, bodyB);

				if ((contact->flags & b2_contactTouchingFlag) == 0)
				{
					continue;
				}

				// Add to array of active contacts
				int32_t index = atomic_fetch_add_long(&world->activeContactCount, 1);
				assert(0 <= index && index < world->contactCapacity);
				world->activeContacts[index] = contact;

				// Don't link static bodies into island
				if (otherBody->type == b2_staticBody)
				{
					continue;
				}

				// Link the with the other body if it is awake
				long otherAwakeIndex = atomic_load_long(&otherBody->awakeIndex);
				if (otherAwakeIndex >= 0)
				{
					b2LinkContact(&world->islandBuilder, index, i, otherBody->awakeIndex);
					continue;
				}

				// There are two values that need to be consistent
				// We need to allocate an awake index while incrementing the awake count
				// step 1: long index = word->awakeCount++;
				// And this needs to be stored in the body
				// step 2: otherBody->awakeIndex = index;
				// However between steps 1 and 2, another thread could reach step 2
				// This leaves awakeCount too large and world->awakeBodies can have duplicates
				// We can tag otherBody->awakeIndex with -2 so other threads will not attempt
				// to wake it. However other threads may need to acquire a valid awake index
				// in order to link the body in the union-find.

				// Tag the body as being worked on. -2 is just used temporarily as a
				// value that is not -1, before figuring out the actual index to assign it.
				// (relax, relax)
				long otherAwakeIndexExpected = B2_NULL_INDEX;
				long otherAwakeIndexDesired = -2;
				bool shouldWake =
					atomic_compare_exchange_strong_long(&otherBody->awakeIndex, &otherAwakeIndexExpected, otherAwakeIndexDesired);

				// Is this the thread to wake it?
				if (shouldWake)
				{
					// Wake the body
					otherBody->sleepTime = 0.0f;

					// (relax)
					long awakeIndex = atomic_fetch_add_long(&world->awakeCount, 1);

					assert(awakeIndex < world->bodyCapacity);

					// (relax)
					atomic_store_long(&otherBody->awakeIndex, awakeIndex);

					world->awakeBodies[awakeIndex] = otherBody->object.index;
				}

				// TODO_ERIN race condition: no guarantee the otherBody has a valid awake index yet
				// I could spin here or I could use a mutex above, which I know already works fine
				b2LinkContact(&world->islandBuilder, index, i, atomic_load_long(&otherBody->awakeIndex));
			}
		}
	}

	//// These can only increase and base is always less or equal
	// long baseAwakeCount = atomic_load_long(&world->baseAwakeCount);
	// long awakeCount = atomic_load_long(&world->awakeCount);

	// while (awakeCount > baseAwakeCount)
	//{
	//	long start = baseAwakeCount;
	//	long count = B2_MIN(8, awakeCount - baseAwakeCount);
	//	while (atomic_compare_exchange_weak_long(&world->baseAwakeCount, &baseAwakeCount, count) == false && count > 0)
	//	{
	//		awakeCount = atomic_load_long(&world->awakeCount);
	//		start = baseAwakeCount;
	//		count = B2_MIN(8, awakeCount - baseAwakeCount);
	//	}

	//	if (count > 0)
	//	{
	//		// TODO_ERIN need to feed baseAwakeCount to job
	//		int32_t minRange = 8;
	//		world->enqueueTask(&b2CollideTask, count, minRange, world, world->userTaskContext);
	//	}
	//}

	b2TracyCZoneEnd(collide_task);
}
#endif

static void b2Collide(b2World* world)
{
	b2TracyCZoneNC(collide, "Collide", b2_colorDarkOrchid, true);

	atomic_store_long(&world->contactPointCount, 0);

	int32_t awakeCount = atomic_load_long(&world->awakeCount);
	atomic_store_long(&world->baseAwakeCount, 0);

	int32_t contactCount = b2Array(world->contactArray).count;
	B2_MAYBE_UNUSED(contactCount);

	assert(world->activeContactCount == 0);
	assert(world->invalidContactCount == 0);

#if B2_VALIDATE == 1
	memset(world->activeContacts, 0, world->contactCapacity * sizeof(b2Contact*));
#endif

	if (awakeCount == 0)
	{
		b2TracyCZoneEnd(collide);
		return;
	}

	if (g_parallel)
	{
		int32_t minRange = 8;
		int32_t baseCount = 0;
		while (baseCount < awakeCount)
		{
			world->enqueueTask(&b2CollideTask, awakeCount - baseCount, minRange, world, world->userTaskContext);
			world->finishTasks(world->userTaskContext);

			// (relaxed)
			atomic_store_long(&world->baseAwakeCount, awakeCount);
			baseCount = awakeCount;

			// (relaxed)
			awakeCount = atomic_load_long(&world->awakeCount);
		}
	}
	else
	{
		b2CollideTask(0, awakeCount, world);
	}

	// Serially destroy contacts
	b2TracyCZoneNC(destroy_contacts, "Destroy Contact", b2_colorCoral, true);

	// Relaxed
	int32_t invalidContactCount = atomic_load_long(&world->invalidContactCount);
	for (int32_t i = 0; i < invalidContactCount; ++i)
	{
		b2Contact* contact = world->invalidContacts[i];
		b2DestroyContact(world, contact);
	}
	atomic_store_long(&world->invalidContactCount, 0);
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

// Solve with union-find islands
static void b2Solve(b2World* world, b2StepContext* context)
{
	b2TracyCZoneNC(solve, "Solve", b2_colorMistyRose, true);
	b2TracyCZoneNC(island_builder, "Finish Islands", b2_colorDarkSalmon, true);

	b2Timer timer = b2CreateTimer();

	// Finish building islands
	b2IslandBuilder* builder = &world->islandBuilder;
	int32_t jointCount = atomic_load_long(&context->activeJointCount);
	assert(jointCount <= world->jointPool.count);
	int32_t contactCount = atomic_load_long(&world->activeContactCount);
	assert(contactCount <= b2Array(world->contactArray).count);
	int32_t awakeCount = atomic_load_long(&world->awakeCount);
	assert(awakeCount <= builder->bodyCapacity);
	b2StackAllocator* allocator = world->stackAllocator;
	b2FinishIslands(builder, world->awakeBodies, awakeCount, jointCount, contactCount, allocator);
	int32_t islandCount = builder->islandCount;
	if (islandCount == 0)
	{
		return;
	}

	// Now create the island solvers
	b2Island** islands = b2AllocateStackItem(allocator, islandCount * sizeof(b2Island*), "islands");
	for (int32_t i = 0; i < islandCount; ++i)
	{
		islands[i] = b2CreateIsland(builder, i, world, context);
	}

	b2TracyCZoneEnd(island_builder);

	world->profile.buildIslands = b2GetMillisecondsAndReset(&timer);

	b2TracyCZoneNC(island_solver, "Island Solver", b2_colorSeaGreen, true);

#if B2_ISLAND_PARALLEL_FOR == 1
	if (g_parallel)
	{
		int32_t minRange = 1;
		world->enqueueTask(&b2IslandParallelForTask, islandCount, minRange, islands, world->userTaskContext);
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

	// Complete and destroy islands (reverse order for stack allocator)
	// This rebuilds the awake body array
	world->awakeCount = 0;
	for (int32_t i = islandCount - 1; i >= 0; --i)
	{
		b2Island* island = islands[i];
		b2CompleteIsland(island);
		b2DestroyIsland(island);
	}

	b2FreeStackItem(allocator, islands);

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
	context.activeJoints = b2AllocateStackItem(world->stackAllocator, world->jointPool.capacity * sizeof(b2Joint*), "active joints");

	// Start island builder with maximal capacities
	const int32_t bodyCapacity = world->bodyPool.count;
	const int32_t jointCapacity = world->jointPool.count;
	const int32_t contactCapacity = b2Array(world->contactArray).count;
	b2StartIslands(&world->islandBuilder, bodyCapacity, jointCapacity, contactCapacity, world->stackAllocator);

	// Link active joints into island builder
	b2LinkActiveJoints(world, &context);

	// Update contacts. Builds contact islands. Destroy invalid contacts.
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

	// TODO_ERIN crash if paused
	b2ResetIslands(&world->islandBuilder, world->stackAllocator);
	b2FreeStackItem(world->stackAllocator, context.activeJoints);

	if (context.dt > 0.0f)
	{
		world->inv_dt0 = context.inv_dt;
	}

	// Reset active contacts
	atomic_store_long(&world->activeContactCount, 0);

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
				else if (b->awakeIndex == B2_NULL_INDEX)
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

b2Profile b2World_GetProfile(b2WorldId worldId)
{
	b2World* world = b2GetWorldFromId(worldId);
	return world->profile;
}

b2Statistics b2World_GetStatistics(b2WorldId worldId)
{
	b2World* world = b2GetWorldFromId(worldId);
	b2Statistics s = {0};
	s.bodyCount = world->bodyPool.count;
	s.contactCount = b2Array(world->contactArray).count;
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

void b2AddAwakeContact(b2World* world, b2Contact* contact)
{
	long index = atomic_fetch_add_long(&world->awakeContactCount, 1);
	assert(index < world->awakeContactCapacity);
	world->awakeContacts[index] = contact->object.index;
	contact->awakeIndex = index;
}

void b2RemoveAwakeContact(b2World* world, b2Contact* contact)
{
	int32_t awakeIndex = contact->awakeIndex;
	assert(awakeIndex != B2_NULL_INDEX && awakeIndex < world->awakeContactCount);
	world->awakeContacts[awakeIndex] = B2_NULL_INDEX;
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
