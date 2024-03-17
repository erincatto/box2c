// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#define _CRT_SECURE_NO_WARNINGS

#include "world.h"

#include "aabb.h"
#include "allocate.h"
#include "stack_allocator.h"
#include "array.h"
#include "bitset.h"
#include "block_allocator.h"
#include "block_array.h"
#include "body.h"
#include "broad_phase.h"
#include "constraint_graph.h"
#include "contact.h"
#include "core.h"
#include "ctz.h"
#include "island.h"
#include "joint.h"
#include "pool.h"
#include "shape.h"
#include "solver.h"
#include "solver_set.h"
#include "util.h"

// needed for dll export
#include "box2d/box2d.h"
#include "box2d/color.h"
#include "box2d/constants.h"
#include "box2d/debug_draw.h"
#include "box2d/distance.h"
#include "box2d/event_types.h"
#include "box2d/timer.h"

#include <float.h>
#include <stdio.h>
#include <string.h>

b2World b2_worlds[b2_maxWorlds];

b2World* b2GetWorldFromId(b2WorldId id)
{
	B2_ASSERT(1 <= id.index1 && id.index1 <= b2_maxWorlds);
	b2World* world = b2_worlds + (id.index1 - 1);
	B2_ASSERT(id.index1 == world->worldId + 1);
	B2_ASSERT(id.revision == world->revision);
	return world;
}

b2World* b2GetWorld(int index)
{
	B2_ASSERT(0 <= index && index < b2_maxWorlds);
	b2World* world = b2_worlds + index;
	B2_ASSERT(world->worldId == index);
	return world;
}

b2World* b2GetWorldLocked(int index)
{
	B2_ASSERT(0 <= index && index < b2_maxWorlds);
	b2World* world = b2_worlds + index;
	B2_ASSERT(world->worldId == index);
	if (world->locked)
	{
		B2_ASSERT(false);
		return NULL;
	}

	return world;
}

static void* b2DefaultAddTaskFcn(b2TaskCallback* task, int32_t count, int32_t minRange, void* taskContext, void* userContext)
{
	B2_MAYBE_UNUSED(minRange);
	B2_MAYBE_UNUSED(userContext);
	task(0, count, 0, taskContext);
	return NULL;
}

static void b2DefaultFinishTaskFcn(void* userTask, void* userContext)
{
	B2_MAYBE_UNUSED(userTask);
	B2_MAYBE_UNUSED(userContext);
}

static void* b2DefaultAddPinnedTaskFcn(b2PinnedTaskFcn* task, int32_t threadIndex, void* taskContext, void* userContext)
{
	B2_MAYBE_UNUSED(userContext);
	task(threadIndex, taskContext);
	return NULL;
}

b2WorldId b2CreateWorld(const b2WorldDef* def)
{
	_Static_assert(b2_maxWorlds < UINT16_MAX, "b2_maxWorlds limit exceeded");

	int worldId = B2_NULL_INDEX;
	for (int i = 0; i < b2_maxWorlds; ++i)
	{
		if (b2_worlds[i].inUse == false)
		{
			worldId = i;
			break;
		}
	}

	if (worldId == B2_NULL_INDEX)
	{
		return (b2WorldId){0};
	}

	b2InitializeContactRegisters();

	b2World* world = b2_worlds + worldId;
	*world = (b2World){0};

	world->worldId = (uint16_t)worldId;
	world->inUse = true;

	world->blockAllocator = b2CreateBlockAllocator();
	world->stackAllocator = b2CreateStackAllocator(def->stackAllocatorCapacity);
	b2CreateBroadPhase(&world->broadPhase);
	b2CreateGraph(&world->constraintGraph, &world->blockAllocator, def->bodyCapacity);

	// pools
	world->bodyIdPool = b2CreateIdPool();
	world->bodyLookupArray = b2CreateArray(sizeof(b2BodyLookup), def->bodyCapacity);
	world->solverSetArray = b2CreateArray(sizeof(b2SolverSet), 8);

	// add empty static, active, and disabled body sets
	world->solverSetIdPool = b2CreateIdPool();
	b2SolverSet set = {0};
	set.solverSetId = b2AllocId(&world->solverSetIdPool);
	B2_ASSERT(set.solverSetId == b2_staticSet);
	b2Array_Push(world->solverSetArray, set);
	set.solverSetId = b2AllocId(&world->solverSetIdPool);
	B2_ASSERT(set.solverSetId == b2_awakeSet);
	b2Array_Push(world->solverSetArray, set);
	set.solverSetId = b2AllocId(&world->solverSetIdPool);
	B2_ASSERT(set.solverSetId == b2_disabledSet);
	b2Array_Push(world->solverSetArray, set);

	world->shapePool = b2CreatePool(sizeof(b2Shape), B2_MAX(def->shapeCapacity, 1));
	world->shapes = (b2Shape*)world->shapePool.memory;

	world->chainPool = b2CreatePool(sizeof(b2ChainShape), 4);
	world->chains = (b2ChainShape*)world->chainPool.memory;

	world->contactIdPool = b2CreateIdPool();
	world->contactLookupArray = b2CreateArray(sizeof(b2ContactLookup), 16);

	world->jointIdPool = b2CreateIdPool();
	world->jointLookupArray = b2CreateArray(sizeof(b2JointLookup), 16);

	world->islandIdPool = b2CreateIdPool();
	world->islandLookupArray = b2CreateArray(sizeof(b2IslandLookup), 8);

	world->bodyMoveEventArray = b2CreateArray(sizeof(b2BodyMoveEvent), 4);
	world->sensorBeginEventArray = b2CreateArray(sizeof(b2SensorBeginTouchEvent), 4);
	world->sensorEndEventArray = b2CreateArray(sizeof(b2SensorEndTouchEvent), 4);
	world->contactBeginArray = b2CreateArray(sizeof(b2ContactBeginTouchEvent), 4);
	world->contactEndArray = b2CreateArray(sizeof(b2ContactEndTouchEvent), 4);

	world->stepIndex = 0;
	world->activeTaskCount = 0;
	world->taskCount = 0;
	world->gravity = def->gravity;
	world->restitutionThreshold = def->restitutionThreshold;
	world->contactPushoutVelocity = def->contactPushoutVelocity;
	world->contactHertz = def->contactHertz;
	world->contactDampingRatio = def->contactDampingRatio;
	world->jointHertz = def->jointHertz;
	world->jointDampingRatio = def->jointDampingRatio;
	world->enableSleep = def->enableSleep;
	world->locked = false;
	world->enableWarmStarting = true;
	world->enableContinuous = def->enableContinous;
	world->userTreeTask = NULL;

	if (def->workerCount > 0 && def->enqueueTask != NULL && def->finishTask != NULL)
	{
		world->workerCount = B2_MIN(def->workerCount, b2_maxWorkers);
		world->enqueueTaskFcn = def->enqueueTask;
		world->finishTaskFcn = def->finishTask;
		world->addPinnedTaskFcn = def->addPinnedTask;
		world->finishPinnedTaskFcn = def->finishPinnedTask;
		world->userTaskContext = def->userTaskContext;
	}
	else
	{
		world->workerCount = 1;
		world->enqueueTaskFcn = b2DefaultAddTaskFcn;
		world->finishTaskFcn = b2DefaultFinishTaskFcn;
		world->addPinnedTaskFcn = b2DefaultAddPinnedTaskFcn;
		world->finishPinnedTaskFcn = b2DefaultFinishTaskFcn;
		world->userTaskContext = NULL;
	}

	world->taskContextArray = b2CreateArray(sizeof(b2TaskContext), world->workerCount);
	for (uint32_t i = 0; i < world->workerCount; ++i)
	{
		world->taskContextArray[i].contactStateBitSet = b2CreateBitSet(def->contactCapacity);
		world->taskContextArray[i].shapeBitSet = b2CreateBitSet(def->shapeCapacity);
		world->taskContextArray[i].awakeIslandBitSet = b2CreateBitSet(256);
	}

	// add one to worldId so that 0 represents a null b2WorldId
	return (b2WorldId){(uint16_t)(worldId + 1), world->revision};
}

void b2DestroyWorld(b2WorldId worldId)
{
	b2World* world = b2GetWorldFromId(worldId);

	for (uint32_t i = 0; i < world->workerCount; ++i)
	{
		b2DestroyBitSet(&world->taskContextArray[i].contactStateBitSet);
		b2DestroyBitSet(&world->taskContextArray[i].shapeBitSet);
		b2DestroyBitSet(&world->taskContextArray[i].awakeIslandBitSet);
	}

	b2DestroyArray(world->taskContextArray, sizeof(b2TaskContext));

	b2DestroyArray(world->bodyMoveEventArray, sizeof(b2BodyMoveEvent));
	b2DestroyArray(world->sensorBeginEventArray, sizeof(b2SensorBeginTouchEvent));
	b2DestroyArray(world->sensorEndEventArray, sizeof(b2SensorEndTouchEvent));
	b2DestroyArray(world->contactBeginArray, sizeof(b2ContactBeginTouchEvent));
	b2DestroyArray(world->contactEndArray, sizeof(b2ContactEndTouchEvent));

	int32_t chainCapacity = world->chainPool.capacity;
	for (int32_t i = 0; i < chainCapacity; ++i)
	{
		b2ChainShape* chain = world->chains + i;
		if (b2IsValidObject(&chain->object))
		{
			b2Free(chain->shapeIndices, chain->count * sizeof(int32_t));
		}
	}

	b2DestroyPool(&world->shapePool);
	b2DestroyPool(&world->chainPool);

	b2DestroyIdPool(&world->bodyIdPool);
	b2DestroyIdPool(&world->contactIdPool);
	b2DestroyIdPool(&world->jointIdPool);
	b2DestroyIdPool(&world->islandIdPool);
	b2DestroyIdPool(&world->solverSetIdPool);
	b2DestroyArray(world->bodyLookupArray, sizeof(b2BodyLookup));
	b2DestroyArray(world->contactLookupArray, sizeof(b2ContactLookup));
	b2DestroyArray(world->jointLookupArray, sizeof(b2JointLookup));
	b2DestroyArray(world->islandLookupArray, sizeof(b2IslandLookup));

	// The data in the body sets all comes from the block allocator so no
	// need to destroy the set contents.
	b2DestroyArray(world->solverSetArray, sizeof(b2SolverSet));

	b2DestroyGraph(&world->constraintGraph);
	b2DestroyBroadPhase(&world->broadPhase);

	b2DestroyBlockAllocator(&world->blockAllocator);
	b2DestroyStackAllocator(&world->stackAllocator);

	// Wipe world but preserve revision
	uint16_t revision = world->revision;
	*world = (b2World){0};
	world->revision = revision + 1;
}

static void b2CollideTask(int32_t startIndex, int32_t endIndex, uint32_t threadIndex, void* context)
{
	b2TracyCZoneNC(collide_task, "Collide Task", b2_colorDodgerBlue1, true);

	b2StepContext* stepContext = context;
	b2World* world = stepContext->world;
	B2_ASSERT(threadIndex < world->workerCount);
	b2TaskContext* taskContext = world->taskContextArray + threadIndex;
	b2Contact** contacts = stepContext->contacts;
	b2Shape* shapes = world->shapes;
	b2BodyLookup* bodyLookup = world->bodyLookupArray;

	B2_ASSERT(startIndex < endIndex);

	for (int i = startIndex; i < endIndex; ++i)
	{
		b2Contact* contact = contacts[i];

		// Reset contact awake index. Contacts must be added to the awake contact array
		// each time step in the island solver.

		b2Shape* shapeA = shapes + contact->shapeIndexA;
		b2Shape* shapeB = shapes + contact->shapeIndexB;

		b2BodyLookup lookupA = bodyLookup[shapeA->bodyId];
		b2BodyLookup lookupB = bodyLookup[shapeB->bodyId];

		if (lookupA.setIndex != b2_awakeSet && lookupB.setIndex != b2_awakeSet)
		{
			B2_ASSERT(lookupA.setIndex != b2_disabledSet);
			B2_ASSERT(lookupB.setIndex != b2_disabledSet);
			B2_ASSERT(lookupA.setIndex >= b2_firstSleepingSet || lookupB.setIndex >= b2_firstSleepingSet);
			// contact needs to be moved to sleeping set, but what if both bodies are sleeping in different sets?
			// perhaps there should be a separate place for sleeping non-touching contacts
			// certainly when a non-touching contact is woken up, it should not go into the constraint graph
			// but we are iterating contacts in the constraint graph, so where should awake non-touching contacts go?
			// perhaps the world should hold an array of non-touching awake contacts and non-touching sleeping contacts
			// the non-touching sleeping contacts should be moved to the non-touching awake contacts whenever either body
			// becomes awake
		}

		// Do proxies still overlap?
		bool overlap = b2AABB_Overlaps(shapeA->fatAABB, shapeB->fatAABB);
		if (overlap == false)
		{
			contact->flags |= b2_contactDisjoint;
			b2SetBit(&taskContext->contactStateBitSet, contact->contactId);
		}
		else
		{
			bool wasTouching = (contact->flags & b2_contactTouchingFlag);
			B2_ASSERT(wasTouching || contact->islandId == B2_NULL_INDEX);

			// Update contact respecting shape/body order (A,B)
			b2Body* bodyA = b2GetBodyFromRawId(world, shapeA->bodyId);
			b2Body* bodyB = b2GetBodyFromRawId(world, shapeB->bodyId);
			b2UpdateContact(world, contact, shapeA, bodyA, shapeB, bodyB);

			bool touching = (contact->flags & b2_contactTouchingFlag) != 0;

			// State changes that affect island connectivity
			if (touching == true && wasTouching == false)
			{
				contact->flags |= b2_contactStartedTouching;
				b2SetBit(&taskContext->contactStateBitSet, contact->contactId);
			}
			else if (touching == false && wasTouching == true)
			{
				contact->flags |= b2_contactStoppedTouching;
				b2SetBit(&taskContext->contactStateBitSet, contact->contactId);
			}
		}
	}

	b2TracyCZoneEnd(collide_task);
}

static void b2UpdateTreesTask(int32_t startIndex, int32_t endIndex, uint32_t threadIndex, void* context)
{
	B2_MAYBE_UNUSED(startIndex);
	B2_MAYBE_UNUSED(endIndex);
	B2_MAYBE_UNUSED(threadIndex);

	b2TracyCZoneNC(tree_task, "Rebuild Trees", b2_colorSnow1, true);

	b2World* world = context;
	b2BroadPhase_RebuildTrees(&world->broadPhase);

	b2TracyCZoneEnd(tree_task);
}

static void b2SyncContactLookup(b2World* world, b2Contact* contact, int setIndex)
{
	B2_ASSERT(0 <= contact->contactId && contact->contactId < b2Array(world->contactLookupArray).count);
	b2ContactLookup* lookup = world->contactLookupArray + contact->contactId;
	lookup->setIndex = setIndex;
	lookup->colorIndex = contact->colorIndex;
	lookup->contactIndex = contact->localIndex;
}

static b2Contact* b2AddNonTouchingContact(b2World* world, b2Contact* contact, int setIndex)
{
	B2_ASSERT(0 <= setIndex && setIndex < b2Array(world->solverSetArray).count);
	b2SolverSet* set = world->solverSetArray + setIndex;
	b2Contact* newContact = b2AddContact(&world->blockAllocator, &set->contacts);
	memcpy(newContact, contact, sizeof(b2Contact));
	return newContact;
}

static void b2RemoveNonTouchingContact(b2World* world, int setIndex, int indexInSet)
{
	B2_ASSERT(0 <= setIndex && setIndex < b2Array(world->solverSetArray).count);
	b2SolverSet* set = world->solverSetArray + setIndex;
	int movedIndex = b2RemoveContact(&world->blockAllocator, &set->contacts, indexInSet);
	if (movedIndex != B2_NULL_INDEX)
	{
		b2Contact* movedContact = set->contacts.data + indexInSet;
		B2_ASSERT(movedContact->colorIndex == B2_NULL_INDEX);
		B2_ASSERT(movedContact->localIndex == movedIndex);
		movedContact->localIndex = indexInSet;

		B2_ASSERT(0 <= movedContact->contactId && movedContact->contactId < b2Array(world->contactLookupArray).count);
		b2ContactLookup* lookup = world->contactLookupArray + movedContact->contactId;
		B2_ASSERT(lookup->setIndex == setIndex);
		B2_ASSERT(lookup->contactIndex == movedIndex);
		B2_ASSERT(lookup->colorIndex == B2_NULL_INDEX);
		lookup->contactIndex = movedContact->localIndex;
	}
}

// Narrow-phase collision
static void b2Collide(b2StepContext* context)
{
	b2World* world = context->world;

	B2_ASSERT(world->workerCount > 0);

	b2TracyCZoneNC(collide, "Collide", b2_colorDarkOrchid, true);

	// Tasks that can be done in parallel with the narrow-phase
	// - rebuild the collision tree for dynamic and kinematic bodies to keep their query performance good
	world->userTreeTask = world->enqueueTaskFcn(&b2UpdateTreesTask, 1, 1, world, world->userTaskContext);
	world->taskCount += 1;
	world->activeTaskCount += world->userTreeTask == NULL ? 0 : 1;

	// gather contacts into a single array for easier parallel-for
	int contactCount = 0;
	b2GraphColor* colors = world->constraintGraph.colors;
	for (int i = 0; i < b2_graphColorCount; ++i)
	{
		contactCount += colors[i].contacts.count;
	}

	int nonTouchingCount = world->solverSetArray[b2_awakeSet].contacts.count;
	contactCount += nonTouchingCount;

	if (contactCount == 0)
	{
		b2TracyCZoneEnd(collide);
		return;
	}

	b2Contact** contacts = b2AllocateStackItem(&world->stackAllocator, contactCount * sizeof(b2Contact), "contacts");

	int contactIndex = 0;
	for (int i = 0; i < b2_graphColorCount; ++i)
	{
		b2GraphColor* color = colors + i;
		int count = color->contacts.count;
		b2Contact* base = color->contacts.data;
		for (int j = 0; j < count; ++j)
		{
			contacts[contactIndex] = base + j;
			contactIndex += 1;
		}
	}

	{
		b2Contact* base = world->solverSetArray[b2_awakeSet].contacts.data;
		for (int i = 0; i < nonTouchingCount; ++i)
		{
			contacts[contactIndex] = base + i;
			contactIndex += 1;
		}
	}

	B2_ASSERT(contactIndex == contactCount);

	context->contacts = contacts;

	// Contact bit set on ids because contact pointers are unstable as they move between touching and not touching.
	int contactIdCapacity = b2GetIdCapacity(&world->contactIdPool);
	for (uint32_t i = 0; i < world->workerCount; ++i)
	{
		b2SetBitCountAndClear(&world->taskContextArray[i].contactStateBitSet, contactIdCapacity);
	}

	// Task should take at least 40us on a 4GHz CPU (10K cycles)
	int32_t minRange = 64;
	void* userCollideTask = world->enqueueTaskFcn(&b2CollideTask, contactCount, minRange, context, world->userTaskContext);
	world->taskCount += 1;
	if (userCollideTask != NULL)
	{
		world->finishTaskFcn(userCollideTask, world->userTaskContext);
	}

	b2FreeStackItem(&world->stackAllocator, contacts);
	context->contacts = NULL;
	contacts = NULL;

	// Serially update contact state
	b2TracyCZoneNC(contact_state, "Contact State", b2_colorCoral, true);

	// Bitwise OR all contact bits
	b2BitSet* bitSet = &world->taskContextArray[0].contactStateBitSet;
	for (uint32_t i = 1; i < world->workerCount; ++i)
	{
		b2InPlaceUnion(bitSet, &world->taskContextArray[i].contactStateBitSet);
	}

	const b2Shape* shapes = world->shapes;
	int16_t worldId = world->worldId;

	// Process contact state changes. Iterate over set bits
	for (uint32_t k = 0; k < bitSet->blockCount; ++k)
	{
		uint64_t bits = bitSet->bits[k];
		while (bits != 0)
		{
			uint32_t ctz = b2CTZ64(bits);
			int contactId = (int)(64 * k + ctz);

			// todo optimize this access
			b2Contact* contact = b2GetContactFromRawId(world, contactId);

			const b2Shape* shapeA = shapes + contact->shapeIndexA;
			const b2Shape* shapeB = shapes + contact->shapeIndexB;
			b2ShapeId shapeIdA = {shapeA->object.index + 1, worldId, shapeA->object.revision};
			b2ShapeId shapeIdB = {shapeB->object.index + 1, worldId, shapeB->object.revision};
			uint32_t flags = contact->flags;

			if (flags & b2_contactDisjoint)
			{
				// Was touching?
				if ((flags & b2_contactTouchingFlag) != 0 && (flags & b2_contactEnableContactEvents) != 0)
				{
					b2ContactEndTouchEvent event = {shapeIdA, shapeIdB};
					b2Array_Push(world->contactEndArray, event);
				}

				// Bounding boxes no longer overlap
				b2DestroyContact(world, contact);
				contact = NULL;
			}
			else if (flags & b2_contactStartedTouching)
			{
				contact->flags &= ~b2_contactStartedTouching;

				B2_ASSERT(contact->islandId == B2_NULL_INDEX);
				if ((flags & b2_contactSensorFlag) != 0 && (flags & b2_contactEnableSensorEvents) != 0)
				{
					if (shapeA->isSensor)
					{
						b2SensorBeginTouchEvent event = {shapeIdA, shapeIdB};
						b2Array_Push(world->sensorBeginEventArray, event);
					}

					if (shapeB->isSensor)
					{
						b2SensorBeginTouchEvent event = {shapeIdB, shapeIdA};
						b2Array_Push(world->sensorBeginEventArray, event);
					}
				}
				else
				{
					if (flags & b2_contactEnableContactEvents)
					{
						b2ContactBeginTouchEvent event = {shapeIdA, shapeIdB};
						b2Array_Push(world->contactBeginArray, event);
					}

					B2_ASSERT(contact->colorIndex == B2_NULL_INDEX);
					int localIndex = contact->localIndex;
					B2_ASSERT(0 <= localIndex && localIndex < world->solverSetArray[b2_awakeSet].contacts.count);
					contact = b2AddContactToGraph(world, contact);
					b2RemoveNonTouchingContact(world, b2_awakeSet, localIndex);
					b2SyncContactLookup(world, contact, b2_awakeSet);
					b2LinkContact(world, contact);
					contact = NULL;
				}

			}
			else if (contact->flags & b2_contactStoppedTouching)
			{
				contact->flags &= ~b2_contactStoppedTouching;

				if ((flags & b2_contactSensorFlag) != 0 && (flags & b2_contactEnableSensorEvents) != 0)
				{
					if (shapeA->isSensor)
					{
						b2SensorEndTouchEvent event = {shapeIdA, shapeIdB};
						b2Array_Push(world->sensorEndEventArray, event);
					}

					if (shapeB->isSensor)
					{
						b2SensorEndTouchEvent event = {shapeIdB, shapeIdA};
						b2Array_Push(world->sensorEndEventArray, event);
					}
				}
				else
				{
					if (contact->flags & b2_contactEnableContactEvents)
					{
						b2ContactEndTouchEvent event = {shapeIdA, shapeIdB};
						b2Array_Push(world->contactEndArray, event);
					}

					b2UnlinkContact(world, contact);
					b2SolverSet* set = world->solverSetArray + b2_awakeSet;
					int storageIndex = set->contacts.count;
					b2Contact* newContact = b2AddNonTouchingContact(world, contact, b2_awakeSet);
					b2RemoveContactFromGraph(world, contact);
					newContact->colorIndex = B2_NULL_INDEX;
					newContact->localIndex = storageIndex;
					b2SyncContactLookup(world, newContact, b2_awakeSet);
					contact = NULL;
				}
			}
			
			{
				// #todo hit events
			}

			// Clear the smallest set bit
			bits = bits & (bits - 1);
		}
	}

	b2ValidateWorld(world);

	b2TracyCZoneEnd(contact_state);
	b2TracyCZoneEnd(collide);
}

void b2World_Step(b2WorldId worldId, float timeStep, int32_t subStepCount)
{
	b2World* world = b2GetWorldFromId(worldId);
	B2_ASSERT(world->locked == false);
	if (world->locked)
	{
		return;
	}

	// Prepare to capture events
	// Ensure user does not access stale data if there is an early return
	b2Array_Clear(world->bodyMoveEventArray);
	b2Array_Clear(world->sensorBeginEventArray);
	b2Array_Clear(world->sensorEndEventArray);
	b2Array_Clear(world->contactBeginArray);
	b2Array_Clear(world->contactEndArray);

	world->profile = (b2Profile){0};

	if (timeStep == 0.0f)
	{
		// todo would be useful to still process collision while paused
		return;
	}

	b2TracyCZoneNC(world_step, "Step", b2_colorChartreuse, true);

	world->locked = true;
	world->activeTaskCount = 0;
	world->taskCount = 0;

	b2Timer stepTimer = b2CreateTimer();

	// Update collision pairs and create contacts
	{
		b2Timer timer = b2CreateTimer();
		b2UpdateBroadPhasePairs(world);
		world->profile.pairs = b2GetMilliseconds(&timer);
	}

	_Alignas(64) b2StepContext context = {0};
	context.world = world;
	context.dt = timeStep;
	context.subStepCount = B2_MAX(1, subStepCount);

	if (timeStep > 0.0f)
	{
		context.inv_dt = 1.0f / timeStep;
		context.h = timeStep / context.subStepCount;
		context.inv_h = context.subStepCount * context.inv_dt;
	}
	else
	{
		context.inv_dt = 0.0f;
		context.h = 0.0f;
		context.inv_h = 0.0f;
	}

	world->inv_h = context.inv_h;

	// Hertz values get reduced for large time steps
	float contactHertz = B2_MIN(world->contactHertz, 0.25f * context.inv_h);
	float jointHertz = B2_MIN(world->jointHertz, 0.125f * context.inv_h);

	context.contactSoftness = b2MakeSoft(contactHertz, world->contactDampingRatio, context.h);
	context.staticSoftness = b2MakeSoft(2.0f * contactHertz, world->contactDampingRatio, context.h);
	context.jointSoftness = b2MakeSoft(jointHertz, world->jointDampingRatio, context.h);

	context.restitutionThreshold = world->restitutionThreshold;
	context.maxBiasVelocity = b2_maxTranslation * context.inv_dt;
	context.enableWarmStarting = world->enableWarmStarting;

	// Update contacts
	{
		b2Timer timer = b2CreateTimer();
		b2Collide(&context);
		world->profile.collide = b2GetMilliseconds(&timer);
	}

	// Integrate velocities, solve velocity constraints, and integrate positions.
	if (context.dt > 0.0f)
	{
		b2Timer timer = b2CreateTimer();
		b2Solve(world, &context);
		world->profile.solve = b2GetMilliseconds(&timer);
	}

	world->locked = false;

	world->profile.step = b2GetMilliseconds(&stepTimer);

	B2_ASSERT(b2GetStackAllocation(&world->stackAllocator) == 0);

	// Ensure stack is large enough
	b2GrowStack(&world->stackAllocator);

	// Make sure all tasks that were started were also finished
	B2_ASSERT(world->activeTaskCount == 0);

	b2TracyCZoneEnd(world_step);
}

static void b2DrawShape(b2DebugDraw* draw, b2Shape* shape, b2Transform xf, b2Color color)
{
	switch (shape->type)
	{
		case b2_capsuleShape:
		{
			b2Capsule* capsule = &shape->capsule;
			b2Vec2 p1 = b2TransformPoint(xf, capsule->point1);
			b2Vec2 p2 = b2TransformPoint(xf, capsule->point2);
			draw->DrawSolidCapsule(p1, p2, capsule->radius, color, draw->context);
		}
		break;

		case b2_circleShape:
		{
			b2Circle* circle = &shape->circle;
			b2Vec2 center = b2TransformPoint(xf, circle->point);
			b2Vec2 axis = b2RotateVector(xf.q, (b2Vec2){1.0f, 0.0f});
			draw->DrawSolidCircle(center, circle->radius, axis, color, draw->context);
		}
		break;

		case b2_polygonShape:
		{
			b2Color fillColor = {0.5f * color.r, 0.5f * color.g, 0.5f * color.b, 0.5f};

			b2Polygon* poly = &shape->polygon;
			int32_t count = poly->count;
			B2_ASSERT(count <= b2_maxPolygonVertices);
			b2Vec2 vertices[b2_maxPolygonVertices];

			for (int32_t i = 0; i < count; ++i)
			{
				vertices[i] = b2TransformPoint(xf, poly->vertices[i]);
			}

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

		case b2_segmentShape:
		{
			b2Segment* segment = &shape->segment;
			b2Vec2 p1 = b2TransformPoint(xf, segment->point1);
			b2Vec2 p2 = b2TransformPoint(xf, segment->point2);
			draw->DrawSegment(p1, p2, color, draw->context);
		}
		break;

		case b2_smoothSegmentShape:
		{
			b2Segment* segment = &shape->smoothSegment.segment;
			b2Vec2 p1 = b2TransformPoint(xf, segment->point1);
			b2Vec2 p2 = b2TransformPoint(xf, segment->point2);
			draw->DrawSegment(p1, p2, color, draw->context);
			draw->DrawPoint(p2, 4.0f, color, draw->context);
			draw->DrawSegment(p1, b2Lerp(p1, p2, 0.1f), b2MakeColor(b2_colorPaleGreen4), draw->context);
		}
		break;

		default:
			break;
	}
}

void b2World_Draw(b2WorldId worldId, b2DebugDraw* draw)
{
	b2World* world = b2GetWorldFromId(worldId);
	B2_ASSERT(world->locked == false);
	if (world->locked)
	{
		return;
	}

	if (draw->drawShapes)
	{
		int setCount = b2Array(world->solverSetArray).count;
		for (int setIndex = 0; setIndex < setCount; ++setIndex)
		{
			bool isAwake = (setIndex == b2_awakeSet);
			b2SolverSet* set = world->solverSetArray + setIndex;
			int bodyCount = set->bodies.count;
			for (int32_t bodyIndex = 0; bodyIndex < bodyCount; ++bodyIndex)
			{
				b2Body* body = set->bodies.data + bodyIndex;
				b2Transform xf = b2MakeTransform(body);
				int32_t shapeIndex = body->shapeList;
				while (shapeIndex != B2_NULL_INDEX)
				{
					b2Shape* shape = world->shapes + shapeIndex;
					b2Color color;

					if (body->type == b2_dynamicBody && body->mass == 0.0f)
					{
						// Bad body
						color = b2MakeColor(b2_colorRed);
					}
					else if (body->isEnabled == false)
					{
						color = b2MakeColor(b2_colorSlateGray2);
					}
					else if (shape->isSensor)
					{
						color = b2MakeColor(b2_colorWheat);
					}
					else if (body->isSpeedCapped)
					{
						color = b2MakeColor(b2_colorYellow);
					}
					else if (body->isFast)
					{
						color = b2MakeColor(b2_colorSalmon);
					}
					else if (body->type == b2_staticBody)
					{
						color = b2MakeColor(b2_colorPaleGreen);
					}
					else if (body->type == b2_kinematicBody)
					{
						color = (b2Color){0.5f, 0.5f, 0.9f, 1.0f};
					}
					else if (isAwake)
					{
						color = b2MakeColor(b2_colorPink3);
					}
					else
					{
						color = b2MakeColor(b2_colorGray);
					}

					b2DrawShape(draw, shape, xf, color);
					shapeIndex = shape->nextShapeIndex;
				}
			}
		}
	}

	#if 0
	// #todo
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
	#endif

	if (draw->drawAABBs)
	{
		b2Color color = {0.9f, 0.3f, 0.9f, 1.0f};

		int setCount = b2Array(world->solverSetArray).count;
		for (int setIndex = 0; setIndex < setCount; ++setIndex)
		{
			b2SolverSet* set = world->solverSetArray + setIndex;
			int bodyCount = set->bodies.count;
			for (int32_t bodyIndex = 0; bodyIndex < bodyCount; ++bodyIndex)
			{
				b2Body* body = set->bodies.data + bodyIndex;

				char buffer[32];
				snprintf(buffer, 32, "%d", body->bodyId);
				draw->DrawString(body->position, buffer, draw->context);

				int32_t shapeIndex = body->shapeList;
				while (shapeIndex != B2_NULL_INDEX)
				{
					b2Shape* shape = world->shapes + shapeIndex;
					b2AABB aabb = shape->fatAABB;

					b2Vec2 vs[4] = {{aabb.lowerBound.x, aabb.lowerBound.y},
									{aabb.upperBound.x, aabb.lowerBound.y},
									{aabb.upperBound.x, aabb.upperBound.y},
									{aabb.lowerBound.x, aabb.upperBound.y}};

					draw->DrawPolygon(vs, 4, color, draw->context);

					shapeIndex = shape->nextShapeIndex;
				}
			}
		}
	}

	if (draw->drawMass)
	{
		b2Vec2 offset = {0.1f, 0.1f};
		int setCount = b2Array(world->solverSetArray).count;
		for (int setIndex = 0; setIndex < setCount; ++setIndex)
		{
			b2SolverSet* set = world->solverSetArray + setIndex;
			int bodyCount = set->bodies.count;
			for (int32_t bodyIndex = 0; bodyIndex < bodyCount; ++bodyIndex)
			{
				b2Body* body = set->bodies.data + bodyIndex;

				b2Transform transform = {body->position, body->rotation};
				draw->DrawTransform(transform, draw->context);

				b2Vec2 p = b2TransformPoint(transform, offset);

				char buffer[32];
				snprintf(buffer, 32, "%.2f", body->mass);
				draw->DrawString(p, buffer, draw->context);
			}
		}
	}

	#if 0
	// #todo
	if (draw->drawContacts)
	{
		const float k_impulseScale = 1.0f;
		const float k_axisScale = 0.3f;
		b2Color speculativeColor = {0.3f, 0.3f, 0.3f, 1.0f};
		b2Color addColor = {0.3f, 0.95f, 0.3f, 1.0f};
		b2Color persistColor = {0.3f, 0.3f, 0.95f, 1.0f};
		b2Color normalColor = {0.9f, 0.9f, 0.9f, 1.0f};
		b2Color impulseColor = {0.9f, 0.9f, 0.3f, 1.0f};
		b2Color frictionColor = {0.9f, 0.9f, 0.3f, 1.0f};

		b2HexColor colors[b2_graphColorCount + 1] = {
			b2_colorRed,  b2_colorOrange,	 b2_colorYellow,	b2_colorGreen, b2_colorCyan, b2_colorBlue, b2_colorViolet,
			b2_colorPink, b2_colorChocolate, b2_colorGoldenrod, b2_colorCoral, b2_colorAqua, b2_colorBlack};

		int count = b2GetArrayCount(world->awakeContactArray);

		for (int32_t i = 0; i < count; ++i)
		{
			int index = world->awakeContactArray[i];
			if (index == B2_NULL_INDEX)
			{
				continue;
			}

			b2Contact* contact = world->contacts + index;
			int pointCount = contact->manifold.pointCount;
			int colorIndex = contact->colorIndex;
			b2Vec2 normal = contact->manifold.normal;
			char buffer[32];

			for (int j = 0; j < pointCount; ++j)
			{
				b2ManifoldPoint* point = contact->manifold.points + j;

				if (draw->drawGraphColors && 0 <= colorIndex && colorIndex <= b2_graphColorCount)
				{
					// graph color
					float pointSize = colorIndex == b2_graphColorCount ? 7.5f : 5.0f;
					draw->DrawPoint(point->point, pointSize, b2MakeColor(colors[colorIndex]), draw->context);
					// g_draw.DrawString(point->position, "%d", point->color);
				}
				else if (point->separation > b2_linearSlop)
				{
					// Speculative
					draw->DrawPoint(point->point, 5.0f, speculativeColor, draw->context);
				}
				else if (point->persisted == false)
				{
					// Add
					draw->DrawPoint(point->point, 10.0f, addColor, draw->context);
				}
				else if (point->persisted == true)
				{
					// Persist
					draw->DrawPoint(point->point, 5.0f, persistColor, draw->context);
				}

				if (draw->drawContactNormals)
				{
					b2Vec2 p1 = point->point;
					b2Vec2 p2 = b2MulAdd(p1, k_axisScale, normal);
					draw->DrawSegment(p1, p2, normalColor, draw->context);
				}
				else if (draw->drawContactImpulses)
				{
					b2Vec2 p1 = point->point;
					b2Vec2 p2 = b2MulAdd(p1, k_impulseScale * point->normalImpulse, normal);
					draw->DrawSegment(p1, p2, impulseColor, draw->context);
					snprintf(buffer, B2_ARRAY_COUNT(buffer), "%.2f", point->normalImpulse);
					draw->DrawString(p1, buffer, draw->context);
				}

				if (draw->drawFrictionImpulses)
				{
					b2Vec2 tangent = b2RightPerp(normal);
					b2Vec2 p1 = point->point;
					b2Vec2 p2 = b2MulAdd(p1, k_impulseScale * point->tangentImpulse, tangent);
					draw->DrawSegment(p1, p2, frictionColor, draw->context);
					snprintf(buffer, B2_ARRAY_COUNT(buffer), "%.2f", point->normalImpulse);
					draw->DrawString(p1, buffer, draw->context);
				}
			}
		}
	}
	#endif
}

b2BodyEvents b2World_GetBodyEvents(b2WorldId worldId)
{
	b2World* world = b2GetWorldFromId(worldId);
	B2_ASSERT(world->locked == false);
	if (world->locked)
	{
		return (b2BodyEvents){0};
	}

	int count = b2Array(world->bodyMoveEventArray).count;
	b2BodyEvents events = {world->bodyMoveEventArray, count};
	return events;
}

b2SensorEvents b2World_GetSensorEvents(b2WorldId worldId)
{
	b2World* world = b2GetWorldFromId(worldId);
	B2_ASSERT(world->locked == false);
	if (world->locked)
	{
		return (b2SensorEvents){0};
	}

	int beginCount = b2Array(world->sensorBeginEventArray).count;
	int endCount = b2Array(world->sensorEndEventArray).count;

	b2SensorEvents events = {world->sensorBeginEventArray, world->sensorEndEventArray, beginCount, endCount};
	return events;
}

b2ContactEvents b2World_GetContactEvents(b2WorldId worldId)
{
	b2World* world = b2GetWorldFromId(worldId);
	B2_ASSERT(world->locked == false);
	if (world->locked)
	{
		return (b2ContactEvents){0};
	}

	int beginCount = b2Array(world->contactBeginArray).count;
	int endCount = b2Array(world->contactEndArray).count;

	b2ContactEvents events = {world->contactBeginArray, world->contactEndArray, beginCount, endCount};
	return events;
}

bool b2World_IsValid(b2WorldId id)
{
	if (id.index1 < 1 || b2_maxWorlds < id.index1)
	{
		return false;
	}

	b2World* world = b2_worlds + (id.index1 - 1);

	if (world->worldId != id.index1 - 1)
	{
		// world is not allocated
		return false;
	}

	return id.revision == world->revision;
}

bool b2Body_IsValid(b2BodyId id)
{
	if (id.world0 < 0 || b2_maxWorlds <= id.world0)
	{
		// invalid world
		return false;
	}

	b2World* world = b2_worlds + id.world0;
	if (world->worldId != id.world0)
	{
		// world is free
		return false;
	}

	if (id.index1 < 1 || b2Array(world->bodyLookupArray).count < id.index1)
	{
		// invalid index
		return false;
	}

	b2BodyLookup lookup = world->bodyLookupArray[id.index1 - 1];
	if (lookup.setIndex == B2_NULL_INDEX)
	{
		// this was freed
		return false;
	}

	B2_ASSERT(lookup.bodyIndex != B2_NULL_INDEX);

	if (lookup.revision != id.revision)
	{
		// this id is orphaned
		return false;
	}

	return true;
}

bool b2Shape_IsValid(b2ShapeId id)
{
	if (id.world0 < 0 || b2_maxWorlds <= id.world0)
	{
		return false;
	}

	b2World* world = b2_worlds + id.world0;
	if (world->worldId != id.world0)
	{
		// world is free
		return false;
	}

	if (id.index1 < 1 || world->shapePool.capacity < id.index1)
	{
		return false;
	}

	b2Shape* shape = world->shapes + (id.index1 - 1);
	if (b2IsValidObject(&shape->object) == false)
	{
		return false;
	}

	return id.revision == shape->object.revision;
}

bool b2Chain_IsValid(b2ChainId id)
{
	if (id.world0 < 0 || b2_maxWorlds <= id.world0)
	{
		return false;
	}

	b2World* world = b2_worlds + id.world0;
	if (world->worldId != id.world0)
	{
		// world is free
		return false;
	}

	if (id.index1 < 1 || world->chainPool.capacity < id.index1)
	{
		return false;
	}

	b2ChainShape* chain = world->chains + (id.index1 - 1);
	if (b2IsValidObject(&chain->object) == false)
	{
		return false;
	}

	return id.revision == chain->object.revision;
}

bool b2Joint_IsValid(b2JointId id)
{
	if (id.world0 < 0 || b2_maxWorlds <= id.world0)
	{
		return false;
	}

	b2World* world = b2_worlds + id.world0;
	if (world->worldId != id.world0)
	{
		// world is free
		return false;
	}

	if (id.index1 < 1 || b2Array(world->jointLookupArray).count < id.index1)
	{
		return false;
	}

	b2JointLookup lookup = world->jointLookupArray[id.index1 - 1];
	if (lookup.setIndex == B2_NULL_INDEX)
	{
		// this was freed
		return false;
	}

	B2_ASSERT(lookup.jointIndex != B2_NULL_INDEX);

	if (lookup.revision != id.revision)
	{
		// this id is orphaned
		return false;
	}

	return true;
}

void b2World_EnableSleeping(b2WorldId worldId, bool flag)
{
	b2World* world = b2GetWorldFromId(worldId);
	B2_ASSERT(world->locked == false);
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
		int setCount = b2Array(world->solverSetArray).count;
		for (int i = b2_firstSleepingSet; i < setCount; ++i)
		{
			b2SolverSet* set = world->solverSetArray + i;
			if (set->bodies.count > 0)
			{
				b2WakeSolverSet(world, i);
			}
		}
	}
}

void b2World_EnableWarmStarting(b2WorldId worldId, bool flag)
{
	b2World* world = b2GetWorldFromId(worldId);
	B2_ASSERT(world->locked == false);
	if (world->locked)
	{
		return;
	}

	world->enableWarmStarting = flag;
}

void b2World_EnableContinuous(b2WorldId worldId, bool flag)
{
	b2World* world = b2GetWorldFromId(worldId);
	B2_ASSERT(world->locked == false);
	if (world->locked)
	{
		return;
	}

	world->enableContinuous = flag;
}

void b2World_SetRestitutionThreshold(b2WorldId worldId, float value)
{
	b2World* world = b2GetWorldFromId(worldId);
	B2_ASSERT(world->locked == false);
	if (world->locked)
	{
		return;
	}

	world->restitutionThreshold = B2_CLAMP(value, 0.0f, FLT_MAX);
}

void b2World_SetContactTuning(b2WorldId worldId, float hertz, float dampingRatio, float pushOut)
{
	b2World* world = b2GetWorldFromId(worldId);
	B2_ASSERT(world->locked == false);
	if (world->locked)
	{
		return;
	}

	world->contactHertz = B2_CLAMP(hertz, 0.0f, FLT_MAX);
	world->contactDampingRatio = B2_CLAMP(dampingRatio, 0.0f, FLT_MAX);
	world->contactPushoutVelocity = B2_CLAMP(pushOut, 0.0f, FLT_MAX);
}

b2Profile b2World_GetProfile(b2WorldId worldId)
{
	b2World* world = b2GetWorldFromId(worldId);
	return world->profile;
}

b2Counters b2World_GetCounters(b2WorldId worldId)
{
	b2World* world = b2GetWorldFromId(worldId);
	b2Counters s = {0};
	s.bodyCount = b2GetIdCount(&world->bodyIdPool);
	s.shapeCount = world->shapePool.count;
	s.contactCount = b2GetIdCount(&world->contactIdPool);
	s.jointCount = b2GetIdCount(&world->jointIdPool);
	s.islandCount = b2GetIdCount(&world->islandIdPool);

	b2DynamicTree* tree = world->broadPhase.trees + b2_dynamicBody;
	s.treeHeight = b2DynamicTree_GetHeight(tree);
	s.stackUsed = b2GetMaxStackAllocation(&world->stackAllocator);
	s.byteCount = b2GetByteCount();
	s.taskCount = world->taskCount;
	for (int32_t i = 0; i <= b2_graphColorCount; ++i)
	{
		s.colorCounts[i] = world->constraintGraph.colors[i].contacts.count + world->constraintGraph.colors[i].joints.count;
	}
	return s;
}

typedef struct WorldQueryContext
{
	b2World* world;
	b2OverlapResultFcn* fcn;
	b2QueryFilter filter;
	void* userContext;
} WorldQueryContext;

static bool TreeQueryCallback(int32_t proxyId, int32_t shapeIndex, void* context)
{
	B2_MAYBE_UNUSED(proxyId);

	WorldQueryContext* worldContext = context;
	b2World* world = worldContext->world;

	B2_ASSERT(0 <= shapeIndex && shapeIndex < world->shapePool.capacity);

	b2Shape* shape = world->shapes + shapeIndex;
	b2Filter shapeFilter = shape->filter;
	b2QueryFilter queryFilter = worldContext->filter;

	if ((shapeFilter.categoryBits & queryFilter.maskBits) == 0 || (shapeFilter.maskBits & queryFilter.categoryBits) == 0)
	{
		return true;
	}

	B2_ASSERT(shape->object.index == shape->object.next);

	b2ShapeId shapeId = {shape->object.index + 1, world->worldId, shape->object.revision};
	bool result = worldContext->fcn(shapeId, worldContext->userContext);
	return result;
}

void b2World_OverlapAABB(b2WorldId worldId, b2AABB aabb, b2QueryFilter filter, b2OverlapResultFcn* fcn, void* context)
{
	b2World* world = b2GetWorldFromId(worldId);
	B2_ASSERT(world->locked == false);
	if (world->locked)
	{
		return;
	}

	B2_ASSERT(b2AABB_IsValid(aabb));

	WorldQueryContext worldContext = {world, fcn, filter, context};

	for (int32_t i = 0; i < b2_bodyTypeCount; ++i)
	{
		b2DynamicTree_Query(world->broadPhase.trees + i, aabb, TreeQueryCallback, &worldContext);
	}
}

typedef struct WorldOverlapContext
{
	b2World* world;
	b2OverlapResultFcn* fcn;
	b2QueryFilter filter;
	b2DistanceProxy proxy;
	b2Transform transform;
	void* userContext;
} WorldOverlapContext;

static bool TreeOverlapCallback(int32_t proxyId, int32_t shapeIndex, void* context)
{
	B2_MAYBE_UNUSED(proxyId);

	WorldOverlapContext* worldContext = context;
	b2World* world = worldContext->world;

	B2_ASSERT(0 <= shapeIndex && shapeIndex < world->shapePool.capacity);

	b2Shape* shape = world->shapes + shapeIndex;
	b2Filter shapeFilter = shape->filter;
	b2QueryFilter queryFilter = worldContext->filter;

	if ((shapeFilter.categoryBits & queryFilter.maskBits) == 0 || (shapeFilter.maskBits & queryFilter.categoryBits) == 0)
	{
		return true;
	}

	B2_ASSERT(shape->object.index == shape->object.next);

	b2Body* body = b2GetBodyFromRawId(world, shape->bodyId);

	b2DistanceInput input;
	input.proxyA = worldContext->proxy;
	input.proxyB = b2MakeShapeDistanceProxy(shape);
	input.transformA = worldContext->transform;
	input.transformB = b2MakeTransform(body);
	input.useRadii = true;

	b2DistanceCache cache = {0};
	b2DistanceOutput output = b2ShapeDistance(&cache, &input);

	if (output.distance > 0.0f)
	{
		return true;
	}

	b2ShapeId shapeId = {shape->object.index + 1, world->worldId, shape->object.revision};
	bool result = worldContext->fcn(shapeId, worldContext->userContext);
	return result;
}

void b2World_OverlapCircle(b2WorldId worldId, const b2Circle* circle, b2Transform transform, b2QueryFilter filter,
						   b2OverlapResultFcn* fcn, void* context)
{
	b2World* world = b2GetWorldFromId(worldId);
	B2_ASSERT(world->locked == false);
	if (world->locked)
	{
		return;
	}

	B2_ASSERT(b2Vec2_IsValid(transform.p));
	B2_ASSERT(b2Rot_IsValid(transform.q));

	b2AABB aabb = b2ComputeCircleAABB(circle, transform);
	WorldOverlapContext worldContext = {
		world, fcn, filter, b2MakeProxy(&circle->point, 1, circle->radius), transform, context,
	};

	for (int32_t i = 0; i < b2_bodyTypeCount; ++i)
	{
		b2DynamicTree_Query(world->broadPhase.trees + i, aabb, TreeOverlapCallback, &worldContext);
	}
}

void b2World_OverlapCapsule(b2WorldId worldId, const b2Capsule* capsule, b2Transform transform, b2QueryFilter filter,
							b2OverlapResultFcn* fcn, void* context)
{
	b2World* world = b2GetWorldFromId(worldId);
	B2_ASSERT(world->locked == false);
	if (world->locked)
	{
		return;
	}

	B2_ASSERT(b2Vec2_IsValid(transform.p));
	B2_ASSERT(b2Rot_IsValid(transform.q));

	b2AABB aabb = b2ComputeCapsuleAABB(capsule, transform);
	WorldOverlapContext worldContext = {
		world, fcn, filter, b2MakeProxy(&capsule->point1, 2, capsule->radius), transform, context,
	};

	for (int32_t i = 0; i < b2_bodyTypeCount; ++i)
	{
		b2DynamicTree_Query(world->broadPhase.trees + i, aabb, TreeOverlapCallback, &worldContext);
	}
}

void b2World_OverlapPolygon(b2WorldId worldId, const b2Polygon* polygon, b2Transform transform, b2QueryFilter filter,
							b2OverlapResultFcn* fcn, void* context)
{
	b2World* world = b2GetWorldFromId(worldId);
	B2_ASSERT(world->locked == false);
	if (world->locked)
	{
		return;
	}

	B2_ASSERT(b2Vec2_IsValid(transform.p));
	B2_ASSERT(b2Rot_IsValid(transform.q));

	b2AABB aabb = b2ComputePolygonAABB(polygon, transform);
	WorldOverlapContext worldContext = {
		world, fcn, filter, b2MakeProxy(polygon->vertices, polygon->count, polygon->radius), transform, context,
	};

	for (int32_t i = 0; i < b2_bodyTypeCount; ++i)
	{
		b2DynamicTree_Query(world->broadPhase.trees + i, aabb, TreeOverlapCallback, &worldContext);
	}
}

typedef struct WorldRayCastContext
{
	b2World* world;
	b2CastResultFcn* fcn;
	b2QueryFilter filter;
	float fraction;
	void* userContext;
} WorldRayCastContext;

static float RayCastCallback(const b2RayCastInput* input, int32_t proxyId, int32_t shapeIndex, void* context)
{
	B2_MAYBE_UNUSED(proxyId);

	WorldRayCastContext* worldContext = context;
	b2World* world = worldContext->world;

	B2_ASSERT(0 <= shapeIndex && shapeIndex < world->shapePool.capacity);

	b2Shape* shape = world->shapes + shapeIndex;
	b2Filter shapeFilter = shape->filter;
	b2QueryFilter queryFilter = worldContext->filter;

	if ((shapeFilter.categoryBits & queryFilter.maskBits) == 0 || (shapeFilter.maskBits & queryFilter.categoryBits) == 0)
	{
		return input->maxFraction;
	}

	b2Body* body = b2GetBodyFromRawId(world, shape->bodyId);

	b2Transform transform = b2MakeTransform(body);
	b2CastOutput output = b2RayCastShape(input, shape, transform);

	if (output.hit)
	{
		b2ShapeId shapeId = {shapeIndex + 1, world->worldId, shape->object.revision};
		float fraction = worldContext->fcn(shapeId, output.point, output.normal, output.fraction, worldContext->userContext);
		worldContext->fraction = fraction;
		return fraction;
	}

	return input->maxFraction;
}

void b2World_RayCast(b2WorldId worldId, b2Vec2 origin, b2Vec2 translation, b2QueryFilter filter, b2CastResultFcn* fcn,
					 void* context)
{
	b2World* world = b2GetWorldFromId(worldId);
	B2_ASSERT(world->locked == false);
	if (world->locked)
	{
		return;
	}

	B2_ASSERT(b2Vec2_IsValid(origin));
	B2_ASSERT(b2Vec2_IsValid(translation));

	b2RayCastInput input = {origin, translation, 1.0f};

	WorldRayCastContext worldContext = {world, fcn, filter, 1.0f, context};

	for (int32_t i = 0; i < b2_bodyTypeCount; ++i)
	{
		b2DynamicTree_RayCast(world->broadPhase.trees + i, &input, filter.maskBits, RayCastCallback, &worldContext);

		if (worldContext.fraction == 0.0f)
		{
			return;
		}

		input.maxFraction = worldContext.fraction;
	}
}

// This callback finds the closest hit. This is the most common callback used in games.
static float b2RayCastClosestFcn(b2ShapeId shapeId, b2Vec2 point, b2Vec2 normal, float fraction, void* context)
{
	b2RayResult* rayResult = (b2RayResult*)context;
	rayResult->shapeId = shapeId;
	rayResult->point = point;
	rayResult->normal = normal;
	rayResult->fraction = fraction;
	rayResult->hit = true;
	return fraction;
}

b2RayResult b2World_RayCastClosest(b2WorldId worldId, b2Vec2 origin, b2Vec2 translation, b2QueryFilter filter)
{
	b2RayResult result = {0};

	b2World* world = b2GetWorldFromId(worldId);
	B2_ASSERT(world->locked == false);
	if (world->locked)
	{
		return result;
	}

	B2_ASSERT(b2Vec2_IsValid(origin));
	B2_ASSERT(b2Vec2_IsValid(translation));

	b2RayCastInput input = {origin, translation, 1.0f};
	WorldRayCastContext worldContext = {world, b2RayCastClosestFcn, filter, 1.0f, &result};

	for (int32_t i = 0; i < b2_bodyTypeCount; ++i)
	{
		b2DynamicTree_RayCast(world->broadPhase.trees + i, &input, filter.maskBits, RayCastCallback, &worldContext);

		if (worldContext.fraction == 0.0f)
		{
			return result;
		}

		input.maxFraction = worldContext.fraction;
	}

	return result;
}

static float ShapeCastCallback(const b2ShapeCastInput* input, int32_t proxyId, int32_t shapeIndex, void* context)
{
	B2_MAYBE_UNUSED(proxyId);

	WorldRayCastContext* worldContext = context;
	b2World* world = worldContext->world;

	B2_ASSERT(0 <= shapeIndex && shapeIndex < world->shapePool.capacity);

	b2Shape* shape = world->shapes + shapeIndex;
	b2Filter shapeFilter = shape->filter;
	b2QueryFilter queryFilter = worldContext->filter;

	if ((shapeFilter.categoryBits & queryFilter.maskBits) == 0 || (shapeFilter.maskBits & queryFilter.categoryBits) == 0)
	{
		return input->maxFraction;
	}

	b2Body* body = b2GetBodyFromRawId(world, shape->bodyId);

	b2Transform transform = b2MakeTransform(body);
	b2CastOutput output = b2ShapeCastShape(input, shape, transform);

	if (output.hit)
	{
		b2ShapeId shapeId = {shapeIndex + 1, world->worldId, shape->object.revision};
		float fraction = worldContext->fcn(shapeId, output.point, output.normal, output.fraction, worldContext->userContext);
		worldContext->fraction = fraction;
		return fraction;
	}

	return input->maxFraction;
}

void b2World_CircleCast(b2WorldId worldId, const b2Circle* circle, b2Transform originTransform, b2Vec2 translation,
						b2QueryFilter filter, b2CastResultFcn* fcn, void* context)
{
	b2World* world = b2GetWorldFromId(worldId);
	B2_ASSERT(world->locked == false);
	if (world->locked)
	{
		return;
	}

	B2_ASSERT(b2Vec2_IsValid(originTransform.p));
	B2_ASSERT(b2Rot_IsValid(originTransform.q));
	B2_ASSERT(b2Vec2_IsValid(translation));

	b2ShapeCastInput input;
	input.points[0] = b2TransformPoint(originTransform, circle->point);
	input.count = 1;
	input.radius = circle->radius;
	input.translation = translation;
	input.maxFraction = 1.0f;

	WorldRayCastContext worldContext = {world, fcn, filter, 1.0f, context};

	for (int32_t i = 0; i < b2_bodyTypeCount; ++i)
	{
		b2DynamicTree_ShapeCast(world->broadPhase.trees + i, &input, filter.maskBits, ShapeCastCallback, &worldContext);

		if (worldContext.fraction == 0.0f)
		{
			return;
		}

		input.maxFraction = worldContext.fraction;
	}
}

void b2World_CapsuleCast(b2WorldId worldId, const b2Capsule* capsule, b2Transform originTransform, b2Vec2 translation,
						 b2QueryFilter filter, b2CastResultFcn* fcn, void* context)
{
	b2World* world = b2GetWorldFromId(worldId);
	B2_ASSERT(world->locked == false);
	if (world->locked)
	{
		return;
	}

	B2_ASSERT(b2Vec2_IsValid(originTransform.p));
	B2_ASSERT(b2Rot_IsValid(originTransform.q));
	B2_ASSERT(b2Vec2_IsValid(translation));

	b2ShapeCastInput input;
	input.points[0] = b2TransformPoint(originTransform, capsule->point1);
	input.points[1] = b2TransformPoint(originTransform, capsule->point2);
	input.count = 2;
	input.radius = capsule->radius;
	input.translation = translation;
	input.maxFraction = 1.0f;

	WorldRayCastContext worldContext = {world, fcn, filter, 1.0f, context};

	for (int32_t i = 0; i < b2_bodyTypeCount; ++i)
	{
		b2DynamicTree_ShapeCast(world->broadPhase.trees + i, &input, filter.maskBits, ShapeCastCallback, &worldContext);

		if (worldContext.fraction == 0.0f)
		{
			return;
		}

		input.maxFraction = worldContext.fraction;
	}
}

void b2World_PolygonCast(b2WorldId worldId, const b2Polygon* polygon, b2Transform originTransform, b2Vec2 translation,
						 b2QueryFilter filter, b2CastResultFcn* fcn, void* context)
{
	b2World* world = b2GetWorldFromId(worldId);
	B2_ASSERT(world->locked == false);
	if (world->locked)
	{
		return;
	}

	B2_ASSERT(b2Vec2_IsValid(originTransform.p));
	B2_ASSERT(b2Rot_IsValid(originTransform.q));
	B2_ASSERT(b2Vec2_IsValid(translation));

	b2ShapeCastInput input;
	for (int i = 0; i < polygon->count; ++i)
	{
		input.points[i] = b2TransformPoint(originTransform, polygon->vertices[i]);
	}
	input.count = polygon->count;
	input.radius = polygon->radius;
	input.translation = translation;
	input.maxFraction = 1.0f;

	WorldRayCastContext worldContext = {world, fcn, filter, 1.0f, context};

	for (int32_t i = 0; i < b2_bodyTypeCount; ++i)
	{
		b2DynamicTree_ShapeCast(world->broadPhase.trees + i, &input, filter.maskBits, ShapeCastCallback, &worldContext);

		if (worldContext.fraction == 0.0f)
		{
			return;
		}

		input.maxFraction = worldContext.fraction;
	}
}

#if 0

void b2World_ShiftOrigin(b2WorldId worldId, b2Vec2 newOrigin)
{
	B2_ASSERT(m_locked == false);
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

void b2World_Dump()
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

void b2World_SetPreSolveCallback(b2WorldId worldId, b2PreSolveFcn* fcn, void* context)
{
	b2World* world = b2GetWorldFromId(worldId);
	world->preSolveFcn = fcn;
	world->preSolveContext = context;
}

void b2World_SetGravity(b2WorldId worldId, b2Vec2 gravity)
{
	b2World* world = b2GetWorldFromId(worldId);
	world->gravity = gravity;
}

b2Vec2 b2World_GetGravity(b2WorldId worldId)
{
	b2World* world = b2GetWorldFromId(worldId);
	return world->gravity;
}

#if B2_VALIDATE
void b2ValidateWorld(b2World* world)
{
	B2_ASSERT(b2GetIdCapacity(&world->bodyIdPool) == b2Array(world->bodyLookupArray).count);
	B2_ASSERT(b2GetIdCapacity(&world->contactIdPool) == b2Array(world->contactLookupArray).count);
	B2_ASSERT(b2GetIdCapacity(&world->jointIdPool) == b2Array(world->jointLookupArray).count);
	B2_ASSERT(b2GetIdCapacity(&world->islandIdPool) == b2Array(world->islandLookupArray).count);
	B2_ASSERT(b2GetIdCapacity(&world->solverSetIdPool) == b2Array(world->solverSetArray).count);

	int activeSetCount = 0;
	int totalBodyCount = 0;
	int totalJointCount = 0;
	int totalContactCount = 0;
	int totalIslandCount = 0;
	for (int setIndex = 0; setIndex < b2Array(world->solverSetArray).count; ++setIndex)
	{
		b2SolverSet* set = world->solverSetArray + setIndex;
		if (set->solverSetId != B2_NULL_INDEX)
		{
			activeSetCount += 1;

			if (setIndex == b2_staticSet)
			{
				B2_ASSERT(set->contacts.count == 0);
				B2_ASSERT(set->joints.count == 0);
				B2_ASSERT(set->islands.count == 0);
				B2_ASSERT(set->states.count == 0);
			}
			else if (setIndex == b2_awakeSet)
			{
				B2_ASSERT(set->bodies.count == set->states.count);
				B2_ASSERT(set->joints.count == 0);
			}
			else if (setIndex == b2_disabledSet)
			{
				B2_ASSERT(set->contacts.count == 0);
				B2_ASSERT(set->islands.count == 0);
				B2_ASSERT(set->states.count == 0);
			}
			else
			{
				B2_ASSERT(set->states.count == 0);
			}

			{
				b2BodyLookup* lookups = world->bodyLookupArray;
				int lookupCount = b2Array(lookups).count;
				B2_ASSERT(set->bodies.count >= 0);
				totalBodyCount += set->bodies.count;
				for (int i = 0; i < set->bodies.count; ++i)
				{
					b2Body* body = set->bodies.data + i;
					B2_ASSERT(0 <= body->bodyId && body->bodyId < lookupCount);
					b2BodyLookup lookup = lookups[body->bodyId];
					B2_ASSERT(lookup.setIndex == setIndex);
					B2_ASSERT(lookup.bodyIndex == i);
					B2_ASSERT(lookup.revision == body->revision);

					// todo: validate body - contact - joint graph
				}
			}

			{
				b2ContactLookup* lookups = world->contactLookupArray;
				int lookupCount = b2Array(lookups).count;
				B2_ASSERT(set->contacts.count >= 0);
				totalContactCount += set->contacts.count;
				for (int i = 0; i < set->contacts.count; ++i)
				{
					b2Contact* contact = set->contacts.data + i;
					B2_ASSERT(0 <= contact->contactId && contact->contactId < lookupCount);
					if (setIndex == b2_awakeSet)
					{
						// contact should be non-touching if awake
						// or it could be this contact hasn't been transferred yet
						B2_ASSERT(contact->manifold.pointCount == 0 || (contact->flags & b2_contactStartedTouching) != 0);
					}
					b2ContactLookup lookup = lookups[contact->contactId];
					B2_ASSERT(lookup.setIndex == setIndex);
					B2_ASSERT(lookup.colorIndex == B2_NULL_INDEX);
					B2_ASSERT(lookup.contactIndex == i);
					B2_ASSERT(contact->colorIndex == B2_NULL_INDEX);
					B2_ASSERT(contact->localIndex == i);

					// todo: validate body - contact - joint graph
				}
			}

			{
				b2JointLookup* lookups = world->jointLookupArray;
				int lookupCount = b2Array(lookups).count;
				B2_ASSERT(set->joints.count >= 0);
				totalJointCount += set->joints.count;
				for (int i = 0; i < set->joints.count; ++i)
				{
					b2Joint* joint = set->joints.data + i;
					B2_ASSERT(0 <= joint->jointId && joint->jointId < lookupCount);
					b2JointLookup lookup = lookups[joint->jointId];
					B2_ASSERT(lookup.setIndex == setIndex);
					B2_ASSERT(lookup.colorIndex == B2_NULL_INDEX);
					B2_ASSERT(lookup.jointIndex == i);
					B2_ASSERT(joint->colorIndex == B2_NULL_INDEX);
					B2_ASSERT(joint->localIndex == i);

					// todo: validate body - contact - joint graph
				}
			}

			{
				b2IslandLookup* lookups = world->islandLookupArray;
				int lookupCount = b2Array(lookups).count;
				B2_ASSERT(set->islands.count >= 0);
				totalIslandCount += set->islands.count;
				for (int i = 0; i < set->islands.count; ++i)
				{
					b2Island* island = set->islands.data + i;
					B2_ASSERT(0 <= island->islandId && island->islandId < lookupCount);
					b2IslandLookup lookup = lookups[island->islandId];
					B2_ASSERT(lookup.setIndex == setIndex);
					B2_ASSERT(lookup.islandIndex == i);

					// todo: validate body - contact - joint graph
				}
			}
		}
		else
		{
			B2_ASSERT(set->bodies.count == 0);
			B2_ASSERT(set->contacts.count == 0);
			B2_ASSERT(set->joints.count == 0);
			B2_ASSERT(set->islands.count == 0);
			B2_ASSERT(set->states.count == 0);
		}
	}

	int setIdCount = b2GetIdCount(&world->solverSetIdPool);
	B2_ASSERT(activeSetCount == setIdCount);

	int bodyIdCount = b2GetIdCount(&world->bodyIdPool);
	B2_ASSERT(totalBodyCount == bodyIdCount);

	int islandIdCount = b2GetIdCount(&world->islandIdPool);
	B2_ASSERT(totalIslandCount == islandIdCount);

	for (int colorIndex = 0; colorIndex < b2_graphColorCount; ++colorIndex)
	{
		b2GraphColor* color = world->constraintGraph.colors + colorIndex;
		{
			b2ContactLookup* lookups = world->contactLookupArray;
			int lookupCount = b2Array(lookups).count;
			B2_ASSERT(color->contacts.count >= 0);
			totalContactCount += color->contacts.count;
			for (int i = 0; i < color->contacts.count; ++i)
			{
				b2Contact* contact = color->contacts.data + i;
				B2_ASSERT(0 <= contact->contactId && contact->contactId < lookupCount);
				// contact should be touching in the constraint graph or awaiting transfer to non-touching
				B2_ASSERT(contact->manifold.pointCount > 0 ||
					(contact->flags & (b2_contactStoppedTouching | b2_contactDisjoint)) != 0);
				b2ContactLookup lookup = lookups[contact->contactId];
				B2_ASSERT(lookup.setIndex == b2_awakeSet);
				B2_ASSERT(lookup.colorIndex == colorIndex);
				B2_ASSERT(lookup.contactIndex == i);
				B2_ASSERT(contact->colorIndex == colorIndex);
				B2_ASSERT(contact->localIndex == i);
			}
		}

		{
			b2JointLookup* lookups = world->jointLookupArray;
			int lookupCount = b2Array(lookups).count;
			B2_ASSERT(color->joints.count >= 0);
			totalJointCount += color->joints.count;
			for (int i = 0; i < color->joints.count; ++i)
			{
				b2Joint* joint = color->joints.data + i;
				B2_ASSERT(0 <= joint->jointId && joint->jointId < lookupCount);
				b2JointLookup lookup = lookups[joint->jointId];
				B2_ASSERT(lookup.setIndex == b2_awakeSet);
				B2_ASSERT(lookup.colorIndex == colorIndex);
				B2_ASSERT(lookup.jointIndex == i);
				B2_ASSERT(joint->colorIndex == colorIndex);
				B2_ASSERT(joint->localIndex == i);
			}
		}
	}

	int contactIdCount = b2GetIdCount(&world->contactIdPool);
	B2_ASSERT(totalContactCount == contactIdCount);

	int jointIdCount = b2GetIdCount(&world->jointIdPool);
	B2_ASSERT(totalJointCount == jointIdCount);
}
#else
void b2ValidateWorld(b2World* world)
{
	B2_MAYBE_UNUSED(world);
}
#endif
