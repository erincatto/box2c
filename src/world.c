// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#define _CRT_SECURE_NO_WARNINGS

#include "world.h"

#include "allocate.h"
#include "array.h"
#include "bitset.h"
#include "block_allocator.h"
#include "body.h"
#include "broad_phase.h"
#include "contact.h"
#include "core.h"
#include "graph.h"
#include "island.h"
#include "joint.h"
#include "pool.h"
#include "shape.h"
#include "solver_data.h"
#include "stack_allocator.h"

#include "box2d/aabb.h"
#include "box2d/box2d.h"
#include "box2d/constants.h"
#include "box2d/debug_draw.h"
#include "box2d/distance.h"
#include "box2d/timer.h"

#include <stdio.h>
#include <string.h>

b2World b2_worlds[b2_maxWorlds];
bool b2_parallel = true;
int b2_collideMinRange = 64;
int b2_islandMinRange = 1;

b2World* b2GetWorldFromId(b2WorldId id)
{
	B2_ASSERT(0 <= id.index && id.index < b2_maxWorlds);
	b2World* world = b2_worlds + id.index;
	B2_ASSERT(id.revision == world->revision);
	return world;
}

b2World* b2GetWorldFromIndex(int16_t index)
{
	B2_ASSERT(0 <= index && index < b2_maxWorlds);
	b2World* world = b2_worlds + index;
	B2_ASSERT(world->blockAllocator != NULL);
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

static void b2DefaultFinishAllTasksFcn(void* userContext)
{
	B2_MAYBE_UNUSED(userContext);
}

b2WorldId b2CreateWorld(const b2WorldDef* def)
{
	b2WorldId id = b2_nullWorldId;
	for (int16_t i = 0; i < b2_maxWorlds; ++i)
	{
		if (b2_worlds[i].blockAllocator == NULL)
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
	b2World* world = b2_worlds + id.index;
	*world = empty;

	world->index = id.index;

	world->blockAllocator = b2CreateBlockAllocator();
	world->stackAllocator = b2CreateStackAllocator(def->stackAllocatorCapacity);

	b2CreateBroadPhase(&world->broadPhase);
	b2CreateGraph(&world->graph, def->bodyCapacity, def->contactCapacity, def->jointCapacity);

	// pools
	world->bodyPool = b2CreatePool(sizeof(b2Body), B2_MAX(def->bodyCapacity, 1));
	world->bodies = (b2Body*)world->bodyPool.memory;

	world->shapePool = b2CreatePool(sizeof(b2Shape), B2_MAX(def->shapeCapacity, 1));
	world->shapes = (b2Shape*)world->shapePool.memory;

	world->contactPool = b2CreatePool(sizeof(b2Contact), B2_MAX(def->contactCapacity, 1));
	world->contacts = (b2Contact*)world->contactPool.memory;

	world->jointPool = b2CreatePool(sizeof(b2Joint), B2_MAX(def->jointCapacity, 1));
	world->joints = (b2Joint*)world->jointPool.memory;

	world->islandPool = b2CreatePool(sizeof(b2Island), B2_MAX(def->bodyCapacity, 1));
	world->islands = (b2Island*)world->islandPool.memory;

	world->awakeIslandArray = b2CreateArray(sizeof(int32_t), B2_MAX(def->bodyCapacity, 1));

	world->awakeContactArray = b2CreateArray(sizeof(int32_t), B2_MAX(def->contactCapacity, 1));
	world->contactAwakeIndexArray = b2CreateArray(sizeof(int32_t), world->contactPool.capacity);

	world->stepId = 0;

	// Globals start at 0. It should be fine for this to roll over.
	world->revision += 1;

	world->gravity = def->gravity;
	world->restitutionThreshold = def->restitutionThreshold;
	world->inv_dt0 = 0.0f;
	world->enableSleep = true;
	world->locked = false;
	world->enableWarmStarting = true;
	world->enableContinuous = true;
	world->profile = b2_emptyProfile;
	world->userTreeTask = NULL;
	world->splitIslandIndex = B2_NULL_INDEX;

	id.revision = world->revision;

	if (def->workerCount > 0 && def->enqueueTask != NULL && def->finishTask != NULL && def->finishAllTasks != NULL)
	{
		world->workerCount = B2_MIN(def->workerCount, b2_maxWorkers);
		world->enqueueTaskFcn = def->enqueueTask;
		world->finishTaskFcn = def->finishTask;
		world->finishAllTasksFcn = def->finishAllTasks;
		world->userTaskContext = def->userTaskContext;
	}
	else
	{
		world->workerCount = 1;
		world->enqueueTaskFcn = b2DefaultAddTaskFcn;
		world->finishTaskFcn = b2DefaultFinishTaskFcn;
		world->finishAllTasksFcn = b2DefaultFinishAllTasksFcn;
		world->userTaskContext = NULL;
	}

	world->taskContextArray = b2CreateArray(sizeof(b2TaskContext), world->workerCount);
	for (uint32_t i = 0; i < world->workerCount; ++i)
	{
		world->taskContextArray[i].contactStateBitSet = b2CreateBitSet(def->contactCapacity);
		world->taskContextArray[i].awakeContactBitSet = b2CreateBitSet(def->contactCapacity);
		world->taskContextArray[i].shapeBitSet = b2CreateBitSet(def->shapeCapacity);
		world->taskContextArray[i].awakeIslandBitSet = b2CreateBitSet(256);
	}

	return id;
}

void b2DestroyWorld(b2WorldId id)
{
	b2World* world = b2GetWorldFromId(id);

	for (uint32_t i = 0; i < world->workerCount; ++i)
	{
		b2DestroyBitSet(&world->taskContextArray[i].contactStateBitSet);
		b2DestroyBitSet(&world->taskContextArray[i].awakeContactBitSet);
		b2DestroyBitSet(&world->taskContextArray[i].shapeBitSet);
		b2DestroyBitSet(&world->taskContextArray[i].awakeIslandBitSet);
	}

	b2DestroyArray(world->taskContextArray, sizeof(b2TaskContext));

	b2DestroyArray(world->awakeContactArray, sizeof(int32_t));

	b2DestroyArray(world->awakeIslandArray, sizeof(int32_t));
	b2DestroyArray(world->contactAwakeIndexArray, sizeof(int32_t));

	b2DestroyPool(&world->islandPool);
	b2DestroyPool(&world->jointPool);
	b2DestroyPool(&world->contactPool);
	b2DestroyPool(&world->shapePool);
	b2DestroyPool(&world->bodyPool);

	b2DestroyGraph(&world->graph);
	b2DestroyBroadPhase(&world->broadPhase);

	b2DestroyBlockAllocator(world->blockAllocator);
	b2DestroyStackAllocator(world->stackAllocator);

	memset(world, 0, sizeof(b2World));
}

static void b2CollideTask(int32_t startIndex, int32_t endIndex, uint32_t threadIndex, void* context)
{
	b2TracyCZoneNC(collide_task, "Collide Task", b2_colorDodgerBlue1, true);

	b2World* world = context;
	B2_ASSERT(threadIndex < world->workerCount);
	b2TaskContext* taskContext = world->taskContextArray + threadIndex;
	b2Shape* shapes = world->shapes;
	b2Body* bodies = world->bodies;
	b2Contact* contacts = world->contacts;
	int32_t awakeCount = b2Array(world->awakeContactArray).count;
	int32_t* awakeContactArray = world->awakeContactArray;
	int32_t* contactAwakeIndexArray = world->contactAwakeIndexArray;

	B2_MAYBE_UNUSED(awakeCount);
	B2_ASSERT(startIndex < endIndex);
	B2_ASSERT(endIndex <= awakeCount);

	for (int32_t awakeIndex = startIndex; awakeIndex < endIndex; ++awakeIndex)
	{
		int32_t contactIndex = awakeContactArray[awakeIndex];
		if (contactIndex == B2_NULL_INDEX)
		{
			// Contact was destroyed
			continue;
		}

		B2_ASSERT(0 <= contactIndex && contactIndex < world->contactPool.capacity);
		b2Contact* contact = contacts + contactIndex;

		B2_ASSERT(contactAwakeIndexArray[contactIndex] == awakeIndex);
		B2_ASSERT(contact->object.index == contactIndex && contact->object.index == contact->object.next);

		// Reset contact awake index. Contacts must be added to the awake contact array
		// each time step in the island solver.
		contactAwakeIndexArray[contactIndex] = B2_NULL_INDEX;

		b2Shape* shapeA = shapes + contact->shapeIndexA;
		b2Shape* shapeB = shapes + contact->shapeIndexB;

		// Do proxies still overlap?
		bool overlap = b2AABB_Overlaps(shapeA->fatAABB, shapeB->fatAABB);
		if (overlap == false)
		{
			contact->flags |= b2_contactDisjoint;
			b2SetBit(&taskContext->contactStateBitSet, awakeIndex);
		}
		else
		{
			bool wasTouching = (contact->flags & b2_contactTouchingFlag);
			B2_ASSERT(wasTouching || contact->islandIndex == B2_NULL_INDEX);

			// Update contact respecting shape/body order (A,B)
			b2Body* bodyA = bodies + shapeA->bodyIndex;
			b2Body* bodyB = bodies + shapeB->bodyIndex;
			b2UpdateContact(world, contact, shapeA, bodyA, shapeB, bodyB);

			bool touching = (contact->flags & b2_contactTouchingFlag) != 0;

			// State changes that affect island connectivity
			if (touching == true && wasTouching == false)
			{
				contact->flags |= b2_contactStartedTouching;
				b2SetBit(&taskContext->contactStateBitSet, awakeIndex);
			}
			else if (touching == false && wasTouching == true)
			{
				contact->flags |= b2_contactStoppedTouching;
				b2SetBit(&taskContext->contactStateBitSet, awakeIndex);
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

// Narrow-phase collision
static void b2Collide(b2World* world)
{
	B2_ASSERT(world->workerCount > 0);

	b2TracyCZoneNC(collide, "Collide", b2_colorDarkOrchid, true);

	// Tasks that can be done in parallel with the narrow-phase 
	// - rebuild the collision tree for dynamic and kinematic bodies to keep their query performance good
	if (b2_parallel)
	{
		world->userTreeTask = world->enqueueTaskFcn(&b2UpdateTreesTask, 1, 1, world, world->userTaskContext);
	}
	else
	{
		b2UpdateTreesTask(0, 1, 0, world);
		world->userTreeTask = NULL;
	}

	int32_t awakeContactCount = b2Array(world->awakeContactArray).count;

	if (awakeContactCount == 0)
	{
		b2TracyCZoneEnd(collide);
		return;
	}

	for (uint32_t i = 0; i < world->workerCount; ++i)
	{
		b2SetBitCountAndClear(&world->taskContextArray[i].contactStateBitSet, awakeContactCount);
	}

	if (b2_parallel)
	{
		// Task should take at least 40us on a 4GHz CPU (10K cycles)
		int32_t minRange = b2_collideMinRange;
		void* userCollideTask = world->enqueueTaskFcn(&b2CollideTask, awakeContactCount, minRange, world, world->userTaskContext);
		world->finishTaskFcn(userCollideTask, world->userTaskContext);
	}
	else
	{
		b2CollideTask(0, awakeContactCount, 0, world);
	}

	// Serially update contact state
	b2TracyCZoneNC(contact_state, "Contact State", b2_colorCoral, true);

	// Bitwise OR all contact bits
	b2BitSet* bitSet = &world->taskContextArray[0].contactStateBitSet;
	for (uint32_t i = 1; i < world->workerCount; ++i)
	{
		b2InPlaceUnion(bitSet, &world->taskContextArray[i].contactStateBitSet);
	}

	// Process contact state changes. Iterate over set bits
	uint64_t word;
	for (uint32_t k = 0; k < bitSet->wordCount; ++k)
	{
		word = bitSet->bits[k];
		while (word != 0)
		{
			uint32_t ctz = b2CTZ(word);
			uint32_t awakeIndex = 64 * k + ctz;
			B2_ASSERT(awakeIndex < (uint32_t)awakeContactCount);

			int32_t contactIndex = world->awakeContactArray[awakeIndex];
			B2_ASSERT(contactIndex != B2_NULL_INDEX);

			b2Contact* contact = world->contacts + contactIndex;

			if (contact->flags & b2_contactDisjoint)
			{
				// Bounding boxes no longer overlap
				b2DestroyContact(world, contact);
			}
			else if (contact->flags & b2_contactStartedTouching)
			{
				B2_ASSERT(contact->islandIndex == B2_NULL_INDEX);
				b2LinkContact(world, contact);
				b2AddContactToGraph(world, contact);
				contact->flags &= ~b2_contactStartedTouching;
			}
			else
			{
				B2_ASSERT(contact->flags & b2_contactStoppedTouching);
				if (contact->colorIndex == B2_NULL_INDEX)
				{
					contact->colorIndex = B2_NULL_INDEX;
				}

				b2UnlinkContact(world, contact);
				b2RemoveContactFromGraph(world, contact);
				contact->flags &= ~b2_contactStoppedTouching;
			}

			// Clear the smallest set bit
			word = word & (word - 1);
		}
	}

	b2TracyCZoneEnd(contact_state);

	b2TracyCZoneEnd(collide);
}

struct b2ContinuousContext
{
	b2World* world;
	b2Body* fastBody;
	b2Shape* fastShape;
	b2Sweep sweep;
	float fraction;
};

static bool b2ContinuousQueryCallback(int32_t proxyId, int32_t shapeIndex, void* context)
{
	B2_MAYBE_UNUSED(proxyId);

	struct b2ContinuousContext* continuousContext = context;
	b2Shape* fastShape = continuousContext->fastShape;

	// Skip same shape
	if (shapeIndex == fastShape->object.index)
	{
		return true;
	}

	b2World* world = continuousContext->world;
	B2_ASSERT(0 <= shapeIndex && shapeIndex < world->shapePool.capacity);
	b2Shape* shape = world->shapes + shapeIndex;
	B2_ASSERT(shape->object.index == shape->object.next);

	// Skip same body
	if (shape->bodyIndex == fastShape->bodyIndex)
	{
		return true;
	}

	// Skip filtered shapes
	bool canCollide = b2ShouldShapesCollide(fastShape->filter, shape->filter);
	if (canCollide == false)
	{
		return true;
	}

	B2_ASSERT(0 <= shape->bodyIndex && shape->bodyIndex < world->bodyPool.capacity);
	b2Body* body = world->bodies + shape->bodyIndex;
	B2_ASSERT(body->type == b2_staticBody);

	// Skip filtered bodies
	canCollide = b2ShouldBodiesCollide(world, continuousContext->fastBody, body);
	if (canCollide == false)
	{
		return true;
	}

	b2TOIInput input;
	input.proxyA = b2Shape_MakeDistanceProxy(shape);
	input.proxyB = b2Shape_MakeDistanceProxy(fastShape);
	input.sweepA = b2MakeSweep(body);
	input.sweepB = continuousContext->sweep;
	input.tMax = continuousContext->fraction;

	b2TOIOutput output = b2TimeOfImpact(&input);
	if (0.0f < output.t && output.t < continuousContext->fraction)
	{
		continuousContext->fraction = output.t;
	}

	return true;
}

// Continuous collision of dynamic versus static
static void b2SolveContinuous(b2World* world, int32_t bodyIndex)
{
	b2Body* fastBody = world->bodies + bodyIndex;
	B2_ASSERT(b2ObjectValid(&fastBody->object));
	B2_ASSERT(fastBody->type == b2_dynamicBody && fastBody->isFast);

	b2Shape* shapes = world->shapes;

	b2Sweep sweep = b2MakeSweep(fastBody);

	b2Transform xf1;
	xf1.q = b2MakeRot(sweep.a1);
	xf1.p = b2Sub(sweep.c1, b2RotateVector(xf1.q, sweep.localCenter));

	b2Transform xf2;
	xf2.q = b2MakeRot(sweep.a2);
	xf2.p = b2Sub(sweep.c2, b2RotateVector(xf2.q, sweep.localCenter));

	b2DynamicTree* staticTree = world->broadPhase.trees + b2_staticBody;

	struct b2ContinuousContext context;
	context.world = world;
	context.sweep = sweep;
	context.fastBody = fastBody;
	context.fraction = 1.0f;

	int32_t shapeIndex = fastBody->shapeList;
	while (shapeIndex != B2_NULL_INDEX)
	{
		b2Shape* fastShape = shapes + shapeIndex;
		B2_ASSERT(fastShape->isFast == true);

		// Clear flag (keep set on body)
		fastShape->isFast = false;

		context.fastShape = fastShape;

		b2AABB box1 = fastShape->aabb;
		b2AABB box2 = b2Shape_ComputeAABB(fastShape, xf2);
		b2AABB box = b2AABB_Union(box1, box2);

		// Store this for later
		fastShape->aabb = box2;

		b2DynamicTree_Query(staticTree, box, b2ContinuousQueryCallback, &context);

		shapeIndex = fastShape->nextShapeIndex;
	}

	if (context.fraction < 1.0f)
	{
		// Handle time of impact event

		b2Vec2 c = b2Lerp(sweep.c1, sweep.c2, context.fraction);
		float a = sweep.a1 + context.fraction * (sweep.a2 - sweep.a1);

		// Advance body
		fastBody->angle0 = a;
		fastBody->angle = a;
		fastBody->position0 = c;
		fastBody->position = c;

		b2Transform xf;
		xf.q = b2MakeRot(a);
		xf.p = b2Sub(c, b2RotateVector(fastBody->transform.q, sweep.localCenter));
		fastBody->transform = xf;

		// Prepare AABBs for broad-phase
		shapeIndex = fastBody->shapeList;
		while (shapeIndex != B2_NULL_INDEX)
		{
			b2Shape* shape = shapes + shapeIndex;

			// Must recompute aabb at the interpolated transform
			shape->aabb = b2Shape_ComputeAABB(shape, xf);

			if (b2AABB_Contains(shape->fatAABB, shape->aabb) == false)
			{
				shape->fatAABB = b2AABB_Extend(shape->aabb);
				shape->enlargedAABB = true;
				fastBody->enlargeAABB = true;
			}

			shapeIndex = shape->nextShapeIndex;
		}
	}
	else
	{
		// No time of impact event

		// Advance body
		fastBody->angle0 = fastBody->angle;
		fastBody->position0 = fastBody->position;

		// Prepare AABBs for broad-phase
		shapeIndex = fastBody->shapeList;
		while (shapeIndex != B2_NULL_INDEX)
		{
			b2Shape* shape = shapes + shapeIndex;

			// shape->aabb is still valid

			if (b2AABB_Contains(shape->fatAABB, shape->aabb) == false)
			{
				shape->fatAABB = b2AABB_Extend(shape->aabb);
				shape->enlargedAABB = true;
				fastBody->enlargeAABB = true;
			}

			shapeIndex = shape->nextShapeIndex;
		}
	}
}

static void b2ContinuousParallelForTask(int32_t startIndex, int32_t endIndex, uint32_t threadIndex, void* taskContext)
{
	B2_MAYBE_UNUSED(threadIndex);

	b2TracyCZoneNC(continuous_task, "Continuous Task", b2_colorAqua, true);

	b2World* world = taskContext;

	B2_ASSERT(startIndex <= endIndex);
	B2_ASSERT(startIndex <= world->bodyPool.capacity);
	B2_ASSERT(endIndex <= world->bodyPool.capacity);

	for (int32_t i = startIndex; i < endIndex; ++i)
	{
		int32_t index = world->fastBodies[i];
		b2SolveContinuous(world, index);
	}

	b2TracyCZoneEnd(continuous_task);
}

// Solve with graph coloring
static void b2Solve(b2World* world, b2StepContext* context)
{
	b2TracyCZoneNC(solve, "Solve", b2_colorMistyRose, true);

	b2Timer timer = b2CreateTimer();

	world->stepId += 1;

	b2MergeAwakeIslands(world);

	world->profile.buildIslands = b2GetMillisecondsAndReset(&timer);

	b2TracyCZoneNC(graph_solver, "Graph", b2_colorSeaGreen, true);

	// Solve constraints using graph coloring
	b2SolveGraph(world, context);

	b2ValidateNoEnlarged(&world->broadPhase);

	b2TracyCZoneEnd(graph_solver);

	world->profile.solveIslands = b2GetMillisecondsAndReset(&timer);

	b2TracyCZoneNC(awake_islands, "Awake Islands", b2_colorGainsboro, true);

	// TODO_ERIN this code is related to body finalization b2SolveGraph. Reorganize?

	// Prepare awake contact bit set so that putting islands to sleep can clear bits
	// for the associated contacts.
	b2BitSet* awakeContactBitSet = &world->taskContextArray[0].awakeContactBitSet;
	for (uint32_t i = 1; i < world->workerCount; ++i)
	{
		b2InPlaceUnion(awakeContactBitSet, &world->taskContextArray[i].awakeContactBitSet);
	}

	{
		b2BitSet* bitSet = &world->taskContextArray[0].awakeIslandBitSet;
		for (uint32_t i = 1; i < world->workerCount; ++i)
		{
			b2InPlaceUnion(bitSet, &world->taskContextArray[i].awakeIslandBitSet);
		}

		b2Body* bodies = world->bodies;
		b2Contact* contacts = world->contacts;
		b2Joint* joints = world->joints;

		int32_t count = b2Array(world->awakeIslandArray).count;
		for (int32_t i = 0; i < count; ++i)
		{
			int32_t islandIndex = world->awakeIslandArray[i];
			if (b2GetBit(bitSet, islandIndex) == true)
			{
				continue;
			}

			// Put island to sleep
			b2Island* island = world->islands + islandIndex;
			island->awakeIndex = B2_NULL_INDEX;

			// Put contacts to sleep. Remember only touching contacts are in the island.
			// So a body may have more contacts than those in the island.
			// This is expensive on the main thread, but this only happens when an island goes
			// to sleep.
			int32_t bodyIndex = island->headBody;
			while (bodyIndex != B2_NULL_INDEX)
			{
				b2Body* body = bodies + bodyIndex;
				int32_t contactKey = body->contactList;
				while (contactKey != B2_NULL_INDEX)
				{
					int32_t contactIndex = contactKey >> 1;
					int32_t edgeIndex = contactKey & 1;
					b2Contact* contact = contacts + contactIndex;

					// IMPORTANT: clear awake contact bit
					b2ClearBit(awakeContactBitSet, contactIndex);

					contactKey = contact->edges[edgeIndex].nextKey;
				}

				bodyIndex = body->islandNext;
			}

			// Remove edges from graph
			int32_t contactIndex = island->headContact;
			while (contactIndex != B2_NULL_INDEX)
			{
				b2Contact* contact = contacts + contactIndex;
				b2RemoveContactFromGraph(world, contact);

				contactIndex = contact->islandNext;
			}

			int32_t jointIndex = island->headJoint;
			while (jointIndex != B2_NULL_INDEX)
			{
				b2Joint* joint = joints + jointIndex;
				// TODO_JOINT_GRAPH
				// b2RemoveJointFromGraph(world, joint);
				jointIndex = joint->islandNext;
			}
		}

		// Clear awake island array
		b2Array_Clear(world->awakeIslandArray);

		// Use bitSet to build awake island array. No need to add edges.
		uint64_t word;
		uint32_t wordCount = bitSet->wordCount;
		uint64_t* bits = bitSet->bits;
		int32_t awakeIndex = 0;
		for (uint32_t k = 0; k < wordCount; ++k)
		{
			word = bits[k];
			while (word != 0)
			{
				uint32_t ctz = b2CTZ(word);
				uint32_t islandIndex = 64 * k + ctz;

				b2Array_Push(world->awakeIslandArray, islandIndex);

				// Reference index. This tells the island and bodies they are awake.
				world->islands[islandIndex].awakeIndex = awakeIndex;
				awakeIndex += 1;

				// Clear the smallest set bit
				word = word & (word - 1);
			}
		}
	}

	b2TracyCZoneEnd(awake_islands);

	b2TracyCZoneNC(awake_contacts, "Awake Contacts", b2_colorYellowGreen, true);

	// Build awake contact array
	{
		b2Array_Clear(world->awakeContactArray);

		int32_t* contactAwakeIndexArray = world->contactAwakeIndexArray;

		// Iterate the bit set
		// The order of the awake contact array doesn't matter, but I don't want duplicates. It is possible
		// that body A or body B or both bodies wake the contact.
		uint64_t word;
		uint32_t wordCount = awakeContactBitSet->wordCount;
		uint64_t* bits = awakeContactBitSet->bits;
		for (uint32_t k = 0; k < wordCount; ++k)
		{
			word = bits[k];
			while (word != 0)
			{
				uint32_t ctz = b2CTZ(word);
				uint32_t contactIndex = 64 * k + ctz;

				B2_ASSERT(contactAwakeIndexArray[contactIndex] == B2_NULL_INDEX);

				// This cache miss is brutal but is necessary to make contact destruction reasonably quick.
				contactAwakeIndexArray[contactIndex] = b2Array(world->awakeContactArray).count;

				// This is fast
				b2Array_Push(world->awakeContactArray, contactIndex);

				// Clear the smallest set bit
				word = word & (word - 1);
			}
		}
	}

	b2TracyCZoneEnd(awake_contacts);

	// Finish the user tree task that was queued early in the time step. This must be done before touching the broadphase.
	if (b2_parallel)
	{
		if (world->userTreeTask != NULL)
		{
			world->finishTaskFcn(world->userTreeTask, world->userTaskContext);
			world->userTreeTask = NULL;
		}
	}

	b2TracyCZoneNC(broad_phase, "Broadphase", b2_colorPurple, true);

	b2TracyCZoneNC(enlarge_proxies, "Enlarge Proxies", b2_colorDarkTurquoise, true);

	// Enlarge broad-phase proxies and build move array
	{
		b2BroadPhase* broadPhase = &world->broadPhase;

		// Gather bits for all shapes that have enlarged AABBs
		b2BitSet* bitSet = &world->taskContextArray[0].shapeBitSet;
		for (uint32_t i = 1; i < world->workerCount; ++i)
		{
			b2InPlaceUnion(bitSet, &world->taskContextArray[i].shapeBitSet);
		}

		// Apply shape AABB changes to broadphase. This also create the move array which must be
		// ordered to ensure determinism.
		b2Shape* shapes = world->shapes;
		uint64_t word;
		uint32_t wordCount = bitSet->wordCount;
		uint64_t* bits = bitSet->bits;
		for (uint32_t k = 0; k < wordCount; ++k)
		{
			word = bits[k];
			while (word != 0)
			{
				uint32_t ctz = b2CTZ(word);
				uint32_t shapeIndex = 64 * k + ctz;

				b2Shape* shape = shapes + shapeIndex;
				B2_ASSERT(b2ObjectValid(&shape->object));
				if (shape->isFast == false)
				{
					b2BroadPhase_EnlargeProxy(broadPhase, shape->proxyKey, shape->fatAABB);
				}
				else
				{
					// Shape is fast. It's aabb will be enlarged in continuous collision.
					b2BufferMove(broadPhase, shape->proxyKey);
				}

				// Clear the smallest set bit
				word = word & (word - 1);
			}
		}
	}

	b2TracyCZoneEnd(enlarge_proxies);

	b2ValidateBroadphase(&world->broadPhase);

	world->profile.broadphase = b2GetMilliseconds(&timer);

	b2TracyCZoneEnd(broad_phase);

	// TODO_ERIN continuous
#if 0
	b2TracyCZoneNC(continuous_collision, "Continuous", b2_colorDarkGoldenrod, true);

	// Parallel continuous collision
	if (b2_parallel)
	{
		int32_t minRange = 8;
		void* userContinuousTask =
			world->enqueueTaskFcn(&b2ContinuousParallelForTask, world->fastBodyCount, minRange, world, world->userTaskContext);
		world->finishTaskFcn(userContinuousTask, world->userTaskContext);
	}
	else
	{
		b2ContinuousParallelForTask(0, world->fastBodyCount, 0, world);
	}

	// Serially enlarge broad-phase proxies for fast shapes
	{
		b2BroadPhase* broadPhase = &world->broadPhase;
		b2Body* bodies = world->bodies;
		b2Shape* shapes = world->shapes;
		int32_t* fastBodies = world->fastBodies;
		int32_t fastBodyCount = world->fastBodyCount;
		b2DynamicTree* tree = broadPhase->trees + b2_dynamicBody;

		// Warning: this loop has non-deterministic order
		for (int32_t i = 0; i < fastBodyCount; ++i)
		{
			b2Body* fastBody = bodies + fastBodies[i];
			if (fastBody->enlargeAABB == false)
			{
				continue;
			}

			// clear flag
			fastBody->enlargeAABB = false;

			int32_t shapeIndex = fastBody->shapeList;
			while (shapeIndex != B2_NULL_INDEX)
			{
				b2Shape* shape = shapes + shapeIndex;
				if (shape->enlargedAABB == false)
				{
					continue;
				}

				// clear flag
				shape->enlargedAABB = false;

				int32_t proxyKey = shape->proxyKey;
				int32_t proxyId = B2_PROXY_ID(proxyKey);
				B2_ASSERT(B2_PROXY_TYPE(proxyKey) == b2_dynamicBody);

				// all fast shapes should already be in the move buffer

				b2DynamicTree_EnlargeProxy(tree, proxyId, shape->fatAABB);

				shapeIndex = shape->nextShapeIndex;
			}
		}
	}

	b2TracyCZoneEnd(continuous_collision);

	b2FreeStackItem(world->stackAllocator, world->fastBodies);
	world->fastBodies = NULL;

	world->profile.continuous = b2GetMilliseconds(&timer);
#endif

	world->profile.continuous = 0.0f;

	b2TracyCZoneEnd(solve);
}

void b2World_Step(b2WorldId worldId, float timeStep, int32_t velocityIterations, int32_t positionIterations)
{
	if (timeStep == 0.0f)
	{
		// TODO_ERIN would be useful to still process collision while paused
		return;
	}

	b2TracyCZoneNC(world_step, "Step", b2_colorChartreuse, true);

	b2World* world = b2GetWorldFromId(worldId);
	B2_ASSERT(world->locked == false);
	if (world->locked)
	{
		return;
	}

	world->profile = b2_emptyProfile;

	b2Timer stepTimer = b2CreateTimer();

	// Update collision pairs and create contacts
	{
		b2Timer timer = b2CreateTimer();
		b2UpdateBroadPhasePairs(world);
		world->profile.pairs = b2GetMilliseconds(&timer);
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
	context.enableWarmStarting = world->enableWarmStarting;
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

	if (b2_parallel)
	{
		// This finishes tree rebuild and split island tasks
		world->finishAllTasksFcn(world->userTaskContext);
	}

	B2_ASSERT(b2GetStackAllocation(world->stackAllocator) == 0);

	// Ensure stack is large enough
	b2GrowStack(world->stackAllocator);

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
				else if (b->isFast)
				{
					b2DrawShape(draw, shape, xf, b2MakeColor(b2_colorSalmon, 1.0f));
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

		int32_t count = world->bodyPool.capacity;
		for (int32_t i = 0; i < count; ++i)
		{
			b2Body* b = world->bodies + i;
			if (b->object.next != i)
			{
				continue;
			}

			char buffer[32];
			sprintf(buffer, "%d", b->object.index);
			draw->DrawString(b->position, buffer, draw->context);

			int32_t shapeIndex = b->shapeList;
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

	if (draw->drawMass)
	{
		b2Vec2 offset = {0.1f, 0.1f};
		b2Body* bodies = world->bodies;
		int32_t bodyCapacity = world->bodyPool.capacity;
		for (int32_t i = 0; i < bodyCapacity; ++i)
		{
			b2Body* body = bodies + i;
			if (b2ObjectValid(&body->object) == false)
			{
				continue;
			}

			draw->DrawTransform(body->transform, draw->context);

			b2Vec2 p = b2TransformPoint(body->transform, offset);

			char buffer[32];
			sprintf(buffer, "%.1f", body->mass);
			draw->DrawString(p, buffer, draw->context);
		}
	}
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
		int32_t count = world->islandPool.capacity;
		for (int32_t i = 0; i < count; ++i)
		{
			b2Island* island = world->islands + i;
			if (island->object.next != i)
			{
				continue;
			}

			b2WakeIsland(island);
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

void b2World_EnableContinuo(b2WorldId worldId, bool flag)
{
	b2World* world = b2GetWorldFromId(worldId);
	B2_ASSERT(world->locked == false);
	if (world->locked)
	{
		return;
	}

	world->enableContinuous = flag;
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
	s.stackCapacity = b2GetStackCapacity(world->stackAllocator);
	s.stackUsed = b2GetMaxStackAllocation(world->stackAllocator);
	s.byteCount = b2GetByteCount();
	for (int32_t i = 0; i <= b2_graphColorCount; ++i)
	{
		s.colorCounts[i] = world->graph.occupancy[i];
	}
	return s;
}

#if 0
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

static bool TreeQueryCallback(int32_t proxyId, int32_t shapeIndex, void* context)
{
	B2_MAYBE_UNUSED(proxyId);

	WorldQueryContext* worldContext = (WorldQueryContext*)context;
	b2World* world = worldContext->world;

	B2_ASSERT(0 <= shapeIndex && shapeIndex < world->shapePool.capacity);

	b2Shape* shape = world->shapes + shapeIndex;
	B2_ASSERT(shape->object.index == shape->object.next);

	b2ShapeId shapeId = {shape->object.index, world->index, shape->object.revision};
	bool result = worldContext->fcn(shapeId, worldContext->userContext);
	return result;
}

void b2World_QueryAABB(b2WorldId worldId, b2AABB aabb, b2QueryCallbackFcn* fcn, void* context)
{
	b2World* world = b2GetWorldFromId(worldId);
	B2_ASSERT(world->locked == false);
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
