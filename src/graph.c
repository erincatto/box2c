// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "graph.h"

#include "allocate.h"
#include "array.h"
#include "body.h"
#include "contact.h"
#include "contact_solver.h"
#include "core.h"
#include "joint.h"
#include "shape.h"
#include "solver_data.h"
#include "stack_allocator.h"
#include "world.h"

#include "box2d/aabb.h"

#include <limits.h>
#include <stdatomic.h>
#include <stdbool.h>

typedef struct b2WorkerContext
{
	b2SolverTaskContext* context;
	int32_t workerIndex;
} b2WorkerContext;

void b2CreateGraph(b2Graph* graph, int32_t bodyCapacity, int32_t contactCapacity)
{
	bodyCapacity = B2_MAX(bodyCapacity, 8);
	contactCapacity = B2_MAX(contactCapacity, 8);

	for (int32_t i = 0; i < b2_graphColorCount; ++i)
	{
		b2GraphColor* color = graph->colors + i;
		color->bodySet = b2CreateBitSet(bodyCapacity);
		b2SetBitCountAndClear(&color->bodySet, bodyCapacity);

		color->contactArray = b2CreateArray(sizeof(int32_t), contactCapacity);
	}
}

void b2DestroyGraph(b2Graph* graph)
{
	for (int32_t i = 0; i < b2_graphColorCount; ++i)
	{
		b2GraphColor* color = graph->colors + i;
		b2DestroyBitSet(&color->bodySet);
		b2DestroyArray(color->contactArray, sizeof(int32_t));
	}
}

void b2AddContactToGraph(b2World* world, b2Contact* contact)
{
	B2_ASSERT(contact->colorContactIndex == B2_NULL_INDEX);
	B2_ASSERT(contact->colorIndex == B2_NULL_INDEX);

	b2Graph* graph = &world->graph;

	int32_t bodyIndexA = contact->edges[0].bodyIndex;
	int32_t bodyIndexB = contact->edges[1].bodyIndex;

	b2BodyType typeA = world->bodies[bodyIndexA].type;
	b2BodyType typeB = world->bodies[bodyIndexB].type;

	if (typeA == b2_dynamicBody && typeB == b2_dynamicBody)
	{
		for (int32_t i = 0; i < b2_graphColorCount; ++i)
		{
			b2GraphColor* color = graph->colors + i;
			if (b2GetBit(&color->bodySet, bodyIndexA) || b2GetBit(&color->bodySet, bodyIndexB))
			{
				continue;
			}

			b2SetBitGrow(&color->bodySet, bodyIndexA);
			b2SetBitGrow(&color->bodySet, bodyIndexB);

			contact->colorContactIndex = b2Array(color->contactArray).count;
			b2Array_Push(color->contactArray, contact->object.index);
			contact->colorIndex = i;
			contact->flags &= ~b2_contactStatic;
			break;
		}
	}
	else if (typeA == b2_dynamicBody)
	{
		// Static contacts never in color 0
		for (int32_t i = 1; i < b2_graphColorCount; ++i)
		{
			b2GraphColor* color = graph->colors + i;
			if (b2GetBit(&color->bodySet, bodyIndexA))
			{
				continue;
			}

			b2SetBitGrow(&color->bodySet, bodyIndexA);

			contact->colorContactIndex = b2Array(color->contactArray).count;
			b2Array_Push(color->contactArray, contact->object.index);
			contact->colorIndex = i;
			break;
		}
	}
	else if (typeB == b2_dynamicBody)
	{
		// Static contacts never in color 0
		for (int32_t i = 1; i < b2_graphColorCount; ++i)
		{
			b2GraphColor* color = graph->colors + i;
			if (b2GetBit(&color->bodySet, bodyIndexB))
			{
				continue;
			}

			b2SetBitGrow(&color->bodySet, bodyIndexB);

			contact->colorContactIndex = b2Array(color->contactArray).count;
			b2Array_Push(color->contactArray, contact->object.index);
			contact->colorIndex = i;
			break;
		}
	}

	B2_ASSERT(contact->colorIndex != B2_NULL_INDEX && contact->colorContactIndex != B2_NULL_INDEX);
}

void b2RemoveContactFromGraph(b2World* world, b2Contact* contact)
{
	B2_ASSERT(contact->colorIndex != B2_NULL_INDEX);
	B2_ASSERT(contact->colorContactIndex != B2_NULL_INDEX);

	b2Graph* graph = &world->graph;

	B2_ASSERT(0 <= contact->colorIndex && contact->colorIndex < b2_graphColorCount);
	int32_t bodyIndexA = contact->edges[0].bodyIndex;
	int32_t bodyIndexB = contact->edges[1].bodyIndex;

	b2BodyType typeA = world->bodies[bodyIndexA].type;
	b2BodyType typeB = world->bodies[bodyIndexB].type;

	if (typeA == b2_dynamicBody && typeB == b2_dynamicBody)
	{
		b2GraphColor* color = graph->colors + contact->colorIndex;
		B2_ASSERT(b2GetBit(&color->bodySet, bodyIndexA) && b2GetBit(&color->bodySet, bodyIndexB));

		int32_t colorContactIndex = contact->colorContactIndex;
		b2Array_RemoveSwap(color->contactArray, colorContactIndex);
		if (colorContactIndex < b2Array(color->contactArray).count)
		{
			// Fix index on swapped contact
			int32_t swappedContactIndex = color->contactArray[colorContactIndex];
			world->contacts[swappedContactIndex].colorContactIndex = colorContactIndex;
		}

		b2ClearBit(&color->bodySet, bodyIndexA);
		b2ClearBit(&color->bodySet, bodyIndexB);
	}
	else if (typeA == b2_dynamicBody)
	{
		b2GraphColor* color = graph->colors + contact->colorIndex;
		B2_ASSERT(b2GetBit(&color->bodySet, bodyIndexA));

		int32_t colorContactIndex = contact->colorContactIndex;
		b2Array_RemoveSwap(color->contactArray, colorContactIndex);
		if (colorContactIndex < b2Array(color->contactArray).count)
		{
			// Fix index on swapped contact
			int32_t swappedContactIndex = color->contactArray[colorContactIndex];
			world->contacts[swappedContactIndex].colorContactIndex = colorContactIndex;
		}

		b2ClearBit(&color->bodySet, bodyIndexA);
	}
	else if (typeB == b2_dynamicBody)
	{
		b2GraphColor* color = graph->colors + contact->colorIndex;
		B2_ASSERT(b2GetBit(&color->bodySet, bodyIndexB));

		int32_t colorContactIndex = contact->colorContactIndex;
		b2Array_RemoveSwap(color->contactArray, colorContactIndex);
		if (colorContactIndex < b2Array(color->contactArray).count)
		{
			// Fix index on swapped contact
			int32_t swappedContactIndex = color->contactArray[colorContactIndex];
			world->contacts[swappedContactIndex].colorContactIndex = colorContactIndex;
		}

		b2ClearBit(&color->bodySet, bodyIndexB);
	}

	contact->colorIndex = B2_NULL_INDEX;
	contact->colorContactIndex = B2_NULL_INDEX;
	contact->flags &= ~b2_contactStatic;
}

static void b2IntegrateVelocitiesTask(int32_t startIndex, int32_t endIndex, b2SolverTaskContext* context)
{
	b2TracyCZoneNC(integrate_velocity, "IntVel", b2_colorDeepPink, true);

	b2Vec2 gravity = context->world->gravity;
	b2Body** bodies = context->awakeBodies;
	b2SolverBody* solverBodies = context->solverBodies;
	int32_t* bodyMap = context->bodyMap;

	float h = context->timeStep;

	// Integrate velocities and apply damping. Initialize the body state.
	for (int32_t i = startIndex; i < endIndex; ++i)
	{
		b2Body* body = bodies[i];
		//_m_prefetch(bodies[i + 1]);

		// create body map used to prepare constraints
		B2_ASSERT(body->object.index < context->world->bodyPool.capacity);
		bodyMap[body->object.index] = i;

		float invMass = body->invMass;
		float invI = body->invI;

		b2Vec2 v = body->linearVelocity;
		float w = body->angularVelocity;

		// Integrate velocities
		v = b2Add(v, b2MulSV(h * invMass, b2MulAdd(body->force, body->gravityScale * body->mass, gravity)));
		w = w + h * invI * body->torque;

		// Apply damping.
		// ODE: dv/dt + c * v = 0
		// Solution: v(t) = v0 * exp(-c * t)
		// Time step: v(t + dt) = v0 * exp(-c * (t + dt)) = v0 * exp(-c * t) * exp(-c * dt) = v * exp(-c * dt)
		// v2 = exp(-c * dt) * v1
		// Pade approximation:
		// v2 = v1 * 1 / (1 + c * dt)
		v = b2MulSV(1.0f / (1.0f + h * body->linearDamping), v);
		w *= 1.0f / (1.0f + h * body->angularDamping);

		b2SolverBody* solverBody = solverBodies + i;
		solverBody->linearVelocity = v;
		solverBody->angularVelocity = w;

		solverBody->deltaAngle = 0.0f;
		solverBody->deltaPosition = b2Vec2_zero;

		solverBody->invMass = invMass;
		solverBody->invI = invI;

		solverBody->bodyIndex = body->object.index;
	}

	b2TracyCZoneEnd(integrate_velocity);
}

static void b2PrepareJointsTask(b2SolverTaskContext* context)
{
	b2World* world = context->world;
	b2Joint* joints = world->joints;
	int32_t jointCapacity = world->jointPool.capacity;
	b2StepContext* stepContext = context->stepContext;

	for (int32_t i = 0; i < jointCapacity; ++i)
	{
		b2Joint* joint = joints + i;
		if (b2ObjectValid(&joint->object) == false)
		{
			continue;
		}

		b2PrepareJoint(joint, stepContext);
	}
}

static void b2SolveJointsTask(b2SolverTaskContext* context, bool useBias)
{
	b2World* world = context->world;
	b2Joint* joints = world->joints;
	int32_t jointCapacity = world->jointPool.capacity;
	b2StepContext* stepContext = context->stepContext;

	for (int32_t i = 0; i < jointCapacity; ++i)
	{
		b2Joint* joint = joints + i;
		if (b2ObjectValid(&joint->object) == false)
		{
			continue;
		}

		b2SolveJointVelocity(joint, stepContext, useBias);
	}
}

static void b2IntegratePositionsTask(int32_t startIndex, int32_t endIndex, b2SolverTaskContext* context)
{
	b2TracyCZoneNC(integrate_positions, "IntPos", b2_colorDarkSeaGreen, true);

	b2SolverBody* bodies = context->solverBodies;
	float h = context->subStep;

	B2_ASSERT(startIndex <= endIndex);

	for (int32_t i = startIndex; i < endIndex; ++i)
	{
		b2SolverBody* body = bodies + i;
		body->deltaAngle += h * body->angularVelocity;
		body->deltaPosition = b2MulAdd(body->deltaPosition, h, body->linearVelocity);
	}

	b2TracyCZoneEnd(integrate_positions);
}

static void b2FinalizePositionsTask(int32_t startIndex, int32_t endIndex, b2SolverTaskContext* context, int32_t workerIndex)
{
	b2TracyCZoneNC(finalize_positions, "FinPos", b2_colorViolet, true);

	b2World* world = context->world;
	b2Body* bodies = world->bodies;
	b2SolverBody* solverBodies = context->solverBodies;
	b2Contact* contacts = world->contacts;
	const b2Vec2 aabbMargin = {b2_aabbMargin, b2_aabbMargin};

	b2BitSet* awakeContactBitSet = &world->taskContextArray[workerIndex].awakeContactBitSet;
	b2BitSet* shapeBitSet = &world->taskContextArray[workerIndex].shapeBitSet;

	B2_ASSERT(startIndex <= endIndex);
	B2_ASSERT(startIndex <= world->bodyPool.capacity);
	B2_ASSERT(endIndex <= world->bodyPool.capacity);

	for (int32_t i = startIndex; i < endIndex; ++i)
	{
		b2SolverBody* solverBody = solverBodies + i;

		b2Body* body = bodies + solverBody->bodyIndex;

		body->linearVelocity = solverBody->linearVelocity;
		body->angularVelocity = solverBody->angularVelocity;
		body->position = b2Add(body->position, solverBody->deltaPosition);
		body->angle += solverBody->deltaAngle;

		body->transform.q = b2MakeRot(body->angle);
		body->transform.p = b2Sub(body->position, b2RotateVector(body->transform.q, body->localCenter));

		body->force = b2Vec2_zero;
		body->torque = 0.0f;

		// Update shapes AABBs
		int32_t shapeIndex = body->shapeList;
		while (shapeIndex != B2_NULL_INDEX)
		{
			b2Shape* shape = world->shapes + shapeIndex;

			B2_ASSERT(shape->isFast == false);

			shape->aabb = b2Shape_ComputeAABB(shape, body->transform);

			if (b2AABB_Contains(shape->fatAABB, shape->aabb) == false)
			{
				shape->fatAABB.lowerBound = b2Sub(shape->aabb.lowerBound, aabbMargin);
				shape->fatAABB.upperBound = b2Add(shape->aabb.upperBound, aabbMargin);

				// Bit-set to keep the move array sorted
				b2SetBit(shapeBitSet, shapeIndex);
			}

			shapeIndex = shape->nextShapeIndex;
		}

		// TODO_ERIN legacy
		int32_t contactKey = body->contactList;
		while (contactKey != B2_NULL_INDEX)
		{
			int32_t contactIndex = contactKey >> 1;
			int32_t edgeIndex = contactKey & 1;
			b2Contact* contact = contacts + contactIndex;

			// Bit set to prevent duplicates
			b2SetBit(awakeContactBitSet, contactIndex);
			contactKey = contact->edges[edgeIndex].nextKey;
		}
	}

	b2TracyCZoneEnd(finalize_positions);
}

static void b2ExecuteBlock(b2SolverStage* stage, b2SolverTaskContext* context, int32_t startIndex, int32_t endIndex, int32_t workerIndex)
{
	b2SolverStageType type = stage->type;

	switch (type)
	{
		case b2_stageIntegrateVelocities:
			b2IntegrateVelocitiesTask(startIndex, endIndex, context);
			break;

		case b2_stagePrepareContacts:
			b2PrepareContactsTask(startIndex, endIndex, context, stage->colorIndex);
			break;

		case b2_stageSolveContacts:
			b2SolveContactsTask(startIndex, endIndex, context, stage->colorIndex, true);
			break;

		case b2_stageIntegratePositions:
			b2IntegratePositionsTask(startIndex, endIndex, context);
			break;

		case b2_stageFinalizePositions:
			b2FinalizePositionsTask(startIndex, endIndex, context, workerIndex);
			break;

		case b2_stageCalmContacts:
			b2SolveContactsTask(startIndex, endIndex, context, stage->colorIndex, false);
			break;

		case b2_stageStoreImpulses:
			b2StoreImpulsesTask(startIndex, endIndex, context);
			break;
	}
}

static inline int32_t GetWorkerStartIndex(int32_t workerIndex, int32_t blockCount, int32_t workerCount)
{
	if (blockCount <= workerCount)
	{
		return workerIndex < blockCount ? workerIndex : B2_NULL_INDEX;
	}

	int32_t blocksPerWorker = blockCount / workerCount;
	int32_t remainder = blockCount - blocksPerWorker * workerCount;
	return blocksPerWorker * workerIndex + B2_MIN(remainder, workerIndex);
}

static void b2ExecuteStage(b2SolverStage* stage, b2SolverTaskContext* context, int previousSyncIndex, int syncIndex, int32_t workerIndex)
{
	int32_t completedCount = 0;
	b2SolverBlock* blocks = stage->blocks;
	int32_t blockCount = stage->blockCount;

	int32_t expectedSyncIndex = previousSyncIndex;

	int32_t startIndex = GetWorkerStartIndex(workerIndex, blockCount, context->workerCount);
	if (startIndex == B2_NULL_INDEX)
	{
		return;
	}

	B2_ASSERT(0 <= startIndex && startIndex < blockCount);

	int32_t blockIndex = startIndex;

	// Caution: this can change expectedSyncIndex
	while (atomic_compare_exchange_strong(&blocks[blockIndex].syncIndex, &expectedSyncIndex, syncIndex) == true)
	{
		B2_ASSERT(stage->type != b2_stagePrepareContacts || syncIndex < 2);

		B2_ASSERT(completedCount < blockCount);

		b2ExecuteBlock(stage, context, blocks[blockIndex].startIndex, blocks[blockIndex].endIndex, workerIndex);

		completedCount += 1;
		blockIndex += 1;
		if (blockIndex >= blockCount)
		{
			// Keep looking for work
			blockIndex = 0;
		}

		expectedSyncIndex = previousSyncIndex;
	}

	// Search backwards for blocks
	blockIndex = startIndex - 1;
	while (true)
	{
		if (blockIndex < 0)
		{
			blockIndex = blockCount - 1;
		}

		expectedSyncIndex = previousSyncIndex;

		// Caution: this can change expectedSyncIndex
		if (atomic_compare_exchange_strong(&blocks[blockIndex].syncIndex, &expectedSyncIndex, syncIndex) == false)
		{
			break;
		}

		b2ExecuteBlock(stage, context, blocks[blockIndex].startIndex, blocks[blockIndex].endIndex, workerIndex);
		completedCount += 1;
		blockIndex -= 1;
	}

	(void)atomic_fetch_add(&stage->completionCount, completedCount);
}

static void b2ExecuteMainStage(b2SolverStage* stage, b2SolverTaskContext* context, uint32_t syncBits)
{
	int32_t blockCount = stage->blockCount;
	if (blockCount == 0)
	{
		return;
	}

	if (blockCount == 1)
	{
		b2ExecuteBlock(stage, context, stage->blocks[0].startIndex, stage->blocks[0].endIndex, 0);
	}
	else
	{
		atomic_store(&context->syncBits, syncBits);

		int syncIndex = (syncBits >> 16) & 0xFFFF;
		B2_ASSERT(syncIndex > 0);
		int previousSyncIndex = syncIndex - 1;

		if (stage->type == b2_stagePrepareContacts)
		{
			if (stage->blockCount > 0 && stage->blocks[0].syncIndex > 1)
			{
				stage->type += 0;
			}
		}

		b2ExecuteStage(stage, context, previousSyncIndex, syncIndex, 0);

		while (atomic_load(&stage->completionCount) != blockCount)
		{
			_mm_pause();
		}

		atomic_store(&stage->completionCount, 0);
	}
}

// This should not use the thread index because thread 0 should not be called twice, which is possible with work stealing.
void b2SolverTask(int32_t startIndex, int32_t endIndex, uint32_t threadIndexDontUse, void* taskContext)
{
	B2_MAYBE_UNUSED(startIndex);
	B2_MAYBE_UNUSED(endIndex);
	B2_MAYBE_UNUSED(threadIndexDontUse);

	b2WorkerContext* workerContext = taskContext;
	int32_t workerIndex = workerContext->workerIndex;
	b2SolverTaskContext* context = workerContext->context;
	int32_t activeColorCount = context->activeColorCount;
	b2SolverStage* stages = context->stages;

	if (workerIndex == 0)
	{
		// Main thread synchronizes the workers and does work itself.
		//
		// Stages are re-used for loops so that I don't need more stages for large iteration counts.
		// The sync indices grow monotonically for the body/graph/constraint groupings because they share solver blocks.
		// The stage index and sync indices are combined in to sync bits for atomic synchronization.
		// The workers need to compute the previous sync index for a given stage so that CAS works correctly. This
		// setup makes this easy to do.

		/*
		b2_stageIntegrateVelocities = 0,
		b2_stagePrepareJoints,
		b2_stagePrepareContacts,
		b2_stageSolveJoints,
		b2_stageSolveContacts,
		b2_stageIntegratePositions,
		b2_stageFinalizePositions,
		b2_stageCalmJoints,
		b2_stageCalmContacts,
		b2_stageStoreImpulses
		*/

		if (stages[3].type == b2_stagePrepareContacts)
		{
			for (int32_t i = 0; i < stages[3].blockCount; ++i)
			{
				B2_ASSERT(stages[3].blocks[i].syncIndex == 0);
			}
		}

		int32_t bodySyncIndex = 1;
		int32_t stageIndex = 0;
		uint32_t syncBits = (bodySyncIndex << 16) | stageIndex;
		B2_ASSERT(stages[stageIndex].type == b2_stageIntegrateVelocities);
		b2ExecuteMainStage(stages + stageIndex, context, syncBits);
		stageIndex += 1;
		bodySyncIndex += 1;

		// TODO_ERIN single threaded
		B2_ASSERT(stages[stageIndex].type == b2_stagePrepareJoints);
		b2PrepareJointsTask(context);
		stageIndex += 1;

		int32_t graphSyncIndex = 1;
		for (int32_t colorIndex = 0; colorIndex < activeColorCount; ++colorIndex)
		{
			syncBits = (graphSyncIndex << 16) | stageIndex;
			B2_ASSERT(stages[stageIndex].type == b2_stagePrepareContacts);
			b2ExecuteMainStage(stages + stageIndex, context, syncBits);
			stageIndex += 1;
		}
		graphSyncIndex += 1;

		int32_t velocityIterations = context->velocityIterations;
		for (int32_t i = 0; i < velocityIterations; ++i)
		{
			// stage index restarted each iteration
			int32_t iterStageIndex = stageIndex;

			B2_ASSERT(stages[iterStageIndex].type == b2_stageSolveJoints);
			b2SolveJointsTask(context, true);
			iterStageIndex += 1;

			for (int32_t colorIndex = 0; colorIndex < activeColorCount; ++colorIndex)
			{
				syncBits = (graphSyncIndex << 16) | iterStageIndex;
				B2_ASSERT(stages[iterStageIndex].type == b2_stageSolveContacts);
				b2ExecuteMainStage(stages + iterStageIndex, context, syncBits);
				iterStageIndex += 1;
			}
			graphSyncIndex += 1;

			B2_ASSERT(stages[iterStageIndex].type == b2_stageIntegratePositions);
			syncBits = (bodySyncIndex << 16) | iterStageIndex;
			b2ExecuteMainStage(stages + iterStageIndex, context, syncBits);
			bodySyncIndex += 1;
		}

		stageIndex += 1 + activeColorCount + 1;

		syncBits = (bodySyncIndex << 16) | stageIndex;
		B2_ASSERT(stages[stageIndex].type == b2_stageFinalizePositions);
		b2ExecuteMainStage(stages + stageIndex, context, syncBits);
		stageIndex += 1;

		int32_t calmIterations = context->calmIterations;
		for (int32_t i = 0; i < calmIterations; ++i)
		{
			// stage index restarted each iteration
			int32_t iterStageIndex = stageIndex;

			B2_ASSERT(stages[iterStageIndex].type == b2_stageCalmJoints);
			b2SolveJointsTask(context, false);
			iterStageIndex += 1;

			for (int32_t colorIndex = 0; colorIndex < activeColorCount; ++colorIndex)
			{
				syncBits = (graphSyncIndex << 16) | iterStageIndex;
				B2_ASSERT(stages[iterStageIndex].type == b2_stageCalmContacts);
				b2ExecuteMainStage(stages + iterStageIndex, context, syncBits);
				iterStageIndex += 1;
			}
			graphSyncIndex += 1;
		}

		stageIndex += 1 + activeColorCount;

		uint32_t constraintSyncIndex = 1;
		syncBits = (constraintSyncIndex << 16) | stageIndex;
		B2_ASSERT(stages[stageIndex].type == b2_stageStoreImpulses);
		b2ExecuteMainStage(stages + stageIndex, context, syncBits);

		// Signal workers to finish
		atomic_store(&context->syncBits, UINT_MAX);

		B2_ASSERT(stageIndex + 1 == context->stageCount);
		return;
	}

	// Worker
	uint32_t lastSyncBits = 0;

	while (true)
	{
		// Spin until main thread bumps changes the sync bits
		uint32_t syncBits = atomic_load(&context->syncBits);
		while (syncBits == lastSyncBits)
		{
			_mm_pause();
			syncBits = atomic_load(&context->syncBits);
		}

		if (syncBits == UINT_MAX)
		{
			// sentinel hit
			break;
		}

		int32_t stageIndex = syncBits & 0xFFFF;
		B2_ASSERT(stageIndex < context->stageCount);

		int32_t syncIndex = (syncBits >> 16) & 0xFFFF;
		B2_ASSERT(syncIndex > 0);

		int32_t previousSyncIndex = syncIndex - 1;

		b2SolverStage* stage = stages + stageIndex;
		b2ExecuteStage(stage, context, previousSyncIndex, syncIndex, workerIndex);

		lastSyncBits = syncBits;
	}
}

// Threading:
// 1. build array of awake bodies, maybe copy to contiguous array
// 2. parallel-for integrate velocities
// 3. parallel prepare constraints by color
// Loop sub-steps:
// 4. parallel solve constraints by color
// 5. parallel-for update position deltas (and positions on last iter)
// End Loop
// Loop bias-removal:
// 6. parallel solve constraints by color
// End loop
// 7. parallel-for store impulses
// 8. parallel-for update aabbs, build proxy update set, build awake contact set

// Soft constraints with constraint error substepping. Allows for stiffer contacts with a small performance hit. Includes a
// bias removal stage to help remove excess bias energy.
// http://mmacklin.com/smallsteps.pdf
// https://box2d.org/files/ErinCatto_SoftConstraints_GDC2011.pdf
void b2SolveGraph(b2World* world, b2StepContext* stepContext)
{
	b2TracyCZoneNC(prepare_stages, "Prepare Stages", b2_colorDarkOrange, true);

	b2Graph* graph = &world->graph;
	b2GraphColor* colors = graph->colors;

	int32_t awakeIslandCount = b2Array(world->awakeIslandArray).count;
	int32_t awakeBodyCount = 0;
	for (int32_t i = 0; i < awakeIslandCount; ++i)
	{
		int32_t islandIndex = world->awakeIslandArray[i];
		b2Island* island = world->islands + islandIndex;
		awakeBodyCount += island->bodyCount;
	}

	if (awakeBodyCount == 0)
	{
		return;
	}

	b2Body* bodies = world->bodies;
	b2Body** awakeBodies = b2AllocateStackItem(world->stackAllocator, awakeBodyCount * sizeof(b2Body*), "awake bodies");
	b2SolverBody* solverBodies = b2AllocateStackItem(world->stackAllocator, awakeBodyCount * sizeof(b2SolverBody), "solver bodies");

	int32_t bodyCapacity = world->bodyPool.capacity;
	int32_t* bodyMap = b2AllocateStackItem(world->stackAllocator, bodyCapacity * sizeof(int32_t), "body map");
	memset(bodyMap, 0xFF, bodyCapacity * sizeof(int32_t));

	int32_t index = 0;
	for (int32_t i = 0; i < awakeIslandCount; ++i)
	{
		int32_t islandIndex = world->awakeIslandArray[i];
		b2Island* island = world->islands + islandIndex;
		int32_t bodyIndex = island->headBody;
		while (bodyIndex != B2_NULL_INDEX)
		{
			b2Body* body = bodies + bodyIndex;
			B2_ASSERT(b2ObjectValid(&body->object));
			B2_ASSERT(body->object.index == bodyIndex);

			awakeBodies[index] = body;

			B2_ASSERT(0 < bodyIndex && bodyIndex < bodyCapacity);
			bodyMap[bodyIndex] = index;

			// cache miss
			bodyIndex = body->islandNext;

			index += 1;
		}
	}
	B2_ASSERT(index == awakeBodyCount);

	int32_t workerCount = world->workerCount;
	const int32_t blocksPerWorker = 6;

	int32_t bodyBlockSize = 1 << 5;
	int32_t bodyBlockCount = ((awakeBodyCount - 1) >> 5) + 1;
	if (awakeBodyCount > blocksPerWorker * bodyBlockSize * workerCount)
	{
		bodyBlockSize = awakeBodyCount / (blocksPerWorker * workerCount);
		bodyBlockCount = blocksPerWorker * workerCount;
	}

	int32_t activeColorIndices[b2_graphColorCount];
	int32_t colorConstraintCounts[b2_graphColorCount];
	int32_t colorBlockSize[b2_graphColorCount];
	int32_t colorBlockCounts[b2_graphColorCount];

	int32_t graphBlockSize = 1 << 2;
	int32_t activeColorCount = 0;
	int32_t graphBlockCount = 0;
	int32_t constraintCount = 0;

	int32_t c = 0;
	for (int32_t i = 0; i < b2_graphColorCount; ++i)
	{
		int32_t count = b2Array(colors[i].contactArray).count;
		if (count > 0)
		{
			activeColorIndices[c] = i;
			colorConstraintCounts[c] = count;
			int32_t blockCount = ((count - 1) >> 2) + 1;
			colorBlockSize[c] = graphBlockSize;
			if (count > blocksPerWorker * graphBlockSize * workerCount)
			{
				colorBlockSize[c] = count / (blocksPerWorker * workerCount);
				blockCount = blocksPerWorker * workerCount;
			}

			colorBlockCounts[c] = blockCount;
			graphBlockCount += blockCount;
			constraintCount += count;
			c += 1;
		}
	}
	activeColorCount = c;

	b2ContactConstraint* constraints =
		b2AllocateStackItem(world->stackAllocator, constraintCount * sizeof(b2ContactConstraint), "constraint");

	int32_t base = 0;

	for (int32_t i = 0; i < activeColorCount; ++i)
	{
		int32_t j = activeColorIndices[i];
		colors[j].contactConstraints = constraints + base;
		base += b2Array(colors[j].contactArray).count;
	}

	int32_t storeBlockSize = 1 << 4;
	int32_t storeBlockCount = constraintCount > 0 ? ((constraintCount - 1) >> 4) + 1 : 0;
	if (constraintCount > blocksPerWorker * storeBlockSize * workerCount)
	{
		storeBlockSize = constraintCount / (blocksPerWorker * workerCount);
		storeBlockCount = blocksPerWorker * workerCount;
	}

	/*
	b2_stageIntegrateVelocities = 0,
	b2_stagePrepareJoints,
	b2_stagePrepareContacts,
	b2_stageSolveJoints,
	b2_stageSolveContacts,
	b2_stageIntegratePositions,
	b2_stageFinalizePositions,
	b2_stageCalmJoints,
	b2_stageCalmContacts,
	b2_stageStoreImpulses
	*/

	// TODO_ERIN joint tasks
	int32_t stageCount = 0;

	// b2_stageIntegrateVelocities
	stageCount += 1;
	// b2_stagePrepareJoints
	stageCount += 1;
	// b2_stagePrepareContacts
	stageCount += activeColorCount;
	// b2_stageSolveJoints, b2_stageSolveContacts, b2_stageIntegratePositions
	stageCount += 1 + activeColorCount + 1;
	// b2_stageFinalizePositions
	stageCount += 1;
	// b2_stageCalmJoints, b2_stageCalmContacts
	stageCount += 1 + activeColorCount;
	// b2_stageStoreImpulses
	stageCount += 1;

	b2SolverStage* stages = b2AllocateStackItem(world->stackAllocator, stageCount * sizeof(b2SolverStage), "stages");
	b2SolverBlock* bodyBlocks = b2AllocateStackItem(world->stackAllocator, bodyBlockCount * sizeof(b2SolverBlock), "body blocks");
	b2SolverBlock* graphBlocks = b2AllocateStackItem(world->stackAllocator, graphBlockCount * sizeof(b2SolverBlock), "graph blocks");
	b2SolverBlock* storeBlocks = b2AllocateStackItem(world->stackAllocator, storeBlockCount * sizeof(b2SolverBlock), "store blocks");

	for (int32_t i = 0; i < bodyBlockCount; ++i)
	{
		b2SolverBlock* block = bodyBlocks + i;
		block->startIndex = i * bodyBlockSize;
		block->endIndex = block->startIndex + bodyBlockSize;
		block->syncIndex = 0;
	}
	bodyBlocks[bodyBlockCount - 1].endIndex = awakeBodyCount;

	b2SolverBlock* colorBlocks[b2_graphColorCount];
	b2SolverBlock* baseGraphBlock = graphBlocks;

	for (int32_t i = 0; i < activeColorCount; ++i)
	{
		int32_t blockCount = colorBlockCounts[i];
		int32_t blockSize = colorBlockSize[i];
		for (int32_t j = 0; j < blockCount; ++j)
		{
			b2SolverBlock* block = baseGraphBlock + j;
			block->startIndex = j * blockSize;
			block->endIndex = block->startIndex + blockSize;
			atomic_store(&block->syncIndex, 0);
		}
		baseGraphBlock[blockCount - 1].endIndex = colorConstraintCounts[i];

		colorBlocks[i] = baseGraphBlock;
		baseGraphBlock += blockCount;
	}

	for (int32_t i = 0; i < storeBlockCount; ++i)
	{
		b2SolverBlock* block = storeBlocks + i;
		block->startIndex = i * storeBlockSize;
		block->endIndex = block->startIndex + storeBlockSize;
		block->syncIndex = 0;
	}

	if (storeBlockCount > 0)
	{
		storeBlocks[storeBlockCount - 1].endIndex = constraintCount;
	}

	b2SolverStage* stage = stages;

	// Integrate velocities
	stage->type = b2_stageIntegrateVelocities;
	stage->blocks = bodyBlocks;
	stage->blockCount = bodyBlockCount;
	stage->colorIndex = -1;
	stage->completionCount = 0;
	stage += 1;

	// Prepare joints
	stage->type = b2_stagePrepareJoints;
	stage->blocks = NULL;
	stage->blockCount = 0;
	stage->colorIndex = -1;
	stage->completionCount = 0;
	stage += 1;

	// Prepare constraints
	for (int32_t i = 0; i < activeColorCount; ++i)
	{
		stage->type = b2_stagePrepareContacts;
		stage->blocks = colorBlocks[i];
		stage->blockCount = colorBlockCounts[i];
		stage->colorIndex = activeColorIndices[i];
		stage->completionCount = 0;
		stage += 1;
	}

	// Solve joints
	stage->type = b2_stageSolveJoints;
	stage->blocks = NULL;
	stage->blockCount = 0;
	stage->colorIndex = -1;
	stage->completionCount = 0;
	stage += 1;

	// Solve constraints
	for (int32_t i = 0; i < activeColorCount; ++i)
	{
		stage->type = b2_stageSolveContacts;
		stage->blocks = colorBlocks[i];
		stage->blockCount = colorBlockCounts[i];
		stage->colorIndex = activeColorIndices[i];
		stage->completionCount = 0;
		stage += 1;
	}

	// Integrate positions
	stage->type = b2_stageIntegratePositions;
	stage->blocks = bodyBlocks;
	stage->blockCount = bodyBlockCount;
	stage->colorIndex = -1;
	stage->completionCount = 0;
	stage += 1;

	// Finalize positions
	stage->type = b2_stageFinalizePositions;
	stage->blocks = bodyBlocks;
	stage->blockCount = bodyBlockCount;
	stage->colorIndex = -1;
	stage->completionCount = 0;
	stage += 1;

	// Calm joints
	stage->type = b2_stageCalmJoints;
	stage->blocks = NULL;
	stage->blockCount = 0;
	stage->colorIndex = -1;
	stage->completionCount = 0;
	stage += 1;

	// Calm constraints
	for (int32_t i = 0; i < activeColorCount; ++i)
	{
		stage->type = b2_stageCalmContacts;
		stage->blocks = colorBlocks[i];
		stage->blockCount = colorBlockCounts[i];
		stage->colorIndex = activeColorIndices[i];
		stage->completionCount = 0;
		stage += 1;
	}

	// Store impulses
	stage->type = b2_stageStoreImpulses;
	stage->blocks = storeBlocks;
	stage->blockCount = storeBlockCount;
	stage->colorIndex = -1;
	stage->completionCount = 0;
	stage += 1;

	B2_ASSERT((int32_t)(stage - stages) == stageCount);

	B2_ASSERT(workerCount <= 16);
	b2WorkerContext workerContext[16];

	int32_t velIters = B2_MAX(1, stepContext->velocityIterations);

	stepContext->bodyMap = bodyMap;
	stepContext->solverBodies = solverBodies;
	stepContext->bodyCount = awakeBodyCount;

	b2SolverTaskContext context;
	context.world = world;
	context.graph = graph;
	context.awakeBodies = awakeBodies;
	context.solverBodies = solverBodies;
	context.bodyMap = bodyMap;
	context.stepContext = stepContext;
	context.constraints = constraints;
	context.activeColorCount = activeColorCount;
	context.velocityIterations = velIters;
	context.calmIterations = stepContext->positionIterations;
	context.workerCount = workerCount;
	context.stageCount = stageCount;
	context.stages = stages;
	context.timeStep = stepContext->dt;
	context.invTimeStep = stepContext->inv_dt;
	context.subStep = context.timeStep / velIters;
	context.invSubStep = velIters * stepContext->inv_dt;
	context.syncBits = 0;

	b2TracyCZoneEnd(prepare_stages);

	// Must use worker index because thread 0 can be assigned multiple tasks by enkiTS
	for (int32_t i = 0; i < workerCount; ++i)
	{
		workerContext[i].context = &context;
		workerContext[i].workerIndex = i;
		world->enqueueTaskFcn(b2SolverTask, 1, 1, workerContext + i, world->userTaskContext);
	}

	world->finishAllTasksFcn(world->userTaskContext);

	b2FreeStackItem(world->stackAllocator, storeBlocks);
	b2FreeStackItem(world->stackAllocator, graphBlocks);
	b2FreeStackItem(world->stackAllocator, bodyBlocks);
	b2FreeStackItem(world->stackAllocator, stages);
	b2FreeStackItem(world->stackAllocator, constraints);
	b2FreeStackItem(world->stackAllocator, bodyMap);
	b2FreeStackItem(world->stackAllocator, solverBodies);
	b2FreeStackItem(world->stackAllocator, awakeBodies);
}
