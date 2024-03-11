// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "solver.h"

#include "array.h"
#include "bitset.h"
#include "body.h"
#include "contact.h"
#include "contact_solver.h"
#include "core.h"
#include "ctz.h"
#include "joint.h"
#include "shape.h"
#include "stack_allocator.h"
#include "world.h"

#include "box2d/color.h"
#include "box2d/event_types.h"

// for mm_pause
#include "x86/sse2.h"

#include <limits.h>
#include <stdatomic.h>
#include <stdbool.h>
#include <string.h>

typedef struct b2WorkerContext
{
	b2StepContext* context;
	int workerIndex;
	void* userTask;
} b2WorkerContext;

// Integrate velocities and apply damping
static void b2IntegrateVelocitiesTask(int startIndex, int endIndex, b2StepContext* context)
{
	b2TracyCZoneNC(integrate_velocity, "IntVel", b2_colorDeepPink, true);

	b2BodyState* states = context->states;
	b2Body* bodies = context->bodies;

	b2Vec2 gravity = context->world->gravity;
	float h = context->h;
	float maxLinearSpeed = b2_maxTranslation * context->inv_dt;
	float maxAngularSpeed = b2_maxRotation * context->inv_dt;
	float maxLinearSpeedSquared = maxLinearSpeed * maxLinearSpeed;
	float maxAngularSpeedSquared = maxAngularSpeed * maxAngularSpeed;

	for (int i = startIndex; i < endIndex; ++i)
	{
		const b2Body* body = bodies + i;
		b2BodyState* state = states + i;

		b2Vec2 v = state->linearVelocity;
		float w = state->angularVelocity;

		// Apply forces, torque, gravity, and damping
		// Apply damping.
		// ODE: dv/dt + c * v = 0
		// Solution: v(t) = v0 * exp(-c * t)
		// Time step: v(t + dt) = v0 * exp(-c * (t + dt)) = v0 * exp(-c * t) * exp(-c * dt) = v * exp(-c * dt)
		// v2 = exp(-c * dt) * v1
		// Pade approximation:
		// v2 = v1 * 1 / (1 + c * dt)
		float linearDamping = 1.0f / (1.0f + h * body->linearDamping);
		float angularDamping = 1.0f / (1.0f + h * body->angularDamping);
		b2Vec2 linearVelocityDelta = b2MulSV(h * body->invMass, b2MulAdd(body->force, body->mass * body->gravityScale, gravity));
		float angularVelocityDelta = h * body->invI * body->torque;

		v = b2MulAdd(linearVelocityDelta, linearDamping, v);
		w = angularVelocityDelta + angularDamping * w;

		// Clamp to max linear speed
		if (b2Dot(v, v) > maxLinearSpeedSquared)
		{
			float ratio = maxLinearSpeed / b2Length(v);
			v = b2MulSV(ratio, v);
			state->flags = 1;
		}

		// Clamp to max angular speed
		if (w * w > maxAngularSpeedSquared)
		{
			float ratio = maxAngularSpeed / B2_ABS(w);
			w *= ratio;
			state->flags = 1;
		}

		state->linearVelocity = v;
		state->angularVelocity = w;
	}

	b2TracyCZoneEnd(integrate_velocity);
}

static void b2PrepareJointsTask(int startIndex, int endIndex, b2StepContext* context)
{
	b2TracyCZoneNC(prepare_joints, "PrepJoints", b2_colorOldLace, true);

	b2Joint** joints = context->joints;

	for (int i = startIndex; i < endIndex; ++i)
	{
		b2Joint* joint = joints[i];
		b2PrepareJoint(joint, context);
	}

	b2TracyCZoneEnd(prepare_joints);
}

static void b2WarmStartJointsTask(int startIndex, int endIndex, b2StepContext* context, int colorIndex)
{
	b2TracyCZoneNC(warm_joints, "WarmJoints", b2_colorGold, true);

	b2GraphColor* color = context->graph->colors + colorIndex;
	b2Joint* joints = color->joints.data;
	B2_ASSERT(0 <= startIndex && startIndex < color->joints.count);
	B2_ASSERT(startIndex <= endIndex && endIndex <= color->joints.count);

	for (int i = startIndex; i < endIndex; ++i)
	{
		b2Joint* joint = joints + i;
		b2WarmStartJoint(joint, context);
	}

	b2TracyCZoneEnd(warm_joints);
}

static void b2SolveJointsTask(int startIndex, int endIndex, b2StepContext* context, int colorIndex, bool useBias)
{
	b2TracyCZoneNC(solve_joints, "SolveJoints", b2_colorLemonChiffon, true);

	b2GraphColor* color = context->graph->colors + colorIndex;
	b2Joint* joints = color->joints.data;
	B2_ASSERT(0 <= startIndex && startIndex < color->joints.count);
	B2_ASSERT(startIndex <= endIndex && endIndex <= color->joints.count);

	for (int i = startIndex; i < endIndex; ++i)
	{
		b2Joint* joint = joints + i;
		b2SolveJoint(joint, context, useBias);
	}

	b2TracyCZoneEnd(solve_joints);
}

static void b2IntegratePositionsTask(int startIndex, int endIndex, b2StepContext* context)
{
	b2TracyCZoneNC(integrate_positions, "IntPos", b2_colorDarkSeaGreen, true);

	b2BodyState* states = context->states;
	float h = context->h;

	B2_ASSERT(startIndex <= endIndex);

	for (int i = startIndex; i < endIndex; ++i)
	{
		b2BodyState* state = states + i;
		state->deltaRotation = b2IntegrateRotation(state->deltaRotation, h * state->angularVelocity);
		state->deltaPosition = b2MulAdd(state->deltaPosition, h, state->linearVelocity);
	}

	b2TracyCZoneEnd(integrate_positions);
}

static void b2FinalizeBodiesTask(int startIndex, int endIndex, uint32_t threadIndex, void* taskContext)
{
	b2TracyCZoneNC(finalize_bodies, "FinalizeBodies", b2_colorViolet, true);

	b2StepContext* context = taskContext;
	b2World* world = context->world;
	bool enableSleep = world->enableSleep;
	b2BodyState* states = context->states;
	b2Body* bodies = context->bodies;
	float timeStep = context->dt;

	uint16_t worldIndex = world->worldIndex;
	b2BodyMoveEvent* moveEvents = world->bodyMoveEventArray;

	b2BitSet* shapeBitSet = &world->taskContextArray[threadIndex].shapeBitSet;
	b2BitSet* awakeIslandBitSet = &world->taskContextArray[threadIndex].awakeIslandBitSet;
	bool enableContinuous = world->enableContinuous;

	B2_ASSERT(startIndex <= endIndex);

	// Update sleep
	const float linTolSqr = b2_linearSleepTolerance * b2_linearSleepTolerance;
	const float angTolSqr = b2_angularSleepTolerance * b2_angularSleepTolerance;

	for (int32_t i = startIndex; i < endIndex; ++i)
	{
		const b2BodyState* state = states + i;
		b2Body* body = bodies + i;

		b2Vec2 v = state->linearVelocity;
		float w = state->angularVelocity;

		B2_ASSERT(b2Vec2_IsValid(v));
		B2_ASSERT(b2IsValid(w));

		body->isSpeedCapped = state->flags != 0;

		body->position = b2Add(body->position, state->deltaPosition);
		body->rotation = b2MulRot(state->deltaRotation, body->rotation);
		body->rotation = b2NormalizeRot(body->rotation);
		body->origin = b2Sub(body->position, b2RotateVector(body->rotation, body->localCenter));

		moveEvents[i].transform = b2MakeTransform(body);
		moveEvents[i].bodyId = (b2BodyId){body->bodyId + 1, worldIndex, body->revision};
		moveEvents[i].userData = body->userData;
		moveEvents[i].fellAsleep = false;

		// reset applied force and torque
		body->force = b2Vec2_zero;
		body->torque = 0.0f;

		body->isFast = false;

		if (enableSleep == false || body->enableSleep == false || w * w > angTolSqr || b2Dot(v, v) > linTolSqr)
		{
			body->sleepTime = 0.0f;

			const float saftetyFactor = 0.5f;
			if (enableContinuous && (b2Length(v) + B2_ABS(w) * body->maxExtent) * timeStep > saftetyFactor * body->minExtent)
			{
				// Store in fast array for the continuous collision stage
				// This is deterministic because the order of TOI sweeps doesn't matter
				if (body->isBullet)
				{
					int bulletIndex = atomic_fetch_add(&world->bulletBodyCount, 1);
					world->bulletBodies[bulletIndex] = i;
				}
				else
				{
					int fastIndex = atomic_fetch_add(&world->fastBodyCount, 1);
					world->fastBodies[fastIndex] = i;
				}

				body->isFast = true;
			}
			else
			{
				// Body is safe to advance
				body->position0 = body->position;
				body->rotation0 = body->rotation;
			}
		}
		else
		{
			// Body is safe to advance
			body->position0 = body->position;
			body->rotation0 = body->rotation;
			body->sleepTime += timeStep;
		}

		// Any single body in an island can keep it awake
		if (body->sleepTime < b2_timeToSleep)
		{
			B2_ASSERT(0 <= body->islandIndex && body->islandIndex < world->islandPool.capacity);
			b2SetBit(awakeIslandBitSet, body->islandIndex);
		}

		// Update shapes AABBs
		b2Transform transform = b2MakeTransform(body);
		bool isFast = body->isFast;
		int32_t shapeIndex = body->shapeList;
		while (shapeIndex != B2_NULL_INDEX)
		{
			b2Shape* shape = world->shapes + shapeIndex;

			B2_ASSERT(shape->isFast == false);

			if (isFast)
			{
				// The AABB is updated after continuous collision.
				// Add to moved shapes regardless of AABB changes.
				shape->isFast = true;

				// Bit-set to keep the move array sorted
				b2SetBit(shapeBitSet, shapeIndex);
			}
			else
			{
				b2AABB aabb = b2ComputeShapeAABB(shape, transform);
				aabb.lowerBound.x -= b2_speculativeDistance;
				aabb.lowerBound.y -= b2_speculativeDistance;
				aabb.upperBound.x += b2_speculativeDistance;
				aabb.upperBound.y += b2_speculativeDistance;
				shape->aabb = aabb;

				if (b2AABB_Contains(shape->fatAABB, aabb) == false)
				{
					b2AABB fatAABB;
					fatAABB.lowerBound.x = aabb.lowerBound.x - b2_aabbMargin;
					fatAABB.lowerBound.y = aabb.lowerBound.y - b2_aabbMargin;
					fatAABB.upperBound.x = aabb.upperBound.x + b2_aabbMargin;
					fatAABB.upperBound.y = aabb.upperBound.y + b2_aabbMargin;
					shape->fatAABB = fatAABB;

					// Bit-set to keep the move array sorted
					b2SetBit(shapeBitSet, shapeIndex);
				}
			}

			shapeIndex = shape->nextShapeIndex;
		}
	}

	b2TracyCZoneEnd(finalize_bodies);
}

/*
 typedef enum b2SolverStageType
{
	b2_stagePrepareJoints,
	b2_stagePrepareContacts,
	b2_stageIntegrateVelocities,
	b2_stageWarmStart,
	b2_stageSolve,
	b2_stageIntegratePositions,
	b2_stageRelax,
	b2_stageRestitution,
	b2_stageStoreImpulses
} b2SolverStageType;

typedef enum b2SolverBlockType
{
	b2_bodyBlock,
	b2_jointBlock,
	b2_contactBlock,
	b2_graphJointBlock,
	b2_graphContactBlock
} b2SolverBlockType;
*/

static void b2ExecuteBlock(b2SolverStage* stage, b2StepContext* context, b2SolverBlock* block)
{
	b2SolverStageType stageType = stage->type;
	b2SolverBlockType blockType = block->blockType;
	int32_t startIndex = block->startIndex;
	int32_t endIndex = startIndex + block->count;

	switch (stageType)
	{
		case b2_stagePrepareJoints:
			b2PrepareJointsTask(startIndex, endIndex, context);
			break;

		case b2_stagePrepareContacts:
			b2PrepareContactsTask(startIndex, endIndex, context);
			break;

		case b2_stageIntegrateVelocities:
			b2IntegrateVelocitiesTask(startIndex, endIndex, context);
			break;

		case b2_stageWarmStart:
			if (context->world->enableWarmStarting)
			{
				if (blockType == b2_graphContactBlock)
				{
					b2WarmStartContactsTask(startIndex, endIndex, context, stage->colorIndex);
				}
				else if (blockType == b2_graphJointBlock)
				{
					b2WarmStartJointsTask(startIndex, endIndex, context, stage->colorIndex);
				}
			}
			break;

		case b2_stageSolve:
			if (blockType == b2_graphContactBlock)
			{
				b2SolveContactsTask(startIndex, endIndex, context, stage->colorIndex, true);
			}
			else if (blockType == b2_graphJointBlock)
			{
				b2SolveJointsTask(startIndex, endIndex, context, stage->colorIndex, true);
			}
			break;

		case b2_stageIntegratePositions:
			b2IntegratePositionsTask(startIndex, endIndex, context);
			break;

		case b2_stageRelax:
			if (blockType == b2_graphContactBlock)
			{
				b2SolveContactsTask(startIndex, endIndex, context, stage->colorIndex, false);
			}
			else if (blockType == b2_graphJointBlock)
			{
				b2SolveJointsTask(startIndex, endIndex, context, stage->colorIndex, false);
			}
			break;

		case b2_stageRestitution:
			if (blockType == b2_graphContactBlock)
			{
				b2ApplyRestitutionTask(startIndex, endIndex, context, stage->colorIndex);
			}
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

static void b2ExecuteStage(b2SolverStage* stage, b2StepContext* context, int previousSyncIndex, int syncIndex,
						   int32_t workerIndex)
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

		b2ExecuteBlock(stage, context, blocks + blockIndex);

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

		b2ExecuteBlock(stage, context, blocks + blockIndex);
		completedCount += 1;
		blockIndex -= 1;
	}

	(void)atomic_fetch_add(&stage->completionCount, completedCount);
}

static void b2ExecuteMainStage(b2SolverStage* stage, b2StepContext* context, uint32_t syncBits)
{
	int32_t blockCount = stage->blockCount;
	if (blockCount == 0)
	{
		return;
	}

	if (blockCount == 1)
	{
		b2ExecuteBlock(stage, context, stage->blocks);
	}
	else
	{
		atomic_store(&context->syncBits, syncBits);

		int syncIndex = (syncBits >> 16) & 0xFFFF;
		B2_ASSERT(syncIndex > 0);
		int previousSyncIndex = syncIndex - 1;

		b2ExecuteStage(stage, context, previousSyncIndex, syncIndex, 0);

		// todo consider using the cycle counter as well
		while (atomic_load(&stage->completionCount) != blockCount)
		{
			simde_mm_pause();
		}

		atomic_store(&stage->completionCount, 0);
	}
}

// This should not use the thread index because thread 0 can be called twice by enkiTS.
static void b2SolverTask(int32_t threadIndexDontUse, void* taskContext)
{
	B2_MAYBE_UNUSED(threadIndexDontUse);

	b2WorkerContext* workerContext = taskContext;
	int32_t workerIndex = workerContext->workerIndex;
	b2StepContext* context = workerContext->context;
	int32_t activeColorCount = context->activeColorCount;
	b2SolverStage* stages = context->stages;

	if (workerIndex == 0)
	{
		// Main thread synchronizes the workers and does work itself.
		//
		// Stages are re-used by loops so that I don't need more stages for large iteration counts.
		// The sync indices grow monotonically for the body/graph/constraint groupings because they share solver blocks.
		// The stage index and sync indices are combined in to sync bits for atomic synchronization.
		// The workers need to compute the previous sync index for a given stage so that CAS works correctly. This
		// setup makes this easy to do.

		/*
		b2_stagePrepareJoints,
		b2_stagePrepareContacts,
		b2_stageIntegrateVelocities,
		b2_stageWarmStart,
		b2_stageSolve,
		b2_stageIntegratePositions,
		b2_stageRelax,
		b2_stageRestitution,
		b2_stageStoreImpulses
		*/

		int32_t bodySyncIndex = 1;
		int32_t stageIndex = 0;

		// This stage loops over all awake joints
		uint32_t jointSyncIndex = 1;
		uint32_t syncBits = (jointSyncIndex << 16) | stageIndex;
		B2_ASSERT(stages[stageIndex].type == b2_stagePrepareJoints);
		b2ExecuteMainStage(stages + stageIndex, context, syncBits);
		stageIndex += 1;
		jointSyncIndex += 1;

		// This stage loops over all contact constraints
		uint32_t contactSyncIndex = 1;
		syncBits = (contactSyncIndex << 16) | stageIndex;
		B2_ASSERT(stages[stageIndex].type == b2_stagePrepareContacts);
		b2ExecuteMainStage(stages + stageIndex, context, syncBits);
		stageIndex += 1;
		contactSyncIndex += 1;

		int32_t graphSyncIndex = 1;

		// Single-threaded overflow work. These constraints don't fit in the graph coloring.
		b2PrepareOverflowJoints(context);
		b2PrepareOverflowContacts(context);

		int32_t subStepCount = context->subStepCount;
		for (int32_t i = 0; i < subStepCount; ++i)
		{
			// stage index restarted each iteration
			// syncBits still increases monotonically because the upper bits increase each iteration
			int32_t iterStageIndex = stageIndex;

			// integrate velocities
			syncBits = (bodySyncIndex << 16) | iterStageIndex;
			B2_ASSERT(stages[iterStageIndex].type == b2_stageIntegrateVelocities);
			b2ExecuteMainStage(stages + iterStageIndex, context, syncBits);
			iterStageIndex += 1;
			bodySyncIndex += 1;

			// warm start constraints
			b2WarmStartOverflowJoints(context);
			b2WarmStartOverflowContacts(context);

			for (int32_t colorIndex = 0; colorIndex < activeColorCount; ++colorIndex)
			{
				syncBits = (graphSyncIndex << 16) | iterStageIndex;
				B2_ASSERT(stages[iterStageIndex].type == b2_stageWarmStart);
				b2ExecuteMainStage(stages + iterStageIndex, context, syncBits);
				iterStageIndex += 1;
			}
			graphSyncIndex += 1;

			// solve constraints
			bool useBias = true;
			b2SolveOverflowJoints(context, useBias);
			b2SolveOverflowContacts(context, useBias);

			for (int32_t colorIndex = 0; colorIndex < activeColorCount; ++colorIndex)
			{
				syncBits = (graphSyncIndex << 16) | iterStageIndex;
				B2_ASSERT(stages[iterStageIndex].type == b2_stageSolve);
				b2ExecuteMainStage(stages + iterStageIndex, context, syncBits);
				iterStageIndex += 1;
			}
			graphSyncIndex += 1;

			// integrate positions
			B2_ASSERT(stages[iterStageIndex].type == b2_stageIntegratePositions);
			syncBits = (bodySyncIndex << 16) | iterStageIndex;
			b2ExecuteMainStage(stages + iterStageIndex, context, syncBits);
			iterStageIndex += 1;
			bodySyncIndex += 1;

			// relax constraints
			useBias = false;
			b2SolveOverflowJoints(context, useBias);
			b2SolveOverflowContacts(context, useBias);

			for (int32_t colorIndex = 0; colorIndex < activeColorCount; ++colorIndex)
			{
				syncBits = (graphSyncIndex << 16) | iterStageIndex;
				B2_ASSERT(stages[iterStageIndex].type == b2_stageRelax);
				b2ExecuteMainStage(stages + iterStageIndex, context, syncBits);
				iterStageIndex += 1;
			}
			graphSyncIndex += 1;
		}

		// advance the stage according to the sub-stepping tasks just completed
		// integrate velocities / warm start / solve / integrate positions / relax
		stageIndex += 1 + activeColorCount + activeColorCount + 1 + activeColorCount;

		// Restitution
		{
			b2ApplyOverflowRestitution(context);

			int32_t iterStageIndex = stageIndex;
			for (int32_t colorIndex = 0; colorIndex < activeColorCount; ++colorIndex)
			{
				syncBits = (graphSyncIndex << 16) | iterStageIndex;
				B2_ASSERT(stages[iterStageIndex].type == b2_stageRestitution);
				b2ExecuteMainStage(stages + iterStageIndex, context, syncBits);
				iterStageIndex += 1;
			}
			// graphSyncIndex += 1;
			stageIndex += activeColorCount;
		}

		b2StoreOverflowImpulses(context);

		syncBits = (contactSyncIndex << 16) | stageIndex;
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
		// todo consider using the cycle counter as well
		uint32_t syncBits = atomic_load(&context->syncBits);
		while (syncBits == lastSyncBits)
		{
			simde_mm_pause();
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

// Returns false if there is nothing awake
static bool b2SolveConstraintGraph(b2World* world, b2StepContext* context)
{
	b2TracyCZoneNC(prepare_stages, "Prepare Stages", b2_colorDarkOrange, true);
	b2Timer timer = b2CreateTimer();

	b2ConstraintGraph* graph = &world->graph;
	b2GraphColor* colors = graph->colors;

	// Count awake bodies
	int32_t awakeIslandCount = b2Array(world->awakeIslandArray).count;
	int32_t awakeBodyCount = 0;
	for (int32_t i = 0; i < awakeIslandCount; ++i)
	{
		int32_t islandIndex = world->awakeIslandArray[i];
		b2Island* island = world->islands + islandIndex;
		awakeBodyCount += island->bodyCount;
	}

	// Prepare world to receive fast bodies from body finalization
	world->fastBodyCount = 0;
	world->fastBodies = b2AllocateStackItem(world->stackAllocator, awakeBodyCount * sizeof(int32_t), "fast bodies");

	// Prepare world to receive bullet bodies from body finalization
	world->bulletBodyCount = 0;
	world->bulletBodies = b2AllocateStackItem(world->stackAllocator, awakeBodyCount * sizeof(int32_t), "bullet bodies");

	if (awakeBodyCount == 0)
	{
		for (int32_t i = 0; i < b2_graphColorCount; ++i)
		{
			graph->occupancy[i] = b2Array(colors[i].contactArray).count;
		}
		graph->occupancy[b2_overflowIndex] = b2Array(graph->overflow.contactArray).count;

		return false;
	}

	// Gather constraint graph contact arrays for efficient parallel-for processing
	int contactCount = 0;
	int subsetCount = 0;
	for (int i = 0; i < b2_graphColorCount; ++i)
	{
		if (colors[i].contacts.count > 0)
		{
			context->contactSubsets[subsetCount].contacts = colors[i].contacts.data;
			context->contactSubsets[subsetCount].count = colors[i].contacts.count;
			context->contactSubsets[subsetCount].colorIndex = i;
			subsetCount += 1;
			contactCount += colors[i].contacts.count;
		}
	}

	context->contactSubsetCount = subsetCount;

	// Reserve space for solver body arrays
	b2Body* bodies = world->bodies;
	b2BodyState* bodyStates = b2AllocateStackItem(world->stackAllocator, awakeBodyCount * sizeof(b2BodyState), "body states");
	b2BodyParam* bodyParams = b2AllocateStackItem(world->stackAllocator, awakeBodyCount * sizeof(b2BodyParam), "body params");

	// Deal with void**
	{
		void* bodyMoveEventArray = world->bodyMoveEventArray;
		b2Array_Resize(&bodyMoveEventArray, sizeof(b2BodyMoveEvent), awakeBodyCount);
		world->bodyMoveEventArray = bodyMoveEventArray;
	}

	// Map from solver body to body
	int32_t* solverToBodyMap = b2AllocateStackItem(world->stackAllocator, awakeBodyCount * sizeof(int32_t), "solver body map");

	// Build array of awake bodies and also search for an awake island to split
	b2Vec2 gravity = world->gravity;
	float h = context->h;
	int32_t splitIslandIndex = B2_NULL_INDEX;
	int32_t maxRemovedContacts = 0;
	int32_t splitIslandBodyCount = 0;
	int32_t index = 0;
	for (int32_t i = 0; i < awakeIslandCount; ++i)
	{
		int32_t islandIndex = world->awakeIslandArray[i];
		b2Island* island = world->islands + islandIndex;

		if (island->constraintRemoveCount > maxRemovedContacts)
		{
			maxRemovedContacts = island->constraintRemoveCount;
			splitIslandIndex = islandIndex;
			splitIslandBodyCount = island->bodyCount;
		}

		int32_t bodyIndex = island->headBody;
		while (bodyIndex != B2_NULL_INDEX)
		{
			b2Body* body = bodies + bodyIndex;
			B2_ASSERT(b2IsValidObject(&body->object));
			B2_ASSERT(body->object.index == bodyIndex);

			body->solverIndex = index;
			solverToBodyMap[index] = bodyIndex;

			b2BodyState* state = bodyStates + index;
			state->linearVelocity = body->linearVelocity;
			state->angularVelocity = body->angularVelocity;
			state->flags = 0;
			state->deltaPosition = b2Vec2_zero;
			state->deltaRotation = b2Rot_identity;

			b2BodyParam* param = bodyParams + index;
			param->invMass = body->invMass;
			param->invI = body->invI;

			// Apply damping.
			// ODE: dv/dt + c * v = 0
			// Solution: v(t) = v0 * exp(-c * t)
			// Time step: v(t + dt) = v0 * exp(-c * (t + dt)) = v0 * exp(-c * t) * exp(-c * dt) = v * exp(-c * dt)
			// v2 = exp(-c * dt) * v1
			// Pade approximation:
			// v2 = v1 * 1 / (1 + c * dt)
			param->angularDamping = 1.0f / (1.0f + h * body->angularDamping);
			param->linearDamping = 1.0f / (1.0f + h * body->linearDamping);

			param->linearVelocityDelta =
				b2MulSV(body->invMass * h, b2MulAdd(body->force, body->mass * body->gravityScale, gravity));
			param->angularVelocityDelta = h * body->invI * body->torque;

			bodyIndex = body->islandNext;
			index += 1;
		}
	}
	B2_ASSERT(index == awakeBodyCount);

	// Each worker receives at most M blocks of work. The workers may receive less than there is not sufficient work.
	// Each block of work has a minimum number of elements (block size). This in turn may limit number of blocks.
	// If there are many elements then the block size is increased so there are still at most M blocks of work per worker.
	// M is a tunable number that has two goals:
	// 1. keep M small to reduce overhead
	// 2. keep M large enough for other workers to be able to steal work
	// The block size is a power of two to make math efficient.

	int32_t workerCount = world->workerCount;
	const int32_t blocksPerWorker = 4;
	const int32_t maxBlockCount = blocksPerWorker * workerCount;

	// Configure blocks for tasks that parallel-for bodies
	int32_t bodyBlockSize = 1 << 5;
	int32_t bodyBlockCount;
	if (awakeBodyCount > bodyBlockSize * maxBlockCount)
	{
		// Too many blocks, increase block size
		bodyBlockSize = awakeBodyCount / maxBlockCount;
		bodyBlockCount = maxBlockCount;
	}
	else
	{
		bodyBlockCount = ((awakeBodyCount - 1) >> 5) + 1;
	}

	// Configure blocks for tasks parallel-for each active graph color
	// The blocks are a mix of SIMD contact blocks and joint blocks
	int32_t activeColorIndices[b2_graphColorCount];

	int32_t colorContactCounts[b2_graphColorCount];
	int32_t colorContactBlockSizes[b2_graphColorCount];
	int32_t colorContactBlockCounts[b2_graphColorCount];

	int32_t colorJointCounts[b2_graphColorCount];
	int32_t colorJointBlockSizes[b2_graphColorCount];
	int32_t colorJointBlockCounts[b2_graphColorCount];

	int32_t activeColorCount = 0;
	int32_t graphBlockCount = 0;
	int32_t contactCount = 0;
	int32_t jointCount = 0;

	int32_t c = 0;
	for (int32_t i = 0; i < b2_graphColorCount; ++i)
	{
		int32_t colorContactCount = b2Array(colors[i].contactArray).count;
		int32_t colorJointCount = b2Array(colors[i].jointArray).count;

		graph->occupancy[i] = colorContactCount + colorJointCount;

		if (colorContactCount + colorJointCount > 0)
		{
			activeColorIndices[c] = i;

			// 8-way SIMD
			int32_t colorContactCountSIMD = colorContactCount > 0 ? ((colorContactCount - 1) >> 3) + 1 : 0;

			colorContactCounts[c] = colorContactCountSIMD;
			colorContactBlockSizes[c] = blocksPerWorker;
			if (colorContactCountSIMD > blocksPerWorker * maxBlockCount)
			{
				// Too many contact blocks
				colorContactBlockSizes[c] = colorContactCountSIMD / maxBlockCount;
				colorContactBlockCounts[c] = maxBlockCount;
			}
			else if (colorContactCountSIMD > 0)
			{
				colorContactBlockCounts[c] = ((colorContactCountSIMD - 1) >> 2) + 1;
			}
			else
			{
				colorContactBlockCounts[c] = 0;
			}

			colorJointCounts[c] = colorJointCount;
			colorJointBlockSizes[c] = blocksPerWorker;
			if (colorJointCount > blocksPerWorker * maxBlockCount)
			{
				// Too many joint blocks
				colorJointBlockSizes[c] = colorJointCount / maxBlockCount;
				colorJointBlockCounts[c] = maxBlockCount;
			}
			else if (colorJointCount > 0)
			{
				colorJointBlockCounts[c] = ((colorJointCount - 1) >> 2) + 1;
			}
			else
			{
				colorJointBlockCounts[c] = 0;
			}

			graphBlockCount += colorContactBlockCounts[c] + colorJointBlockCounts[c];
			contactCount += colorContactCountSIMD;
			jointCount += colorJointCount;
			c += 1;
		}
	}
	activeColorCount = c;

	b2ContactConstraintSIMD* contactConstraints =
		b2AllocateStackItem(world->stackAllocator, contactCount * sizeof(b2ContactConstraintSIMD), "contact constraint");

	int32_t* contactIndices = b2AllocateStackItem(world->stackAllocator, 8 * contactCount * sizeof(int32_t), "contact indices");
	int32_t* jointIndices = b2AllocateStackItem(world->stackAllocator, jointCount * sizeof(int32_t), "joint indices");

	int32_t overflowContactCount = b2Array(graph->overflow.contactArray).count;
	graph->occupancy[b2_overflowIndex] = overflowContactCount;
	graph->overflow.contactConstraints = b2AllocateStackItem(
		world->stackAllocator, overflowContactCount * sizeof(b2ContactConstraint), "overflow contact constraint");

	// Distribute transient constraints to each graph color
	{
		int32_t base = 0;
		int32_t jointBaseIndex = 0;
		for (int32_t i = 0; i < activeColorCount; ++i)
		{
			int32_t j = activeColorIndices[i];
			b2GraphColor* color = colors + j;

			int32_t colorJointCount = b2Array(color->jointArray).count;
			memcpy(jointIndices + jointBaseIndex, color->jointArray, colorJointCount * sizeof(int32_t));
			jointBaseIndex += colorJointCount;

			int32_t colorContactCount = b2Array(color->contactArray).count;

			if (colorContactCount == 0)
			{
				color->contactConstraints = NULL;
				continue;
			}

			color->contactConstraints = contactConstraints + base;

			for (int32_t k = 0; k < colorContactCount; ++k)
			{
				contactIndices[8 * base + k] = color->contactArray[k];
			}

			// remainder
			int32_t colorContactCountSIMD = ((colorContactCount - 1) >> 3) + 1;
			for (int32_t k = colorContactCount; k < 8 * colorContactCountSIMD; ++k)
			{
				contactIndices[8 * base + k] = B2_NULL_INDEX;
			}

			base += colorContactCountSIMD;
		}
	}

	// Define work blocks for preparing contacts and storing contact impulses
	int32_t contactBlockSize = blocksPerWorker;
	int32_t contactBlockCount = contactCount > 0 ? ((contactCount - 1) >> 2) + 1 : 0;
	if (contactCount > contactBlockSize * maxBlockCount)
	{
		// Too many blocks, increase block size
		contactBlockSize = contactCount / maxBlockCount;
		contactBlockCount = maxBlockCount;
	}

	// Define work blocks for preparing joints
	int32_t jointBlockSize = blocksPerWorker;
	int32_t jointBlockCount = jointCount > 0 ? ((jointCount - 1) >> 2) + 1 : 0;
	if (jointCount > jointBlockSize * maxBlockCount)
	{
		// Too many blocks, increase block size
		jointBlockSize = jointCount / maxBlockCount;
		jointBlockCount = maxBlockCount;
	}

	/*
	b2_stagePrepareJoints,
	b2_stagePrepareContacts,
	b2_stageIntegrateVelocities,
	b2_stageWarmStart,
	b2_stageSolve,
	b2_stageIntegratePositions,
	b2_stageRelax,
	b2_stageRestitution,
	b2_stageStoreImpulses
	*/

	int32_t stageCount = 0;

	// b2_stagePrepareJoints
	stageCount += 1;
	// b2_stagePrepareContacts
	stageCount += 1;
	// b2_stageIntegrateVelocities
	stageCount += 1;
	// b2_stageWarmStart
	stageCount += activeColorCount;
	// b2_stageSolve
	stageCount += activeColorCount;
	// b2_stageIntegratePositions
	stageCount += 1;
	// b2_stageRelax
	stageCount += activeColorCount;
	// b2_stageRestitution
	stageCount += activeColorCount;
	// b2_stageStoreImpulses
	stageCount += 1;

	b2SolverStage* stages = b2AllocateStackItem(world->stackAllocator, stageCount * sizeof(b2SolverStage), "stages");
	b2SolverBlock* bodyBlocks = b2AllocateStackItem(world->stackAllocator, bodyBlockCount * sizeof(b2SolverBlock), "body blocks");
	b2SolverBlock* contactBlocks =
		b2AllocateStackItem(world->stackAllocator, contactBlockCount * sizeof(b2SolverBlock), "contact blocks");
	b2SolverBlock* jointBlocks =
		b2AllocateStackItem(world->stackAllocator, jointBlockCount * sizeof(b2SolverBlock), "joint blocks");
	b2SolverBlock* graphBlocks =
		b2AllocateStackItem(world->stackAllocator, graphBlockCount * sizeof(b2SolverBlock), "graph blocks");

	// Split an awake island. This modifies:
	// - stack allocator
	// - awake island array
	// - island pool
	// - island indices on bodies, contacts, and joints
	// I'm squeezing this task in here because it may be expensive and this is a safe place to put it.
	// Note: cannot split islands in parallel with FinalizeBodies
	world->splitIslandIndex = splitIslandIndex;
	void* splitIslandTask = NULL;
	if (splitIslandIndex != B2_NULL_INDEX)
	{
		splitIslandTask = world->enqueueTaskFcn(&b2SplitIslandTask, 1, 1, world, world->userTaskContext);
		world->taskCount += 1;
		world->activeTaskCount += splitIslandTask == NULL ? 0 : 1;
	}

	// Prepare body work blocks
	for (int32_t i = 0; i < bodyBlockCount; ++i)
	{
		b2SolverBlock* block = bodyBlocks + i;
		block->startIndex = i * bodyBlockSize;
		block->count = (int16_t)bodyBlockSize;
		block->blockType = b2_bodyBlock;
		block->syncIndex = 0;
	}
	bodyBlocks[bodyBlockCount - 1].count = (int16_t)(awakeBodyCount - (bodyBlockCount - 1) * bodyBlockSize);

	// Prepare joint work blocks
	for (int32_t i = 0; i < jointBlockCount; ++i)
	{
		b2SolverBlock* block = jointBlocks + i;
		block->startIndex = i * jointBlockSize;
		block->count = (int16_t)jointBlockSize;
		block->blockType = b2_jointBlock;
		block->syncIndex = 0;
	}

	if (jointBlockCount > 0)
	{
		jointBlocks[jointBlockCount - 1].count = (int16_t)(jointCount - (jointBlockCount - 1) * jointBlockSize);
	}

	// Prepare contact work blocks
	for (int32_t i = 0; i < contactBlockCount; ++i)
	{
		b2SolverBlock* block = contactBlocks + i;
		block->startIndex = i * contactBlockSize;
		block->count = (int16_t)contactBlockSize;
		block->blockType = b2_contactBlock;
		block->syncIndex = 0;
	}

	if (contactBlockCount > 0)
	{
		contactBlocks[contactBlockCount - 1].count = (int16_t)(contactCount - (contactBlockCount - 1) * contactBlockSize);
	}

	// Prepare graph work blocks
	b2SolverBlock* graphColorBlocks[b2_graphColorCount];
	b2SolverBlock* baseGraphBlock = graphBlocks;

	for (int32_t i = 0; i < activeColorCount; ++i)
	{
		graphColorBlocks[i] = baseGraphBlock;

		int32_t colorJointBlockCount = colorJointBlockCounts[i];
		int32_t colorJointBlockSize = colorJointBlockSizes[i];
		for (int32_t j = 0; j < colorJointBlockCount; ++j)
		{
			b2SolverBlock* block = baseGraphBlock + j;
			block->startIndex = j * colorJointBlockSize;
			block->count = (int16_t)colorJointBlockSize;
			block->blockType = b2_graphJointBlock;
			block->syncIndex = 0;
		}

		if (colorJointBlockCount > 0)
		{
			baseGraphBlock[colorJointBlockCount - 1].count =
				(int16_t)(colorJointCounts[i] - (colorJointBlockCount - 1) * colorJointBlockSize);
			baseGraphBlock += colorJointBlockCount;
		}

		int32_t colorContactBlockCount = colorContactBlockCounts[i];
		int32_t colorContactBlockSize = colorContactBlockSizes[i];
		for (int32_t j = 0; j < colorContactBlockCount; ++j)
		{
			b2SolverBlock* block = baseGraphBlock + j;
			block->startIndex = j * colorContactBlockSize;
			block->count = (int16_t)colorContactBlockSize;
			block->blockType = b2_graphContactBlock;
			block->syncIndex = 0;
		}

		if (colorContactBlockCount > 0)
		{
			baseGraphBlock[colorContactBlockCount - 1].count =
				(int16_t)(colorContactCounts[i] - (colorContactBlockCount - 1) * colorContactBlockSize);
			baseGraphBlock += colorContactBlockCount;
		}
	}

	B2_ASSERT((ptrdiff_t)(baseGraphBlock - graphBlocks) == graphBlockCount);

	b2SolverStage* stage = stages;

	// Prepare joints
	stage->type = b2_stagePrepareJoints;
	stage->blocks = jointBlocks;
	stage->blockCount = jointBlockCount;
	stage->colorIndex = -1;
	stage->completionCount = 0;
	stage += 1;

	// Prepare contacts
	stage->type = b2_stagePrepareContacts;
	stage->blocks = contactBlocks;
	stage->blockCount = contactBlockCount;
	stage->colorIndex = -1;
	stage->completionCount = 0;
	stage += 1;

	// Integrate velocities
	stage->type = b2_stageIntegrateVelocities;
	stage->blocks = bodyBlocks;
	stage->blockCount = bodyBlockCount;
	stage->colorIndex = -1;
	stage->completionCount = 0;
	stage += 1;

	// Warm start
	for (int32_t i = 0; i < activeColorCount; ++i)
	{
		stage->type = b2_stageWarmStart;
		stage->blocks = graphColorBlocks[i];
		stage->blockCount = colorJointBlockCounts[i] + colorContactBlockCounts[i];
		stage->colorIndex = activeColorIndices[i];
		stage->completionCount = 0;
		stage += 1;
	}

	// Solve graph
	for (int32_t i = 0; i < activeColorCount; ++i)
	{
		stage->type = b2_stageSolve;
		stage->blocks = graphColorBlocks[i];
		stage->blockCount = colorJointBlockCounts[i] + colorContactBlockCounts[i];
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

	// Relax constraints
	for (int32_t i = 0; i < activeColorCount; ++i)
	{
		stage->type = b2_stageRelax;
		stage->blocks = graphColorBlocks[i];
		stage->blockCount = colorJointBlockCounts[i] + colorContactBlockCounts[i];
		stage->colorIndex = activeColorIndices[i];
		stage->completionCount = 0;
		stage += 1;
	}

	// Restitution
	// Note: joint blocks mixed in, could have joint limit restitution
	for (int32_t i = 0; i < activeColorCount; ++i)
	{
		stage->type = b2_stageRestitution;
		stage->blocks = graphColorBlocks[i];
		stage->blockCount = colorJointBlockCounts[i] + colorContactBlockCounts[i];
		stage->colorIndex = activeColorIndices[i];
		stage->completionCount = 0;
		stage += 1;
	}

	// Store impulses
	stage->type = b2_stageStoreImpulses;
	stage->blocks = contactBlocks;
	stage->blockCount = contactBlockCount;
	stage->colorIndex = -1;
	stage->completionCount = 0;
	stage += 1;

	B2_ASSERT((int32_t)(stage - stages) == stageCount);

	B2_ASSERT(workerCount <= b2_maxWorkers);
	b2WorkerContext workerContext[b2_maxWorkers];

	context->world = world;
	context->graph = graph;
	context->contactConstraints = contactConstraints;
	context->joints = joints;
	context->contacts = contacts;
	context->activeColorCount = activeColorCount;
	context->workerCount = workerCount;
	context->stageCount = stageCount;
	context->stages = stages;
	context->syncBits = 0;

	world->profile.prepareTasks = b2GetMillisecondsAndReset(&timer);

	b2TracyCZoneEnd(prepare_stages);

	// Must use worker index because thread 0 can be assigned multiple tasks by enkiTS
	for (int32_t i = 0; i < workerCount; ++i)
	{
		workerContext[i].context = context;
		workerContext[i].workerIndex = i;
		workerContext[i].userTask = world->addPinnedTaskFcn(b2SolverTask, i, workerContext + i, world->userTaskContext);
		world->taskCount += 1;
		world->activeTaskCount += workerContext[i].userTask == NULL ? 0 : 1;
	}

	// Finish split
	if (splitIslandTask != NULL)
	{
		world->finishTaskFcn(splitIslandTask, world->userTaskContext);
		world->activeTaskCount -= 1;
	}

	world->splitIslandIndex = B2_NULL_INDEX;

	// Finish solve
	for (int32_t i = 0; i < workerCount; ++i)
	{
		if (workerContext[i].userTask != NULL)
		{
			world->finishPinnedTaskFcn(workerContext[i].userTask, world->userTaskContext);
			world->activeTaskCount -= 1;
		}
	}

	// Prepare contact, shape, and island bit sets used in body finalization.
	int32_t contactCapacity = world->contactPool.capacity;
	int32_t shapeCapacity = world->shapePool.capacity;
	int32_t islandCapacity = world->islandPool.capacity + splitIslandBodyCount;
	for (uint32_t i = 0; i < world->workerCount; ++i)
	{
		b2SetBitCountAndClear(&world->taskContextArray[i].awakeContactBitSet, contactCapacity);
		b2SetBitCountAndClear(&world->taskContextArray[i].shapeBitSet, shapeCapacity);
		b2SetBitCountAndClear(&world->taskContextArray[i].awakeIslandBitSet, islandCapacity);
	}

	// Finalize bodies. Must happen after the constraint solver and after island splitting.
	void* finalizeBodiesTask = world->enqueueTaskFcn(b2FinalizeBodiesTask, awakeBodyCount, 64, context, world->userTaskContext);
	world->taskCount += 1;
	if (finalizeBodiesTask != NULL)
	{
		world->finishTaskFcn(finalizeBodiesTask, world->userTaskContext);
	}

	b2FreeStackItem(world->stackAllocator, graphBlocks);
	b2FreeStackItem(world->stackAllocator, jointBlocks);
	b2FreeStackItem(world->stackAllocator, contactBlocks);
	b2FreeStackItem(world->stackAllocator, bodyBlocks);
	b2FreeStackItem(world->stackAllocator, stages);
	b2FreeStackItem(world->stackAllocator, graph->overflow.contactConstraints);
	b2FreeStackItem(world->stackAllocator, jointIndices);
	b2FreeStackItem(world->stackAllocator, contactIndices);
	b2FreeStackItem(world->stackAllocator, contactConstraints);
	b2FreeStackItem(world->stackAllocator, solverToBodyMap);
	b2FreeStackItem(world->stackAllocator, bodyParams);
	b2FreeStackItem(world->stackAllocator, bodyStates);

	world->profile.solverTasks = b2GetMillisecondsAndReset(&timer);

	b2TracyCZoneNC(awake_islands, "Awake Islands", b2_colorGainsboro, true);

	// Prepare awake contact bit set so that putting islands to sleep can clear bits
	// for the associated contacts.
	b2BitSet* awakeContactBitSet = &world->taskContextArray[0].awakeContactBitSet;
	for (uint32_t i = 1; i < world->workerCount; ++i)
	{
		b2InPlaceUnion(awakeContactBitSet, &world->taskContextArray[i].awakeContactBitSet);
	}

	{
		b2BitSet* awakeIslandBitSet = &world->taskContextArray[0].awakeIslandBitSet;
		for (uint32_t i = 1; i < world->workerCount; ++i)
		{
			b2InPlaceUnion(awakeIslandBitSet, &world->taskContextArray[i].awakeIslandBitSet);
		}

		b2Contact* contacts = world->contacts;
		b2Joint* joints = world->joints;
		b2Island* islands = world->islands;

		int32_t count = b2Array(world->awakeIslandArray).count;
		for (int32_t i = 0; i < count; ++i)
		{
			int32_t islandIndex = world->awakeIslandArray[i];
			if (b2GetBit(awakeIslandBitSet, islandIndex) == true)
			{
				continue;
			}

			// There are no bodies keeping the island awake, so put the island to sleep
			b2Island* island = islands + islandIndex;
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

				// Report to the user that this body fell asleep
				world->bodyMoveEventArray[body->solverIndex].fellAsleep = true;

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
				b2RemoveJointFromGraph(world, joint);
				jointIndex = joint->islandNext;
			}
		}

		// Use bitSet to put islands to sleep.
		uint64_t word;
		uint32_t wordCount = awakeIslandBitSet->blockCount;
		uint64_t* bits = awakeIslandBitSet->bits;
		for (uint32_t k = 0; k < wordCount; ++k)
		{
			word = bits[k];
			while (word != 0)
			{
				uint32_t ctz = b2CTZ64(word);
				uint32_t islandIndex = 64 * k + ctz;

				B2_ASSERT(b2IsValidObject(&islands[islandIndex].object));

				// An island with outstanding contact removals, it cannot sleep
				b2Island* island = islands + islandIndex;
				if (island->constraintRemoveCount == 0)
				{
					b2SleepIsland(island);
				}

				// Clear the smallest set bit
				word = word & (word - 1);
			}
		}
	}

#if B2_VALIDATE
	for (int32_t i = 0; i < world->islandPool.capacity; ++i)
	{
		b2Island* island = world->islands + i;
		if (b2IsValidObject(&island->object) == false)
		{
			continue;
		}

		b2ValidateIsland(island, true);
	}
#endif

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
		uint32_t wordCount = awakeContactBitSet->blockCount;
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

	world->profile.awakeUpdate = b2GetMilliseconds(&timer);

	b2TracyCZoneEnd(awake_contacts);

	return true;
}

struct b2ContinuousContext
{
	b2World* world;
	b2Body* fastBody;
	b2Shape* fastShape;
	b2Vec2 centroid1, centroid2;
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
	B2_ASSERT(body->type == b2_staticBody || continuousContext->fastBody->isBullet);

	// Skip bullets
	if (body->isBullet)
	{
		return true;
	}

	// Skip filtered bodies
	canCollide = b2ShouldBodiesCollide(world, continuousContext->fastBody, body);
	if (canCollide == false)
	{
		return true;
	}

	// Prevent pausing on smooth segment junctions
	if (shape->type == b2_smoothSegmentShape)
	{
		b2Transform transform = b2MakeTransform(body);
		b2Vec2 p1 = b2TransformPoint(transform, shape->smoothSegment.segment.point1);
		b2Vec2 p2 = b2TransformPoint(transform, shape->smoothSegment.segment.point2);
		b2Vec2 e = b2Sub(p2, p1);
		b2Vec2 c1 = continuousContext->centroid1;
		b2Vec2 c2 = continuousContext->centroid2;
		float offset1 = b2Cross(b2Sub(c1, p1), e);
		float offset2 = b2Cross(b2Sub(c2, p1), e);

		if (offset1 < 0.0f || offset2 > 0.0f)
		{
			// Started behind or finished in front
			return true;
		}
	}

	b2TOIInput input;
	input.proxyA = b2MakeShapeDistanceProxy(shape);
	input.proxyB = b2MakeShapeDistanceProxy(fastShape);
	input.sweepA = b2MakeSweep(body);
	input.sweepB = continuousContext->sweep;
	input.tMax = continuousContext->fraction;

	b2TOIOutput output = b2TimeOfImpact(&input);
	if (0.0f < output.t && output.t < continuousContext->fraction)
	{
		continuousContext->fraction = output.t;
	}
	else if (0.0f == output.t)
	{
		// fallback to TOI of a small circle around the fast shape centroid
		b2Vec2 centroid = b2GetShapeCentroid(fastShape);
		input.proxyB = b2MakeProxy(&centroid, 1, b2_speculativeDistance);
		output = b2TimeOfImpact(&input);
		if (0.0f < output.t && output.t < continuousContext->fraction)
		{
			continuousContext->fraction = output.t;
		}
	}

	return true;
}

// Continuous collision of dynamic versus static
static void b2SolveContinuous(b2World* world, int32_t bodyIndex)
{
	b2Body* fastBody = world->bodies + bodyIndex;
	B2_ASSERT(b2IsValidObject(&fastBody->object));
	B2_ASSERT(fastBody->type == b2_dynamicBody && fastBody->isFast);

	b2Shape* shapes = world->shapes;

	b2Sweep sweep = b2MakeSweep(fastBody);

	b2Transform xf1;
	xf1.q = sweep.q1;
	xf1.p = b2Sub(sweep.c1, b2RotateVector(sweep.q1, sweep.localCenter));

	b2Transform xf2;
	xf2.q = sweep.q2;
	xf2.p = b2Sub(sweep.c2, b2RotateVector(sweep.q2, sweep.localCenter));

	b2DynamicTree* staticTree = world->broadPhase.trees + b2_staticBody;
	b2DynamicTree* kinematicTree = world->broadPhase.trees + b2_kinematicBody;
	b2DynamicTree* dynamicTree = world->broadPhase.trees + b2_dynamicBody;

	struct b2ContinuousContext context;
	context.world = world;
	context.sweep = sweep;
	context.fastBody = fastBody;
	context.fraction = 1.0f;

	bool isBullet = fastBody->isBullet;

	int32_t shapeIndex = fastBody->shapeList;
	while (shapeIndex != B2_NULL_INDEX)
	{
		b2Shape* fastShape = shapes + shapeIndex;
		B2_ASSERT(fastShape->isFast == true);

		// Clear flag (keep set on body)
		fastShape->isFast = false;

		context.fastShape = fastShape;
		context.centroid1 = b2TransformPoint(xf1, fastShape->localCentroid);
		context.centroid2 = b2TransformPoint(xf2, fastShape->localCentroid);

		b2AABB box1 = fastShape->aabb;
		b2AABB box2 = b2ComputeShapeAABB(fastShape, xf2);
		b2AABB box = b2AABB_Union(box1, box2);

		// Store this for later
		fastShape->aabb = box2;

		b2DynamicTree_Query(staticTree, box, b2ContinuousQueryCallback, &context);

		if (isBullet)
		{
			b2DynamicTree_Query(kinematicTree, box, b2ContinuousQueryCallback, &context);
			b2DynamicTree_Query(dynamicTree, box, b2ContinuousQueryCallback, &context);
		}

		shapeIndex = fastShape->nextShapeIndex;
	}

	if (context.fraction < 1.0f)
	{
		// Handle time of impact event
		b2Rot q = b2NLerp(sweep.q1, sweep.q2, context.fraction);
		b2Vec2 c = b2Lerp(sweep.c1, sweep.c2, context.fraction);
		b2Vec2 origin = b2Sub(c, b2RotateVector(q, sweep.localCenter));

		// Advance body
		fastBody->rotation0 = q;
		fastBody->rotation = q;
		fastBody->position0 = c;
		fastBody->position = c;

		b2Transform transform = {origin, q};
		fastBody->origin = origin;

		// Prepare AABBs for broad-phase
		shapeIndex = fastBody->shapeList;
		while (shapeIndex != B2_NULL_INDEX)
		{
			b2Shape* shape = shapes + shapeIndex;

			// Must recompute aabb at the interpolated transform
			b2AABB aabb = b2ComputeShapeAABB(shape, transform);
			aabb.lowerBound.x -= b2_speculativeDistance;
			aabb.lowerBound.y -= b2_speculativeDistance;
			aabb.upperBound.x += b2_speculativeDistance;
			aabb.upperBound.y += b2_speculativeDistance;
			shape->aabb = aabb;

			if (b2AABB_Contains(shape->fatAABB, aabb) == false)
			{
				b2AABB fatAABB;
				fatAABB.lowerBound.x = aabb.lowerBound.x - b2_aabbMargin;
				fatAABB.lowerBound.y = aabb.lowerBound.y - b2_aabbMargin;
				fatAABB.upperBound.x = aabb.upperBound.x + b2_aabbMargin;
				fatAABB.upperBound.y = aabb.upperBound.y + b2_aabbMargin;
				shape->fatAABB = fatAABB;

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
		fastBody->rotation0 = fastBody->rotation;
		fastBody->position0 = fastBody->position;

		// Prepare AABBs for broad-phase
		shapeIndex = fastBody->shapeList;
		while (shapeIndex != B2_NULL_INDEX)
		{
			b2Shape* shape = shapes + shapeIndex;

			// shape->aabb is still valid

			if (b2AABB_Contains(shape->fatAABB, shape->aabb) == false)
			{
				b2AABB fatAABB;
				fatAABB.lowerBound.x = shape->aabb.lowerBound.x - b2_aabbMargin;
				fatAABB.lowerBound.y = shape->aabb.lowerBound.y - b2_aabbMargin;
				fatAABB.upperBound.x = shape->aabb.upperBound.x + b2_aabbMargin;
				fatAABB.upperBound.y = shape->aabb.upperBound.y + b2_aabbMargin;
				shape->fatAABB = fatAABB;

				shape->enlargedAABB = true;
				fastBody->enlargeAABB = true;
			}

			shapeIndex = shape->nextShapeIndex;
		}
	}
}

static void b2FastBodyTask(int32_t startIndex, int32_t endIndex, uint32_t threadIndex, void* taskContext)
{
	B2_MAYBE_UNUSED(threadIndex);

	b2TracyCZoneNC(fast_body_task, "Fast Body Task", b2_colorAqua, true);

	b2World* world = taskContext;

	B2_ASSERT(startIndex <= endIndex);
	B2_ASSERT(startIndex <= world->bodyPool.capacity);
	B2_ASSERT(endIndex <= world->bodyPool.capacity);

	for (int32_t i = startIndex; i < endIndex; ++i)
	{
		int32_t index = world->fastBodies[i];
		b2SolveContinuous(world, index);
	}

	b2TracyCZoneEnd(fast_body_task);
}

static void b2BulletBodyTask(int32_t startIndex, int32_t endIndex, uint32_t threadIndex, void* taskContext)
{
	B2_MAYBE_UNUSED(threadIndex);

	b2TracyCZoneNC(bullet_body_task, "Bullet Body Task", b2_colorLightSkyBlue, true);

	b2World* world = taskContext;

	B2_ASSERT(startIndex <= endIndex);
	B2_ASSERT(startIndex <= world->bodyPool.capacity);
	B2_ASSERT(endIndex <= world->bodyPool.capacity);

	for (int32_t i = startIndex; i < endIndex; ++i)
	{
		int32_t index = world->bulletBodies[i];
		b2SolveContinuous(world, index);
	}

	b2TracyCZoneEnd(bullet_body_task);
}

// Solve with graph coloring
void b2Solve(b2World* world, b2StepContext* context)
{
	b2TracyCZoneNC(solve, "Solve", b2_colorMistyRose, true);

	b2Timer timer = b2CreateTimer();

	world->stepId += 1;

	b2MergeAwakeIslands(world);

	world->profile.buildIslands = b2GetMillisecondsAndReset(&timer);

	b2TracyCZoneNC(graph_solver, "Graph", b2_colorSeaGreen, true);

	// Solve constraints using graph coloring
	bool anyAwake = b2SolveConstraintGraph(world, context);

	b2TracyCZoneEnd(graph_solver);

	world->profile.solveConstraints = b2GetMillisecondsAndReset(&timer);

	// Finish the user tree task that was queued early in the time step. This must be done before touching the broadphase.
	if (world->userTreeTask != NULL)
	{
		world->finishTaskFcn(world->userTreeTask, world->userTaskContext);
		world->activeTaskCount -= 1;
		world->userTreeTask = NULL;
	}

	b2ValidateNoEnlarged(&world->broadPhase);

	b2TracyCZoneNC(broad_phase, "Broadphase", b2_colorPurple, true);

	b2TracyCZoneNC(enlarge_proxies, "Enlarge Proxies", b2_colorDarkTurquoise, true);

	// Enlarge broad-phase proxies and build move array
	// #todo this is a hack to deal with stale shapeBitSet when no bodies are awake because they were all destroyed
	if (anyAwake)
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
		uint32_t wordCount = bitSet->blockCount;
		uint64_t* bits = bitSet->bits;
		for (uint32_t k = 0; k < wordCount; ++k)
		{
			word = bits[k];
			while (word != 0)
			{
				uint32_t ctz = b2CTZ(word);
				uint32_t shapeIndex = 64 * k + ctz;

				b2Shape* shape = shapes + shapeIndex;
				B2_ASSERT(b2IsValidObject(&shape->object));
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

	b2TracyCZoneNC(continuous_collision, "Continuous", b2_colorDarkGoldenrod, true);

	// Parallel continuous collision
	{
		// fast bodies
		int32_t minRange = 8;
		void* userFastBodyTask =
			world->enqueueTaskFcn(&b2FastBodyTask, world->fastBodyCount, minRange, world, world->userTaskContext);
		world->taskCount += 1;
		if (userFastBodyTask != NULL)
		{
			world->finishTaskFcn(userFastBodyTask, world->userTaskContext);
		}
	}

	{
		// bullet bodies
		int32_t minRange = 8;
		void* userBulletBodyTask =
			world->enqueueTaskFcn(&b2BulletBodyTask, world->bulletBodyCount, minRange, world, world->userTaskContext);
		world->taskCount += 1;
		if (userBulletBodyTask != NULL)
		{
			world->finishTaskFcn(userBulletBodyTask, world->userTaskContext);
		}
	}

	// Serially enlarge broad-phase proxies for fast shapes
	{
		b2BroadPhase* broadPhase = &world->broadPhase;
		b2Body* bodies = world->bodies;
		b2Shape* shapes = world->shapes;
		int32_t* fastBodies = world->fastBodies;
		int32_t fastBodyCount = world->fastBodyCount;
		b2DynamicTree* tree = broadPhase->trees + b2_dynamicBody;

		// This loop has non-deterministic order but it shouldn't affect the result
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
					shapeIndex = shape->nextShapeIndex;
					continue;
				}

				// clear flag
				shape->enlargedAABB = false;

				int32_t proxyKey = shape->proxyKey;
				int32_t proxyId = B2_PROXY_ID(proxyKey);
				B2_ASSERT(B2_PROXY_TYPE(proxyKey) == b2_dynamicBody);

				// all fast shapes should already be in the move buffer
				B2_ASSERT(b2ContainsKey(&broadPhase->moveSet, proxyKey + 1));

				b2DynamicTree_EnlargeProxy(tree, proxyId, shape->fatAABB);

				shapeIndex = shape->nextShapeIndex;
			}
		}
	}

	// Serially enlarge broad-phase proxies for bullet shapes
	{
		b2BroadPhase* broadPhase = &world->broadPhase;
		b2Body* bodies = world->bodies;
		b2Shape* shapes = world->shapes;
		int32_t* bulletBodies = world->bulletBodies;
		int32_t bulletBodyCount = world->bulletBodyCount;
		b2DynamicTree* tree = broadPhase->trees + b2_dynamicBody;

		// This loop has non-deterministic order but it shouldn't affect the result
		for (int32_t i = 0; i < bulletBodyCount; ++i)
		{
			b2Body* bulletBody = bodies + bulletBodies[i];
			if (bulletBody->enlargeAABB == false)
			{
				continue;
			}

			// clear flag
			bulletBody->enlargeAABB = false;

			int32_t shapeIndex = bulletBody->shapeList;
			while (shapeIndex != B2_NULL_INDEX)
			{
				b2Shape* shape = shapes + shapeIndex;
				if (shape->enlargedAABB == false)
				{
					shapeIndex = shape->nextShapeIndex;
					continue;
				}

				// clear flag
				shape->enlargedAABB = false;

				int32_t proxyKey = shape->proxyKey;
				int32_t proxyId = B2_PROXY_ID(proxyKey);
				B2_ASSERT(B2_PROXY_TYPE(proxyKey) == b2_dynamicBody);

				// all fast shapes should already be in the move buffer
				B2_ASSERT(b2ContainsKey(&broadPhase->moveSet, proxyKey + 1));

				b2DynamicTree_EnlargeProxy(tree, proxyId, shape->fatAABB);

				shapeIndex = shape->nextShapeIndex;
			}
		}
	}

	b2TracyCZoneEnd(continuous_collision);

	b2FreeStackItem(world->stackAllocator, world->bulletBodies);
	world->bulletBodies = NULL;
	world->bulletBodyCount = 0;

	b2FreeStackItem(world->stackAllocator, world->fastBodies);
	world->fastBodies = NULL;
	world->fastBodyCount = 0;

	world->profile.continuous = b2GetMilliseconds(&timer);

	b2TracyCZoneEnd(solve);
}
