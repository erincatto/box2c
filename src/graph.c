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
#include <string.h>

// Kinematic bodies have to be treated like dynamic bodies in graph coloring. Unlike static bodies, we cannot use a dummy solver body for
// kinematic bodies. We cannot access a kinematic body from multiple threads efficiently because the SIMD solver body scatter would write to
// the same kinematic body from multiple threads. Even if these writes don't modify the body, they will cause horrible cache stalls. To make
// this feasible I would need a way to block these writes.

extern bool b2_parallel;

typedef struct b2WorkerContext
{
	b2SolverTaskContext* context;
	int32_t workerIndex;
	void* userTask;
} b2WorkerContext;

void b2CreateGraph(b2Graph* graph, int32_t bodyCapacity, int32_t contactCapacity, int32_t jointCapacity)
{
	memset(graph, 0, sizeof(b2Graph));

	bodyCapacity = B2_MAX(bodyCapacity, 8);
	contactCapacity = B2_MAX(contactCapacity, 8);
	jointCapacity = B2_MAX(jointCapacity, 8);

	for (int32_t i = 0; i < b2_graphColorCount; ++i)
	{
		b2GraphColor* color = graph->colors + i;
		color->bodySet = b2CreateBitSet(bodyCapacity);
		b2SetBitCountAndClear(&color->bodySet, bodyCapacity);

		color->contactArray = b2CreateArray(sizeof(int32_t), contactCapacity);
		color->jointArray = b2CreateArray(sizeof(int32_t), jointCapacity);
		color->contactConstraints = NULL;
	}

	graph->overflow.contactArray = b2CreateArray(sizeof(int32_t), contactCapacity);
	graph->overflow.jointArray = b2CreateArray(sizeof(int32_t), jointCapacity);
	graph->overflow.contactConstraints = NULL;
}

void b2DestroyGraph(b2Graph* graph)
{
	for (int32_t i = 0; i < b2_graphColorCount; ++i)
	{
		b2GraphColor* color = graph->colors + i;
		b2DestroyBitSet(&color->bodySet);
		b2DestroyArray(color->contactArray, sizeof(int32_t));
		b2DestroyArray(color->jointArray, sizeof(int32_t));
	}

	b2DestroyArray(graph->overflow.contactArray, sizeof(int32_t));
	b2DestroyArray(graph->overflow.jointArray, sizeof(int32_t));
}

void b2AddContactToGraph(b2World* world, b2Contact* contact)
{
	B2_ASSERT(contact->colorIndex == B2_NULL_INDEX);
	B2_ASSERT(contact->colorSubIndex == B2_NULL_INDEX);

	b2Graph* graph = &world->graph;

	int32_t bodyIndexA = contact->edges[0].bodyIndex;
	int32_t bodyIndexB = contact->edges[1].bodyIndex;

	b2BodyType typeA = world->bodies[bodyIndexA].type;
	b2BodyType typeB = world->bodies[bodyIndexB].type;
	B2_ASSERT(typeA != b2_staticBody || typeB != b2_staticBody);

	if (typeA != b2_staticBody && typeB != b2_staticBody)
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

			contact->colorSubIndex = b2Array(color->contactArray).count;
			b2Array_Push(color->contactArray, contact->object.index);
			contact->colorIndex = i;
			break;
		}
	}
	else if (typeA != b2_staticBody)
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

			contact->colorSubIndex = b2Array(color->contactArray).count;
			b2Array_Push(color->contactArray, contact->object.index);
			contact->colorIndex = i;
			break;
		}
	}
	else if (typeB != b2_staticBody)
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

			contact->colorSubIndex = b2Array(color->contactArray).count;
			b2Array_Push(color->contactArray, contact->object.index);
			contact->colorIndex = i;
			break;
		}
	}

	// Overflow
	if (contact->colorIndex == B2_NULL_INDEX)
	{
		contact->colorSubIndex = b2Array(graph->overflow.contactArray).count;
		b2Array_Push(graph->overflow.contactArray, contact->object.index);
		contact->colorIndex = b2_overflowIndex;
	}
}

void b2RemoveContactFromGraph(b2World* world, b2Contact* contact)
{
	B2_ASSERT(contact->colorIndex != B2_NULL_INDEX);
	B2_ASSERT(contact->colorSubIndex != B2_NULL_INDEX);

	b2Graph* graph = &world->graph;

	// Overflow
	if (contact->colorIndex == b2_overflowIndex)
	{
		int32_t colorSubIndex = contact->colorSubIndex;
		b2Array_RemoveSwap(graph->overflow.contactArray, colorSubIndex);
		if (colorSubIndex < b2Array(graph->overflow.contactArray).count)
		{
			// Fix index on swapped contact
			int32_t swappedIndex = graph->overflow.contactArray[colorSubIndex];
			B2_ASSERT(world->contacts[swappedIndex].colorIndex == b2_overflowIndex);
			world->contacts[swappedIndex].colorSubIndex = colorSubIndex;
		}

		contact->colorIndex = B2_NULL_INDEX;
		contact->colorSubIndex = B2_NULL_INDEX;

		return;
	}

	B2_ASSERT(0 <= contact->colorIndex && contact->colorIndex < b2_graphColorCount);
	int32_t bodyIndexA = contact->edges[0].bodyIndex;
	int32_t bodyIndexB = contact->edges[1].bodyIndex;

	b2BodyType typeA = world->bodies[bodyIndexA].type;
	b2BodyType typeB = world->bodies[bodyIndexB].type;
	B2_ASSERT(typeA != b2_staticBody || typeB != b2_staticBody);

	if (typeA != b2_staticBody && typeB != b2_staticBody)
	{
		b2GraphColor* color = graph->colors + contact->colorIndex;
		B2_ASSERT(b2GetBit(&color->bodySet, bodyIndexA) && b2GetBit(&color->bodySet, bodyIndexB));

		int32_t colorSubIndex = contact->colorSubIndex;
		b2Array_RemoveSwap(color->contactArray, colorSubIndex);
		if (colorSubIndex < b2Array(color->contactArray).count)
		{
			// Fix index on swapped contact
			int32_t swappedIndex = color->contactArray[colorSubIndex];
			world->contacts[swappedIndex].colorSubIndex = colorSubIndex;
		}

		b2ClearBit(&color->bodySet, bodyIndexA);
		b2ClearBit(&color->bodySet, bodyIndexB);
	}
	else if (typeA != b2_staticBody)
	{
		b2GraphColor* color = graph->colors + contact->colorIndex;
		B2_ASSERT(b2GetBit(&color->bodySet, bodyIndexA));

		int32_t colorSubIndex = contact->colorSubIndex;
		b2Array_RemoveSwap(color->contactArray, colorSubIndex);
		if (colorSubIndex < b2Array(color->contactArray).count)
		{
			// Fix index on swapped contact
			int32_t swappedIndex = color->contactArray[colorSubIndex];
			world->contacts[swappedIndex].colorSubIndex = colorSubIndex;
		}

		b2ClearBit(&color->bodySet, bodyIndexA);
	}
	else if (typeB != b2_staticBody)
	{
		b2GraphColor* color = graph->colors + contact->colorIndex;
		B2_ASSERT(b2GetBit(&color->bodySet, bodyIndexB));

		int32_t colorSubIndex = contact->colorSubIndex;
		b2Array_RemoveSwap(color->contactArray, colorSubIndex);
		if (colorSubIndex < b2Array(color->contactArray).count)
		{
			// Fix index on swapped contact
			int32_t swappedIndex = color->contactArray[colorSubIndex];
			world->contacts[swappedIndex].colorSubIndex = colorSubIndex;
		}

		b2ClearBit(&color->bodySet, bodyIndexB);
	}

	contact->colorIndex = B2_NULL_INDEX;
	contact->colorSubIndex = B2_NULL_INDEX;
}

void b2AddJointToGraph(b2World* world, b2Joint* joint)
{
	B2_ASSERT(joint->colorIndex == B2_NULL_INDEX);
	B2_ASSERT(joint->colorSubIndex == B2_NULL_INDEX);

	b2Graph* graph = &world->graph;

	int32_t bodyIndexA = joint->edges[0].bodyIndex;
	int32_t bodyIndexB = joint->edges[1].bodyIndex;

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

			joint->colorSubIndex = b2Array(color->jointArray).count;
			b2Array_Push(color->jointArray, joint->object.index);
			joint->colorIndex = i;
			break;
		}
	}
	else if (typeA == b2_dynamicBody)
	{
		for (int32_t i = 0; i < b2_graphColorCount; ++i)
		{
			b2GraphColor* color = graph->colors + i;
			if (b2GetBit(&color->bodySet, bodyIndexA))
			{
				continue;
			}

			b2SetBitGrow(&color->bodySet, bodyIndexA);

			joint->colorSubIndex = b2Array(color->jointArray).count;
			b2Array_Push(color->jointArray, joint->object.index);
			joint->colorIndex = i;
			break;
		}
	}
	else if (typeB == b2_dynamicBody)
	{
		for (int32_t i = 0; i < b2_graphColorCount; ++i)
		{
			b2GraphColor* color = graph->colors + i;
			if (b2GetBit(&color->bodySet, bodyIndexB))
			{
				continue;
			}

			b2SetBitGrow(&color->bodySet, bodyIndexB);

			joint->colorSubIndex = b2Array(color->jointArray).count;
			b2Array_Push(color->jointArray, joint->object.index);
			joint->colorIndex = i;
			break;
		}
	}

	// Overflow
	if (joint->colorIndex == B2_NULL_INDEX)
	{
		joint->colorSubIndex = b2Array(graph->overflow.jointArray).count;
		b2Array_Push(graph->overflow.jointArray, joint->object.index);
		joint->colorIndex = b2_overflowIndex;
	}
}

void b2RemoveJointFromGraph(b2World* world, b2Joint* joint)
{
	B2_ASSERT(joint->colorIndex != B2_NULL_INDEX);
	B2_ASSERT(joint->colorSubIndex != B2_NULL_INDEX);

	b2Graph* graph = &world->graph;

	// Overflow
	if (joint->colorIndex == b2_overflowIndex)
	{
		int32_t colorSubIndex = joint->colorSubIndex;
		b2Array_RemoveSwap(graph->overflow.jointArray, colorSubIndex);
		if (colorSubIndex < b2Array(graph->overflow.jointArray).count)
		{
			// Fix index on swapped joint
			int32_t swappedIndex = graph->overflow.jointArray[colorSubIndex];
			B2_ASSERT(world->joints[swappedIndex].colorIndex == b2_overflowIndex);
			world->joints[swappedIndex].colorSubIndex = colorSubIndex;
		}

		joint->colorIndex = B2_NULL_INDEX;
		joint->colorSubIndex = B2_NULL_INDEX;

		return;
	}

	B2_ASSERT(0 <= joint->colorIndex && joint->colorIndex < b2_graphColorCount);
	int32_t bodyIndexA = joint->edges[0].bodyIndex;
	int32_t bodyIndexB = joint->edges[1].bodyIndex;

	b2BodyType typeA = world->bodies[bodyIndexA].type;
	b2BodyType typeB = world->bodies[bodyIndexB].type;

	if (typeA == b2_dynamicBody && typeB == b2_dynamicBody)
	{
		b2GraphColor* color = graph->colors + joint->colorIndex;
		B2_ASSERT(b2GetBit(&color->bodySet, bodyIndexA) && b2GetBit(&color->bodySet, bodyIndexB));

		int32_t colorSubIndex = joint->colorSubIndex;
		b2Array_RemoveSwap(color->jointArray, colorSubIndex);
		if (colorSubIndex < b2Array(color->jointArray).count)
		{
			// Fix index on swapped joint
			int32_t swappedIndex = color->jointArray[colorSubIndex];
			world->joints[swappedIndex].colorSubIndex = colorSubIndex;
		}

		b2ClearBit(&color->bodySet, bodyIndexA);
		b2ClearBit(&color->bodySet, bodyIndexB);
	}
	else if (typeA == b2_dynamicBody)
	{
		b2GraphColor* color = graph->colors + joint->colorIndex;
		B2_ASSERT(b2GetBit(&color->bodySet, bodyIndexA));

		int32_t colorSubIndex = joint->colorSubIndex;
		b2Array_RemoveSwap(color->jointArray, colorSubIndex);
		if (colorSubIndex < b2Array(color->jointArray).count)
		{
			// Fix index on swapped joint
			int32_t swappedIndex = color->jointArray[colorSubIndex];
			world->joints[swappedIndex].colorSubIndex = colorSubIndex;
		}

		b2ClearBit(&color->bodySet, bodyIndexA);
	}
	else if (typeB == b2_dynamicBody)
	{
		b2GraphColor* color = graph->colors + joint->colorIndex;
		B2_ASSERT(b2GetBit(&color->bodySet, bodyIndexB));

		int32_t colorSubIndex = joint->colorSubIndex;
		b2Array_RemoveSwap(color->jointArray, colorSubIndex);
		if (colorSubIndex < b2Array(color->jointArray).count)
		{
			// Fix index on swapped joint
			int32_t swappedIndex = color->jointArray[colorSubIndex];
			world->joints[swappedIndex].colorSubIndex = colorSubIndex;
		}

		b2ClearBit(&color->bodySet, bodyIndexB);
	}

	joint->colorIndex = B2_NULL_INDEX;
	joint->colorSubIndex = B2_NULL_INDEX;
}

static void b2IntegrateVelocitiesTask(int32_t startIndex, int32_t endIndex, b2SolverTaskContext* context)
{
	b2TracyCZoneNC(integrate_velocity, "IntVel", b2_colorDeepPink, true);

	b2Vec2 gravity = context->world->gravity;
	b2Body** bodies = context->awakeBodies;
	b2SolverBody* solverBodies = context->solverBodies;
	int32_t* bodyToSolverMap = context->bodyToSolverMap;

	float h = context->timeStep;

	// Integrate velocities and apply damping. Initialize the body state.
	for (int32_t i = startIndex; i < endIndex; ++i)
	{
		b2Body* body = bodies[i];
		//_m_prefetch(bodies[i + 1]);

		// create body map used to prepare constraints
		B2_ASSERT(body->object.index < context->world->bodyPool.capacity);
		bodyToSolverMap[body->object.index] = i;

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
	}

	b2TracyCZoneEnd(integrate_velocity);
}

static void b2PrepareJoints(int32_t startIndex, int32_t endIndex, b2SolverTaskContext* context)
{
	b2TracyCZoneNC(prepare_joints, "PrepJoints", b2_colorOldLace, true);

	b2World* world = context->world;
	b2Joint* joints = world->joints;
	b2StepContext* stepContext = context->stepContext;
	const int32_t* jointIndices = context->jointIndices;

	for (int32_t i = startIndex; i < endIndex; ++i)
	{
		int32_t index = jointIndices[i];
		B2_ASSERT(0 <= index && index < world->jointPool.capacity);

		b2Joint* joint = joints + index;
		B2_ASSERT(b2ObjectValid(&joint->object) == true);

		b2PrepareJoint(joint, stepContext);
	}

	b2TracyCZoneEnd(prepare_joints);
}

static void b2WarmStartJoints(int32_t startIndex, int32_t endIndex, b2SolverTaskContext* context, int32_t colorIndex)
{
	b2TracyCZoneNC(warm_joints, "WarmJoints", b2_colorGold, true);

	b2World* world = context->world;
	b2Joint* joints = world->joints;
	b2StepContext* stepContext = context->stepContext;
	int32_t* jointIndices = context->graph->colors[colorIndex].jointArray;

	for (int32_t i = startIndex; i < endIndex; ++i)
	{
		int32_t index = jointIndices[i];
		B2_ASSERT(0 <= index && index < world->jointPool.capacity);

		b2Joint* joint = joints + index;
		B2_ASSERT(b2ObjectValid(&joint->object) == true);

		b2WarmStartJoint(joint, stepContext);
	}

	b2TracyCZoneEnd(warm_joints);
}

static void b2SolveJoints(int32_t startIndex, int32_t endIndex, b2SolverTaskContext* context, int32_t colorIndex, bool useBias)
{
	b2TracyCZoneNC(solve_joints, "SolveJoints", b2_colorLemonChiffon, true);

	b2World* world = context->world;
	b2Joint* joints = world->joints;
	b2StepContext* stepContext = context->stepContext;
	int32_t* jointIndices = context->graph->colors[colorIndex].jointArray;

	for (int32_t i = startIndex; i < endIndex; ++i)
	{
		int32_t index = jointIndices[i];
		B2_ASSERT(0 <= index && index < world->jointPool.capacity);

		b2Joint* joint = joints + index;
		B2_ASSERT(b2ObjectValid(&joint->object) == true);

		b2SolveJointVelocity(joint, stepContext, useBias);
	}

	b2TracyCZoneEnd(solve_joints);
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

static void b2FinalizeBodiesTask(int32_t startIndex, int32_t endIndex, uint32_t threadIndex, void* taskContext)
{
	b2TracyCZoneNC(finalize_bodies, "FinalizeBodies", b2_colorViolet, true);

	b2SolverTaskContext* context = taskContext;
	b2World* world = context->world;
	bool enableSleep = world->enableSleep;
	b2Body* bodies = world->bodies;
	const b2SolverBody* solverBodies = context->solverBodies;
	b2Contact* contacts = world->contacts;
	const int32_t* solverToBodyMap = context->solverToBodyMap;
	const b2Vec2 aabbMargin = {b2_aabbMargin, b2_aabbMargin};
	float timeStep = context->timeStep;

	b2BitSet* awakeContactBitSet = &world->taskContextArray[threadIndex].awakeContactBitSet;
	b2BitSet* shapeBitSet = &world->taskContextArray[threadIndex].shapeBitSet;
	b2BitSet* awakeIslandBitSet = &world->taskContextArray[threadIndex].awakeIslandBitSet;
	bool enableContinuous = world->enableContinuous;

	B2_ASSERT(startIndex <= endIndex);
	B2_ASSERT(startIndex <= world->bodyPool.capacity);
	B2_ASSERT(endIndex <= world->bodyPool.capacity);

	// Update sleep
	const float linTolSqr = b2_linearSleepTolerance * b2_linearSleepTolerance;
	const float angTolSqr = b2_angularSleepTolerance * b2_angularSleepTolerance;

	for (int32_t i = startIndex; i < endIndex; ++i)
	{
		const b2SolverBody* solverBody = solverBodies + i;

		int32_t bodyIndex = solverToBodyMap[i];
		b2Body* body = bodies + bodyIndex;
		B2_ASSERT(b2ObjectValid(&body->object));

		b2Vec2 v = solverBody->linearVelocity;
		float w = solverBody->angularVelocity;

		body->linearVelocity = v;
		body->angularVelocity = w;

		body->position = b2Add(body->position, solverBody->deltaPosition);
		body->angle += solverBody->deltaAngle;

		body->transform.q = b2MakeRot(body->angle);
		body->transform.p = b2Sub(body->position, b2RotateVector(body->transform.q, body->localCenter));

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
				int fastIndex = atomic_fetch_add(&world->fastBodyCount, 1);
				world->fastBodies[fastIndex] = bodyIndex;
				body->isFast = true;
			}
			else
			{
				// Body is safe to advance
				body->position0 = body->position;
				body->angle0 = body->angle;
			}
		}
		else
		{
			// Body is safe to advance
			body->position0 = body->position;
			body->angle0 = body->angle;
			body->sleepTime += timeStep;
		}

		// Any single body in an island can keep it awake
		if (body->sleepTime < b2_timeToSleep)
		{
			B2_ASSERT(0 <= body->islandIndex && body->islandIndex < world->islandPool.capacity);
			b2SetBit(awakeIslandBitSet, body->islandIndex);
		}

		// Update shapes AABBs
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
				shape->aabb = b2Shape_ComputeAABB(shape, body->transform);

				if (b2AABB_Contains(shape->fatAABB, shape->aabb) == false)
				{
					shape->fatAABB.lowerBound = b2Sub(shape->aabb.lowerBound, aabbMargin);
					shape->fatAABB.upperBound = b2Add(shape->aabb.upperBound, aabbMargin);

					// Bit-set to keep the move array sorted
					b2SetBit(shapeBitSet, shapeIndex);
				}
			}
			
			shapeIndex = shape->nextShapeIndex;
		}

		// Wake contacts
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

	b2TracyCZoneEnd(finalize_bodies);
}

/*
 typedef enum b2SolverStageType
{
	b2_stageIntegrateVelocities,
	b2_stagePrepareJoints,
	b2_stagePrepareContacts,
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

static void b2ExecuteBlock(b2SolverStage* stage, b2SolverTaskContext* context, b2SolverBlock* block)
{
	b2SolverStageType stageType = stage->type;
	b2SolverBlockType blockType = block->blockType;
	int32_t startIndex = block->startIndex;
	int32_t endIndex = startIndex + block->count;

	switch (stageType)
	{
		case b2_stageIntegrateVelocities:
			b2IntegrateVelocitiesTask(startIndex, endIndex, context);
			break;

		case b2_stagePrepareJoints:
			b2PrepareJoints(startIndex, endIndex, context);
			break;

		case b2_stagePrepareContacts:
			b2PrepareContactsSIMD(startIndex, endIndex, context);
			break;

		case b2_stageWarmStart:
			if (blockType == b2_graphContactBlock)
			{
				b2WarmStartContactsSIMD(startIndex, endIndex, context, stage->colorIndex);
			}
			else if (blockType == b2_graphJointBlock)
			{
				b2WarmStartJoints(startIndex, endIndex, context, stage->colorIndex);
			}
			break;

		case b2_stageSolve:
			if (blockType == b2_graphContactBlock)
			{
				b2SolveContactsSIMD(startIndex, endIndex, context, stage->colorIndex, true);
			}
			else if (blockType == b2_graphJointBlock)
			{
				b2SolveJoints(startIndex, endIndex, context, stage->colorIndex, true);
			}
			break;

		case b2_stageIntegratePositions:
			b2IntegratePositionsTask(startIndex, endIndex, context);
			break;

		case b2_stageRelax:
			if (blockType == b2_graphContactBlock)
			{
				b2SolveContactsSIMD(startIndex, endIndex, context, stage->colorIndex, false);
			}
			else if (blockType == b2_graphJointBlock)
			{
				b2SolveJoints(startIndex, endIndex, context, stage->colorIndex, false);
			}
			break;

		case b2_stageRestitution:
			if (blockType == b2_graphContactBlock)
			{
				b2ApplyRestitutionSIMD(startIndex, endIndex, context, stage->colorIndex);
			}
			break;

		case b2_stageStoreImpulses:
			b2StoreImpulsesSIMD(startIndex, endIndex, context);
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

static void b2ExecuteMainStage(b2SolverStage* stage, b2SolverTaskContext* context, uint32_t syncBits)
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

		while (atomic_load(&stage->completionCount) != blockCount)
		{
			_mm_pause();
		}

		atomic_store(&stage->completionCount, 0);
	}
}

// This should not use the thread index because thread 0 can be called twice by enkiTS.
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
		b2_stageWarmStart,
		b2_stageSolve,
		b2_stageIntegratePositions,
		b2_stageRelax,
		b2_stageRestitution,
		b2_stageStoreImpulses
		*/

		int32_t bodySyncIndex = 1;
		int32_t stageIndex = 0;
		uint32_t syncBits = (bodySyncIndex << 16) | stageIndex;
		B2_ASSERT(stages[stageIndex].type == b2_stageIntegrateVelocities);
		b2ExecuteMainStage(stages + stageIndex, context, syncBits);
		stageIndex += 1;
		bodySyncIndex += 1;

		uint32_t jointSyncIndex = 1;
		syncBits = (jointSyncIndex << 16) | stageIndex;
		B2_ASSERT(stages[stageIndex].type == b2_stagePrepareJoints);
		b2ExecuteMainStage(stages + stageIndex, context, syncBits);
		stageIndex += 1;
		jointSyncIndex += 1;

		uint32_t constraintSyncIndex = 1;
		syncBits = (constraintSyncIndex << 16) | stageIndex;
		B2_ASSERT(stages[stageIndex].type == b2_stagePrepareContacts);
		b2ExecuteMainStage(stages + stageIndex, context, syncBits);
		stageIndex += 1;
		constraintSyncIndex += 1;

		int32_t graphSyncIndex = 1;
		for (int32_t colorIndex = 0; colorIndex < activeColorCount; ++colorIndex)
		{
			syncBits = (graphSyncIndex << 16) | stageIndex;
			B2_ASSERT(stages[stageIndex].type == b2_stageWarmStart);
			b2ExecuteMainStage(stages + stageIndex, context, syncBits);
			stageIndex += 1;
		}
		graphSyncIndex += 1;

		b2PrepareAndWarmStartOverflowContacts(context);

		int32_t velocityIterations = context->velocityIterations;
		for (int32_t i = 0; i < velocityIterations; ++i)
		{
			// stage index restarted each iteration
			int32_t iterStageIndex = stageIndex;

			for (int32_t colorIndex = 0; colorIndex < activeColorCount; ++colorIndex)
			{
				syncBits = (graphSyncIndex << 16) | iterStageIndex;
				B2_ASSERT(stages[iterStageIndex].type == b2_stageSolve);
				b2ExecuteMainStage(stages + iterStageIndex, context, syncBits);
				iterStageIndex += 1;
			}
			graphSyncIndex += 1;

			b2SolveOverflowContacts(context, true);

			B2_ASSERT(stages[iterStageIndex].type == b2_stageIntegratePositions);
			syncBits = (bodySyncIndex << 16) | iterStageIndex;
			b2ExecuteMainStage(stages + iterStageIndex, context, syncBits);
			bodySyncIndex += 1;
		}

		stageIndex += activeColorCount + 1;

		int32_t relaxIterations = context->relaxIterations;
		for (int32_t i = 0; i < relaxIterations; ++i)
		{
			// stage index restarted each iteration
			int32_t iterStageIndex = stageIndex;

			for (int32_t colorIndex = 0; colorIndex < activeColorCount; ++colorIndex)
			{
				syncBits = (graphSyncIndex << 16) | iterStageIndex;
				B2_ASSERT(stages[iterStageIndex].type == b2_stageRelax);
				b2ExecuteMainStage(stages + iterStageIndex, context, syncBits);
				iterStageIndex += 1;
			}
			graphSyncIndex += 1;

			b2SolveOverflowContacts(context, false);
		}

		stageIndex += activeColorCount;

		// Restitution
		{
			int32_t iterStageIndex = stageIndex;
			for (int32_t colorIndex = 0; colorIndex < activeColorCount; ++colorIndex)
			{
				syncBits = (graphSyncIndex << 16) | iterStageIndex;
				B2_ASSERT(stages[iterStageIndex].type == b2_stageRestitution);
				b2ExecuteMainStage(stages + iterStageIndex, context, syncBits);
				iterStageIndex += 1;
			}
			graphSyncIndex += 1;

			b2ApplyOverflowRestitution(context);
		}

		stageIndex += activeColorCount;

		b2StoreOverflowImpulses(context);

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

// TODO_ERIN this comment is out of data
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
void b2SolveGraph(b2World* world, b2StepContext* stepContext)
{
	b2TracyCZoneNC(prepare_stages, "Prepare Stages", b2_colorDarkOrange, true);

	b2Graph* graph = &world->graph;
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
	// TODO_ERIN scope problem
	world->fastBodyCount = 0;
	world->fastBodies = b2AllocateStackItem(world->stackAllocator, awakeBodyCount * sizeof(int32_t), "fast bodies");

	if (awakeBodyCount == 0)
	{
		for (int32_t i = 0; i < b2_graphColorCount; ++i)
		{
			graph->occupancy[i] = b2Array(colors[i].contactArray).count;
		}
		graph->occupancy[b2_overflowIndex] = b2Array(graph->overflow.contactArray).count;

		return;
	}

	// Reserve space for awake bodies
	b2Body* bodies = world->bodies;
	b2Body** awakeBodies = b2AllocateStackItem(world->stackAllocator, awakeBodyCount * sizeof(b2Body*), "awake bodies");
	b2SolverBody* solverBodies = b2AllocateStackItem(world->stackAllocator, awakeBodyCount * sizeof(b2SolverBody), "solver bodies");

	// Map from solver body to body
	// TODO_ERIN have body directly reference solver body for user access?
	int32_t* solverToBodyMap = b2AllocateStackItem(world->stackAllocator, awakeBodyCount * sizeof(int32_t), "solver body map");

	// Map from world body to solver body
	// TODO_ERIN eliminate this?
	int32_t bodyCapacity = world->bodyPool.capacity;
	int32_t* bodyToSolverMap = b2AllocateStackItem(world->stackAllocator, bodyCapacity * sizeof(int32_t), "body map");
	memset(bodyToSolverMap, 0xFF, bodyCapacity * sizeof(int32_t));

	// Build array of awake bodies and also search for an awake island to split
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
			B2_ASSERT(b2ObjectValid(&body->object));
			B2_ASSERT(body->object.index == bodyIndex);

			awakeBodies[index] = body;

			B2_ASSERT(0 < bodyIndex && bodyIndex < bodyCapacity);
			bodyToSolverMap[bodyIndex] = index;
			solverToBodyMap[index] = bodyIndex;

			// cache miss
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
			colorContactBlockSizes[c] = 4;
			if (colorContactCountSIMD > 4 * maxBlockCount)
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
			colorJointBlockSizes[c] = 4;
			if (colorJointCount > 4 * maxBlockCount)
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
	graph->overflow.contactConstraints =
		b2AllocateStackItem(world->stackAllocator, overflowContactCount * sizeof(b2ContactConstraint), "overflow contact constraint");

	// Distribute transient constraints to each graph color
	int32_t base = 0;
	int32_t jointIndex = 0;
	for (int32_t i = 0; i < activeColorCount; ++i)
	{
		int32_t j = activeColorIndices[i];
		b2GraphColor* color = colors + j;

		int32_t colorJointCount = b2Array(color->jointArray).count;
		memcpy(jointIndices + jointIndex, color->jointArray, colorJointCount * sizeof(int32_t));
		jointIndex += colorJointCount;

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

	// Define work blocks for preparing contacts and storing contact impulses
	int32_t contactBlockSize = 4;
	int32_t contactBlockCount = contactCount > 0 ? ((contactCount - 1) >> 2) + 1 : 0;
	if (contactCount > contactBlockSize * maxBlockCount)
	{
		// Too many blocks, increase block size
		contactBlockSize = contactCount / maxBlockCount;
		contactBlockCount = maxBlockCount;
	}

	// Define work blocks for preparing joints
	int32_t jointBlockSize = 4;
	int32_t jointBlockCount = jointCount > 0 ? ((jointCount - 1) >> 2) + 1 : 0;
	if (jointCount > jointBlockSize * maxBlockCount)
	{
		// Too many blocks, increase block size
		jointBlockSize = jointCount / maxBlockCount;
		jointBlockCount = maxBlockCount;
	}

	/*
	b2_stageIntegrateVelocities = 0,
	b2_stagePrepareJoints,
	b2_stagePrepareContacts,
	b2_stageWarmStart,
	b2_stageSolve,
	b2_stageIntegratePositions,
	b2_stageRelax,
	b2_stageRestitution,
	b2_stageStoreImpulses
	*/

	// TODO_ERIN joint tasks
	int32_t stageCount = 0;

	// b2_stageIntegrateVelocities
	stageCount += 1;
	// b2_stagePrepareJoints
	stageCount += 1;
	// b2_stagePrepareContacts
	stageCount += 1;
	// b2_stageWarmStart
	stageCount += activeColorCount;
	// b2_stageSolve, b2_stageIntegratePositions
	stageCount += activeColorCount + 1;
	// b2_stageRelax
	stageCount += activeColorCount;
	// b2_stageRestitution
	stageCount += activeColorCount;
	// b2_stageStoreImpulses
	stageCount += 1;

	b2SolverStage* stages = b2AllocateStackItem(world->stackAllocator, stageCount * sizeof(b2SolverStage), "stages");
	b2SolverBlock* bodyBlocks = b2AllocateStackItem(world->stackAllocator, bodyBlockCount * sizeof(b2SolverBlock), "body blocks");
	b2SolverBlock* contactBlocks = b2AllocateStackItem(world->stackAllocator, contactBlockCount * sizeof(b2SolverBlock), "contact blocks");
	b2SolverBlock* jointBlocks = b2AllocateStackItem(world->stackAllocator, jointBlockCount * sizeof(b2SolverBlock), "joint blocks");
	b2SolverBlock* graphBlocks = b2AllocateStackItem(world->stackAllocator, graphBlockCount * sizeof(b2SolverBlock), "graph blocks");

	// TODO_ERIN cannot do this in parallel with FinalizeBodies
	// Split an awake island. This modifies:
	// - stack allocator
	// - awake island array
	// - island pool
	// - island indices on bodies, contacts, and joints
	// I'm squeezing this task in here because it may be expensive and this
	// is a safe place to put it.
	world->splitIslandIndex = splitIslandIndex;
	void* splitIslandTask = NULL;
	if (splitIslandIndex != B2_NULL_INDEX)
	{
		if (b2_parallel)
		{
			splitIslandTask = world->enqueueTaskFcn(&b2SplitIslandTask, 1, 1, world, world->userTaskContext);
		}
		else
		{
			b2SplitIslandTask(0, 1, 0, world);
			world->splitIslandIndex = B2_NULL_INDEX;
		}
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
			baseGraphBlock[colorJointBlockCount - 1].count = (int16_t)(colorJointCounts[i] - (colorJointBlockCount - 1) * colorJointBlockSize);
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
			baseGraphBlock[colorContactBlockCount - 1].count = (int16_t)(colorContactCounts[i] - (colorContactBlockCount - 1) * colorContactBlockSize);
			baseGraphBlock += colorContactBlockCount;
		}
	}

	B2_ASSERT((ptrdiff_t)(baseGraphBlock - graphBlocks) == graphBlockCount);

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

	int32_t velIters = B2_MAX(1, stepContext->velocityIterations);

	stepContext->solverBodies = solverBodies;
	stepContext->solverToBodyMap = solverToBodyMap;
	stepContext->bodyToSolverMap = bodyToSolverMap;

	b2SolverTaskContext context;
	context.world = world;
	context.graph = graph;
	context.awakeBodies = awakeBodies;
	context.solverBodies = solverBodies;
	context.bodyToSolverMap = bodyToSolverMap;
	context.solverToBodyMap = solverToBodyMap;
	context.stepContext = stepContext;
	context.contactConstraints = contactConstraints;
	context.jointIndices = jointIndices;
	context.contactIndices = contactIndices;
	context.activeColorCount = activeColorCount;
	context.velocityIterations = velIters;
	context.relaxIterations = stepContext->relaxIterations;
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
	if (b2_parallel)
	{
		for (int32_t i = 0; i < workerCount; ++i)
		{
			workerContext[i].context = &context;
			workerContext[i].workerIndex = i;
			workerContext[i].userTask = world->enqueueTaskFcn(b2SolverTask, 1, 1, workerContext + i, world->userTaskContext);
		}
	}
	else
	{
		// This relies on work stealing
		for (int32_t i = 0; i < workerCount; ++i)
		{
			workerContext[i].context = &context;
			workerContext[i].workerIndex = i;
			workerContext[i].userTask = NULL;
			b2SolverTask(0, 1, 0, workerContext + i);
		}
	}

	// Finish split
	if (splitIslandTask != NULL)
	{
		world->finishTaskFcn(splitIslandTask, world->userTaskContext);
		world->splitIslandIndex = B2_NULL_INDEX;
	}

	// Finish solve
	if (b2_parallel)
	{
		for (int32_t i = 0; i < workerCount; ++i)
		{
			world->finishTaskFcn(workerContext[i].userTask, world->userTaskContext);
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
	void* finalizeBodiesTask = NULL;
	if (b2_parallel)
	{
		finalizeBodiesTask = world->enqueueTaskFcn(b2FinalizeBodiesTask, awakeBodyCount, 16, &context, world->userTaskContext);
		world->finishTaskFcn(finalizeBodiesTask, world->userTaskContext);
	}
	else
	{
		b2FinalizeBodiesTask(0, awakeBodyCount, 0, &context);
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
	b2FreeStackItem(world->stackAllocator, bodyToSolverMap);
	b2FreeStackItem(world->stackAllocator, solverToBodyMap);
	b2FreeStackItem(world->stackAllocator, solverBodies);
	b2FreeStackItem(world->stackAllocator, awakeBodies);
}
