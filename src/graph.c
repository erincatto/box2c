// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "graph.h"

#include "allocate.h"
#include "array.h"
#include "body.h"
#include "contact.h"
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

#define maxBaumgarteVelocity 3.0f

typedef struct
{
	b2Vec2 rA, rB;
	b2Vec2 localAnchorA, localAnchorB;
	float separation;
	float normalImpulse;
	float tangentImpulse;
	float normalMass;
	float tangentMass;
} b2ConstraintPoint;

typedef struct b2Constraint
{
	b2Contact* contact;
	int32_t indexA;
	int32_t indexB;
	b2ConstraintPoint points[2];
	b2Vec2 normal;
	float friction;
	float massCoefficient;
	float biasCoefficient;
	float impulseCoefficient;
	int32_t pointCount;
} b2Constraint;

typedef enum
{
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
} b2SolverStageType;

// Each block of work has a sync index that gets incremented when a worker claims the block. This ensures only a single worker claims a
// block, yet lets work be distributed dynamically across multiple workers (work stealing). This also reduces contention on a single block
// index atomic. For non-iterative stages the sync index is simply set to one. For iterative stages (solver iteration) the same block of
// work is executed once per iteration and the atomic sync index is shared across iterations, so it increases monotonically.
typedef struct
{
	int32_t startIndex;
	int32_t endIndex;
	_Atomic int syncIndex;
} b2SolverBlock;

// Each stage must be completed before going to the next stage.
// Non-iterative stages use a stage instance once while iterative stages re-use the same instance each iteration.
typedef struct
{
	b2SolverStageType type;
	b2SolverBlock* blocks;
	int32_t blockCount;
	int32_t colorIndex;
	_Atomic int completionCount;
} b2SolverStage;

typedef struct
{
	b2World* world;
	b2Body** awakeBodies;
	b2SolverBody* solverBodies;
	b2Graph* graph;

	const b2StepContext* stepContext;
	b2Constraint* constraints;
	int32_t activeColorCount;
	int32_t velocityIterations;
	int32_t calmIterations;
	int32_t workerCount;

	float timeStep;
	float invTimeStep;
	float subStep;
	float invSubStep;

	b2SolverStage* stages;
	int32_t stageCount;

	// sync index (16-bits) | stage type (16-bits)
	_Atomic unsigned int syncBits;
} b2SolverTaskContext;

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

	float h = context->timeStep;

	// Integrate velocities and apply damping. Initialize the body state.
	for (int32_t i = startIndex; i < endIndex; ++i)
	{
		b2Body* body = bodies[i];
		//_m_prefetch(bodies[i + 1]);

		B2_ASSERT(body->solverIndex == i);

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
	const b2StepContext* stepContext = context->stepContext;

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

static void b2PrepareContactsTask(int32_t startIndex, int32_t endIndex, b2SolverTaskContext* context, int32_t colorIndex)
{
	b2TracyCZoneNC(prepare_contact, "Prepare Contact", b2_colorYellow, true);

	b2World* world = context->world;
	b2Graph* graph = context->graph;
	b2GraphColor* color = graph->colors + colorIndex;
	int32_t* contactIndices = color->contactArray;
	b2Contact* contacts = world->contacts;
	b2Body* bodies = world->bodies;
	b2SolverBody* solverBodies = context->solverBodies;

	// 30 is a bit soft, 60 oscillates too much
	// const float contactHertz = 45.0f;
	// const float contactHertz = B2_MAX(15.0f, stepContext->inv_dt * stepContext->velocityIterations / 8.0f);
	const float contactHertz = 45.0f;

	float h = context->timeStep;
	bool enableWarmStarting = world->enableWarmStarting;

	B2_ASSERT(startIndex <= b2Array(color->contactArray).count);
	B2_ASSERT(endIndex <= b2Array(color->contactArray).count);

	for (int32_t i = startIndex; i < endIndex; ++i)
	{
		b2Contact* contact = contacts + contactIndices[i];

		const b2Manifold* manifold = &contact->manifold;
		int32_t pointCount = manifold->pointCount;

		B2_ASSERT(0 < pointCount && pointCount <= 2);

		b2Body* bodyA = bodies + contact->edges[0].bodyIndex;
		b2Body* bodyB = bodies + contact->edges[1].bodyIndex;

		int32_t indexA = bodyA->solverIndex;
		int32_t indexB = bodyB->solverIndex;

		b2Constraint* constraint = color->contacts + i;
		constraint->contact = contact;
		constraint->indexA = indexA;
		constraint->indexB = indexB;
		constraint->normal = manifold->normal;
		constraint->friction = contact->friction;
		constraint->pointCount = pointCount;

		b2SolverBody* solverBodyA;
		b2Vec2 vA;
		float wA;
		float mA;
		float iA;

		if (indexA != B2_NULL_INDEX)
		{
			solverBodyA = solverBodies + indexA;
			vA = solverBodyA->linearVelocity;
			wA = solverBodyA->angularVelocity;
			mA = solverBodyA->invMass;
			iA = solverBodyA->invI;
		}
		else
		{
			solverBodyA = NULL;
			vA.x = vA.y = 0.0;
			wA = 0.0f;
			mA = 0.0f;
			iA = 0.0f;
		}

		B2_ASSERT(indexB != B2_NULL_INDEX);
		b2SolverBody* solverBodyB = solverBodies + indexB;
		b2Vec2 vB = solverBodyB->linearVelocity;
		float wB = solverBodyB->angularVelocity;
		float mB = solverBodyB->invMass;
		float iB = solverBodyB->invI;

		// Stiffer for static contacts to avoid bodies getting pushed through the ground
		const float hertz = mA == 0.0f ? 2.0f * contactHertz : contactHertz;
		const float zeta = 1.0f;
		float omega = 2.0f * b2_pi * hertz;
		float c = h * omega * (2.0f * zeta + h * omega);
		constraint->impulseCoefficient = 1.0f / (1.0f + c);
		constraint->massCoefficient = c * constraint->impulseCoefficient;
		constraint->biasCoefficient = omega / (2.0f * zeta + h * omega);

		b2Vec2 cA = bodyA->position;
		b2Vec2 cB = bodyB->position;
		b2Rot qA = bodyA->transform.q;
		b2Rot qB = bodyB->transform.q;

		b2Vec2 normal = constraint->normal;
		b2Vec2 tangent = b2RightPerp(constraint->normal);

		for (int32_t j = 0; j < pointCount; ++j)
		{
			const b2ManifoldPoint* mp = manifold->points + j;
			b2ConstraintPoint* cp = constraint->points + j;

			cp->normalImpulse = mp->normalImpulse;
			cp->tangentImpulse = mp->tangentImpulse;

			cp->rA = b2Sub(mp->point, cA);
			cp->rB = b2Sub(mp->point, cB);
			cp->localAnchorA = b2InvRotateVector(qA, cp->rA);
			cp->localAnchorB = b2InvRotateVector(qB, cp->rB);

			float rnA = b2Cross(cp->rA, normal);
			float rnB = b2Cross(cp->rB, normal);
			float kNormal = mA + mB + iA * rnA * rnA + iB * rnB * rnB;

			float rtA = b2Cross(cp->rA, tangent);
			float rtB = b2Cross(cp->rB, tangent);
			float kTangent = mA + mB + iA * rtA * rtA + iB * rtB * rtB;

			cp->tangentMass = kTangent > 0.0f ? 1.0f / kTangent : 0.0f;
			cp->separation = mp->separation;
			cp->normalMass = kNormal > 0.0f ? 1.0f / kNormal : 0.0f;

			// Warm start
			if (enableWarmStarting)
			{
				b2Vec2 P = b2Add(b2MulSV(cp->normalImpulse, normal), b2MulSV(cp->tangentImpulse, tangent));
				wA -= iA * b2Cross(cp->rA, P);
				vA = b2MulAdd(vA, -mA, P);
				wB += iB * b2Cross(cp->rB, P);
				vB = b2MulAdd(vB, mB, P);
			}
		}

		if (solverBodyA != NULL)
		{
			solverBodyA->linearVelocity = vA;
			solverBodyA->angularVelocity = wA;
		}
		solverBodyB->linearVelocity = vB;
		solverBodyB->angularVelocity = wB;
	}

	b2TracyCZoneEnd(prepare_contact);
}

static void b2SolveJointsTask(b2SolverTaskContext* context, bool useBias)
{
	b2World* world = context->world;
	b2Joint* joints = world->joints;
	int32_t jointCapacity = world->jointPool.capacity;
	const b2StepContext* stepContext = context->stepContext;

	for (int32_t i = 0; i < jointCapacity; ++i)
	{
		b2Joint* joint = joints + i;
		if (b2ObjectValid(&joint->object) == false)
		{
			continue;
		}

		b2SolveJointVelocitySoft(joint, stepContext, useBias);
	}
}

static void b2SolveContactsTask(int32_t startIndex, int32_t endIndex, b2SolverTaskContext* context, int32_t colorIndex, bool useBias)
{
	b2TracyCZoneNC(solve_contact, "Solve Contact", b2_colorAliceBlue, true);

	b2Graph* graph = context->graph;
	b2GraphColor* color = graph->colors + colorIndex;
	b2SolverBody* bodies = context->solverBodies;
	b2Constraint* constraints = color->contacts;

	float inv_dt = context->invTimeStep;

	B2_ASSERT(startIndex <= endIndex);
	B2_ASSERT(startIndex <= b2Array(color->contactArray).count);
	B2_ASSERT(endIndex <= b2Array(color->contactArray).count);

	for (int32_t i = startIndex; i < endIndex; ++i)
	{
		b2Constraint* constraint = constraints + i;

		int32_t indexA = constraint->indexA;
		b2SolverBody* bodyA;
		b2Vec2 vA;
		float wA;
		float mA;
		float iA;
		b2Vec2 dpA;
		float daA;

		if (indexA != B2_NULL_INDEX)
		{
			bodyA = bodies + indexA;
			vA = bodyA->linearVelocity;
			wA = bodyA->angularVelocity;
			dpA = bodyA->deltaPosition;
			daA = bodyA->deltaAngle;
			mA = bodyA->invMass;
			iA = bodyA->invI;
		}
		else
		{
			bodyA = NULL;
			vA = b2Vec2_zero;
			wA = 0.0f;
			dpA = b2Vec2_zero;
			daA = 0.0f;
			mA = 0.0f;
			iA = 0.0f;
		}

		b2SolverBody* bodyB = bodies + constraint->indexB;
		b2Vec2 vB = bodyB->linearVelocity;
		float wB = bodyB->angularVelocity;
		b2Vec2 dpB = bodyB->deltaPosition;
		float daB = bodyB->deltaAngle;
		float mB = bodyB->invMass;
		float iB = bodyB->invI;

		int32_t pointCount = constraint->pointCount;
		b2Vec2 normal = constraint->normal;
		b2Vec2 tangent = b2RightPerp(normal);
		float friction = constraint->friction;
		float biasCoefficient = constraint->biasCoefficient;
		float massCoefficient = constraint->massCoefficient;
		float impulseCoefficient = constraint->impulseCoefficient;

		for (int32_t j = 0; j < pointCount; ++j)
		{
			b2ConstraintPoint* cp = constraint->points + j;

			// Relative velocity at contact
			b2Vec2 vrB = b2Add(vB, b2CrossSV(wB, cp->rB));
			b2Vec2 vrA = b2Add(vA, b2CrossSV(wA, cp->rA));
			b2Vec2 dv = b2Sub(vrB, vrA);

			// Compute change in separation (small angle approximation of sin(angle) == angle)
			b2Vec2 prB = b2Add(dpB, b2CrossSV(daB, cp->rB));
			b2Vec2 prA = b2Add(dpA, b2CrossSV(daA, cp->rA));
			float ds = b2Dot(b2Sub(prB, prA), normal);
			float s = cp->separation + ds;
			float bias = 0.0f;
			float massScale = 1.0f;
			float impulseScale = 0.0f;
			if (s > 0.0f)
			{
				// TODO_ERIN what time to use?
				// Speculative (inverse of full time step)
				bias = s * inv_dt;
			}
			else if (useBias)
			{
				bias = B2_MAX(biasCoefficient * s, -maxBaumgarteVelocity);
				// bias = cp->biasCoefficient * s;
				massScale = massCoefficient;
				impulseScale = impulseCoefficient;
			}

			// Compute normal impulse
			float vn = b2Dot(dv, normal);
			float impulse = -cp->normalMass * massScale * (vn + bias) - impulseScale * cp->normalImpulse;
			// float impulse = -cp->normalMass * (vn + bias + cp->gamma * cp->normalImpulse);

			// Clamp the accumulated impulse
			float newImpulse = B2_MAX(cp->normalImpulse + impulse, 0.0f);
			impulse = newImpulse - cp->normalImpulse;
			cp->normalImpulse = newImpulse;

			// Apply contact impulse
			b2Vec2 P = b2MulSV(impulse, normal);
			vA = b2MulSub(vA, mA, P);
			wA -= iA * b2Cross(cp->rA, P);

			vB = b2MulAdd(vB, mB, P);
			wB += iB * b2Cross(cp->rB, P);
		}

		for (int32_t j = 0; j < pointCount; ++j)
		{
			b2ConstraintPoint* cp = constraint->points + j;

			// Relative velocity at contact
			b2Vec2 vrB = b2Add(vB, b2CrossSV(wB, cp->rB));
			b2Vec2 vrA = b2Add(vA, b2CrossSV(wA, cp->rA));
			b2Vec2 dv = b2Sub(vrB, vrA);

			// Compute tangent force
			float vt = b2Dot(dv, tangent);
			float lambda = cp->tangentMass * (-vt);

			// Clamp the accumulated force
			float maxFriction = friction * cp->normalImpulse;
			float newImpulse = B2_CLAMP(cp->tangentImpulse + lambda, -maxFriction, maxFriction);
			lambda = newImpulse - cp->tangentImpulse;
			cp->tangentImpulse = newImpulse;

			// Apply contact impulse
			b2Vec2 P = b2MulSV(lambda, tangent);

			vA = b2MulSub(vA, mA, P);
			wA -= iA * b2Cross(cp->rA, P);

			vB = b2MulAdd(vB, mB, P);
			wB += iB * b2Cross(cp->rB, P);
		}

		if (bodyA != NULL)
		{
			bodyA->linearVelocity = vA;
			bodyA->angularVelocity = wA;
		}
		bodyB->linearVelocity = vB;
		bodyB->angularVelocity = wB;
	}

	b2TracyCZoneEnd(solve_contact);
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

static void b2FinalizePositionsTask(int32_t startIndex, int32_t endIndex, b2SolverTaskContext* context, uint32_t threadIndex)
{
	b2TracyCZoneNC(finalize_positions, "FinPos", b2_colorViolet, true);

	b2World* world = context->world;
	b2Body* bodies = world->bodies;
	b2SolverBody* solverBodies = context->solverBodies;
	b2Contact* contacts = world->contacts;
	const b2Vec2 aabbMargin = {b2_aabbMargin, b2_aabbMargin};

	b2BitSet* awakeContactBitSet = &world->taskContextArray[threadIndex].awakeContactBitSet;
	b2BitSet* shapeBitSet = &world->taskContextArray[threadIndex].shapeBitSet;

	B2_ASSERT(startIndex <= endIndex);
	B2_ASSERT(startIndex <= world->bodyPool.capacity);
	B2_ASSERT(endIndex <= world->bodyPool.capacity);

	for (int32_t i = startIndex; i < endIndex; ++i)
	{
		b2SolverBody* solverBody = solverBodies + i;

		b2Body* body = bodies + solverBody->bodyIndex;
		B2_ASSERT(body->solverIndex == i);

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

static void b2StoreImpulsesTask(int32_t startIndex, int32_t endIndex, b2SolverTaskContext* context)
{
	b2TracyCZoneNC(store_impulses, "Store", b2_colorFirebrick, true);

	b2Constraint* constraints = context->constraints;

	for (int32_t i = startIndex; i < endIndex; ++i)
	{
		b2Constraint* constraint = constraints + i;
		b2Contact* contact = constraint->contact;

		b2Manifold* manifold = &contact->manifold;

		for (int32_t j = 0; j < constraint->pointCount; ++j)
		{
			manifold->points[j].normalImpulse = constraint->points[j].normalImpulse;
			manifold->points[j].tangentImpulse = constraint->points[j].tangentImpulse;
		}
	}

	b2TracyCZoneEnd(store_impulses);
}

static void b2ExecuteBlock(b2SolverStage* stage, b2SolverTaskContext* context, int32_t startIndex, int32_t endIndex, uint32_t threadIndex)
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
			b2FinalizePositionsTask(startIndex, endIndex, context, threadIndex);
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

static void b2ExecuteStage(b2SolverStage* stage, b2SolverTaskContext* context, int32_t workerIndex, int previousSyncIndex, int syncIndex,
						   uint32_t threadIndex)
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
		B2_ASSERT(completedCount < blockCount);

		b2ExecuteBlock(stage, context, blocks[blockIndex].startIndex, blocks[blockIndex].endIndex, threadIndex);

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

		b2ExecuteBlock(stage, context, blocks[blockIndex].startIndex, blocks[blockIndex].endIndex, threadIndex);
		completedCount += 1;
		blockIndex -= 1;
	}

	(void)atomic_fetch_add(&stage->completionCount, completedCount);
}

static void b2ExecuteMainStage(b2SolverStage* stage, b2SolverTaskContext* context, int32_t workerIndex, uint32_t syncBits)
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

		b2ExecuteStage(stage, context, workerIndex, previousSyncIndex, syncIndex, 0);

		while (atomic_load(&stage->completionCount) != blockCount)
		{
			_mm_pause();
		}

		atomic_store(&stage->completionCount, 0);
	}
}

void b2SolverTask(int32_t startIndex, int32_t endIndex, uint32_t threadIndex, void* taskContext)
{
	B2_MAYBE_UNUSED(startIndex);
	B2_MAYBE_UNUSED(endIndex);

	b2WorkerContext* workerContext = taskContext;
	int32_t workerIndex = workerContext->workerIndex;
	b2SolverTaskContext* context = workerContext->context;
	int32_t activeColorCount = context->activeColorCount;
	b2SolverStage* stages = context->stages;

	if (threadIndex == 0)
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
				if (stages[3].blocks[i].syncIndex > 0)
				{
					i += 0;
				}
			}
		}

		int32_t bodySyncIndex = 1;
		int32_t stageIndex = 0;
		uint32_t syncBits = (bodySyncIndex << 16) | stageIndex;
		B2_ASSERT(stages[stageIndex].type == b2_stageIntegrateVelocities);
		b2ExecuteMainStage(stages + stageIndex, context, workerIndex, syncBits);
		stageIndex += 1;
		bodySyncIndex += 1;

		if (stages[3].type == b2_stagePrepareContacts)
		{
			for (int32_t i = 0; i < stages[3].blockCount; ++i)
			{
				if (stages[3].blocks[i].syncIndex > 0)
				{
					i += 0;
				}
			}
		}

		// TODO_ERIN single threaded
		B2_ASSERT(stages[stageIndex].type == b2_stagePrepareJoints);
		b2PrepareJointsTask(context);
		stageIndex += 1;

		if (stages[3].type == b2_stagePrepareContacts)
		{
			for (int32_t i = 0; i < stages[3].blockCount; ++i)
			{
				if (stages[3].blocks[i].syncIndex > 0)
				{
					i += 0;
				}
			}
		}

		int32_t graphSyncIndex = 1;
		for (int32_t colorIndex = 0; colorIndex < activeColorCount; ++colorIndex)
		{
			syncBits = (graphSyncIndex << 16) | stageIndex;
			B2_ASSERT(stages[stageIndex].type == b2_stagePrepareContacts);
			for (int32_t i = 0; i < stages[stageIndex].blockCount; ++i)
			{
				if (stages[stageIndex].blocks[i].syncIndex > 0)
				{
					i += 0;
				}
			}
			b2ExecuteMainStage(stages + stageIndex, context, workerIndex, syncBits);
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
				b2ExecuteMainStage(stages + iterStageIndex, context, workerIndex, syncBits);
				iterStageIndex += 1;
			}
			graphSyncIndex += 1;

			B2_ASSERT(stages[iterStageIndex].type == b2_stageIntegratePositions);
			syncBits = (bodySyncIndex << 16) | iterStageIndex;
			b2ExecuteMainStage(stages + iterStageIndex, context, workerIndex, syncBits);
			bodySyncIndex += 1;
		}

		stageIndex += 1 + activeColorCount + 1;

		syncBits = (bodySyncIndex << 16) | stageIndex;
		B2_ASSERT(stages[stageIndex].type == b2_stageFinalizePositions);
		b2ExecuteMainStage(stages + stageIndex, context, workerIndex, syncBits);
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
				b2ExecuteMainStage(stages + iterStageIndex, context, workerIndex, syncBits);
				iterStageIndex += 1;
			}
			graphSyncIndex += 1;
		}

		stageIndex += 1 + activeColorCount;

		uint32_t constraintSyncIndex = 1;
		syncBits = (constraintSyncIndex << 16) | stageIndex;
		B2_ASSERT(stages[stageIndex].type == b2_stageStoreImpulses);
		b2ExecuteMainStage(stages + stageIndex, context, workerIndex, syncBits);

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
		b2ExecuteStage(stage, context, workerIndex, previousSyncIndex, syncIndex, threadIndex);

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
void b2SolveGraph(b2World* world, const b2StepContext* stepContext)
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

			awakeBodies[index] = body;

			// cache miss
			bodyIndex = body->islandNext;
			body->solverIndex = index;

			index += 1;
		}
	}
	B2_ASSERT(index == awakeBodyCount);

	int32_t workerCount = world->workerCount;
	const int32_t blocksPerWorker = 6;

	int32_t bodyBlockSize = 1 << 3;
	int32_t bodyBlockCount = ((awakeBodyCount - 1) >> 3) + 1;
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

	b2Constraint* constraints = b2AllocateStackItem(world->stackAllocator, constraintCount * sizeof(b2Constraint), "constraint");
	int32_t base = 0;

	for (int32_t i = 0; i < activeColorCount; ++i)
	{
		int32_t j = activeColorIndices[i];
		colors[j].contacts = constraints + base;
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
	//b2SolverBlock* graphBlocks = b2AllocateStackItem(world->stackAllocator, graphBlockCount * sizeof(b2SolverBlock), "graph blocks");
	
	B2_ASSERT(graphBlockCount <= 32);
	static b2SolverBlock graphBlocks[32];

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

	for (int32_t i = 0; i < graphBlockCount; ++i)
	{
		if (graphBlocks[i].syncIndex > 0)
		{
			i += 0;
		}
	}

	B2_ASSERT((int32_t)(stage - stages) == stageCount);

	B2_ASSERT(workerCount <= 16);
	b2WorkerContext workerContext[16];

	int32_t velIters = B2_MAX(1, stepContext->velocityIterations);

	b2SolverTaskContext context;
	context.stepContext = stepContext;
	context.world = world;
	context.awakeBodies = awakeBodies;
	context.solverBodies = solverBodies;
	context.graph = graph;
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
	atomic_store(&context.syncBits, 0);

	b2TracyCZoneEnd(prepare_stages);

	for (int32_t i = 0; i < graphBlockCount; ++i)
	{
		if (graphBlocks[i].syncIndex > 0)
		{
			i += 0;
		}
	}

	// TODO_ERIN use workerIndex or threadIndex?
	for (int32_t i = 0; i < workerCount; ++i)
	{
		for (int32_t j = 0; j < graphBlockCount; ++j)
		{
			if (graphBlocks[j].syncIndex > 0)
			{
				j += 0;
			}
		}

		workerContext[i].context = &context;
		workerContext[i].workerIndex = i;
		world->enqueueTaskFcn(b2SolverTask, 1, 1, workerContext + i, world->userTaskContext);
	}

	for (int32_t i = 0; i < graphBlockCount; ++i)
	{
		if (graphBlocks[i].syncIndex > 0)
		{
			i += 0;
		}
	}

	world->finishAllTasksFcn(world->userTaskContext);

	b2FreeStackItem(world->stackAllocator, storeBlocks);
	//b2FreeStackItem(world->stackAllocator, graphBlocks);
	b2FreeStackItem(world->stackAllocator, bodyBlocks);
	b2FreeStackItem(world->stackAllocator, stages);
	b2FreeStackItem(world->stackAllocator, constraints);
	b2FreeStackItem(world->stackAllocator, solverBodies);
	b2FreeStackItem(world->stackAllocator, awakeBodies);
}
