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

#include <stdbool.h>
//#include <stdlib.h>

#define maxBaumgarteVelocity 3.0f

typedef enum b2SolverStage
{
	b2_stageIntegrateVelocities = 0,
	b2_stagePrepareContacts,
	b2_stagePrepareJoints,
	b2_stageSolveJoints,
	b2_stageSolveContacts,
	b2_stageIntegratePositions,
	b2_stageCalmJoints,
	b2_stageCalmContacts,
	b2_stageFinalizePositions,
	b2_stageStoreImpulses
} b2SolverStage;

typedef struct b2SolverTaskEntry
{
	uint16_t startIndex;
	uint16_t endIndex;

	// b2SolverStage
	uint8_t stage;
	uint8_t color;
} b2SolverTaskEntry;

typedef struct b2SolverTaskContext
{
	b2World* world;
	b2Body** awakeBodies;
	b2Graph* graph;

	b2SolverTaskEntry* taskEntries;
	int32_t taskCount;
	int32_t* segmentIndices;

	_Atomic int startIndex;
	_Atomic int endIndex;
	_Atomic int completionCount;
} b2SolverTaskContext;

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

	graph->solverTaskEntries = b2CreateArray(sizeof(b2SolverTaskEntry), 32);
}

void b2DestroyGraph(b2Graph* graph)
{
	for (int32_t i = 0; i < b2_graphColorCount; ++i)
	{
		b2GraphColor* color = graph->colors + i;
		b2DestroyBitSet(&color->bodySet);
		b2DestroyArray(color->contactArray, sizeof(int32_t));
	}

	b2DestroyArray(graph->solverTaskEntries, sizeof(b2SolverTaskEntry));
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

static void b2IntegrateVelocities2(b2World* world, b2Body** bodies, int32_t bodyCount, float h)
{
	b2Vec2 gravity = world->gravity;

	// Integrate velocities and apply damping. Initialize the body state.
	for (int32_t i = 0; i < bodyCount; ++i)
	{
		b2Body* body = bodies[i];

		if (body->type != b2_dynamicBody)
		{
			continue;
		}

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

		body->linearVelocity = v;
		body->angularVelocity = w;

		body->deltaAngle = 0.0f;
		body->deltaPosition = b2Vec2_zero;
	}
}

typedef struct b2BodyContext
{
	b2World* world;
	b2Body** bodies;
	float h;
} b2BodyContext;

static void b2UpdateDeltasTask(int32_t startIndex, int32_t endIndex, uint32_t threadIndex, void* taskContext)
{
	B2_MAYBE_UNUSED(threadIndex);

	b2TracyCZoneNC(update_deltas, "Deltas", b2_colorDarkSeaGreen, true);

	b2BodyContext* bodyContext = taskContext;
	b2Body** bodies = bodyContext->bodies;
	float h = bodyContext->h;

	B2_ASSERT(startIndex <= endIndex);

	for (int32_t i = startIndex; i < endIndex; ++i)
	{
		b2Body* body = bodies[i];

		body->deltaAngle += h * body->angularVelocity;
		body->deltaPosition = b2MulAdd(body->deltaPosition, h, body->linearVelocity);
	}

	b2TracyCZoneEnd(update_deltas);
}

static void b2UpdateDeltas(b2World* world, b2Body** bodies, int32_t count, float h)
{
	if (count == 0)
	{
		return;
	}

	b2BodyContext context = {world, bodies, h};

	int32_t minRange = 128;
	if (count < minRange)
	{
		b2UpdateDeltasTask(0, count, 0, &context);
	}
	else
	{
		void* userTask = world->enqueueTaskFcn(&b2UpdateDeltasTask, count, minRange, &context, world->userTaskContext);
		world->finishTaskFcn(userTask, world->userTaskContext);
	}
}

static void b2UpdatePositionsTask(int32_t startIndex, int32_t endIndex, uint32_t threadIndex, void* taskContext)
{
	b2TracyCZoneNC(update_positions, "Positions", b2_colorViolet, true);

	b2BodyContext* bodyContext = taskContext;
	b2World* world = bodyContext->world;
	b2Body** bodies = bodyContext->bodies;
	b2Contact* contacts = world->contacts;
	const b2Vec2 aabbMargin = {b2_aabbMargin, b2_aabbMargin};
	float h = bodyContext->h;

	b2BitSet* awakeContactBitSet = &world->taskContextArray[threadIndex].awakeContactBitSet;
	b2BitSet* shapeBitSet = &world->taskContextArray[threadIndex].shapeBitSet;

	B2_ASSERT(startIndex <= endIndex);
	B2_ASSERT(startIndex <= world->bodyPool.capacity);
	B2_ASSERT(endIndex <= world->bodyPool.capacity);

	for (int32_t i = startIndex; i < endIndex; ++i)
	{
		b2Body* body = bodies[i];

		// Final substep
		body->deltaAngle += h * body->angularVelocity;
		body->deltaPosition = b2MulAdd(body->deltaPosition, h, body->linearVelocity);

		body->position = b2Add(body->position, body->deltaPosition);
		body->angle += body->deltaAngle;

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

	b2TracyCZoneEnd(update_positions);
}

static void b2UpdatePositions(b2World* world, b2Body** bodies, int32_t count, float h)
{
	if (count == 0)
	{
		return;
	}

	b2BodyContext context = {world, bodies, h};

	int32_t minRange = 32;
	if (count < minRange)
	{
		b2UpdatePositionsTask(0, count, 0, &context);
	}
	else
	{
		void* userTask = world->enqueueTaskFcn(&b2UpdatePositionsTask, count, minRange, &context, world->userTaskContext);
		world->finishTaskFcn(userTask, world->userTaskContext);
	}
}

typedef struct b2ConstraintPoint
{
	b2Vec2 rA, rB;
	b2Vec2 rAf, rBf;
	b2Vec2 localAnchorA, localAnchorB;
	float tangentSeparation;
	float separation;
	float normalImpulse;
	float tangentImpulse;
	float normalMass;
	float tangentMass;
	float gamma;
	float massCoefficient;
	float biasCoefficient;
	float impulseCoefficient;
	float baumgarte;
	bool frictionValid;
} b2ConstraintPoint;

typedef struct b2Constraint
{
	b2Contact* contact;
	int32_t indexA;
	int32_t indexB;
	b2ConstraintPoint points[2];
	b2Vec2 normal;
	float friction;
	int32_t pointCount;
} b2Constraint;

typedef struct b2GraphContext
{
	b2World* world;
	b2GraphColor* color;
	float timeStep;
	float contactHertz;
	bool enableWarmStarting;
} b2GraphContext;

static void b2PrepareSoftContactTask(int32_t startIndex, int32_t endIndex, uint32_t threadIndex, void* taskContext)
{
	B2_MAYBE_UNUSED(threadIndex);

	b2TracyCZoneNC(prepare_contact, "Prepare Contact", b2_colorYellow, true);

	b2GraphContext* graphContext = taskContext;
	b2GraphColor* color = graphContext->color;
	int32_t* contactIndices = color->contactArray;
	b2Contact* contacts = graphContext->world->contacts;
	b2Body* bodies = graphContext->world->bodies;

	float contactHertz = graphContext->contactHertz;
	float h = graphContext->timeStep;
	bool enableWarmStarting = graphContext->enableWarmStarting;

	B2_ASSERT(startIndex <= endIndex);
	B2_ASSERT(startIndex <= b2Array(color->contactArray).count);
	B2_ASSERT(endIndex <= b2Array(color->contactArray).count);

	for (int32_t i = startIndex; i < endIndex; ++i)
	{
		b2Contact* contact = contacts + contactIndices[i];

		const b2Manifold* manifold = &contact->manifold;
		int32_t pointCount = manifold->pointCount;

		B2_ASSERT(0 < pointCount && pointCount <= 2);

		int32_t indexA = contact->edges[0].bodyIndex;
		int32_t indexB = contact->edges[1].bodyIndex;
		b2Body* bodyA = bodies + indexA;
		b2Body* bodyB = bodies + indexB;

		b2Constraint* constraint = color->contacts + i;
		constraint->contact = contact;
		constraint->indexA = indexA;
		constraint->indexB = indexB;
		constraint->normal = manifold->normal;
		constraint->friction = contact->friction;
		constraint->pointCount = pointCount;

		float mA = bodyA->invMass;
		float iA = bodyA->invI;
		float mB = bodyB->invMass;
		float iB = bodyB->invI;

		b2Vec2 cA = bodyA->position;
		b2Vec2 cB = bodyB->position;
		b2Rot qA = b2MakeRot(bodyA->angle);
		b2Rot qB = b2MakeRot(bodyB->angle);

		b2Vec2 vA = bodyA->linearVelocity;
		float wA = bodyA->angularVelocity;
		b2Vec2 vB = bodyB->linearVelocity;
		float wB = bodyB->angularVelocity;

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

			// Stiffer for static contacts to avoid bodies getting pushed through the ground
			const float hertz = mA == 0.0f ? 2.0f * contactHertz : contactHertz;
			const float zeta = 1.0f;
			float omega = 2.0f * b2_pi * hertz;
			// float d = 2.0f * zeta * omega / kNormal;
			// float k = omega * omega / kNormal;

			// cp->gamma = 1.0f / (h * (d + h * k));
			// cp->gamma = 1.0f / (h * (2.0f * zeta * omega / kNormal + h * omega * omega / kNormal));
			cp->gamma = kNormal / (h * omega * (2.0f * zeta + h * omega));

			cp->separation = mp->separation;

			// cp->bias = h * k * cp->gamma * mp->separation;
			// cp->bias = k / (d + h * k) * mp->separation;
			// cp->bias =
			//	(omega * omega / kNormal) / (2 * zeta * omega / kNormal + h * omega * omega / kNormal) * mp->separation;
			cp->biasCoefficient = omega / (2.0f * zeta + h * omega);
			// cp->gamma = 0.0f;
			// cp->bias = (0.2f / h) * mp->separation;

			// TODO_ERIN this can be expanded
			cp->normalMass = kNormal > 0.0f ? 1.0f / kNormal : 0.0f;
			// cp->normalMass = 1.0f / (kNormal + cp->gamma);

			float c = h * omega * (2.0f * zeta + h * omega);
			cp->impulseCoefficient = 1.0f / (1.0f + c);
			cp->massCoefficient = c * cp->impulseCoefficient;

			// meff = 1.0f / kNormal * 1.0f / (1.0f + 1.0f / (h * omega * (2 * zeta + h * omega)))
			// float impulse = -cp->normalMass * (vn + bias + cp->gamma * cp->normalImpulse);
			// = -meff * mscale * (vn + bias) - imp_scale * impulse

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

		bodyA->linearVelocity = vA;
		bodyA->angularVelocity = wA;
		bodyB->linearVelocity = vB;
		bodyB->angularVelocity = wB;
	}

	b2TracyCZoneEnd(prepare_contact);
}

// h is full time step
static void b2PrepareSoftContact(b2World* world, b2GraphColor* color, float h, float contactHertz, bool warmStart)
{
	int32_t count = b2Array(color->contactArray).count;
	if (count == 0)
	{
		return;
	}

	b2GraphContext context = {world, color, h, contactHertz, warmStart};

	int32_t minRange = 64;
	if (count < minRange)
	{
		b2PrepareSoftContactTask(0, count, 0, &context);
	}
	else
	{
		void* userPrepareTask = world->enqueueTaskFcn(&b2PrepareSoftContactTask, count, minRange, &context, world->userTaskContext);
		world->finishTaskFcn(userPrepareTask, world->userTaskContext);
	}
}

typedef struct b2ContactContext
{
	b2World* world;
	b2GraphColor* color;
	float inv_dt;
	bool removeOverlap;
} b2ContactContext;

static void b2SolveContactTask(int32_t startIndex, int32_t endIndex, uint32_t threadIndex, void* taskContext)
{
	B2_MAYBE_UNUSED(threadIndex);
	b2TracyCZoneNC(solve_contact, "Solve Contact", b2_colorAliceBlue, true);

	b2ContactContext* contactContext = taskContext;
	b2Body* bodies = contactContext->world->bodies;
	b2Constraint* constraints = contactContext->color->contacts;

	float inv_dt = contactContext->inv_dt;
	bool removeOverlap = contactContext->removeOverlap;

	B2_ASSERT(startIndex <= endIndex);
	B2_ASSERT(startIndex <= b2Array(contactContext->color->contactArray).count);
	B2_ASSERT(endIndex <= b2Array(contactContext->color->contactArray).count);

	for (int32_t i = startIndex; i < endIndex; ++i)
	{
		b2Constraint* constraint = constraints + i;

		b2Body* bodyA = bodies + constraint->indexA;
		b2Body* bodyB = bodies + constraint->indexB;

		float mA = bodyA->invMass;
		float iA = bodyA->invI;
		float mB = bodyB->invMass;
		float iB = bodyB->invI;
		int32_t pointCount = constraint->pointCount;

		b2Vec2 vA = bodyA->linearVelocity;
		float wA = bodyA->angularVelocity;
		b2Vec2 vB = bodyB->linearVelocity;
		float wB = bodyB->angularVelocity;

		const b2Vec2 dpA = bodyA->deltaPosition;
		const float daA = bodyA->deltaAngle;
		const b2Vec2 dpB = bodyB->deltaPosition;
		const float daB = bodyB->deltaAngle;

		b2Vec2 normal = constraint->normal;
		b2Vec2 tangent = b2RightPerp(normal);
		float friction = constraint->friction;

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
				// Speculative (inverse of full time step)
				bias = s * inv_dt;
			}
			else if (removeOverlap)
			{
				bias = B2_MAX(cp->biasCoefficient * s, -maxBaumgarteVelocity);
				// bias = cp->biasCoefficient * s;
				massScale = cp->massCoefficient;
				impulseScale = cp->impulseCoefficient;
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

		bodyA->linearVelocity = vA;
		bodyA->angularVelocity = wA;
		bodyB->linearVelocity = vB;
		bodyB->angularVelocity = wB;
	}

	b2TracyCZoneEnd(solve_contact);
}

// inv_dt is full time step inverse
static void b2SolveSoftContact(b2World* world, b2GraphColor* color, float inv_dt, bool removeOverlap)
{
	int32_t count = b2Array(color->contactArray).count;
	if (count == 0)
	{
		return;
	}

	b2ContactContext context = {world, color, inv_dt, removeOverlap};

	int32_t minRange = 128;
	if (count < minRange)
	{
		b2SolveContactTask(0, count, 0, &context);
	}
	else
	{
		void* userSolveTask = world->enqueueTaskFcn(&b2SolveContactTask, count, minRange, &context, world->userTaskContext);
		world->finishTaskFcn(userSolveTask, world->userTaskContext);
	}
}

static void b2StoreImpulses(b2Constraint* constraints, int32_t constraintCount)
{
	for (int32_t i = 0; i < constraintCount; ++i)
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
}

void b2SolverTask(int32_t startIndex, int32_t endIndex, uint32_t threadIndex, void* taskContext)
{
	B2_MAYBE_UNUSED(startIndex);
	B2_MAYBE_UNUSED(endIndex);

	b2SolverTaskContext* context = taskContext;
	B2_MAYBE_UNUSED(context);

	if (threadIndex == 0)
	{
		// Manage and execute tasks
	}
	else
	{
		// Execute tasks
	}
}

void b2SolveGraph(b2World* world, , const b2StepContext* stepContext)
{
	b2Graph* graph = &world->graph;
	b2GraphColor* colors = graph->colors;
	b2Joint* joints = world->joints;

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
	b2Body** awakeBodies = b2AllocateStackItem(world->stackAllocator, awakeBodyCount * sizeof(b2Body*), "body pointers");
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

			awakeBodies[index++] = body;
			bodyIndex = body->islandNext;
		}
	}

	int32_t bodyBlockSize = 1 << 6;
	int32_t bodyTaskCount = ((awakeBodyCount - 1) >> 6) + 1;

	B2_ASSERT(index == awakeBodyCount);

	int32_t perColorTaskCount[b2_graphColorCount];

	int32_t contactBlockSize = 1 << 5;
	int32_t constraintTaskCount = 0;
	int32_t constraintCount = 0;
	for (int32_t i = 0; i < b2_graphColorCount; ++i)
	{
		int32_t count = b2Array(colors[i].contactArray).count;
		perColorTaskCount[i] = count > 0 ? ((count - 1) >> 5) + 1 : 0;
		constraintTaskCount += perColorTaskCount[i];
		constraintCount += count;
	}

	b2Constraint* constraints = b2AllocateStackItem(world->stackAllocator, constraintCount * sizeof(b2Constraint), "constraint");
	int32_t base = 0;

	for (int32_t i = 0; i < b2_graphColorCount; ++i)
	{
		colors[i].contacts = constraints + base;
		base += b2Array(colors[i].contactArray).count;
	}

	int32_t storeBlockSize = 1 << 6;
	int32_t storeTaskCount = constraintCount > 0 ? ((constraintCount - 1) >> 6) + 1 : 0;
	int32_t velIters = stepContext->velocityIterations;
	int32_t posIters = stepContext->positionIterations;

	// TODO_ERIN joint tasks
	int32_t taskCount = bodyTaskCount + constraintTaskCount + velIters * (constraintTaskCount + bodyTaskCount) +
						posIters * (constraintTaskCount) + bodyTaskCount + storeTaskCount;
	
	b2SolverTaskEntry* entries = b2AllocateStackItem(world->stackAllocator, taskCount * sizeof(b2SolverTaskEntry), "task entries");

	int32_t taskIndex = 0;
	for (int32_t i = 0; i < bodyTaskCount; ++i)
	{
		int32_t startIndex = i * bodyBlockSize;
		int32_t endIndex = B2_MIN(startIndex + bodyBlockSize, awakeBodyCount);
		entries[taskIndex++] = (b2SolverTaskEntry){startIndex, endIndex, b2_stageIntegrateVelocities, 0xFF};
	}

	for (int32_t i = 0; i < b2_graphColorCount; ++i)
	{
		int32_t colorConstraintCount = b2Array(colors[i].contactArray).count;
		int32_t colorTaskCount = perColorTaskCount[i];

		for (int32_t j = 0; j < colorTaskCount; ++j)
		{
			int32_t startIndex = j * contactBlockSize;
			int32_t endIndex = B2_MIN(startIndex + contactBlockSize, colorConstraintCount);
			entries[taskIndex++] = (b2SolverTaskEntry){startIndex, endIndex, b2_stagePrepareContacts, (uint8_t)i};
		}
	}
	
	for (int32_t iter = 0; iter < velIters; ++iter)
	{
		for (int32_t i = 0; i < b2_graphColorCount; ++i)
		{
			int32_t colorConstraintCount = b2Array(colors[i].contactArray).count;
			int32_t colorTaskCount = perColorTaskCount[i];

			for (int32_t j = 0; j < colorTaskCount; ++j)
			{
				int32_t startIndex = j * contactBlockSize;
				int32_t endIndex = B2_MIN(startIndex + contactBlockSize, colorConstraintCount);
				entries[taskIndex++] = (b2SolverTaskEntry){startIndex, endIndex, b2_stageSolveContacts, (uint8_t)i};
			}
		}
	
		for (int32_t i = 0; i < bodyTaskCount; ++i)
		{
			int32_t startIndex = i * bodyBlockSize;
			int32_t endIndex = B2_MIN(startIndex + bodyBlockSize, awakeBodyCount);
			entries[taskIndex++] = (b2SolverTaskEntry){startIndex, endIndex, b2_stageIntegratePositions, 0xFF};
		}
	}
	
	for (int32_t iter = 0; iter < posIters; ++iter)
	{
		for (int32_t i = 0; i < b2_graphColorCount; ++i)
		{
			int32_t colorConstraintCount = b2Array(colors[i].contactArray).count;
			int32_t colorTaskCount = perColorTaskCount[i];

			for (int32_t j = 0; j < colorTaskCount; ++j)
			{
				int32_t startIndex = j * contactBlockSize;
				int32_t endIndex = B2_MIN(startIndex + contactBlockSize, colorConstraintCount);
				entries[taskIndex++] = (b2SolverTaskEntry){startIndex, endIndex, b2_stageCalmContacts, (uint8_t)i};
			}
		}
	}

	for (int32_t i = 0; i < bodyTaskCount; ++i)
	{
		int32_t startIndex = i * bodyBlockSize;
		int32_t endIndex = B2_MIN(startIndex + bodyBlockSize, awakeBodyCount);
		entries[taskIndex++] = (b2SolverTaskEntry){startIndex, endIndex, b2_stageFinalizePositions, 0xFF};
	}

	for (int32_t i = 0; i < storeTaskCount; ++i)
	{
		int32_t startIndex = i * storeBlockSize;
		int32_t endIndex = B2_MIN(startIndex + storeBlockSize, constraintCount);
		entries[taskIndex++] = (b2SolverTaskEntry){startIndex, endIndex, b2_stageStoreImpulses, 0xFF};
	}

	B2_ASSERT(taskIndex == taskCount);

	/*
	 typedef enum b2SolverStage
	{
		b2_stageIntegrateVelocities = 0,
		b2_stagePrepareContacts,
		b2_stagePrepareJoints,
		b2_stageSolveJoints,
		b2_stageSolveContacts,
		b2_stageIntegratePositions,
		b2_stageCalmJoints,
		b2_stageCalmContacts,
		b2_stageFinalizePositions,
		b2_stageStoreImpulses
	} b2SolverStage;
	*/

	b2SolverTaskContext context;
	context.world = world;
	context.awakeBodies = awakeBodies;
	context.graph = graph;
	context.taskCount = taskCount;
	context.taskEnties = entries;
	context.startIndex = 0;
	context.endIndex = 0;
	context.completionCount = 0;

	b2FreeStackItem(world->stackAllocator, entries);
	b2FreeStackItem(world->stackAllocator, constraints);
	b2FreeStackItem(world->stackAllocator, awakeBodies);
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
void b2SolveGraphSoftStep(b2World* world, const b2StepContext* stepContext)
{
	b2Graph* graph = &world->graph;
	b2GraphColor* colors = graph->colors;
	b2Joint* joints = world->joints;

	int32_t awakeIslandCount = b2Array(world->awakeIslandArray).count;
	int32_t awakeBodyCount = 0;
	for (int32_t i = 0; i < awakeIslandCount; ++i)
	{
		int32_t islandIndex = world->awakeIslandArray[i];
		b2Island* island = world->islands + islandIndex;
		awakeBodyCount += island->bodyCount;
	}

	b2Body* bodies = world->bodies;
	b2Body** awakeBodies = b2AllocateStackItem(world->stackAllocator, awakeBodyCount * sizeof(b2Body*), "body pointers");
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

			awakeBodies[index++] = body;
			bodyIndex = body->islandNext;
		}
	}

	B2_ASSERT(index == awakeBodyCount);

	int32_t constraintCount = 0;
	for (int32_t i = 0; i < b2_graphColorCount; ++i)
	{
		constraintCount += b2Array(colors[i].contactArray).count;
	}

	b2Constraint* constraints = b2AllocateStackItem(world->stackAllocator, constraintCount * sizeof(b2Constraint), "constraint");
	int32_t base = 0;

	for (int32_t i = 0; i < b2_graphColorCount; ++i)
	{
		colors[i].contacts = constraints + base;
		base += b2Array(colors[i].contactArray).count;
	}

	B2_ASSERT(base == constraintCount);

	// Full step apply gravity
	b2IntegrateVelocities2(world, awakeBodies, awakeBodyCount, stepContext->dt);

	// 30 is a bit soft, 60 oscillates too much
	// const float contactHertz = 45.0f;
	// const float contactHertz = B2_MAX(15.0f, stepContext->inv_dt * stepContext->velocityIterations / 8.0f);
	const float contactHertz = 30.0f;

	for (int32_t i = 0; i < b2_graphColorCount; ++i)
	{
		// Soft constraints initialized with full time step
		bool warmStart = stepContext->enableWarmStarting;
		b2PrepareSoftContact(world, colors + i, stepContext->dt, contactHertz, warmStart);
	}

	int32_t jointCapacity = world->jointPool.capacity;

	for (int32_t i = 0; i < jointCapacity; ++i)
	{
		b2Joint* joint = joints + i;
		if (b2ObjectValid(&joint->object) == false)
		{
			continue;
		}

		b2PrepareJoint(joint, stepContext);
	}

	int32_t substepCount = stepContext->velocityIterations;
	float h = stepContext->dt / substepCount;

	for (int32_t substep = 0; substep < substepCount; ++substep)
	{
		// One constraint iteration
		for (int32_t i = 0; i < jointCapacity; ++i)
		{
			b2Joint* joint = joints + i;
			if (b2ObjectValid(&joint->object) == false)
			{
				continue;
			}

			bool removeOverlap = true;
			b2SolveJointVelocitySoft(joint, stepContext, removeOverlap);
		}

		for (int32_t i = 0; i < b2_graphColorCount; ++i)
		{
			bool removeOverlap = true;
			b2SolveSoftContact(world, colors + i, h, removeOverlap);
		}

		if (substep < substepCount - 1)
		{
			b2UpdateDeltas(world, awakeBodies, awakeBodyCount, h);
		}
		else
		{
			b2UpdatePositions(world, awakeBodies, awakeBodyCount, h);
		}
	}

	int32_t positionIterations = stepContext->positionIterations;
	for (int32_t iter = 0; iter < positionIterations; ++iter)
	{
		for (int32_t i = 0; i < jointCapacity; ++i)
		{
			b2Joint* joint = joints + i;
			if (b2ObjectValid(&joint->object) == false)
			{
				continue;
			}

			bool removeOverlap = false;
			b2SolveJointVelocitySoft(joint, stepContext, removeOverlap);
		}

		for (int32_t i = 0; i < b2_graphColorCount; ++i)
		{
			bool removeOverlap = false;
			b2SolveSoftContact(world, colors + i, h, removeOverlap);
		}
	}

	b2StoreImpulses(constraints, constraintCount);

	b2FreeStackItem(world->stackAllocator, constraints);
	b2FreeStackItem(world->stackAllocator, awakeBodies);
}
