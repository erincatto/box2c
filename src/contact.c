// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "contact.h"

#include "array.h"
#include "block_allocator.h"
#include "body.h"
#include "core.h"
#include "island.h"
#include "shape.h"
#include "table.h"
#include "world.h"

#include "box2d/distance.h"
#include "box2d/manifold.h"
#include "box2d/timer.h"

#include <float.h>
#include <math.h>

// Contacts and determinism
// A deterministic simulation requires contacts to exist in the same order in b2Island no matter the thread count.
// The order must reproduce from run to run. This is necessary because the Gauss-Seidel constraint solver is order dependent.
//
// Creation:
// - Contacts are created using results from b2UpdateBroadPhasePairs
// - These results are ordered according to the order of the broad-phase move array
// - The move array is ordered according to the shape creation order using a bitset.
// - The island/shape/body order is determined by creation order
// - Logically contacts are only created for awake bodies, so they are immediately added to the awake contact array (serially)
//
// Island linking:
// - The awake contact array is built from the body-contact graph for all awake bodies in awake islands.
// - Awake contacts are solved in parallel and they generate contact state changes.
// - These state changes may link islands together using union find.
// - The state changes are ordered using a bit array that encompasses all contacts
// - As long as contacts are created in deterministic order, island link order is deterministic.
// - This keeps the order of contacts in islands deterministic

// Friction mixing law. The idea is to allow either shape to drive the friction to zero.
// For example, anything slides on ice.
static inline float b2MixFriction(float friction1, float friction2)
{
	return sqrtf(friction1 * friction2);
}

// Restitution mixing law. The idea is allow for anything to bounce off an inelastic surface.
// For example, a superball bounces on anything.
static inline float b2MixRestitution(float restitution1, float restitution2)
{
	return restitution1 > restitution2 ? restitution1 : restitution2;
}

typedef b2Manifold b2ManifoldFcn(const b2Shape* shapeA, b2Transform xfA, const b2Shape* shapeB, b2Transform xfB,
								 b2DistanceCache* cache);

struct b2ContactRegister
{
	b2ManifoldFcn* fcn;
	bool primary;
};

static struct b2ContactRegister s_registers[b2_shapeTypeCount][b2_shapeTypeCount];
static bool s_initialized = false;

static b2Manifold b2CircleManifold(const b2Shape* shapeA, b2Transform xfA, const b2Shape* shapeB, b2Transform xfB,
								   b2DistanceCache* cache)
{
	B2_MAYBE_UNUSED(cache);
	return b2CollideCircles(&shapeA->circle, xfA, &shapeB->circle, xfB);
}

static b2Manifold b2CapsuleAndCircleManifold(const b2Shape* shapeA, b2Transform xfA, const b2Shape* shapeB, b2Transform xfB,
											 b2DistanceCache* cache)
{
	B2_MAYBE_UNUSED(cache);
	return b2CollideCapsuleAndCircle(&shapeA->capsule, xfA, &shapeB->circle, xfB);
}

static b2Manifold b2CapsuleManifold(const b2Shape* shapeA, b2Transform xfA, const b2Shape* shapeB, b2Transform xfB,
									b2DistanceCache* cache)
{
	return b2CollideCapsules(&shapeA->capsule, xfA, &shapeB->capsule, xfB, cache);
}

static b2Manifold b2PolygonAndCircleManifold(const b2Shape* shapeA, b2Transform xfA, const b2Shape* shapeB, b2Transform xfB,
											 b2DistanceCache* cache)
{
	B2_MAYBE_UNUSED(cache);
	return b2CollidePolygonAndCircle(&shapeA->polygon, xfA, &shapeB->circle, xfB);
}

static b2Manifold b2PolygonAndCapsuleManifold(const b2Shape* shapeA, b2Transform xfA, const b2Shape* shapeB, b2Transform xfB,
											  b2DistanceCache* cache)
{
	return b2CollidePolygonAndCapsule(&shapeA->polygon, xfA, &shapeB->capsule, xfB, cache);
}

static b2Manifold b2PolygonManifold(const b2Shape* shapeA, b2Transform xfA, const b2Shape* shapeB, b2Transform xfB,
									b2DistanceCache* cache)
{
	return b2CollidePolygons(&shapeA->polygon, xfA, &shapeB->polygon, xfB, cache);
}

static b2Manifold b2SegmentAndCircleManifold(const b2Shape* shapeA, b2Transform xfA, const b2Shape* shapeB, b2Transform xfB,
											 b2DistanceCache* cache)
{
	B2_MAYBE_UNUSED(cache);
	return b2CollideSegmentAndCircle(&shapeA->segment, xfA, &shapeB->circle, xfB);
}

static b2Manifold b2SegmentAndCapsuleManifold(const b2Shape* shapeA, b2Transform xfA, const b2Shape* shapeB, b2Transform xfB,
											  b2DistanceCache* cache)
{
	return b2CollideSegmentAndCapsule(&shapeA->segment, xfA, &shapeB->capsule, xfB, cache);
}

static b2Manifold b2SegmentAndPolygonManifold(const b2Shape* shapeA, b2Transform xfA, const b2Shape* shapeB, b2Transform xfB,
											  b2DistanceCache* cache)
{
	return b2CollideSegmentAndPolygon(&shapeA->segment, xfA, &shapeB->polygon, xfB, cache);
}

static b2Manifold b2SmoothSegmentAndCircleManifold(const b2Shape* shapeA, b2Transform xfA, const b2Shape* shapeB, b2Transform xfB,
												   b2DistanceCache* cache)
{
	B2_MAYBE_UNUSED(cache);
	return b2CollideSmoothSegmentAndCircle(&shapeA->smoothSegment, xfA, &shapeB->circle, xfB);
}

static b2Manifold b2SmoothSegmentAndCapsuleManifold(const b2Shape* shapeA, b2Transform xfA, const b2Shape* shapeB, b2Transform xfB,
												   b2DistanceCache* cache)
{
	return b2CollideSmoothSegmentAndCapsule(&shapeA->smoothSegment, xfA, &shapeB->capsule, xfB, cache);
}

static b2Manifold b2SmoothSegmentAndPolygonManifold(const b2Shape* shapeA, b2Transform xfA, const b2Shape* shapeB,
													b2Transform xfB, b2DistanceCache* cache)
{
	return b2CollideSmoothSegmentAndPolygon(&shapeA->smoothSegment, xfA, &shapeB->polygon, xfB, cache);
}

static void b2AddType(b2ManifoldFcn* fcn, enum b2ShapeType type1, enum b2ShapeType type2)
{
	B2_ASSERT(0 <= type1 && type1 < b2_shapeTypeCount);
	B2_ASSERT(0 <= type2 && type2 < b2_shapeTypeCount);

	s_registers[type1][type2].fcn = fcn;
	s_registers[type1][type2].primary = true;

	if (type1 != type2)
	{
		s_registers[type2][type1].fcn = fcn;
		s_registers[type2][type1].primary = false;
	}
}

void b2InitializeContactRegisters(void)
{
	if (s_initialized == false)
	{
		b2AddType(b2CircleManifold, b2_circleShape, b2_circleShape);
		b2AddType(b2CapsuleAndCircleManifold, b2_capsuleShape, b2_circleShape);
		b2AddType(b2CapsuleManifold, b2_capsuleShape, b2_capsuleShape);
		b2AddType(b2PolygonAndCircleManifold, b2_polygonShape, b2_circleShape);
		b2AddType(b2PolygonAndCapsuleManifold, b2_polygonShape, b2_capsuleShape);
		b2AddType(b2PolygonManifold, b2_polygonShape, b2_polygonShape);
		b2AddType(b2SegmentAndCircleManifold, b2_segmentShape, b2_circleShape);
		b2AddType(b2SegmentAndCapsuleManifold, b2_segmentShape, b2_capsuleShape);
		b2AddType(b2SegmentAndPolygonManifold, b2_segmentShape, b2_polygonShape);
		b2AddType(b2SmoothSegmentAndCircleManifold, b2_smoothSegmentShape, b2_circleShape);
		b2AddType(b2SmoothSegmentAndCapsuleManifold, b2_smoothSegmentShape, b2_capsuleShape);
		b2AddType(b2SmoothSegmentAndPolygonManifold, b2_smoothSegmentShape, b2_polygonShape);
		s_initialized = true;
	}
}

void b2CreateContact(b2World* world, b2Shape* shapeA, b2Shape* shapeB)
{
	b2ShapeType type1 = shapeA->type;
	b2ShapeType type2 = shapeB->type;

	B2_ASSERT(0 <= type1 && type1 < b2_shapeTypeCount);
	B2_ASSERT(0 <= type2 && type2 < b2_shapeTypeCount);

	if (s_registers[type1][type2].fcn == NULL)
	{
		// For example, no segment vs segment collision
		return;
	}

	if (s_registers[type1][type2].primary == false)
	{
		// flip order
		b2CreateContact(world, shapeB, shapeA);
		return;
	}

	b2Contact* contact = (b2Contact*)b2AllocObject(&world->contactPool);
	world->contacts = (b2Contact*)world->contactPool.memory;

	int32_t contactIndex = contact->object.index;

	contact->flags = b2_contactEnabledFlag;

	if (shapeA->isSensor || shapeB->isSensor)
	{
		contact->flags |= b2_contactSensorFlag;
	}

	contact->shapeIndexA = shapeA->object.index;
	contact->shapeIndexB = shapeB->object.index;
	contact->cache = b2_emptyDistanceCache;
	contact->manifold = b2_emptyManifold;
	contact->friction = b2MixFriction(shapeA->friction, shapeB->friction);
	contact->restitution = b2MixRestitution(shapeA->restitution, shapeB->restitution);
	contact->tangentSpeed = 0.0f;
	contact->islandIndex = B2_NULL_INDEX;
	contact->islandPrev = B2_NULL_INDEX;
	contact->islandNext = B2_NULL_INDEX;
	contact->colorSubIndex = B2_NULL_INDEX;
	contact->colorIndex = B2_NULL_INDEX;
	contact->isMarked = false;

	b2Body* bodyA = world->bodies + shapeA->bodyIndex;
	b2Body* bodyB = world->bodies + shapeB->bodyIndex;

	// Connect to body A
	{
		contact->edges[0].bodyIndex = shapeA->bodyIndex;
		contact->edges[0].prevKey = B2_NULL_INDEX;
		contact->edges[0].nextKey = bodyA->contactList;

		int32_t keyA = (contactIndex << 1) | 0;
		if (bodyA->contactList != B2_NULL_INDEX)
		{
			b2Contact* contactA = world->contacts + (bodyA->contactList >> 1);
			b2ContactEdge* edgeA = contactA->edges + (bodyA->contactList & 1);
			edgeA->prevKey = keyA;
		}
		bodyA->contactList = keyA;
		bodyA->contactCount += 1;
	}

	// Connect to body B
	{
		contact->edges[1].bodyIndex = shapeB->bodyIndex;
		contact->edges[1].prevKey = B2_NULL_INDEX;
		contact->edges[1].nextKey = bodyB->contactList;

		int32_t keyB = (contactIndex << 1) | 1;
		if (bodyB->contactList != B2_NULL_INDEX)
		{
			b2Contact* contactB = world->contacts + (bodyB->contactList >> 1);
			b2ContactEdge* edgeB = contactB->edges + (bodyB->contactList & 1);
			edgeB->prevKey = keyB;
		}
		bodyB->contactList = keyB;
		bodyB->contactCount += 1;
	}

	// A contact should only be created from an awake body
	B2_ASSERT(b2IsBodyAwake(world, bodyA) || b2IsBodyAwake(world, bodyB));

	int32_t awakeIndex = b2Array(world->awakeContactArray).count;
	b2Array_Push(world->awakeContactArray, contactIndex);

	if (contactIndex == b2Array(world->contactAwakeIndexArray).count)
	{
		b2Array_Push(world->contactAwakeIndexArray, awakeIndex);
	}
	else
	{
		B2_ASSERT(contactIndex < b2Array(world->contactAwakeIndexArray).count);
		world->contactAwakeIndexArray[contactIndex] = awakeIndex;
	}

	// Add to pair set for fast lookup
	uint64_t pairKey = B2_SHAPE_PAIR_KEY(contact->shapeIndexA, contact->shapeIndexB);
	b2AddKey(&world->broadPhase.pairSet, pairKey);
}

void b2DestroyContact(b2World* world, b2Contact* contact)
{
	// Remove pair from set
	uint64_t pairKey = B2_SHAPE_PAIR_KEY(contact->shapeIndexA, contact->shapeIndexB);
	b2RemoveKey(&world->broadPhase.pairSet, pairKey);

	b2ContactEdge* edgeA = contact->edges + 0;
	b2ContactEdge* edgeB = contact->edges + 1;

	b2Body* bodyA = world->bodies + edgeA->bodyIndex;
	b2Body* bodyB = world->bodies + edgeB->bodyIndex;

	if (contact->colorIndex != B2_NULL_INDEX)
	{
		b2RemoveContactFromGraph(world, contact);
	}

	// if (contactListener && contact->IsTouching())
	//{
	//	contactListener->EndContact(contact);
	// }

	// Remove from body A
	if (edgeA->prevKey != B2_NULL_INDEX)
	{
		b2Contact* prevContact = world->contacts + (edgeA->prevKey >> 1);
		b2ContactEdge* prevEdge = prevContact->edges + (edgeA->prevKey & 1);
		prevEdge->nextKey = edgeA->nextKey;
	}

	if (edgeA->nextKey != B2_NULL_INDEX)
	{
		b2Contact* nextContact = world->contacts + (edgeA->nextKey >> 1);
		b2ContactEdge* nextEdge = nextContact->edges + (edgeA->nextKey & 1);
		nextEdge->prevKey = edgeA->prevKey;
	}

	int32_t edgeKeyA = (contact->object.index << 1) | 0;
	if (bodyA->contactList == edgeKeyA)
	{
		bodyA->contactList = edgeA->nextKey;
	}

	bodyA->contactCount -= 1;

	// Remove from body B
	if (edgeB->prevKey != B2_NULL_INDEX)
	{
		b2Contact* prevContact = world->contacts + (edgeB->prevKey >> 1);
		b2ContactEdge* prevEdge = prevContact->edges + (edgeB->prevKey & 1);
		prevEdge->nextKey = edgeB->nextKey;
	}

	if (edgeB->nextKey != B2_NULL_INDEX)
	{
		b2Contact* nextContact = world->contacts + (edgeB->nextKey >> 1);
		b2ContactEdge* nextEdge = nextContact->edges + (edgeB->nextKey & 1);
		nextEdge->prevKey = edgeB->prevKey;
	}

	int32_t contactIndex = contact->object.index;

	int32_t edgeKeyB = (contactIndex << 1) | 1;
	if (bodyB->contactList == edgeKeyB)
	{
		bodyB->contactList = edgeB->nextKey;
	}

	bodyB->contactCount -= 1;

	if (contact->islandIndex != B2_NULL_INDEX)
	{
		b2UnlinkContact(world, contact);
	}

	// Remove from awake contact array
	b2Array_Check(world->contactAwakeIndexArray, contactIndex);
	int32_t awakeIndex = world->contactAwakeIndexArray[contactIndex];
	if (awakeIndex != B2_NULL_INDEX)
	{
		B2_ASSERT(0 <= awakeIndex && awakeIndex < b2Array(world->awakeContactArray).count);
		world->awakeContactArray[awakeIndex] = B2_NULL_INDEX;
		world->contactAwakeIndexArray[contactIndex] = B2_NULL_INDEX;
	}

	b2FreeObject(&world->contactPool, &contact->object);
}

bool b2ShouldShapesCollide(b2Filter filterA, b2Filter filterB)
{
	if (filterA.groupIndex == filterB.groupIndex && filterA.groupIndex != 0)
	{
		return filterA.groupIndex > 0;
	}

	bool collide = (filterA.maskBits & filterB.categoryBits) != 0 && (filterA.categoryBits & filterB.maskBits) != 0;
	return collide;
}

static bool b2TestShapeOverlap(const b2Shape* shapeA, b2Transform xfA, const b2Shape* shapeB, b2Transform xfB)
{
	b2DistanceInput input;
	input.proxyA = b2MakeShapeDistanceProxy(shapeA);
	input.proxyB = b2MakeShapeDistanceProxy(shapeB);
	input.transformA = xfA;
	input.transformB = xfB;
	input.useRadii = true;

	b2DistanceCache cache = {0};
	b2DistanceOutput output = b2ShapeDistance(&cache, &input);

	return output.distance < 10.0f * FLT_EPSILON;
}

// Update the contact manifold and touching status.
// Note: do not assume the shape AABBs are overlapping or are valid.
void b2UpdateContact(b2World* world, b2Contact* contact, b2Shape* shapeA, b2Body* bodyA, b2Shape* shapeB, b2Body* bodyB)
{
	b2Manifold oldManifold = contact->manifold;

	B2_ASSERT(shapeA->object.index == contact->shapeIndexA);
	B2_ASSERT(shapeB->object.index == contact->shapeIndexB);

	b2ShapeId shapeIdA = {shapeA->object.index, world->index, shapeA->object.revision};
	b2ShapeId shapeIdB = {shapeB->object.index, world->index, shapeB->object.revision};

	// Re-enable this contact.
	contact->flags |= b2_contactEnabledFlag;

	bool touching = false;
	contact->manifold.pointCount = 0;

	// bool wasTouching = (contact->flags & b2_contactTouchingFlag) == b2_contactTouchingFlag;

	bool sensorA = shapeA->isSensor;
	bool sensorB = shapeB->isSensor;
	bool sensor = sensorA || sensorB;

	// Is this contact a sensor?
	if (sensor)
	{
		touching = b2TestShapeOverlap(shapeA, bodyA->transform, shapeB, bodyB->transform);

		// Sensors don't generate manifolds.
	}
	else
	{
		// Compute TOI
		b2ManifoldFcn* fcn = s_registers[shapeA->type][shapeB->type].fcn;

		contact->manifold = fcn(shapeA, bodyA->transform, shapeB, bodyB->transform, &contact->cache);

		touching = contact->manifold.pointCount > 0;

		// Match old contact ids to new contact ids and copy the
		// stored impulses to warm start the solver.
		for (int32_t i = 0; i < contact->manifold.pointCount; ++i)
		{
			b2ManifoldPoint* mp2 = contact->manifold.points + i;
			mp2->anchorA = b2Sub(mp2->point, bodyA->position);
			mp2->anchorB = b2Sub(mp2->point, bodyB->position);
			mp2->normalImpulse = 0.0f;
			mp2->tangentImpulse = 0.0f;
			mp2->persisted = false;
			uint16_t id2 = mp2->id;

			for (int32_t j = 0; j < oldManifold.pointCount; ++j)
			{
				b2ManifoldPoint* mp1 = oldManifold.points + j;

				if (mp1->id == id2)
				{
					mp2->normalImpulse = mp1->normalImpulse;
					mp2->tangentImpulse = mp1->tangentImpulse;
					mp2->persisted = true;
					break;
				}
			}
		}

		if (touching && world->preSolveFcn)
		{
			// TODO_ERIN this call assumes thread safety
			int32_t colorIndex = contact->colorIndex;
			bool collide = world->preSolveFcn(shapeIdA, shapeIdB, &contact->manifold, colorIndex, world->preSolveContext);
			if (collide == false)
			{
				// disable contact
				contact->flags &= ~b2_contactEnabledFlag;
			}
		}
	}

	if (touching)
	{
		contact->flags |= b2_contactTouchingFlag;
	}
	else
	{
		contact->flags &= ~b2_contactTouchingFlag;
	}

	// if (wasTouching == false && touching == true && world->callbacks.beginContactFcn)
	//{
	//	world->callbacks.beginContactFcn(shapeIdA, shapeIdB);
	// }

	// if (wasTouching == true && touching == false && world->callbacks.endContactFcn)
	//{
	//	world->callbacks.endContactFcn(shapeIdA, shapeIdB);
	// }
}
