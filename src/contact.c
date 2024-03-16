// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "contact.h"

#include "array.h"
#include "body.h"
#include "core.h"
#include "island.h"
#include "shape.h"
#include "table.h"
#include "util.h"
#include "solver_set.h"
#include "world.h"

#include "box2d/distance.h"
#include "box2d/event_types.h"
#include "box2d/manifold.h"

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

static void b2AddType(b2ManifoldFcn* fcn, b2ShapeType type1, b2ShapeType type2)
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

	int bodyKeyA = shapeA->bodyId;
	int bodyKeyB = shapeB->bodyId;
	B2_ASSERT(0 <= bodyKeyA && bodyKeyA < b2Array(world->bodyLookupArray).count);
	B2_ASSERT(0 <= bodyKeyB && bodyKeyB < b2Array(world->bodyLookupArray).count);
	b2BodyLookup lookupA = world->bodyLookupArray[bodyKeyA];
	b2BodyLookup lookupB = world->bodyLookupArray[bodyKeyB];

	// Contacts should only be created from an awake body
	B2_ASSERT(lookupA.setIndex == b2_awakeSet || lookupB.setIndex == b2_awakeSet);

	b2Body* bodyA = b2GetBodyFromRawId(world, shapeA->bodyId);
	b2Body* bodyB = b2GetBodyFromRawId(world, shapeB->bodyId);

	b2SolverSet* set = world->solverSetArray + b2_awakeSet;

	// Create contact key and lookup
	int contactKey = b2AllocId(&world->contactIdPool);
	if (contactKey == b2Array(world->contactLookupArray).count)
	{
		b2Array_Push(world->contactLookupArray, (b2ContactLookup){0});
	}

	b2ContactLookup* lookup = world->contactLookupArray + contactKey;
	lookup->setIndex = b2_awakeSet;
	lookup->colorIndex = B2_NULL_INDEX;
	lookup->contactIndex = set->contacts.count;

	// Contacts are created as non-touching. Later if they are found to be touching
	// they will link islands and be moved into the constraint graph.
	b2Contact* contact = b2AddContact(&world->blockAllocator, &set->contacts);
	contact->contactId = contactKey;
	contact->flags = 0;

	if (shapeA->isSensor || shapeB->isSensor)
	{
		contact->flags |= b2_contactSensorFlag;
	}

	if (shapeA->enableSensorEvents || shapeB->enableSensorEvents)
	{
		contact->flags |= b2_contactEnableSensorEvents;
	}

	if (shapeA->enableContactEvents || shapeB->enableContactEvents)
	{
		contact->flags |= b2_contactEnableContactEvents;
	}

	if (shapeA->enablePreSolveEvents || shapeB->enablePreSolveEvents)
	{
		contact->flags |= b2_contactEnablePreSolveEvents;
	}

	contact->shapeIndexA = shapeA->object.index;
	contact->shapeIndexB = shapeB->object.index;
	contact->cache = b2_emptyDistanceCache;
	contact->manifold = b2_emptyManifold;
	contact->friction = b2MixFriction(shapeA->friction, shapeB->friction);
	contact->restitution = b2MixRestitution(shapeA->restitution, shapeB->restitution);
	contact->tangentSpeed = 0.0f;
	contact->islandId = B2_NULL_INDEX;
	contact->islandPrev = B2_NULL_INDEX;
	contact->islandNext = B2_NULL_INDEX;
	contact->isMarked = false;

	// Connect to body A
	{
		contact->edges[0].bodyId = bodyKeyA;
		contact->edges[0].prevKey = B2_NULL_INDEX;
		contact->edges[0].nextKey = bodyA->contactList;

		int32_t keyA = (contactKey << 1) | 0;
		if (bodyA->contactList != B2_NULL_INDEX)
		{
			b2Contact* contactA = b2GetContactFromRawId(world, bodyA->contactList >> 1);
			b2ContactEdge* edgeA = contactA->edges + (bodyA->contactList & 1);
			edgeA->prevKey = keyA;
		}
		bodyA->contactList = keyA;
		bodyA->contactCount += 1;
	}

	// Connect to body B
	{
		contact->edges[1].bodyId = bodyKeyB;
		contact->edges[1].prevKey = B2_NULL_INDEX;
		contact->edges[1].nextKey = bodyB->contactList;

		int32_t keyB = (contactKey << 1) | 1;
		if (bodyB->contactList != B2_NULL_INDEX)
		{
			b2Contact* contactB = b2GetContactFromRawId(world, bodyB->contactList >> 1);
			b2ContactEdge* edgeB = contactB->edges + (bodyB->contactList & 1);
			edgeB->prevKey = keyB;
		}
		bodyB->contactList = keyB;
		bodyB->contactCount += 1;
	}

	// Add to pair set for fast lookup
	uint64_t pairKey = B2_SHAPE_PAIR_KEY(contact->shapeIndexA, contact->shapeIndexB);
	b2AddKey(&world->broadPhase.pairSet, pairKey);
}

// A contact is destroyed when:
// - broad-phase proxies stop overlapping
// - a body is destroyed
// - a body is disabled
// - a body changes type from dynamic to kinematic or static
// - a shape is destroyed
// - contact filtering is modified
// - a body becomes a sensor (check this!!!)
void b2DestroyContact(b2World* world, b2Contact* contact)
{
	// Remove pair from set
	uint64_t pairKey = B2_SHAPE_PAIR_KEY(contact->shapeIndexA, contact->shapeIndexB);
	b2RemoveKey(&world->broadPhase.pairSet, pairKey);

	b2ContactEdge* edgeA = contact->edges + 0;
	b2ContactEdge* edgeB = contact->edges + 1;

	b2Body* bodyA = b2GetBodyFromRawId(world, edgeA->bodyId);
	b2Body* bodyB = b2GetBodyFromRawId(world, edgeB->bodyId);

	// if (contactListener && contact->IsTouching())
	//{
	//	contactListener->EndContact(contact);
	// }

	// Remove from body A
	if (edgeA->prevKey != B2_NULL_INDEX)
	{
		b2Contact* prevContact = b2GetContactFromRawId(world, edgeA->prevKey >> 1);
		b2ContactEdge* prevEdge = prevContact->edges + (edgeA->prevKey & 1);
		prevEdge->nextKey = edgeA->nextKey;
	}

	if (edgeA->nextKey != B2_NULL_INDEX)
	{
		b2Contact* nextContact = b2GetContactFromRawId(world, edgeA->nextKey >> 1);
		b2ContactEdge* nextEdge = nextContact->edges + (edgeA->nextKey & 1);
		nextEdge->prevKey = edgeA->prevKey;
	}

	int contactId = contact->contactId;

	int32_t edgeKeyA = (contactId << 1) | 0;
	if (bodyA->contactList == edgeKeyA)
	{
		bodyA->contactList = edgeA->nextKey;
	}

	bodyA->contactCount -= 1;

	// Remove from body B
	if (edgeB->prevKey != B2_NULL_INDEX)
	{
		b2Contact* prevContact = b2GetContactFromRawId(world, edgeB->prevKey >> 1);
		b2ContactEdge* prevEdge = prevContact->edges + (edgeB->prevKey & 1);
		prevEdge->nextKey = edgeB->nextKey;
	}

	if (edgeB->nextKey != B2_NULL_INDEX)
	{
		b2Contact* nextContact = b2GetContactFromRawId(world, edgeB->nextKey >> 1);
		b2ContactEdge* nextEdge = nextContact->edges + (edgeB->nextKey & 1);
		nextEdge->prevKey = edgeB->prevKey;
	}

	int32_t edgeKeyB = (contactId << 1) | 1;
	if (bodyB->contactList == edgeKeyB)
	{
		bodyB->contactList = edgeB->nextKey;
	}

	bodyB->contactCount -= 1;

	if (contact->islandId != B2_NULL_INDEX)
	{
		b2UnlinkContact(world, contact);
	}

	// Remove contact from the array that owns it
	b2ContactLookup* lookup = world->contactLookupArray + contactId;

	if (contact->colorIndex != B2_NULL_INDEX)
	{
		// contact is an active constraint
		B2_ASSERT(lookup->setIndex == b2_awakeSet);
		b2RemoveContactFromGraph(world, contact);
	}
	else
	{
		// contact is non-touching or is sleeping
		B2_ASSERT(lookup->setIndex != b2_awakeSet || contact->manifold.pointCount == 0);
		b2SolverSet* set = world->solverSetArray + lookup->setIndex;
		int movedIndex = b2RemoveContact(&world->blockAllocator, &set->contacts, lookup->contactIndex);
		if (movedIndex != B2_NULL_INDEX)
		{
			b2Contact* movedContact = set->contacts.data + lookup->contactIndex;
			movedContact->localIndex = lookup->contactIndex;
			int movedKey = movedContact->contactId;
			b2ContactLookup* movedLookup = world->contactLookupArray + movedKey;
			movedLookup->contactIndex = lookup->contactIndex;
		}
	}

	lookup->setIndex = B2_NULL_INDEX;
	lookup->colorIndex = B2_NULL_INDEX;
	lookup->contactIndex = B2_NULL_INDEX;

	b2FreeId(&world->contactIdPool, contactId);

	b2WakeBody(world, edgeA->bodyId);
	b2WakeBody(world, edgeB->bodyId);

	b2ValidateWorld(world);
}

b2Contact* b2GetContactFromRawId(b2World* world, int contactId)
{
	B2_ASSERT(0 <= contactId && contactId < b2Array(world->contactLookupArray).count);
	b2ContactLookup lookup = world->contactLookupArray[contactId];
	B2_ASSERT(0 <= lookup.setIndex && lookup.setIndex < b2Array(world->solverSetArray).count);
	if (lookup.setIndex == b2_awakeSet && lookup.colorIndex != B2_NULL_INDEX)
	{
		// contact lives in constraint graph
		B2_ASSERT(0 <= lookup.colorIndex && lookup.colorIndex < b2_graphColorCount);
		b2GraphColor* color = world->constraintGraph.colors + lookup.colorIndex;
		B2_ASSERT(0 <= lookup.contactIndex && lookup.contactIndex < color->contacts.count);
		return color->contacts.data + lookup.contactIndex;
	}

	b2SolverSet* set = world->solverSetArray + lookup.setIndex;
	B2_ASSERT(0 <= lookup.contactIndex && lookup.contactIndex <= set->contacts.count);
	return set->contacts.data + lookup.contactIndex;
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

	b2ShapeId shapeIdA = {shapeA->object.index, world->worldId, shapeA->object.revision};
	b2ShapeId shapeIdB = {shapeB->object.index, world->worldId, shapeB->object.revision};

	bool touching = false;
	contact->manifold.pointCount = 0;

	bool sensorA = shapeA->isSensor;
	bool sensorB = shapeB->isSensor;
	bool sensor = sensorA || sensorB;

	b2Transform transformA = b2MakeTransform(bodyA);
	b2Transform transformB = b2MakeTransform(bodyB);

	// Is this contact a sensor?
	if (sensor)
	{
		touching = b2TestShapeOverlap(shapeA, transformA, shapeB, transformB);

		// Sensors don't generate manifolds.
	}
	else
	{
		// Compute TOI
		b2ManifoldFcn* fcn = s_registers[shapeA->type][shapeB->type].fcn;

		contact->manifold = fcn(shapeA, transformA, shapeB, transformB, &contact->cache);

		touching = contact->manifold.pointCount > 0;

		// Match old contact ids to new contact ids and copy the
		// stored impulses to warm start the solver.
		for (int32_t i = 0; i < contact->manifold.pointCount; ++i)
		{
			b2ManifoldPoint* mp2 = contact->manifold.points + i;

			// make anchors relative to center of mass
			b2Vec2 centerOffsetA = b2RotateVector(transformA.q, bodyA->localCenter);
			b2Vec2 centerOffsetB = b2RotateVector(transformB.q, bodyB->localCenter);
			mp2->anchorA = b2Sub(mp2->anchorA, centerOffsetA);
			mp2->anchorB = b2Sub(mp2->anchorB, centerOffsetB);
			
			mp2->normalImpulse = 0.0f;
			mp2->tangentImpulse = 0.0f;
			mp2->maxNormalImpulse = 0.0f;
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

		if (touching && world->preSolveFcn && (contact->flags & b2_contactEnablePreSolveEvents) != 0)
		{
			// this call assumes thread safety
			bool collide = world->preSolveFcn(shapeIdA, shapeIdB, &contact->manifold, world->preSolveContext);
			if (collide == false)
			{
				// disable contact
				touching = false;
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
}

#if 0 // todo probably delete this in favor of new API
b2Contact* b2GetContact(b2World* world, b2ContactId contactId)
{
	B2_ASSERT(0 <= contactId.index && contactId.index < world->contactPool.capacity);
	b2Contact* contact = world->contacts + contactId.index;
	B2_ASSERT(b2ObjectValid(&contact->object));
	B2_ASSERT(contact->object.revision == contactId.revision);
	return contact;
}

b2ContactId b2Body_GetFirstContact(b2BodyId bodyId)
{
	b2World* world = b2GetWorld(bodyId.world);
	b2Body* body = b2GetBody(world, bodyId);

	if (body->contactList == B2_NULL_INDEX)
	{
		return b2_nullContactId;
	}

	b2Contact* contact = world->contacts + body->contactList;
	b2ContactId id = {contact->object.index, bodyId.world, contact->object.revision};
	return id;
}

b2ContactId b2Body_GetNextContact(b2BodyId bodyId, b2ContactId contactId)
{
	b2World* world = b2GetWorld(contactId.world);
	b2Body* body = b2GetBody(world, bodyId);
	b2Contact* contact = b2GetContact(world, contactId);

	if (contact->edges[0].bodyIndex == body->object.index)
	{
		if (contact->edges[0].nextKey == B2_NULL_INDEX)
		{
			return b2_nullContactId;
		}

		contact = world->contacts + (contact->edges[0].nextKey >> 1);
	}
	else
	{
		B2_ASSERT(contact->edges[1].bodyIndex == body->object.index);

		if (contact->edges[1].nextKey == B2_NULL_INDEX)
		{
			return b2_nullContactId;
		}

		contact = world->contacts + (contact->edges[1].nextKey >> 1);
	}

	b2ContactId id = {contact->object.index, bodyId.world, contact->object.revision};
	return id;
}

b2ContactData b2Contact_GetData(b2ContactId contactId)
{
	b2World* world = b2GetWorld(contactId.world);
	b2Contact* contact = b2GetContact(world, contactId);
	b2Shape* shapeA = world->shapes + contact->shapeIndexA;
	b2Shape* shapeB = world->shapes + contact->shapeIndexB;
	b2ShapeId idA = {shapeA->object.index, contactId.world, shapeA->object.revision};
	b2ShapeId idB = {shapeB->object.index, contactId.world, shapeB->object.revision};
	b2ContactData data = {idA, idB, contact->manifold};
	return data;
}
#endif
