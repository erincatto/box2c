// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "joint.h"

#include "body.h"
#include "contact.h"
#include "core.h"
#include "shape.h"
#include "solver.h"
#include "solver_set.h"
#include "util.h"
#include "world.h"

// needed for dll export
#include "box2d/box2d.h"
#include "box2d/color.h"
#include "box2d/debug_draw.h"
#include "box2d/joint_types.h"

b2DistanceJointDef b2DefaultDistanceJointDef()
{
	b2DistanceJointDef def = {0};
	def.length = 1.0f;
	def.maxLength = b2_huge;
	return def;
}

b2MotorJointDef b2DefaultMotorJointDef()
{
	b2MotorJointDef def = {0};
	def.maxForce = 1.0f;
	def.maxTorque = 1.0f;
	def.correctionFactor = 0.3f;
	return def;
}

b2MouseJointDef b2DefaultMouseJointDef()
{
	b2MouseJointDef def = {0};
	def.hertz = 4.0f;
	def.dampingRatio = 1.0f;
	return def;
}

b2PrismaticJointDef b2DefaultPrismaticJointDef()
{
	b2PrismaticJointDef def = {0};
	def.localAxisA = (b2Vec2){1.0f, 0.0f};
	return def;
}

b2RevoluteJointDef b2DefaultRevoluteJointDef()
{
	b2RevoluteJointDef def = {0};
	def.drawSize = 0.25f;
	return def;
}

b2WeldJointDef b2DefaultWeldJointDef()
{
	b2WeldJointDef def = {0};
	return def;
}

b2WheelJointDef b2DefaultWheelJointDef()
{
	b2WheelJointDef def = {0};
	def.localAxisA.y = 1.0f;
	def.hertz = 1.0f;
	def.dampingRatio = 0.7f;
	return def;
}

b2JointLookup* b2GetJointLookupFullId(b2World* world, b2JointId jointId)
{
	int id = jointId.index1 - 1;
	b2CheckIndex(world->jointLookupArray, id);
	b2JointLookup* lookup = world->jointLookupArray + id;
	b2CheckIndex(world->solverSetArray, lookup->setIndex);
	B2_ASSERT(lookup->revision == jointId.revision);
	return lookup;
}

b2JointLookup* b2GetJointLookup(b2World* world, int jointId)
{
	b2CheckIndex(world->jointLookupArray, jointId);
	b2JointLookup* lookup = world->jointLookupArray + jointId;
	return lookup;
}

b2Joint* b2GetJointSim(b2World* world, b2JointLookup* lookup)
{
	b2CheckIndex(world->solverSetArray, lookup->setIndex);

	if (lookup->setIndex == b2_awakeSet)
	{
		B2_ASSERT(0 <= lookup->colorIndex && lookup->colorIndex < b2_graphColorCount);
		b2GraphColor* color = world->constraintGraph.colors + lookup->colorIndex;
		B2_ASSERT(0 <= lookup->localIndex && lookup->localIndex < color->joints.count);
		return color->joints.data + lookup->localIndex;
	}

	b2SolverSet* set = world->solverSetArray + lookup->setIndex;
	B2_ASSERT(0 <= lookup->localIndex && lookup->localIndex < set->joints.count);
	return set->joints.data + lookup->localIndex;
}

b2Joint* b2GetJointSimCheckType(b2JointId jointId, b2JointType type)
{
	B2_MAYBE_UNUSED(type);

	b2World* world = b2GetWorld(jointId.world0);
	B2_ASSERT(world->locked == false);
	if (world->locked)
	{
		return NULL;
	}

	b2JointLookup* joint = b2GetJointLookupFullId(world, jointId);
	B2_ASSERT(joint->type == type);
	b2Joint* jointSim = b2GetJointSim(world, joint);
	B2_ASSERT(jointSim->type == type);
	return jointSim;
}

typedef struct b2JointPair
{
	b2JointLookup* lookup;
	b2Joint* joint;
} b2JointPair;

static b2JointPair b2CreateJoint(b2World* world, b2Body* bodyA, b2Body* bodyB, void* userData, float drawSize, b2JointType type, bool collideConnected)
{
	int bodyIdA = bodyA->bodyId;
	int bodyIdB = bodyB->bodyId;
	int maxSetIndex = B2_MAX(bodyA->setIndex, bodyB->setIndex);

	// Create joint id and lookup
	int jointId = b2AllocId(&world->jointIdPool);
	if (jointId == b2Array(world->jointLookupArray).count)
	{
		b2Array_Push(world->jointLookupArray, (b2JointLookup){0});
	}

	b2JointLookup* jointLookup = world->jointLookupArray + jointId;
	jointLookup->userData = userData;
	jointLookup->revision += 1;
	jointLookup->setIndex = B2_NULL_INDEX;
	jointLookup->colorIndex = B2_NULL_INDEX;
	jointLookup->localIndex = B2_NULL_INDEX;
	jointLookup->jointId = jointId;
	jointLookup->islandId = B2_NULL_INDEX;
	jointLookup->islandPrev = B2_NULL_INDEX;
	jointLookup->islandNext = B2_NULL_INDEX;
	jointLookup->drawSize = drawSize;
	jointLookup->type = type;
	jointLookup->collideConnected = collideConnected;
	jointLookup->isMarked = false;

	b2Joint* joint;

	if (bodyA->setIndex == b2_disabledSet || bodyB->setIndex == b2_disabledSet)
	{
		// if either body is disabled, create in disabled set
		b2SolverSet* set = world->solverSetArray + b2_disabledSet;
		jointLookup->setIndex = b2_disabledSet;
		jointLookup->localIndex = set->joints.count;
		joint = b2AddJoint(&world->blockAllocator, &set->joints);
	}
	else if (bodyA->setIndex == b2_staticSet && bodyB->setIndex == b2_staticSet)
	{
		// joint is connecting static bodies
		b2SolverSet* set = world->solverSetArray + b2_staticSet;
		jointLookup->setIndex = b2_staticSet;
		jointLookup->localIndex = set->joints.count;
		joint = b2AddJoint(&world->blockAllocator, &set->joints);
	}
	else if (bodyA->setIndex == b2_awakeSet || bodyB->setIndex == b2_awakeSet)
	{
		// if either body is sleeping, wake it
		if (maxSetIndex >= b2_firstSleepingSet)
		{
			b2WakeSolverSet(world, maxSetIndex);
		}

		joint = b2CreateJointInGraph(world, bodyIdA, bodyIdB, jointLookup);
		jointLookup->setIndex = b2_awakeSet;
	}
	else
	{
		// joint connected between sleeping and/or static bodies
		B2_ASSERT(bodyA->setIndex >= b2_firstSleepingSet || bodyB->setIndex >= b2_firstSleepingSet);
		B2_ASSERT(bodyA->setIndex != b2_staticSet || bodyB->setIndex != b2_staticSet);

		// joint should go into the sleeping set (not static set)
		int setIndex = maxSetIndex;

		b2CheckIndex(world->solverSetArray, setIndex);
		b2SolverSet* set = world->solverSetArray + setIndex;
		jointLookup->setIndex = setIndex;
		jointLookup->localIndex = set->joints.count;
		joint = b2AddJoint(&world->blockAllocator, &set->joints);
		joint->jointId = jointId;

		if (bodyA->setIndex != bodyB->setIndex &&
			bodyA->setIndex >= b2_firstSleepingSet &&
			bodyB->setIndex >= b2_firstSleepingSet)
		{
			// merge sleeping sets
			b2MergeSolverSets(world, bodyA->setIndex, bodyB->setIndex);
			B2_ASSERT(bodyA->setIndex == bodyB->setIndex);

			// fix potentially invalid set index
			setIndex = bodyA->setIndex;

			// Careful! The joint sim pointer was orphaned by the set merge.
			joint = world->solverSetArray[setIndex].joints.data + jointLookup->localIndex;
		}

		B2_ASSERT(jointLookup->setIndex == setIndex);
	}

	joint->jointId = jointId;
	joint->bodyIdA = bodyIdA;
	joint->bodyIdB = bodyIdB;

	// Doubly linked list on bodyA
	jointLookup->edges[0].bodyId = bodyIdA;
	jointLookup->edges[0].prevKey = B2_NULL_INDEX;
	jointLookup->edges[0].nextKey = bodyA->headJointKey;

	int keyA = (jointId << 1) | 0;
	if (bodyA->headJointKey != B2_NULL_INDEX)
	{
		b2JointLookup* jointA = world->jointLookupArray + (bodyA->headJointKey >> 1);
		b2JointEdge* edgeA = jointA->edges + (bodyA->headJointKey & 1);
		edgeA->prevKey = keyA;
	}
	bodyA->headJointKey = keyA;
	bodyA->jointCount += 1;

	// Doubly linked list on bodyB
	jointLookup->edges[1].bodyId = bodyIdB;
	jointLookup->edges[1].prevKey = B2_NULL_INDEX;
	jointLookup->edges[1].nextKey = bodyB->headJointKey;

	int keyB = (jointId << 1) | 1;
	if (bodyB->headJointKey != B2_NULL_INDEX)
	{
		b2JointLookup* jointB = world->jointLookupArray +  (bodyB->headJointKey >> 1);
		b2JointEdge* edgeB = jointB->edges + (bodyB->headJointKey & 1);
		edgeB->prevKey = keyB;
	}
	bodyB->headJointKey = keyB;
	bodyB->jointCount += 1;

	if (jointLookup->setIndex > b2_disabledSet)
	{
		// Add edge to island graph
		b2LinkJoint(world, jointLookup);
	}

	b2ValidateWorld(world);

	return (b2JointPair){jointLookup, joint};
}

static void b2DestroyContactsBetweenBodies(b2World* world, b2Body* bodyA, b2Body* bodyB)
{
	int contactKey;
	int otherBodyId;

	// use the smaller of the two contact lists
	if (bodyA->contactCount < bodyB->contactCount)
	{
		contactKey = bodyA->headContactKey;
		otherBodyId = bodyB->bodyId;
	}
	else
	{
		contactKey = bodyB->headContactKey;
		otherBodyId = bodyA->bodyId;
	}

	// no need to wake bodies when a joint removes collision between them
	bool wakeBodies = false;

	// destroy the contacts
	while (contactKey != B2_NULL_INDEX)
	{
		int contactId = contactKey >> 1;
		int edgeIndex = contactKey & 1;

		b2CheckIndex(world->contactLookupArray, contactId);
		b2ContactLookup* contact = world->contactLookupArray + contactId;
		contactKey = contact->edges[edgeIndex].nextKey;

		int otherEdgeIndex = edgeIndex ^ 1;
		if (contact->edges[otherEdgeIndex].bodyId == otherBodyId)
		{
			// Careful, this removes the contact from the current doubly linked list
			b2DestroyContact(world, contact, wakeBodies);
		}
	}
}

b2JointId b2CreateDistanceJoint(b2WorldId worldId, const b2DistanceJointDef* def)
{
	b2World* world = b2GetWorldFromId(worldId);

	B2_ASSERT(world->locked == false);

	if (world->locked)
	{
		return (b2JointId){0};
	}

	B2_ASSERT(b2Body_IsValid(def->bodyIdA));
	B2_ASSERT(b2Body_IsValid(def->bodyIdB));

	b2Body* bodyA = b2GetBodyFullId(world, def->bodyIdA);
	b2Body* bodyB = b2GetBodyFullId(world, def->bodyIdB);

	b2JointPair pair = b2CreateJoint(world, bodyA, bodyB, def->userData, 1.0f, b2_distanceJoint, def->collideConnected);

	b2Joint* joint = pair.joint;
	joint->type = b2_distanceJoint;
	joint->localOriginAnchorA = def->localAnchorA;
	joint->localOriginAnchorB = def->localAnchorB;

	b2DistanceJoint empty = {0};
	joint->distanceJoint = empty;
	joint->distanceJoint.hertz = def->hertz;
	joint->distanceJoint.dampingRatio = def->dampingRatio;
	joint->distanceJoint.length = def->length;
	joint->distanceJoint.minLength = def->minLength;
	joint->distanceJoint.maxLength = def->maxLength;
	joint->distanceJoint.impulse = 0.0f;
	joint->distanceJoint.lowerImpulse = 0.0f;
	joint->distanceJoint.upperImpulse = 0.0f;

	// If the joint prevents collisions, then destroy all contacts between attached bodies
	if (def->collideConnected == false)
	{
		b2DestroyContactsBetweenBodies(world, bodyA, bodyB);
	}

	b2JointId jointId = {joint->jointId + 1, world->worldId, pair.lookup->revision};
	return jointId;
}

b2JointId b2CreateMotorJoint(b2WorldId worldId, const b2MotorJointDef* def)
{
	b2World* world = b2GetWorldFromId(worldId);

	B2_ASSERT(world->locked == false);

	if (world->locked)
	{
		return (b2JointId){0};
	}

	b2Body* bodyA = b2GetBodyFullId(world, def->bodyIdA);
	b2Body* bodyB = b2GetBodyFullId(world, def->bodyIdB);

	b2JointPair pair = b2CreateJoint(world, bodyA, bodyB, def->userData, 1.0f, b2_motorJoint, def->collideConnected);
	b2Joint* joint = pair.joint;

	joint->type = b2_motorJoint;
	joint->localOriginAnchorA = (b2Vec2){0.0f, 0.0f};
	joint->localOriginAnchorB = (b2Vec2){0.0f, 0.0f};
	joint->motorJoint = (b2MotorJoint){0};
	joint->motorJoint.linearOffset = def->linearOffset;
	joint->motorJoint.angularOffset = def->angularOffset;
	joint->motorJoint.maxForce = def->maxForce;
	joint->motorJoint.maxTorque = def->maxTorque;
	joint->motorJoint.correctionFactor = B2_CLAMP(def->correctionFactor, 0.0f, 1.0f);

	// If the joint prevents collisions, then destroy all contacts between attached bodies
	if (def->collideConnected == false)
	{
		b2DestroyContactsBetweenBodies(world, bodyA, bodyB);
	}

	b2JointId jointId = {joint->jointId + 1, world->worldId, pair.lookup->revision};
	return jointId;
}

b2JointId b2CreateMouseJoint(b2WorldId worldId, const b2MouseJointDef* def)
{
	b2World* world = b2GetWorldFromId(worldId);

	B2_ASSERT(world->locked == false);

	if (world->locked)
	{
		return (b2JointId){0};
	}

	b2Body* bodyA = b2GetBodyFullId(world, def->bodyIdA);
	b2Body* bodyB = b2GetBodyFullId(world, def->bodyIdB);

	b2Transform transformA = b2GetBodyTransformQuick(world, bodyA);
	b2Transform transformB = b2GetBodyTransformQuick(world, bodyB);

	b2JointPair pair = b2CreateJoint(world, bodyA, bodyB, def->userData, 1.0f, b2_mouseJoint, def->collideConnected);

	b2Joint* joint = pair.joint;
	joint->type = b2_mouseJoint;
	joint->localOriginAnchorA = b2InvTransformPoint(transformA, def->target);
	joint->localOriginAnchorB = b2InvTransformPoint(transformB, def->target);

	b2MouseJoint empty = {0};
	joint->mouseJoint = empty;
	joint->mouseJoint.targetA = def->target;
	joint->mouseJoint.hertz = def->hertz;
	joint->mouseJoint.dampingRatio = def->dampingRatio;

	b2JointId jointId = {joint->jointId + 1, world->worldId, pair.lookup->revision};
	return jointId;
}

b2JointId b2CreateRevoluteJoint(b2WorldId worldId, const b2RevoluteJointDef* def)
{
	b2World* world = b2GetWorldFromId(worldId);

	B2_ASSERT(world->locked == false);

	if (world->locked)
	{
		return (b2JointId){0};
	}

	b2Body* bodyA = b2GetBodyFullId(world, def->bodyIdA);
	b2Body* bodyB = b2GetBodyFullId(world, def->bodyIdB);

	b2JointPair pair = b2CreateJoint(world, bodyA, bodyB, def->userData, def->drawSize, b2_revoluteJoint, def->collideConnected);

	b2Joint* joint = pair.joint;
	joint->type = b2_revoluteJoint;
	joint->localOriginAnchorA = def->localAnchorA;
	joint->localOriginAnchorB = def->localAnchorB;

	b2RevoluteJoint empty = {0};
	joint->revoluteJoint = empty;

	joint->revoluteJoint.referenceAngle = def->referenceAngle;
	joint->revoluteJoint.linearImpulse = b2Vec2_zero;
	joint->revoluteJoint.axialMass = 0.0f;
	joint->revoluteJoint.motorImpulse = 0.0f;
	joint->revoluteJoint.lowerImpulse = 0.0f;
	joint->revoluteJoint.upperImpulse = 0.0f;
	joint->revoluteJoint.lowerAngle = B2_MIN(def->lowerAngle, def->upperAngle);
	joint->revoluteJoint.upperAngle = B2_MAX(def->lowerAngle, def->upperAngle);
	joint->revoluteJoint.maxMotorTorque = def->maxMotorTorque;
	joint->revoluteJoint.motorSpeed = def->motorSpeed;
	joint->revoluteJoint.enableLimit = def->enableLimit;
	joint->revoluteJoint.enableMotor = def->enableMotor;

	// If the joint prevents collisions, then destroy all contacts between attached bodies
	if (def->collideConnected == false)
	{
		b2DestroyContactsBetweenBodies(world, bodyA, bodyB);
	}

	b2JointId jointId = {joint->jointId + 1, world->worldId, pair.lookup->revision};
	return jointId;
}

b2JointId b2CreatePrismaticJoint(b2WorldId worldId, const b2PrismaticJointDef* def)
{
	b2World* world = b2GetWorldFromId(worldId);

	B2_ASSERT(world->locked == false);

	if (world->locked)
	{
		return (b2JointId){0};
	}

	b2Body* bodyA = b2GetBodyFullId(world, def->bodyIdA);
	b2Body* bodyB = b2GetBodyFullId(world, def->bodyIdB);

	b2JointPair pair = b2CreateJoint(world, bodyA, bodyB, def->userData, 1.0f, b2_prismaticJoint, def->collideConnected);

	b2Joint* joint = pair.joint;
	joint->type = b2_prismaticJoint;
	joint->localOriginAnchorA = def->localAnchorA;
	joint->localOriginAnchorB = def->localAnchorB;

	b2PrismaticJoint empty = {0};
	joint->prismaticJoint = empty;

	joint->prismaticJoint.localAxisA = b2Normalize(def->localAxisA);
	joint->prismaticJoint.referenceAngle = def->referenceAngle;
	joint->prismaticJoint.impulse = b2Vec2_zero;
	joint->prismaticJoint.axialMass = 0.0f;
	joint->prismaticJoint.motorImpulse = 0.0f;
	joint->prismaticJoint.lowerImpulse = 0.0f;
	joint->prismaticJoint.upperImpulse = 0.0f;
	joint->prismaticJoint.lowerTranslation = def->lowerTranslation;
	joint->prismaticJoint.upperTranslation = def->upperTranslation;
	joint->prismaticJoint.maxMotorForce = def->maxMotorForce;
	joint->prismaticJoint.motorSpeed = def->motorSpeed;
	joint->prismaticJoint.enableLimit = def->enableLimit;
	joint->prismaticJoint.enableMotor = def->enableMotor;

	// If the joint prevents collisions, then destroy all contacts between attached bodies
	if (def->collideConnected == false)
	{
		b2DestroyContactsBetweenBodies(world, bodyA, bodyB);
	}

	b2JointId jointId = {joint->jointId + 1, world->worldId, pair.lookup->revision};
	return jointId;
}

b2JointId b2CreateWeldJoint(b2WorldId worldId, const b2WeldJointDef* def)
{
	b2World* world = b2GetWorldFromId(worldId);

	B2_ASSERT(world->locked == false);

	if (world->locked)
	{
		return (b2JointId){0};
	}

	b2Body* bodyA = b2GetBodyFullId(world, def->bodyIdA);
	b2Body* bodyB = b2GetBodyFullId(world, def->bodyIdB);

	b2JointPair pair = b2CreateJoint(world, bodyA, bodyB, def->userData, 1.0f, b2_weldJoint, def->collideConnected);

	b2Joint* joint = pair.joint;
	joint->type = b2_weldJoint;
	joint->localOriginAnchorA = def->localAnchorA;
	joint->localOriginAnchorB = def->localAnchorB;

	b2WeldJoint empty = {0};
	joint->weldJoint = empty;
	joint->weldJoint.referenceAngle = def->referenceAngle;
	joint->weldJoint.linearHertz = def->linearHertz;
	joint->weldJoint.linearDampingRatio = def->linearDampingRatio;
	joint->weldJoint.angularHertz = def->angularHertz;
	joint->weldJoint.angularDampingRatio = def->angularDampingRatio;
	joint->weldJoint.linearImpulse = b2Vec2_zero;
	joint->weldJoint.angularImpulse = 0.0f;

	// If the joint prevents collisions, then destroy all contacts between attached bodies
	if (def->collideConnected == false)
	{
		b2DestroyContactsBetweenBodies(world, bodyA, bodyB);
	}

	b2JointId jointId = {joint->jointId + 1, world->worldId, pair.lookup->revision};
	return jointId;
}

b2JointId b2CreateWheelJoint(b2WorldId worldId, const b2WheelJointDef* def)
{
	b2World* world = b2GetWorldFromId(worldId);

	B2_ASSERT(world->locked == false);

	if (world->locked)
	{
		return (b2JointId){0};
	}

	b2Body* bodyA = b2GetBodyFullId(world, def->bodyIdA);
	b2Body* bodyB = b2GetBodyFullId(world, def->bodyIdB);

	b2JointPair pair = b2CreateJoint(world, bodyA, bodyB, def->userData, 1.0f, b2_wheelJoint, def->collideConnected);

	b2Joint* joint = pair.joint;
	joint->type = b2_wheelJoint;
	joint->localOriginAnchorA = def->localAnchorA;
	joint->localOriginAnchorB = def->localAnchorB;

	joint->wheelJoint = (b2WheelJoint){0};
	joint->wheelJoint.localAxisA = b2Normalize(def->localAxisA);
	joint->wheelJoint.perpMass = 0.0f;
	joint->wheelJoint.axialMass = 0.0f;
	joint->wheelJoint.motorImpulse = 0.0f;
	joint->wheelJoint.lowerImpulse = 0.0f;
	joint->wheelJoint.upperImpulse = 0.0f;
	joint->wheelJoint.lowerTranslation = def->lowerTranslation;
	joint->wheelJoint.upperTranslation = def->upperTranslation;
	joint->wheelJoint.maxMotorTorque = def->maxMotorTorque;
	joint->wheelJoint.motorSpeed = def->motorSpeed;
	joint->wheelJoint.hertz = def->hertz;
	joint->wheelJoint.dampingRatio = def->dampingRatio;
	joint->wheelJoint.enableLimit = def->enableLimit;
	joint->wheelJoint.enableMotor = def->enableMotor;

	// If the joint prevents collisions, then destroy all contacts between attached bodies
	if (def->collideConnected == false)
	{
		b2DestroyContactsBetweenBodies(world, bodyA, bodyB);
	}

	b2JointId jointId = {joint->jointId + 1, world->worldId, pair.lookup->revision};
	return jointId;
}

void b2DestroyJointInternal(b2World* world, b2JointLookup* joint, bool wakeBodies)
{
	int jointId = joint->jointId;

	b2JointEdge* edgeA = joint->edges + 0;
	b2JointEdge* edgeB = joint->edges + 1;

	int idA = edgeA->bodyId;
	int idB = edgeB->bodyId;
	b2Body* bodyA = b2GetBody(world, idA);
	b2Body* bodyB = b2GetBody(world, idB);

	// Remove from body A
	if (edgeA->prevKey != B2_NULL_INDEX)
	{
		b2JointLookup* prevJoint = world->jointLookupArray + (edgeA->prevKey >> 1);
		b2JointEdge* prevEdge = prevJoint->edges + (edgeA->prevKey & 1);
		prevEdge->nextKey = edgeA->nextKey;
	}

	if (edgeA->nextKey != B2_NULL_INDEX)
	{
		b2JointLookup* nextJoint = world->jointLookupArray + (edgeA->nextKey >> 1);
		b2JointEdge* nextEdge = nextJoint->edges + (edgeA->nextKey & 1);
		nextEdge->prevKey = edgeA->prevKey;
	}

	int edgeKeyA = (jointId << 1) | 0;
	if (bodyA->headJointKey == edgeKeyA)
	{
		bodyA->headJointKey = edgeA->nextKey;
	}

	bodyA->jointCount -= 1;

	// Remove from body B
	if (edgeB->prevKey != B2_NULL_INDEX)
	{
		b2JointLookup* prevJoint = world->jointLookupArray + (edgeB->prevKey >> 1);
		b2JointEdge* prevEdge = prevJoint->edges + (edgeB->prevKey & 1);
		prevEdge->nextKey = edgeB->nextKey;
	}

	if (edgeB->nextKey != B2_NULL_INDEX)
	{
		b2JointLookup* nextJoint = world->jointLookupArray + (edgeB->nextKey >> 1);
		b2JointEdge* nextEdge = nextJoint->edges + (edgeB->nextKey & 1);
		nextEdge->prevKey = edgeB->prevKey;
	}

	int edgeKeyB = (jointId << 1) | 1;
	if (bodyB->headJointKey == edgeKeyB)
	{
		bodyB->headJointKey = edgeB->nextKey;
	}

	bodyB->jointCount -= 1;

	b2UnlinkJoint(world, joint);

	// Remove joint from solver set that owns it
	int setIndex = joint->setIndex;
	int localIndex = joint->localIndex;

	if (setIndex == b2_awakeSet)
	{
		b2RemoveJointFromGraph(world, joint->edges[0].bodyId, joint->edges[1].bodyId, joint->colorIndex, localIndex);
	}
	else
	{
		b2SolverSet* set = world->solverSetArray + setIndex;
		int movedIndex = b2RemoveJoint(&set->joints, localIndex);
		if (movedIndex != B2_NULL_INDEX)
		{
			// Fix lookup on moved joint
			b2Joint* movedJoint = set->joints.data + localIndex;
			int movedId = movedJoint->jointId;
			b2JointLookup* movedLookup = world->jointLookupArray + movedId;
			B2_ASSERT(movedLookup->localIndex == movedIndex);
			movedLookup->localIndex = localIndex;
		}
	}

	// Free lookup and id (preserve lookup revision)
	joint->setIndex = B2_NULL_INDEX;
	joint->localIndex = B2_NULL_INDEX;
	joint->jointId = B2_NULL_INDEX;
	b2FreeId(&world->jointIdPool, jointId);

	if (wakeBodies)
	{
		b2WakeBody(world, bodyA);
		b2WakeBody(world, bodyB);
	}

	b2ValidateWorld(world);
}

void b2DestroyJoint(b2JointId jointId)
{
	b2World* world = b2GetWorld(jointId.world0);
	B2_ASSERT(world->locked == false);

	if (world->locked)
	{
		return;
	}

	b2JointLookup* joint = b2GetJointLookupFullId(world, jointId);

	b2DestroyJointInternal(world, joint, true);
}

b2JointType b2Joint_GetType(b2JointId jointId)
{
	b2World* world = b2GetWorld(jointId.world0);
	b2JointLookup* joint = b2GetJointLookupFullId(world, jointId);
	return joint->type;
}

b2BodyId b2Joint_GetBodyA(b2JointId jointId)
{
	b2World* world = b2GetWorld(jointId.world0);
	b2JointLookup* joint = b2GetJointLookupFullId(world, jointId);
	return b2MakeBodyId(world, joint->edges[0].bodyId);
}

b2BodyId b2Joint_GetBodyB(b2JointId jointId)
{
	b2World* world = b2GetWorld(jointId.world0);
	b2JointLookup* joint = b2GetJointLookupFullId(world, jointId);
	return b2MakeBodyId(world, joint->edges[1].bodyId);
}

b2Vec2 b2Joint_GetLocalAnchorA(b2JointId jointId)
{
	b2World* world = b2GetWorld(jointId.world0);
	b2JointLookup* joint = b2GetJointLookupFullId(world, jointId);
	b2Joint* jointSim = b2GetJointSim(world, joint);
	return jointSim->localOriginAnchorA;
}

b2Vec2 b2Joint_GetLocalAnchorB(b2JointId jointId)
{
	b2World* world = b2GetWorld(jointId.world0);
	b2JointLookup* joint = b2GetJointLookupFullId(world, jointId);
	b2Joint* jointSim = b2GetJointSim(world, joint);
	return jointSim->localOriginAnchorB;
}

void b2Joint_SetCollideConnected(b2JointId jointId, bool shouldCollide)
{
	b2World* world = b2GetWorldLocked(jointId.world0);
	if (world == NULL)
	{
		return;
	}

	b2JointLookup* joint = b2GetJointLookupFullId(world, jointId);
	if (joint->collideConnected == shouldCollide)
	{
		return;
	}

	joint->collideConnected = shouldCollide;

	b2Body* bodyA = b2GetBody(world, joint->edges[0].bodyId);
	b2Body* bodyB = b2GetBody(world, joint->edges[1].bodyId);

	if (shouldCollide)
	{
		// need to tell the broad-phase to look for new pairs for one of the
		// two bodies. Pick the one with the fewest shapes.
		int shapeCountA = bodyA->shapeCount;
		int shapeCountB = bodyB->shapeCount;

		int shapeId = shapeCountA < shapeCountB ? bodyA->headShapeId : bodyB->headShapeId;
		while (shapeId != B2_NULL_INDEX)
		{
			b2Shape* shape = world->shapes + shapeId;

			if (shape->proxyKey != B2_NULL_INDEX)
			{
				b2BufferMove(&world->broadPhase, shape->proxyKey);
			}

			shapeId = shape->nextShapeId;
		}
	}
	else
	{
		b2DestroyContactsBetweenBodies(world, bodyA, bodyB);
	}
}

bool b2Joint_GetCollideConnected(b2JointId jointId)
{
	b2World* world = b2GetWorld(jointId.world0);
	b2JointLookup* joint = b2GetJointLookupFullId(world, jointId);
	return joint->collideConnected;
}

void b2Joint_SetUserData(b2JointId jointId, void* userData)
{
	b2World* world = b2GetWorld(jointId.world0);
	b2JointLookup* jointLookup = b2GetJointLookupFullId(world, jointId);
	jointLookup->userData = userData;
}

void* b2Joint_GetUserData(b2JointId jointId)
{
	b2World* world = b2GetWorld(jointId.world0);
	b2JointLookup* jointLookup = b2GetJointLookupFullId(world, jointId);
	return jointLookup->userData;
}

void b2Joint_WakeBodies(b2JointId jointId)
{
	b2World* world = b2GetWorldLocked(jointId.world0);
	if (world == NULL)
	{
		return;
	}

	b2JointLookup* joint = b2GetJointLookupFullId(world, jointId);
	b2Body* bodyA = world->bodyArray + joint->edges[0].bodyId;
	b2Body* bodyB = world->bodyArray + joint->edges[1].bodyId;

	b2WakeBody(world, bodyA);
	b2WakeBody(world, bodyB);
}

extern void b2PrepareDistanceJoint(b2Joint* base, b2StepContext* context);
extern void b2PrepareMotorJoint(b2Joint* base, b2StepContext* context);
extern void b2PrepareMouseJoint(b2Joint* base, b2StepContext* context);
extern void b2PreparePrismaticJoint(b2Joint* base, b2StepContext* context);
extern void b2PrepareRevoluteJoint(b2Joint* base, b2StepContext* context);
extern void b2PrepareWeldJoint(b2Joint* base, b2StepContext* context);
extern void b2PrepareWheelJoint(b2Joint* base, b2StepContext* context);

void b2PrepareJoint(b2Joint* joint, b2StepContext* context)
{
	switch (joint->type)
	{
		case b2_distanceJoint:
			b2PrepareDistanceJoint(joint, context);
			break;

		case b2_motorJoint:
			b2PrepareMotorJoint(joint, context);
			break;

		case b2_mouseJoint:
			b2PrepareMouseJoint(joint, context);
			break;

		case b2_prismaticJoint:
			b2PreparePrismaticJoint(joint, context);
			break;

		case b2_revoluteJoint:
			b2PrepareRevoluteJoint(joint, context);
			break;

		case b2_weldJoint:
			b2PrepareWeldJoint(joint, context);
			break;

		case b2_wheelJoint:
			b2PrepareWheelJoint(joint, context);
			break;

		default:
			B2_ASSERT(false);
	}
}

extern void b2WarmStartDistanceJoint(b2Joint* base, b2StepContext* context);
extern void b2WarmStartMotorJoint(b2Joint* base, b2StepContext* context);
extern void b2WarmStartMouseJoint(b2Joint* base, b2StepContext* context);
extern void b2WarmStartPrismaticJoint(b2Joint* base, b2StepContext* context);
extern void b2WarmStartRevoluteJoint(b2Joint* base, b2StepContext* context);
extern void b2WarmStartWeldJoint(b2Joint* base, b2StepContext* context);
extern void b2WarmStartWheelJoint(b2Joint* base, b2StepContext* context);

void b2WarmStartJoint(b2Joint* joint, b2StepContext* context)
{
	switch (joint->type)
	{
		case b2_distanceJoint:
			b2WarmStartDistanceJoint(joint, context);
			break;

		case b2_motorJoint:
			b2WarmStartMotorJoint(joint, context);
			break;

		case b2_mouseJoint:
			b2WarmStartMouseJoint(joint, context);
			break;

		case b2_prismaticJoint:
			b2WarmStartPrismaticJoint(joint, context);
			break;

		case b2_revoluteJoint:
			b2WarmStartRevoluteJoint(joint, context);
			break;

		case b2_weldJoint:
			b2WarmStartWeldJoint(joint, context);
			break;

		case b2_wheelJoint:
			b2WarmStartWheelJoint(joint, context);
			break;

		default:
			B2_ASSERT(false);
	}
}

extern void b2SolveDistanceJoint(b2Joint* base, b2StepContext* context, bool useBias);
extern void b2SolveMotorJoint(b2Joint* base, b2StepContext* context, bool useBias);
extern void b2SolveMouseJoint(b2Joint* base, b2StepContext* context);
extern void b2SolvePrismaticJoint(b2Joint* base, b2StepContext* context, bool useBias);
extern void b2SolveRevoluteJoint(b2Joint* base, b2StepContext* context, bool useBias);
extern void b2SolveWeldJoint(b2Joint* base, b2StepContext* context, bool useBias);
extern void b2SolveWheelJoint(b2Joint* base, b2StepContext* context, bool useBias);

void b2SolveJoint(b2Joint* joint, b2StepContext* context, bool useBias)
{
	switch (joint->type)
	{
		case b2_distanceJoint:
			b2SolveDistanceJoint(joint, context, useBias);
			break;

		case b2_motorJoint:
			b2SolveMotorJoint(joint, context, useBias);
			break;

		case b2_mouseJoint:
			b2SolveMouseJoint(joint, context);
			break;

		case b2_prismaticJoint:
			b2SolvePrismaticJoint(joint, context, useBias);
			break;

		case b2_revoluteJoint:
			b2SolveRevoluteJoint(joint, context, useBias);
			break;

		case b2_weldJoint:
			b2SolveWeldJoint(joint, context, useBias);
			break;

		case b2_wheelJoint:
			b2SolveWheelJoint(joint, context, useBias);
			break;

		default:
			B2_ASSERT(false);
	}
}

void b2PrepareOverflowJoints(b2StepContext* context)
{
	b2TracyCZoneNC(prepare_joints, "PrepJoints", b2_colorOldLace, true);

	b2World* world = context->world;
	b2ConstraintGraph* graph = context->graph;
	b2Joint* joints = graph->colors[b2_overflowIndex].joints.data;
	int jointCount = graph->colors[b2_overflowIndex].joints.count;

	for (int i = 0; i < jointCount; ++i)
	{
		b2Joint* joint = joints + i;
		b2PrepareJoint(joint, context);
	}

	b2TracyCZoneEnd(prepare_joints);
}

void b2WarmStartOverflowJoints(b2StepContext* context)
{
	b2TracyCZoneNC(prepare_joints, "PrepJoints", b2_colorOldLace, true);

	b2World* world = context->world;
	b2ConstraintGraph* graph = context->graph;
	b2Joint* joints = graph->colors[b2_overflowIndex].joints.data;
	int jointCount = graph->colors[b2_overflowIndex].joints.count;

	for (int i = 0; i < jointCount; ++i)
	{
		b2Joint* joint = joints + i;
		b2WarmStartJoint(joint, context);
	}

	b2TracyCZoneEnd(prepare_joints);
}

void b2SolveOverflowJoints(b2StepContext* context, bool useBias)
{
	b2TracyCZoneNC(solve_joints, "SolveJoints", b2_colorLemonChiffon, true);

	b2World* world = context->world;
	b2ConstraintGraph* graph = context->graph;
	b2Joint* joints = graph->colors[b2_overflowIndex].joints.data;
	int jointCount = graph->colors[b2_overflowIndex].joints.count;

	for (int i = 0; i < jointCount; ++i)
	{
		b2Joint* joint = joints + i;
		b2SolveJoint(joint, context, useBias);
	}

	b2TracyCZoneEnd(solve_joints);
}

extern void b2DrawDistanceJoint(b2DebugDraw* draw, b2Joint* base, b2Transform transformA, b2Transform transformB);
extern void b2DrawPrismaticJoint(b2DebugDraw* draw, b2Joint* base, b2Transform transformA, b2Transform transformB);
extern void b2DrawRevoluteJoint(b2DebugDraw* draw, b2Joint* base,  b2Transform transformA, b2Transform transformB, float drawSize);
extern void b2DrawWheelJoint(b2DebugDraw* draw, b2Joint* base, b2Transform transformA, b2Transform transformB);

void b2DrawJoint(b2DebugDraw* draw, b2World* world, b2JointLookup* joint)
{
	b2Body* bodyA = b2GetBody(world, joint->edges[0].bodyId);
	b2Body* bodyB = b2GetBody(world, joint->edges[1].bodyId);
	if (bodyA->setIndex == b2_disabledSet || bodyB->setIndex == b2_disabledSet)
	{
		return;
	}

	b2Joint* jointSim = b2GetJointSim(world, joint);

	b2Transform transformA = b2GetBodyTransformQuick(world, bodyA);
	b2Transform transformB = b2GetBodyTransformQuick(world, bodyB);
	b2Vec2 pA = b2TransformPoint(transformA, jointSim->localOriginAnchorA);
	b2Vec2 pB = b2TransformPoint(transformB, jointSim->localOriginAnchorB);

	b2Color color = {0.5f, 0.8f, 0.8f, 1.0f};

	switch (joint->type)
	{
		case b2_distanceJoint:
			b2DrawDistanceJoint(draw, jointSim, transformA, transformB);
			break;

		case b2_mouseJoint:
		{
			b2Vec2 target = jointSim->mouseJoint.targetA;

			b2Color c1 = {0.0f, 1.0f, 0.0f, 1.0f};
			draw->DrawPoint(target, 4.0f, c1, draw->context);
			draw->DrawPoint(pB, 4.0f, c1, draw->context);

			b2Color c2 = {0.8f, 0.8f, 0.8f, 1.0f};
			draw->DrawSegment(target, pB, c2, draw->context);
		}
		break;

		case b2_prismaticJoint:
			b2DrawPrismaticJoint(draw, jointSim, transformA, transformB);
			break;

		case b2_revoluteJoint:
			b2DrawRevoluteJoint(draw, jointSim, transformA, transformB, joint->drawSize);
			break;

		case b2_wheelJoint:
			b2DrawWheelJoint(draw, jointSim, transformA, transformB);
			break;

		default:
			draw->DrawSegment(transformA.p, pA, color, draw->context);
			draw->DrawSegment(pA, pB, color, draw->context);
			draw->DrawSegment(transformB.p, pB, color, draw->context);
	}

	if (draw->drawGraphColors)
	{
		b2HexColor colors[b2_graphColorCount] = {b2_colorRed,		b2_colorOrange,	   b2_colorYellow, b2_colorGreen,
												 b2_colorCyan,		b2_colorBlue,	   b2_colorViolet, b2_colorPink,
												 b2_colorChocolate, b2_colorGoldenrod, b2_colorCoral,  b2_colorBlack};

		int colorIndex = joint->colorIndex;
		if (colorIndex != B2_NULL_INDEX)
		{
			b2Vec2 p = b2Lerp(pA, pB, 0.5f);
			draw->DrawPoint(p, 5.0f, b2MakeColor(colors[colorIndex]), draw->context);
		}
	}
}
