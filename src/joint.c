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

b2Joint* b2GetJoint(b2World* world, int jointId)
{
	B2_ASSERT(0 <= jointId && jointId < b2Array(world->jointLookupArray).count);

	b2JointLookup lookup = world->jointLookupArray[jointId];
	B2_ASSERT(0 <= lookup.setIndex && lookup.setIndex < b2Array(world->solverSetArray).count);

	b2Joint* joint;
	if (lookup.setIndex == b2_awakeSet)
	{
		B2_ASSERT(0 <= lookup.colorIndex && lookup.colorIndex < b2_graphColorCount);
		b2GraphColor* color = world->constraintGraph.colors + lookup.colorIndex;
		B2_ASSERT(0 <= lookup.localIndex && lookup.localIndex < color->joints.count);
		joint = color->joints.data + lookup.localIndex;
	}
	else
	{
		b2SolverSet* set = world->solverSetArray + lookup.setIndex;
		B2_ASSERT(0 <= lookup.localIndex && lookup.localIndex < set->joints.count);
		joint = set->joints.data + lookup.localIndex;
	}

	return joint;
}

b2Joint* b2GetJointCheckRevision(b2World* world, b2JointId jointId)
{
	int id = jointId.index1 - 1;
	b2JointLookup lookup = world->jointLookupArray[id];
	B2_ASSERT(0 <= lookup.setIndex && lookup.setIndex < b2Array(world->solverSetArray).count);
	B2_ASSERT(lookup.revision == jointId.revision);

	b2Joint* joint;
	if (lookup.setIndex == b2_awakeSet)
	{
		B2_ASSERT(0 <= lookup.colorIndex && lookup.colorIndex < b2_graphColorCount);
		b2GraphColor* color = world->constraintGraph.colors + lookup.colorIndex;
		B2_ASSERT(0 <= lookup.localIndex && lookup.localIndex < color->joints.count);
		joint = color->joints.data + lookup.localIndex;
	}
	else
	{
		b2SolverSet* set = world->solverSetArray + lookup.setIndex;
		B2_ASSERT(0 <= lookup.localIndex && lookup.localIndex < set->joints.count);
		joint = set->joints.data + lookup.localIndex;
	}

	return joint;
}

b2Joint* b2GetJointCheckType(b2JointId jointId, b2JointType type)
{
	B2_MAYBE_UNUSED(type);

	b2World* world = b2GetWorld(jointId.world0);
	B2_ASSERT(world->locked == false);
	if (world->locked)
	{
		return NULL;
	}

	b2Joint* joint = b2GetJointCheckRevision(world, jointId);
	B2_ASSERT(joint->type == type);
	return joint;
}

static b2Joint* b2CreateJoint(b2World* world, b2Body* bodyA, b2Body* bodyB)
{
	int bodyKeyA = bodyA->bodyId;
	int bodyKeyB = bodyB->bodyId;
	B2_ASSERT(0 <= bodyKeyA && bodyKeyA < b2Array(world->bodyLookupArray).count);
	B2_ASSERT(0 <= bodyKeyB && bodyKeyB < b2Array(world->bodyLookupArray).count);
	b2BodyLookup lookupA = world->bodyLookupArray[bodyKeyA];
	b2BodyLookup lookupB = world->bodyLookupArray[bodyKeyB];
	int maxSetIndex = B2_MAX(lookupA.setIndex, lookupB.setIndex);

	// Create joint id and lookup
	int jointId = b2AllocId(&world->jointIdPool);
	if (jointId == b2Array(world->jointLookupArray).count)
	{
		b2Array_Push(world->jointLookupArray, (b2JointLookup){0});
	}

	b2JointLookup* lookup = world->jointLookupArray + jointId;
	lookup->revision += 1;

	b2Joint* joint;

	if (lookupA.setIndex == b2_disabledSet || lookupB.setIndex == b2_disabledSet)
	{
		// if either body is disabled, create in disabled set
		b2SolverSet* set = world->solverSetArray + b2_disabledSet;
		joint = b2AddJoint(&world->blockAllocator, &set->joints);
		lookup->setIndex = b2_disabledSet;
	}
	else if (lookupA.setIndex == b2_awakeSet || lookupB.setIndex == b2_awakeSet)
	{
		joint = b2AddJointToGraph(world, bodyA, bodyB, lookup);
		lookup->setIndex = b2_awakeSet;

		// if either body is sleeping, wake it
		if (maxSetIndex >= b2_firstSleepingSet)
		{
			b2WakeSolverSet(world, maxSetIndex);

			// body pointers are now invalid, fix them
			lookupA = world->bodyLookupArray[bodyA->bodyId];
			lookupB = world->bodyLookupArray[bodyB->bodyId];
			B2_ASSERT(lookupA.setIndex == b2_awakeSet);
			B2_ASSERT(lookupB.setIndex == b2_awakeSet);
			b2SolverSet* set = world->solverSetArray + lookupA.setIndex;
			B2_ASSERT(0 <= lookupA.bodyIndex && lookupA.bodyIndex < set->bodies.count);
			B2_ASSERT(0 <= lookupB.bodyIndex && lookupB.bodyIndex < set->bodies.count);
			bodyA = set->bodies.data + lookupA.bodyIndex;
			bodyB = set->bodies.data + lookupB.bodyIndex;
		}
	}
	else if (lookupA.setIndex == b2_staticSet && lookupB.setIndex == b2_staticSet)
	{
		// joint between static bodies
		b2SolverSet* set = world->solverSetArray + b2_staticSet;
		joint = b2AddJoint(&world->blockAllocator, &set->joints);
		setIndex = lookupA.setIndex;
	}
	else
	{
		// joint connected between sleeping bodies
		B2_ASSERT(lookupA.setIndex >= b2_firstSleepingSet && lookupB.setIndex >= b2_firstSleepingSet);
		b2SolverSet* set;
		if (lookupA.setIndex != lookupB.setIndex)
		{
			b2MergeSolverSets(world, lookupA.setIndex, lookupB.setIndex);

			// body pointers are now invalid, fix them
			lookupA = world->bodyLookupArray[bodyA->bodyId];
			lookupB = world->bodyLookupArray[bodyB->bodyId];
			B2_ASSERT(lookupA.setIndex == lookupB.setIndex);
			set = world->solverSetArray + lookupA.setIndex;
			B2_ASSERT(0 <= lookupA.bodyIndex && lookupA.bodyIndex < set->bodies.count);
			B2_ASSERT(0 <= lookupB.bodyIndex && lookupB.bodyIndex < set->bodies.count);
			bodyA = set->bodies.data + lookupA.bodyIndex;
			bodyB = set->bodies.data + lookupB.bodyIndex;
		}
		else
		{
			B2_ASSERT(0 <= lookupA.setIndex && lookupA.setIndex < b2Array(world->solverSetArray).count);
			set = world->solverSetArray + lookupA.setIndex;
		}

		joint = b2AddJoint(&world->blockAllocator, &set->joints);
		setIndex = lookupA.setIndex;
	}


	joint->jointId = jointId;
	joint->revision = lookup->revision;

	// Doubly linked list on bodyA
	joint->edges[0].bodyId = bodyKeyA;
	joint->edges[0].prevKey = B2_NULL_INDEX;
	joint->edges[0].nextKey = bodyA->jointList;

	int32_t keyA = (jointId << 1) | 0;
	if (bodyA->jointList != B2_NULL_INDEX)
	{
		b2Joint* jointA = b2GetJoint(world, bodyA->jointList >> 1);
		b2JointEdge* edgeA = jointA->edges + (bodyA->jointList & 1);
		edgeA->prevKey = keyA;
	}
	bodyA->jointList = keyA;
	bodyA->jointCount += 1;

	// Doubly linked list on bodyB
	joint->edges[1].bodyId = bodyKeyB;
	joint->edges[1].prevKey = B2_NULL_INDEX;
	joint->edges[1].nextKey = bodyB->jointList;

	int32_t keyB = (jointId << 1) | 1;
	if (bodyB->jointList != B2_NULL_INDEX)
	{
		b2Joint* jointB = b2GetJoint(world, bodyB->jointList >> 1);
		b2JointEdge* edgeB = jointB->edges + (bodyB->jointList & 1);
		edgeB->prevKey = keyB;
	}
	bodyB->jointList = keyB;
	bodyB->jointCount += 1;

	joint->islandId = B2_NULL_INDEX;
	joint->islandPrev = B2_NULL_INDEX;
	joint->islandNext = B2_NULL_INDEX;

	joint->drawSize = 1.0f;
	joint->isMarked = false;

	if ((bodyA->type == b2_dynamicBody || bodyB->type == b2_dynamicBody) && bodyA->isEnabled == true && bodyB->isEnabled == true)
	{
		// Add edge to island graph
		b2LinkJoint(world, joint);
	}

	return joint;
}

static void b2DestroyContactsBetweenBodies(b2World* world, b2Body* bodyA, b2Body* bodyB)
{
	int32_t contactKey;
	int32_t otherBodyKey;

	if (bodyA->contactCount < bodyB->contactCount)
	{
		contactKey = bodyA->contactList;
		otherBodyKey = bodyB->bodyId;
	}
	else
	{
		contactKey = bodyB->contactList;
		otherBodyKey = bodyA->bodyId;
	}

	while (contactKey != B2_NULL_INDEX)
	{
		int32_t contactId = contactKey >> 1;
		int32_t edgeIndex = contactKey & 1;

		b2Contact* contact = b2GetContactFromRawId(world, contactId);
		contactKey = contact->edges[edgeIndex].nextKey;

		int32_t otherEdgeIndex = edgeIndex ^ 1;
		if (contact->edges[otherEdgeIndex].bodyId == otherBodyKey)
		{
			// Careful, this removes the contact from the current doubly linked list
			b2DestroyContact(world, contact);
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

	b2Body* bodyA = b2GetBody(world, def->bodyIdA);
	b2Body* bodyB = b2GetBody(world, def->bodyIdB);

	b2Joint* joint = b2CreateJoint(world, bodyA, bodyB);

	joint->type = b2_distanceJoint;
	joint->localOriginAnchorA = def->localAnchorA;
	joint->localOriginAnchorB = def->localAnchorB;
	joint->collideConnected = def->collideConnected;
	joint->userData = def->userData;

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

	b2JointId jointId = {joint->jointId + 1, world->worldId, joint->revision};
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

	b2Body* bodyA = b2GetBody(world, def->bodyIdA);
	b2Body* bodyB = b2GetBody(world, def->bodyIdB);

	b2Joint* joint = b2CreateJoint(world, bodyA, bodyB);

	joint->type = b2_motorJoint;
	joint->localOriginAnchorA = (b2Vec2){0.0f, 0.0f};
	joint->localOriginAnchorB = (b2Vec2){0.0f, 0.0f};
	joint->collideConnected = def->collideConnected;
	joint->userData = def->userData;

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

	b2JointId jointId = {joint->jointId + 1, world->worldId, joint->revision};
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

	b2Body* bodyA = b2GetBody(world, def->bodyIdA);
	b2Body* bodyB = b2GetBody(world, def->bodyIdB);

	b2Joint* joint = b2CreateJoint(world, bodyA, bodyB);

	joint->type = b2_mouseJoint;
	joint->localOriginAnchorA = b2InvTransformPoint(b2MakeTransform(bodyA), def->target);
	joint->localOriginAnchorB = b2InvTransformPoint(b2MakeTransform(bodyB), def->target);
	joint->collideConnected = true;
	joint->userData = def->userData;

	b2MouseJoint empty = {0};
	joint->mouseJoint = empty;
	joint->mouseJoint.targetA = def->target;
	joint->mouseJoint.hertz = def->hertz;
	joint->mouseJoint.dampingRatio = def->dampingRatio;

	b2JointId jointId = {joint->jointId + 1, world->worldId, joint->revision};
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

	b2Body* bodyA = b2GetBody(world, def->bodyIdA);
	b2Body* bodyB = b2GetBody(world, def->bodyIdB);

	b2Joint* joint = b2CreateJoint(world, bodyA, bodyB);

	joint->type = b2_revoluteJoint;
	joint->localOriginAnchorA = def->localAnchorA;
	joint->localOriginAnchorB = def->localAnchorB;
	joint->collideConnected = def->collideConnected;
	joint->userData = def->userData;
	joint->drawSize = def->drawSize;

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

	b2JointId jointId = {joint->jointId + 1, world->worldId, joint->revision};
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

	b2Body* bodyA = b2GetBody(world, def->bodyIdA);
	b2Body* bodyB = b2GetBody(world, def->bodyIdB);

	b2Joint* joint = b2CreateJoint(world, bodyA, bodyB);

	joint->type = b2_prismaticJoint;
	joint->localOriginAnchorA = def->localAnchorA;
	joint->localOriginAnchorB = def->localAnchorB;
	joint->collideConnected = def->collideConnected;
	joint->userData = def->userData;

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

	b2JointId jointId = {joint->jointId + 1, world->worldId, joint->revision};
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

	b2Body* bodyA = b2GetBody(world, def->bodyIdA);
	b2Body* bodyB = b2GetBody(world, def->bodyIdB);

	b2Joint* joint = b2CreateJoint(world, bodyA, bodyB);

	joint->type = b2_weldJoint;
	joint->localOriginAnchorA = def->localAnchorA;
	joint->localOriginAnchorB = def->localAnchorB;
	joint->collideConnected = def->collideConnected;
	joint->userData = def->userData;

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

	b2JointId jointId = {joint->jointId + 1, world->worldId, joint->revision};
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

	b2Body* bodyA = b2GetBody(world, def->bodyIdA);
	b2Body* bodyB = b2GetBody(world, def->bodyIdB);

	b2Joint* joint = b2CreateJoint(world, bodyA, bodyB);

	joint->type = b2_wheelJoint;
	joint->localOriginAnchorA = def->localAnchorA;
	joint->localOriginAnchorB = def->localAnchorB;
	joint->collideConnected = def->collideConnected;
	joint->userData = def->userData;

	// todo test this
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

	b2JointId jointId = {joint->jointId + 1, world->worldId, joint->revision};
	return jointId;
}

void b2DestroyJointInternal(b2World* world, b2Joint* joint, bool wakeBodies)
{
	b2JointEdge* edgeA = joint->edges + 0;
	b2JointEdge* edgeB = joint->edges + 1;

	int idA = edgeA->bodyId;
	int idB = edgeB->bodyId;
	b2Body* bodyA = b2GetBodyFromRawId(world, idA);
	b2Body* bodyB = b2GetBodyFromRawId(world, idB);

	// Remove from body A
	if (edgeA->prevKey != B2_NULL_INDEX)
	{
		b2Joint* prevJoint = b2GetJoint(world, edgeA->prevKey >> 1);
		b2JointEdge* prevEdge = prevJoint->edges + (edgeA->prevKey & 1);
		prevEdge->nextKey = edgeA->nextKey;
	}

	if (edgeA->nextKey != B2_NULL_INDEX)
	{
		b2Joint* nextJoint = b2GetJoint(world, edgeA->nextKey >> 1);
		b2JointEdge* nextEdge = nextJoint->edges + (edgeA->nextKey & 1);
		nextEdge->prevKey = edgeA->prevKey;
	}

	int32_t edgeKeyA = (joint->jointId << 1) | 0;
	if (bodyA->jointList == edgeKeyA)
	{
		bodyA->jointList = edgeA->nextKey;
	}

	bodyA->jointCount -= 1;

	// Remove from body B
	if (edgeB->prevKey != B2_NULL_INDEX)
	{
		b2Joint* prevJoint = b2GetJoint(world, edgeB->prevKey >> 1);
		b2JointEdge* prevEdge = prevJoint->edges + (edgeB->prevKey & 1);
		prevEdge->nextKey = edgeB->nextKey;
	}

	if (edgeB->nextKey != B2_NULL_INDEX)
	{
		b2Joint* nextJoint = b2GetJoint(world, edgeB->nextKey >> 1);
		b2JointEdge* nextEdge = nextJoint->edges + (edgeB->nextKey & 1);
		nextEdge->prevKey = edgeB->prevKey;
	}

	int32_t edgeKeyB = (joint->jointId << 1) | 1;
	if (bodyB->jointList == edgeKeyB)
	{
		bodyB->jointList = edgeB->nextKey;
	}

	bodyB->jointCount -= 1;

	b2UnlinkJoint(world, joint);

	// Remove joint from solver set that owns it
	int jointId = joint->jointId;
	b2CheckIndex(world->jointLookupArray, jointId);
	b2JointLookup* lookup = world->jointLookupArray + jointId;
	b2CheckIndex(world->solverSetArray, lookup->setIndex);

	if (lookup->setIndex == b2_awakeSet)
	{
		b2RemoveJointFromGraph(world, joint);
	}
	else
	{
		b2SolverSet* set = world->solverSetArray + lookup->setIndex;
		int movedIndex = b2RemoveJoint(&world->blockAllocator, &set->joints, lookup->localIndex);
		if (movedIndex != B2_NULL_INDEX)
		{
			// Fix lookup on moved joint
			b2Joint* movedJoint = set->joints.data + lookup->localIndex;
			int movedId = movedJoint->jointId;
			b2JointLookup* movedLookup = world->jointLookupArray + movedId;
			B2_ASSERT(movedLookup->localIndex == movedIndex);
			movedLookup->localIndex = lookup->localIndex;
		}
	}

	// Free lookup and id (preserve lookup revision)
	lookup->setIndex = B2_NULL_INDEX;
	lookup->localIndex = B2_NULL_INDEX;
	b2FreeId(&world->jointIdPool, jointId);

	if (wakeBodies)
	{
		// careful because calling these invalidates pointers
		b2WakeBody(world, idA);
		b2WakeBody(world, idB);
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

	b2Joint* joint = b2GetJointCheckRevision(world, jointId);

	b2DestroyJointInternal(world, joint, true);
}

b2JointType b2Joint_GetType(b2JointId jointId)
{
	b2World* world = b2GetWorld(jointId.world0);
	b2Joint* joint = b2GetJointCheckRevision(world, jointId);
	return joint->type;
}

b2BodyId b2Joint_GetBodyA(b2JointId jointId)
{
	b2World* world = b2GetWorld(jointId.world0);
	b2Joint* joint = b2GetJointCheckRevision(world, jointId);
	return b2MakeBodyId(world, joint->edges[0].bodyId);
}

b2BodyId b2Joint_GetBodyB(b2JointId jointId)
{
	b2World* world = b2GetWorld(jointId.world0);
	b2Joint* joint = b2GetJointCheckRevision(world, jointId);
	return b2MakeBodyId(world, joint->edges[1].bodyId);
}

b2Vec2 b2Joint_GetLocalAnchorA(b2JointId jointId)
{
	b2World* world = b2GetWorld(jointId.world0);
	b2Joint* joint = b2GetJointCheckRevision(world, jointId);
	return joint->localOriginAnchorA;
}

b2Vec2 b2Joint_GetLocalAnchorB(b2JointId jointId)
{
	b2World* world = b2GetWorld(jointId.world0);
	b2Joint* joint = b2GetJointCheckRevision(world, jointId);
	return joint->localOriginAnchorB;
}

void b2Joint_SetCollideConnected(b2JointId jointId, bool shouldCollide)
{
	b2World* world = b2GetWorldLocked(jointId.world0);
	if (world == NULL)
	{
		return;
	}

	b2Joint* joint = b2GetJointCheckRevision(world, jointId);
	if (joint->collideConnected == shouldCollide)
	{
		return;
	}

	joint->collideConnected = shouldCollide;

	b2JointEdge* edgeA = joint->edges + 0;
	b2JointEdge* edgeB = joint->edges + 1;

	int32_t bodyKeyA = edgeA->bodyId;
	int32_t bodyKeyB = edgeB->bodyId;

	b2Body* bodyA = b2GetBodyFromRawId(world, bodyKeyA);
	b2Body* bodyB = b2GetBodyFromRawId(world, bodyKeyB);

	if (shouldCollide)
	{
		// need to tell the broadphase to look for new pairs for one of the
		// two bodies. Pick the one with the fewest shapes.
		int shapeCountA = bodyA->shapeCount;
		int shapeCountB = bodyB->shapeCount;

		int shapeIndex = shapeCountA < shapeCountB ? bodyA->shapeList : bodyB->shapeList;
		while (shapeIndex != B2_NULL_INDEX)
		{
			b2Shape* shape = world->shapes + shapeIndex;

			if (shape->proxyKey != B2_NULL_INDEX)
			{
				b2BufferMove(&world->broadPhase, shape->proxyKey);
			}

			shapeIndex = shape->nextShapeIndex;
		}
	}
	else
	{
		// no collision between bodies, destroy any contact between them
		int contactCountA = bodyA->contactCount;
		int contactCountB = bodyB->contactCount;

		int32_t contactKey = contactCountA < contactCountB ? bodyA->contactList : bodyB->contactList;
		while (contactKey != B2_NULL_INDEX)
		{
			int32_t contactId = contactKey >> 1;
			int32_t edgeIndex = contactKey & 1;

			b2Contact* contact = b2GetContactFromRawId(world, contactId);
			contactKey = contact->edges[edgeIndex].nextKey;

			b2Shape* shapeA = world->shapes + contact->shapeIndexA;
			b2Shape* shapeB = world->shapes + contact->shapeIndexB;

			if ((bodyKeyA == shapeA->bodyId && bodyKeyB == shapeB->bodyId) ||
				(bodyKeyA == shapeB->bodyId && bodyKeyB == shapeA->bodyId))
			{
				b2DestroyContact(world, contact);
			}
		}
	}
}

bool b2Joint_GetCollideConnected(b2JointId jointId)
{
	b2World* world = b2GetWorld(jointId.world0);
	b2Joint* joint = b2GetJointCheckRevision(world, jointId);
	return joint->collideConnected;
}

void b2Joint_SetUserData(b2JointId jointId, void* userData)
{
	b2World* world = b2GetWorld(jointId.world0);
	b2Joint* joint = b2GetJointCheckRevision(world, jointId);
	joint->userData = userData;
}

void* b2Joint_GetUserData(b2JointId jointId)
{
	b2World* world = b2GetWorld(jointId.world0);
	b2Joint* joint = b2GetJointCheckRevision(world, jointId);
	return joint->userData;
}

void b2Joint_WakeBodies(b2JointId jointId)
{
	b2World* world = b2GetWorldLocked(jointId.world0);
	if (world == NULL)
	{
		return;
	}

	b2Joint* joint = b2GetJointCheckRevision(world, jointId);
	int idA = joint->edges[0].bodyId;
	int idB = joint->edges[1].bodyId;

	// careful because calling these invalidates pointers
	b2WakeBody(world, idA);
	b2WakeBody(world, idB);
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

	for (int32_t i = 0; i < jointCount; ++i)
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

	for (int32_t i = 0; i < jointCount; ++i)
	{
		b2Joint* joint = joints + i;
		b2SolveJoint(joint, context, useBias);
	}

	b2TracyCZoneEnd(solve_joints);
}

b2JointId b2Body_GetFirstJoint(b2BodyId bodyId)
{
	b2World* world = b2GetWorld(bodyId.world0);
	b2Body* body = b2GetBody(world, bodyId);

	if (body->jointList == B2_NULL_INDEX)
	{
		return (b2JointId){0};
	}

	b2Joint* joint = b2GetJoint(world, body->jointList);
	b2JointId id = {joint->jointId + 1, bodyId.world0, joint->revision};
	return id;
}

b2JointId b2Body_GetNextJoint(b2BodyId bodyId, b2JointId jointId)
{
	b2World* world = b2GetWorld(bodyId.world0);
	b2Body* body = b2GetBody(world, bodyId);
	b2Joint* joint = b2GetJointCheckRevision(world, jointId);

	if (joint->edges[0].bodyId == body->bodyId)
	{
		if (joint->edges[0].nextKey == B2_NULL_INDEX)
		{
			return (b2JointId){0};
		}

		joint = b2GetJoint(world, joint->edges[0].nextKey >> 1);
	}
	else
	{
		B2_ASSERT(joint->edges[1].bodyId == body->bodyId);

		if (joint->edges[1].nextKey == B2_NULL_INDEX)
		{
			return (b2JointId){0};
		}

		joint = b2GetJoint(world, joint->edges[1].nextKey >> 1);
	}

	b2JointId id = {joint->jointId + 1, bodyId.world0, joint->revision};
	return id;
}

extern void b2DrawDistance(b2DebugDraw* draw, b2Joint* base, b2Body* bodyA, b2Body* bodyB);
extern void b2DrawPrismatic(b2DebugDraw* draw, b2Joint* base, b2Body* bodyA, b2Body* bodyB);
extern void b2DrawRevolute(b2DebugDraw* draw, b2Joint* base, b2Body* bodyA, b2Body* bodyB);
extern void b2DrawWheelJoint(b2DebugDraw* draw, b2Joint* base, b2Body* bodyA, b2Body* bodyB);

void b2DrawJoint(b2DebugDraw* draw, b2World* world, b2Joint* joint)
{
	b2Body* bodyA = b2GetBodyFromRawId(world, joint->edges[0].bodyId);
	b2Body* bodyB = b2GetBodyFromRawId(world, joint->edges[1].bodyId);
	if (bodyA->isEnabled == false || bodyB->isEnabled == false)
	{
		return;
	}

	b2Transform transformA = b2MakeTransform(bodyA);
	b2Transform transformB = b2MakeTransform(bodyB);
	b2Vec2 pA = b2TransformPoint(transformA, joint->localOriginAnchorA);
	b2Vec2 pB = b2TransformPoint(transformB, joint->localOriginAnchorB);

	b2Color color = {0.5f, 0.8f, 0.8f, 1.0f};

	switch (joint->type)
	{
		case b2_distanceJoint:
			b2DrawDistance(draw, joint, bodyA, bodyB);
			break;

		case b2_mouseJoint:
		{
			b2Vec2 target = joint->mouseJoint.targetA;

			b2Color c1 = {0.0f, 1.0f, 0.0f, 1.0f};
			draw->DrawPoint(target, 4.0f, c1, draw->context);
			draw->DrawPoint(pB, 4.0f, c1, draw->context);

			b2Color c2 = {0.8f, 0.8f, 0.8f, 1.0f};
			draw->DrawSegment(target, pB, c2, draw->context);
		}
		break;

		case b2_prismaticJoint:
			b2DrawPrismatic(draw, joint, bodyA, bodyB);
			break;

		case b2_revoluteJoint:
			b2DrawRevolute(draw, joint, bodyA, bodyB);
			break;

		case b2_wheelJoint:
			b2DrawWheelJoint(draw, joint, bodyA, bodyB);
			break;

		default:
			draw->DrawSegment(transformA.p, pA, color, draw->context);
			draw->DrawSegment(pA, pB, color, draw->context);
			draw->DrawSegment(transformB.p, pB, color, draw->context);
	}

	if (draw->drawGraphColors)
	{
		b2HexColor colors[b2_graphColorCount + 1] = {
			b2_colorRed,  b2_colorOrange,	 b2_colorYellow,	b2_colorGreen, b2_colorCyan, b2_colorBlue, b2_colorViolet,
			b2_colorPink, b2_colorChocolate, b2_colorGoldenrod, b2_colorCoral, b2_colorAqua, b2_colorBlack};

		if (joint->colorIndex != B2_NULL_INDEX)
		{
			b2Vec2 p = b2Lerp(pA, pB, 0.5f);
			draw->DrawPoint(p, 5.0f, b2MakeColor(colors[joint->colorIndex]), draw->context);
		}
	}
}
