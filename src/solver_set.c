// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "solver_set.h"

#include "core.h"
#include "body.h"
#include "contact.h"
#include "island.h"
#include "joint.h"
#include "world.h"

#include <string.h>

void b2DestroySolverSet(b2World* world, int setId)
{
	b2SolverSet* set = world->solverSetArray + setId;
	b2DestroyBodySimArray(&world->blockAllocator, &set->sims);
	b2DestroyBodyStateArray(&world->blockAllocator, &set->states);
	b2DestroyContactArray(&world->blockAllocator, &set->contacts);
	b2DestroyJointArray(&world->blockAllocator, &set->joints);
	b2DestroyIslandArray(&world->blockAllocator, &set->islands);
	b2FreeId(&world->solverSetIdPool, setId);
	*set = (b2SolverSet){0};
	set->setId = B2_NULL_INDEX;
}

// Wake a solver set. Does not merge islands.
// Contacts can be in several places:
// 1. non-touching contacts in the disabled set
// 2. non-touching contacts already in the awake set
// 3. touching contacts in the sleeping set
// This handles contact types 1 and 3. Type 2 doesn't need any action.
void b2WakeSolverSet(b2World* world, int setId)
{
	b2ValidateWorld(world);

	B2_ASSERT(setId >= b2_firstSleepingSet);
	b2CheckIndex(world->solverSetArray, setId);
	b2SolverSet* set = world->solverSetArray + setId;
	b2SolverSet* awakeSet = world->solverSetArray + b2_awakeSet;
	b2SolverSet* disabledSet = world->solverSetArray + b2_disabledSet;

	b2BlockAllocator* alloc = &world->blockAllocator;
	b2Body* bodies = world->bodyArray;
	b2ContactLookup* contactLookups = world->contactLookupArray;

	int bodyCount = set->sims.count;
	for (int i = 0; i < bodyCount; ++i)
	{
		b2BodySim* simSrc = set->sims.data + i;
		
		b2Body* body = bodies + simSrc->bodyId;
		B2_ASSERT(body->setIndex == setId);
		body->setIndex = b2_awakeSet;
		body->localIndex = awakeSet->sims.count;

		b2BodySim* simDst = b2AddBodySim(alloc, &awakeSet->sims);
		memcpy(simDst, simSrc, sizeof(b2BodySim));

		b2BodyState* state = b2AddBodyState(alloc, &awakeSet->states);
		*state = b2_identityBodyState;

		// move non-touching contacts from disabled set to awake set
		int contactKey = body->headContactKey;
		while (contactKey != B2_NULL_INDEX)
		{
			int edgeIndex = contactKey & 1;
			int contactId = contactKey >> 1;

			b2CheckIndex(contactLookups, contactId);
			b2ContactLookup* contactLookup = contactLookups + contactId;

			b2Contact* contact = b2GetContactFromRawId(world, contactId);
			contactKey = contact->edges[edgeIndex].nextKey;

			if (contactLookup->setIndex != b2_disabledSet)
			{
				B2_ASSERT(contactLookup->setIndex == b2_awakeSet || contactLookup->setIndex == setId);
				continue;
			}

			B2_ASSERT((contact->flags & b2_contactTouchingFlag) == 0 && contact->manifold.pointCount == 0);

			int contactLocalIndex = contactLookup->localIndex;
			contactLookup->setIndex = b2_awakeSet;
			contactLookup->localIndex = awakeSet->contacts.count;
			b2Contact* awakeContact = b2AddContact(&world->blockAllocator, &awakeSet->contacts);
			memcpy(awakeContact, contact, sizeof(b2Contact));

			int movedContactIndex = b2RemoveContact(&disabledSet->contacts, contactLocalIndex);
			if (movedContactIndex != B2_NULL_INDEX)
			{
				// fix lookup on moved element
				b2Contact* movedContact = disabledSet->contacts.data + contactLocalIndex;
				int movedContactId = movedContact->contactId;
				b2CheckIndex(contactLookups, movedContactId);
				b2ContactLookup* movedContactLookup = contactLookups + movedContactId;
				B2_ASSERT(movedContactLookup->localIndex == movedContactIndex);
				movedContactLookup->localIndex = contactLocalIndex;
			}
		}
	}

	// transfer touching contacts from sleeping set to contact graph
	{
		int contactCount = set->contacts.count;
		for (int i = 0; i < contactCount; ++i)
		{
			b2Contact* contact = set->contacts.data + i;
			b2ContactLookup* contactLookup = contactLookups + contact->contactId;
			B2_ASSERT(contactLookup->setIndex == setId);
			b2AddContactToGraph(world, contact, contactLookup);
			contactLookup->setIndex = b2_awakeSet;
		}
	}

	// transfer joints from sleeping set to awake set
	{
		b2JointLookup* jointLookups = world->jointLookupArray;
		int jointCount = set->joints.count;
		for (int i = 0; i < jointCount; ++i)
		{
			b2Joint* joint = set->joints.data + i;
			b2JointLookup* jointLookup = jointLookups + joint->jointId;
			B2_ASSERT(jointLookup->setIndex == setId);
			b2AddJointToGraph(world, joint, jointLookup);
			jointLookup->setIndex = b2_awakeSet;
		}
	}

	// transfer island from sleeping set to awake set
	// Usually a sleeping set has only one island, but it is possible
	// that joints are created between sleeping islands and they
	// are moved to the same sleeping set.
	{
		b2Island* islands = world->islandArray;
		int islandCount = set->islands.count;
		for (int i = 0; i < islandCount; ++i)
		{
			b2IslandSim* islandSrc = set->islands.data + i;
			b2CheckIndex(islands, islandSrc->islandId);
			b2Island* islandLookup = islands + islandSrc->islandId;
			islandLookup->setIndex = b2_awakeSet;
			islandLookup->localIndex = awakeSet->islands.count;
			b2IslandSim* islandDst = b2AddIsland(alloc, &awakeSet->islands);
			memcpy(islandDst, islandSrc, sizeof(b2IslandSim));
		}
	}

	// destroy the sleeping set
	b2DestroySolverSet(world, setId);

	b2ValidateWorld(world);
}

void b2TrySleepIsland(b2World* world, int islandId)
{
	b2CheckIndex(world->islandArray, islandId);
	b2Island* island = world->islandArray + islandId;
	B2_ASSERT(island->setIndex == b2_awakeSet);

	// cannot put an island to sleep while it has a pending split
	if (island->constraintRemoveCount > 0)
	{
		return;
	}

	// island is sleeping
	// - create new sleeping solver set
	// - move island to sleeping solver set
	// - identify non-touching contacts that should move to sleeping solver set or disabled set
	// - remove old island
	// - fix island lookup
	int sleepSetId = b2AllocId(&world->solverSetIdPool);
	if (sleepSetId == b2Array(world->solverSetArray).count)
	{
		b2SolverSet set = {0};
		set.setId = B2_NULL_INDEX;
		b2Array_Push(world->solverSetArray, set);
	}

	b2SolverSet* sleepSet = world->solverSetArray + sleepSetId;
	*sleepSet = (b2SolverSet){0};

	// grab awake set after creating the sleep set because the solver set array may have been resized
	b2SolverSet* awakeSet = world->solverSetArray + b2_awakeSet;
	B2_ASSERT(0 <= island->localIndex && island->localIndex < awakeSet->islands.count);
	b2IslandSim* islandSim = awakeSet->islands.data + island->localIndex;

	sleepSet->setId = sleepSetId;
	sleepSet->sims = b2CreateBodySimArray(&world->blockAllocator, island->bodyCount);
	sleepSet->contacts = b2CreateContactArray(&world->blockAllocator, island->contactCount);
	sleepSet->joints = b2CreateJointArray(&world->blockAllocator, island->jointCount);

	// move awake bodies to sleeping set
	// this shuffles around bodies in the awake set
	{
		b2SolverSet* disabledSet = world->solverSetArray + b2_disabledSet;
		b2Body* bodies = world->bodyArray;
		b2ContactLookup* contactLookups = world->contactLookupArray;
		int bodyId = island->headBody;
		while (bodyId != B2_NULL_INDEX)
		{
			b2CheckIndex(bodies, bodyId);
			b2Body* body = bodies + bodyId;
			B2_ASSERT(body->setIndex == b2_awakeSet);
			B2_ASSERT(body->islandId == islandId);
			int awakeBodyIndex = body->localIndex;
			B2_ASSERT(0 <= awakeBodyIndex && awakeBodyIndex < awakeSet->sims.count);

			b2BodySim* awakeSim = awakeSet->sims.data + awakeBodyIndex;

			// move body sim to sleep set
			int sleepBodyIndex = sleepSet->sims.count;
			b2BodySim* sleepBody = b2AddBodySim(&world->blockAllocator, &sleepSet->sims);
			memcpy(sleepBody, awakeSim, sizeof(b2BodySim));

			int movedIndex = b2RemoveBodySim(&awakeSet->sims, awakeBodyIndex);
			if (movedIndex != B2_NULL_INDEX)
			{
				// fix local index on moved element
				b2BodySim* movedSim = awakeSet->sims.data + awakeBodyIndex;
				int movedId = movedSim->bodyId;
				b2CheckIndex(bodies, movedId);
				b2Body* movedBody = bodies + movedId;
				B2_ASSERT(movedBody->localIndex == movedIndex);
				movedBody->localIndex = awakeBodyIndex;
			}

			// destroy state, no need to clone
			b2RemoveBodyState(&awakeSet->states, awakeBodyIndex);

			body->setIndex = sleepSetId;
			body->localIndex = sleepBodyIndex;

			// Move non-touching contacts to the disabled set.
			// Non-touching contacts may exist between sleeping islands and there is no clear ownership.
			int contactKey = body->headContactKey;
			while (contactKey != B2_NULL_INDEX)
			{
				int edgeIndex = contactKey & 1;
				int contactId = contactKey >> 1;

				b2CheckIndex(contactLookups, contactId);
				b2ContactLookup* contactLookup = contactLookups + contactId;

				B2_ASSERT(contactLookup->setIndex == b2_awakeSet || contactLookup->setIndex == b2_disabledSet);

				b2Contact* contact = b2GetContactFromRawId(world, contactId);
				contactKey = contact->edges[edgeIndex].nextKey;

				if (contactLookup->setIndex == b2_disabledSet)
				{
					// already disabled by another body in the island
					continue;
				}

				if (contactLookup->colorIndex != B2_NULL_INDEX)
				{
					// contact is touching and will be moved separately
					B2_ASSERT((contact->flags & b2_contactTouchingFlag) != 0 && contact->manifold.pointCount > 0);
					continue;
				}

				// the other body may still be awake, it still may go to sleep and then it will be responsible
				// for moving this contact to the disabled set.
				int otherEdgeIndex = edgeIndex ^ 1;
				int otherBodyId = contact->edges[otherEdgeIndex].bodyId;
				b2CheckIndex(bodies, otherBodyId);
				b2Body* otherBody = bodies + otherBodyId;
				if (otherBody->setIndex == b2_awakeSet)
				{
					continue;
				}

				// move the non-touching contact to the disabled set
				B2_ASSERT((contact->flags & b2_contactTouchingFlag) == 0 && contact->manifold.pointCount == 0);

				int contactLocalIndex = contactLookup->localIndex;
				contactLookup->setIndex = b2_disabledSet;
				contactLookup->localIndex = disabledSet->contacts.count;
				b2Contact* disabledContact = b2AddContact(&world->blockAllocator, &disabledSet->contacts);
				memcpy(disabledContact, contact, sizeof(b2Contact));

				int movedContactIndex = b2RemoveContact(&awakeSet->contacts, contactLocalIndex);
				if (movedContactIndex != B2_NULL_INDEX)
				{
					// fix lookup on moved element
					b2Contact* movedContact = awakeSet->contacts.data + contactLocalIndex;
					int movedContactId = movedContact->contactId;
					b2CheckIndex(contactLookups, movedContactId);
					b2ContactLookup* movedContactLookup = contactLookups + movedContactId;
					B2_ASSERT(movedContactLookup->localIndex == movedContactIndex);
					movedContactLookup->localIndex = contactLocalIndex;
				}
			}

			bodyId = body->islandNext;
		}
	}

	// move touching contacts
	// this shuffles contacts in the awake set
	{
		b2ContactLookup* contactLookups = world->contactLookupArray;
		int contactId = island->headContact;
		while (contactId != B2_NULL_INDEX)
		{
			b2CheckIndex(contactLookups, contactId);
			b2ContactLookup* contactLookup = contactLookups + contactId;
			B2_ASSERT(contactLookup->setIndex == b2_awakeSet);
			int colorIndex = contactLookup->colorIndex;
			int awakeContactIndex = contactLookup->localIndex;

			B2_ASSERT(0 <= colorIndex && colorIndex < b2_graphColorCount);

			b2GraphColor* color = world->constraintGraph.colors + colorIndex;

			B2_ASSERT(0 <= awakeContactIndex && awakeContactIndex < color->contacts.count);

			b2Contact* awakeContact = color->contacts.data + awakeContactIndex;
			B2_ASSERT(awakeContact->islandId == islandId);

			// Remove bodies from graph coloring associated with this constraint
			if (colorIndex != b2_overflowIndex)
			{
				// might clear a bit for a static body, but this has no effect
				b2ClearBit(&color->bodySet, awakeContact->edges[0].bodyId);
				b2ClearBit(&color->bodySet, awakeContact->edges[1].bodyId);
			}

			int sleepContactIndex = sleepSet->contacts.count;
			b2Contact* sleepContact = b2AddContact(&world->blockAllocator, &sleepSet->contacts);
			memcpy(sleepContact, awakeContact, sizeof(b2Contact));

			int movedIndex = b2RemoveContact(&color->contacts, awakeContactIndex);
			if (movedIndex != B2_NULL_INDEX)
			{
				// fix lookup on moved element
				b2Contact* movedContact = color->contacts.data + awakeContactIndex;
				int movedId = movedContact->contactId;
				b2CheckIndex(contactLookups, movedId);
				b2ContactLookup* movedLookup = contactLookups + movedId;
				B2_ASSERT(movedLookup->localIndex == movedIndex);
				movedLookup->localIndex = awakeContactIndex;
			}

			contactLookup->setIndex = sleepSetId;
			contactLookup->colorIndex = B2_NULL_INDEX;
			contactLookup->localIndex = sleepContactIndex;

			contactId = sleepContact->islandNext;
		}
	}

	// move joints
	// this shuffles joints in the awake set
	{
		b2JointLookup* jointLookups = world->jointLookupArray;
		int jointId = island->headJoint;
		while (jointId != B2_NULL_INDEX)
		{
			b2CheckIndex(jointLookups, jointId);
			b2JointLookup* jointLookup = jointLookups + jointId;
			B2_ASSERT(jointLookup->setIndex == b2_awakeSet);
			int colorIndex = jointLookup->colorIndex;
			int awakeJointIndex = jointLookup->localIndex;

			B2_ASSERT(0 <= colorIndex && colorIndex < b2_graphColorCount);

			b2GraphColor* color = world->constraintGraph.colors + colorIndex;

			B2_ASSERT(0 <= awakeJointIndex && awakeJointIndex < color->joints.count);

			b2Joint* awakeJoint = color->joints.data + awakeJointIndex;
			B2_ASSERT(awakeJoint->islandId == islandId);

			if (colorIndex != b2_overflowIndex)
			{
				// might clear a bit for a static body, but this has no effect
				b2ClearBit(&color->bodySet, awakeJoint->edges[0].bodyId);
				b2ClearBit(&color->bodySet, awakeJoint->edges[1].bodyId);
			}

			int sleepJointIndex = sleepSet->joints.count;
			b2Joint* sleepJoint = b2AddJoint(&world->blockAllocator, &sleepSet->joints);
			memcpy(sleepJoint, awakeJoint, sizeof(b2Joint));

			int movedIndex = b2RemoveJoint(&color->joints, awakeJointIndex);
			if (movedIndex != B2_NULL_INDEX)
			{
				// fix lookup on moved element
				b2Joint* movedJoint = color->joints.data + awakeJointIndex;
				int movedId = movedJoint->jointId;
				b2CheckIndex(jointLookups, movedId);
				b2JointLookup* movedLookup = jointLookups + movedId;
				B2_ASSERT(movedLookup->localIndex == movedIndex);
				movedLookup->localIndex = awakeJointIndex;
			}

			jointLookup->setIndex = sleepSetId;
			jointLookup->colorIndex = B2_NULL_INDEX;
			jointLookup->localIndex = sleepJointIndex;

			jointId = sleepJoint->islandNext;
		}
	}

	// move island struct
	{
		B2_ASSERT(island->setIndex == b2_awakeSet);

		int islandIndex = island->localIndex;
		b2IslandSim* sleepIsland = b2AddIsland(&world->blockAllocator, &sleepSet->islands);
		sleepIsland->islandId = islandId;

		int movedIslandIndex = b2RemoveIsland(&awakeSet->islands, islandIndex);
		if (movedIslandIndex != B2_NULL_INDEX)
		{
			// fix index on moved element
			b2IslandSim* movedIslandSim = awakeSet->islands.data + islandIndex;
			int movedIslandId = movedIslandSim->islandId;
			b2CheckIndex(world->islandArray, movedIslandId);
			b2Island* movedIsland = world->islandArray + movedIslandId;
			B2_ASSERT(movedIsland->localIndex == movedIslandIndex);
			movedIsland->localIndex = islandIndex;
		}

		island->setIndex = sleepSetId;
		island->localIndex = 0;
	}

	b2ValidateWorld(world);
}

// This is called when joints are created between sets. I want to allow the sets
// to continue sleeping if both are asleep. Otherwise one set is waked.
// Islands will get merge when the set is waked.
void b2MergeSolverSets(b2World* world, int setId1, int setId2)
{
	B2_ASSERT(setId1 >= b2_firstSleepingSet);
	B2_ASSERT(setId2 >= b2_firstSleepingSet);
	b2CheckIndex(world->solverSetArray, setId1);
	b2CheckIndex(world->solverSetArray, setId2);
	b2SolverSet* set1 = world->solverSetArray + setId1;
	b2SolverSet* set2 = world->solverSetArray + setId2;

	// Move the fewest number of bodies
	if (set1->sims.count < set2->sims.count)
	{
		b2SolverSet* tempSet = set1;
		set1 = set2;
		set2 = tempSet;

		int tempId = setId1;
		setId1 = setId2;
		setId2 = tempId;
	}

	b2BlockAllocator* alloc = &world->blockAllocator;

	// transfer bodies
	{
		b2Body* bodies = world->bodyArray;
		int bodyCount = set2->sims.count;
		for (int i = 0; i < bodyCount; ++i)
		{
			b2BodySim* simSrc = set2->sims.data + i;

			b2Body* body = bodies + simSrc->bodyId;
			B2_ASSERT(body->setIndex == setId2);
			body->setIndex = setId1;
			body->localIndex = set1->sims.count;

			b2BodySim* simDst = b2AddBodySim(alloc, &set1->sims);
			memcpy(simDst, simSrc, sizeof(b2BodySim));
		}
	}

	// transfer contacts
	{
		b2ContactLookup* contactLookups = world->contactLookupArray;
		int contactCount = set2->contacts.count;
		for (int i = 0; i < contactCount; ++i)
		{
			b2Contact* contactSrc = set2->contacts.data + i;
			
			b2ContactLookup* contactLookup = contactLookups + contactSrc->contactId;
			B2_ASSERT(contactLookup->setIndex == setId2);
			contactLookup->setIndex = setId1;
			contactLookup->localIndex = set1->contacts.count;

			b2Contact* contactDst = b2AddContact(alloc, &set1->contacts);
			memcpy(contactDst, contactSrc, sizeof(b2Contact));
		}
	}

	// transfer joints
	{
		b2JointLookup* jointLookups = world->jointLookupArray;
		int jointCount = set2->joints.count;
		for (int i = 0; i < jointCount; ++i)
		{
			b2Joint* jointSrc = set2->joints.data + i;

			b2JointLookup* jointLookup = jointLookups + jointSrc->jointId;
			B2_ASSERT(jointLookup->setIndex == setId2);
			jointLookup->setIndex = setId1;
			jointLookup->localIndex = set1->joints.count;

			b2Joint* jointDst = b2AddJoint(alloc, &set1->joints);
			memcpy(jointDst, jointSrc, sizeof(b2Joint));
		}
	}

	// transfer islands
	{
		b2Island* islandLookups = world->islandArray;
		int islandCount = set2->islands.count;
		for (int i = 0; i < islandCount; ++i)
		{
			b2IslandSim* islandSrc = set2->islands.data + i;
			int islandId = islandSrc->islandId;

			b2CheckIndex(islandLookups, islandId);
			b2Island* islandLookup = islandLookups + islandId;
			islandLookup->setIndex = setId1;
			islandLookup->localIndex = set1->islands.count;
			
			b2IslandSim* islandDst = b2AddIsland(alloc, &set1->islands);
			islandDst->islandId = islandId;
		}
	}

	// destroy the merged set
	b2DestroySolverSet(world, setId2);

	b2ValidateWorld(world);
}
