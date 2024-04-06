// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "solver_set.h"

#include "core.h"
#include "body.h"
#include "contact.h"
#include "island.h"
#include "joint.h"
#include "world.h"
#include "util.h"

void b2DestroySolverSet(b2World* world, int setId)
{
	b2SolverSet* set = world->solverSetArray + setId;
	b2DestroyBodyArray(&world->blockAllocator, &set->sims);
	b2DestroyBodyStateArray(&world->blockAllocator, &set->states);
	b2DestroyContactArray(&world->blockAllocator, &set->contacts);
	b2DestroyJointArray(&world->blockAllocator, &set->joints);
	b2DestroyIslandArray(&world->blockAllocator, &set->islands);
	b2FreeId(&world->solverSetIdPool, setId);
}

void b2WakeSolverSet(b2World* world, int setId)
{
	B2_ASSERT(setId >= b2_firstSleepingSet);
	b2CheckIndex(world->solverSetArray, setId);
	b2SolverSet* set = world->solverSetArray + setId;

	b2SolverSet* awakeSet = world->solverSetArray + b2_awakeSet;

	b2BlockAllocator* alloc = &world->blockAllocator;
	b2Body* bodyLookups = world->bodyArray;
	b2ContactLookup* contactLookups = world->contactLookupArray;
	b2JointLookup* jointLookups = world->jointLookupArray;
	b2IslandLookup* islandLookups = world->islandLookupArray;

	int bodyCount = set->sims.count;
	for (int i = 0; i < bodyCount; ++i)
	{
		b2Body* body1 = set->sims.data + i;
		b2Body* body2 = b2AddBody(alloc, &awakeSet->sims);
		b2Body* lookup = bodyLookups->
	}

	b2DestroySolverSet(world, setId);
}

void b2MergeSolverSets(b2World* world, int setIndex1, int setIndex2)
{

}
