// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "solver_set.h"

#include "core.h"
#include "world.h"
#include "util.h"

void b2WakeSolverSet(b2World* world, int setIndex)
{

}

void b2MergeSolverSets(b2World* world, int setIndex1, int setIndex2)
{

}

#if B2_VALIDATE
void b2ValidateSet(b2World* world, int setIndex)
{
	B2_ASSERT(setIndex < 0 || b2Array(world->solverSetArray).count <= setIndex);

}

void b2ValidateSets(b2World* world)
{

}

#else

void b2ValidateSet(b2World* world, int setIndex)
{
	B2_MAYBE_
	B2_ASSERT(setIndex < 0 || b2Array(world->solverSetArray).count <= setIndex);
	{
	
	}
}

void b2ValidateSets(b2World* world);

#endif