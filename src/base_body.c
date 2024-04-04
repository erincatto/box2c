// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "base_body.h"

#include "array.h"
#include "body.h"
#include "contact.h"
#include "core.h"
#include "solver_set.h"
#include "static_body.h"
#include "world.h"

// needed for dll export
#include "box2d/box2d.h"
#include "box2d/id.h"

b2BodyLookup* b2GetBody(b2World* world, int bodyId)
{
	b2CheckIndex(world->bodyLookupArray, bodyId);
	return world->bodyLookupArray + bodyId;
}

// Get a validated body from a world using an id.
// todo remove this function and instead use B2_ASSERT(b2Body_IsValid(bodyId))
b2BodyLookup* b2GetBodyFullId(b2World* world, b2BodyId bodyId)
{
	B2_ASSERT(b2Body_IsValid(bodyId));

	// id index starts at one so that zero can represent null
	return b2GetBody(world, bodyId.index1 - 1);
}

b2Transform b2GetBodyTransform(b2World* world, int bodyId)
{
	b2CheckIndex(world->bodyLookupArray, bodyId);
	b2BodyLookup lookup = world->bodyLookupArray[bodyId];
	b2CheckIndex(world->solverSetArray, lookup.setIndex);
	b2SolverSet* set = world->solverSetArray + lookup.setIndex;
	B2_ASSERT(0 <= lookup.bodyIndex && lookup.bodyIndex <= set->bodies.count);
	b2Body* body = set->bodies.data + lookup.bodyIndex;
	return body->transform;
}

// Create a b2BodyId from a key.
b2BodyId b2MakeBodyId(b2World* world, int bodyKey)
{
	B2_ASSERT(0 <= bodyKey && bodyKey < b2Array(world->bodyLookupArray).count);
	b2BodyLookup lookup = world->bodyLookupArray[bodyKey];
	B2_ASSERT(0 <= lookup.setIndex && lookup.setIndex < b2Array(world->solverSetArray).count);
	return (b2BodyId){bodyKey + 1, world->worldId, lookup.revision};
}
