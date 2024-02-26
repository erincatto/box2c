// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "box2d/box2d.h"
#include "box2d/geometry.h"
#include "box2d/math.h"
#include "test_macros.h"

#include <stdio.h>

// This is a simple example of building and running a simulation
// using Box2D. Here we create a large ground box and a small dynamic
// box.
// There are no graphics for this example. Box2D is meant to be used
// with your rendering engine in your game engine.
int HelloWorld(void)
{
	// Define the gravity vector.
	b2Vec2 gravity = {0.0f, -10.0f};

	// Construct a world object, which will hold and simulate the rigid bodies.
	b2WorldDef worldDef = b2DefaultWorldDef();
	worldDef.gravity = gravity;

	b2WorldId worldId = b2CreateWorld(&worldDef);

	// Define the ground body.
	b2BodyDef groundBodyDef = b2DefaultBodyDef();
	groundBodyDef.position = (b2Vec2){0.0f, -10.0f};

	// Call the body factory which allocates memory for the ground body
	// from a pool and creates the ground box shape (also from a pool).
	// The body is also added to the world.
	b2BodyId groundBodyId = b2CreateBody(worldId, &groundBodyDef);

	// Define the ground box shape. The extents are the half-widths of the box.
	b2Polygon groundBox = b2MakeBox(50.0f, 10.0f);

	// Add the box shape to the ground body.
	b2ShapeDef groundShapeDef = b2DefaultShapeDef();
	b2CreatePolygonShape(groundBodyId, &groundShapeDef, &groundBox);

	// Define the dynamic body. We set its position and call the body factory.
	b2BodyDef bodyDef = b2DefaultBodyDef();
	bodyDef.type = b2_dynamicBody;
	bodyDef.position = (b2Vec2){0.0f, 4.0f};

	b2BodyId bodyId = b2CreateBody(worldId, &bodyDef);

	// Define another box shape for our dynamic body.
	b2Polygon dynamicBox = b2MakeBox(1.0f, 1.0f);

	// Define the dynamic body shape
	b2ShapeDef shapeDef = b2DefaultShapeDef();

	// Set the box density to be non-zero, so it will be dynamic.
	shapeDef.density = 1.0f;

	// Override the default friction.
	shapeDef.friction = 0.3f;

	// Add the shape to the body.
	b2CreatePolygonShape(bodyId, &shapeDef, &dynamicBox);

	// Prepare for simulation. Typically we use a time step of 1/60 of a
	// second (60Hz) and 4 sub-steps. This provides a high quality simulation
	// in most game scenarios.
	float timeStep = 1.0f / 60.0f;
	int32_t subStepCount = 4;

	b2Vec2 position = b2Body_GetPosition(bodyId);
	float angle = b2Body_GetAngle(bodyId);

	// This is our little game loop.
	for (int32_t i = 0; i < 90; ++i)
	{
		// Instruct the world to perform a single step of simulation.
		// It is generally best to keep the time step and iterations fixed.
		b2World_Step(worldId, timeStep, subStepCount);

		// Now print the position and angle of the body.
		position = b2Body_GetPosition(bodyId);
		angle = b2Body_GetAngle(bodyId);

		//printf("%4.2f %4.2f %4.2f\n", position.x, position.y, angle);
	}

	// When the world destructor is called, all bodies and joints are freed. This can
	// create orphaned ids, so be careful about your world management.
	b2DestroyWorld(worldId);

	ENSURE(B2_ABS(position.x) < 0.01f);
	ENSURE(B2_ABS(position.y - 1.00f) < 0.01f);
	ENSURE(B2_ABS(angle) < 0.01f);

	return 0;
}

int EmptyWorld(void)
{
	b2WorldDef worldDef = b2DefaultWorldDef();
	b2WorldId worldId = b2CreateWorld(&worldDef);
	ENSURE(b2World_IsValid(worldId) == true);

	float timeStep = 1.0f / 60.0f;
	int32_t subStepCount = 1;

	for (int32_t i = 0; i < 60; ++i)
	{
		b2World_Step(worldId, timeStep, subStepCount);
	}

	b2DestroyWorld(worldId);

	ENSURE(b2World_IsValid(worldId) == false);

	return 0;
}

#define BODY_COUNT 10
int DestroyAllBodiesWorld(void)
{
	b2WorldDef worldDef = b2DefaultWorldDef();
	b2WorldId worldId = b2CreateWorld(&worldDef);
	ENSURE(b2World_IsValid(worldId) == true);

	int count = 0;
	bool creating = true;

	b2BodyId bodyIds[BODY_COUNT];
	b2BodyDef bodyDef = b2DefaultBodyDef();
	bodyDef.type = b2_dynamicBody;
	b2Polygon square = b2MakeSquare(0.5f);

	for (int32_t i = 0; i < 2 * BODY_COUNT + 10; ++i)
	{
		if (creating)
		{
			if (count < BODY_COUNT)
			{
				bodyIds[count] = b2CreateBody(worldId, &bodyDef);

				b2ShapeDef shapeDef = b2DefaultShapeDef();
				b2CreatePolygonShape(bodyIds[count], &shapeDef, &square);
				count += 1;
			}
			else
			{
				creating = false;
			}
		}
		else if (count > 0)
		{
			b2DestroyBody(bodyIds[count - 1]);
			bodyIds[count - 1] = b2_nullBodyId;
			count -= 1;
		}

		b2World_Step(worldId, 1.0f / 60.0f, 3);
	}

	b2Counters counters = b2World_GetCounters(worldId);
	ENSURE(counters.bodyCount == 0);

	b2DestroyWorld(worldId);

	ENSURE(b2World_IsValid(worldId) == false);

	return 0;
}

int WorldTest(void)
{
	RUN_SUBTEST(HelloWorld);
	RUN_SUBTEST(EmptyWorld);
	RUN_SUBTEST(DestroyAllBodiesWorld);

	return 0;
}
