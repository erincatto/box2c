// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "box2d/distance.h"
#include "box2d/id.h"
#include "box2d/math.h"

#include "pool.h"

typedef struct b2Polygon b2Polygon;
typedef struct b2World b2World;

// A rigid body
typedef struct b2Body
{
	b2Object object;

	enum b2BodyType type;

	// the body origin transform
	b2Transform transform;
	
	// center of mass position
	b2Vec2 position;

	// rotation in radians
	float angle;

	// location of center of mass relative to the body origin
	b2Vec2 localCenter;

	b2Vec2 speculativePosition;
	float speculativeAngle;

	b2Vec2 linearVelocity;
	float angularVelocity;

	b2Vec2 force;
	float torque;

	int32_t shapeIndex;
	int32_t jointIndex;

	struct b2ContactEdge* contacts;
	int32_t contactCount;

	B2_ATOMIC long awakeIndex;

	uint64_t islandId;

	float mass, invMass;

	// Rotational inertia about the center of mass.
	float I, invI;

	float linearDamping;
	float angularDamping;
	float gravityScale;

	float sleepTime;

	void* userData;
	int16_t world;

	bool enableSleep;
	bool fixedRotation;
	bool isEnabled;
} b2Body;

void b2SetAwake(b2World* world, b2Body* body, bool flag);

bool b2ShouldBodiesCollide(b2World* world, b2Body* bodyA, b2Body* bodyB);

b2ShapeId b2Body_CreatePolygon(b2BodyId bodyId, const b2ShapeDef* def, const b2Polygon* polygon);
void b2Body_DestroyShape(b2ShapeId shapeId);

static inline b2Sweep b2Body_GetSweep(const b2Body* body)
{
	b2Sweep s;
	if (body->type == b2_staticBody)
	{
		s.c1 = body->position;
		s.c2 = body->position;
		s.a1 = body->angle;
		s.a2 = body->angle;
	}
	else
	{
		s.c1 = body->position;
		s.c2 = body->speculativePosition;
		s.a1 = body->angle;
		s.a2 = body->speculativeAngle;
	}

	s.localCenter = body->localCenter;
	return s;
}
