// SPDX-FileCopyrightText: 2022 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "block_allocator.h"
#include "pool.h"

#include "box2d/math.h"

#include "broad_phase.h"

/// This is an internal structure.
typedef struct b2TimeStep
{
	// time step
	float dt;

	 // inverse time step (0 if dt == 0).
	float inv_dt;

	// ratio between current and previous time step (dt * inv_dt0)
	float dtRatio;

	// Velocity iterations for constraint solver. Controls the accuracy of internal forces.
	int32_t velocityIterations;

	// Position iterations for constraint solver. Controls the accuracy of shape overlap and joint alignment.
	int32_t positionIterations;

	// For testing
	bool warmStarting;
} b2TimeStep;


/// The world class manages all physics entities, dynamic simulation,
/// and asynchronous queries. The world also contains efficient memory
/// management facilities.
typedef struct b2World
{
	int32_t index;

	b2BlockAllocator* blockAllocator;
	//b2StackAllocator* stackAllocator;

	b2BroadPhase broadPhase;

	b2Pool bodyPool;
	b2Pool shapePool;

	struct b2Body* bodies;
	struct b2Shape* shapes;
	//b2Joint* m_jointList;
	//int32_t m_jointCount;

	struct b2Contact* contacts;
	int32_t contactCount;

	b2Vec2 gravity;

	//b2DestructionListener* m_destructionListener;
	//b2Draw* m_debugDraw;

	// This is used to compute the time step ratio to support a variable time step.
	float inv_dt0;

	uint16_t revision;

	b2Profile profile;

	bool canSleep;
	bool newContacts;
	bool locked;
	bool warmStarting;
} b2World;

b2World* b2GetWorldFromId(b2WorldId id);
b2World* b2GetWorldFromIndex(int16_t index);

	//void b2Solve(b2World* world, const b2TimeStep* step);

//void b2DrawShape(b2World* world, b2Shape* shape, const b2Transform& xf, const b2Color& color);
