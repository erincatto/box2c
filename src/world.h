// SPDX-FileCopyrightText: 2022 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "block_allocator.h"
//#include "contact_manager.h"
#include "box2d/math.h"
//#include "b2_stack_allocator.h"
//#include "b2_time_step.h"
//#include "b2_world_callbacks.h"

//struct b2AABB;
//struct b2BodyDef;
//struct b2Color;
//struct b2JointDef;
//class b2Body;
//class b2Draw;
//class b2Fixture;
//class b2Joint;

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
	b2BlockAllocator* blockAllocator;
	//b2StackAllocator* stackAllocator;

	//b2ContactManager contactManager;

	struct b2Body* bodies;
	int32_t bodyCount;
	int32_t bodyCapacity;
	int32_t bodyFreeList;

	//b2Joint* m_jointList;
	//int32_t m_jointCount;

	b2Vec2 gravity;

	//b2DestructionListener* m_destructionListener;
	//b2Draw* m_debugDraw;

	// This is used to compute the time step ratio to support a variable time step.
	float inv_dt0;

	uint16_t revision;

	bool canSleep;
	bool newContacts;
	bool locked;
	bool warmStarting;
} b2World;

//void b2Solve(b2World* world, const b2TimeStep* step);

//void b2DrawShape(b2World* world, b2Shape* shape, const b2Transform& xf, const b2Color& color);
