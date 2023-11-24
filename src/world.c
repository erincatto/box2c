// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#define _CRT_SECURE_NO_WARNINGS

#include "world.h"

#include "allocate.h"
#include "array.h"
#include "bitset.h"
#include "block_allocator.h"
#include "body.h"
#include "broad_phase.h"
#include "contact.h"
#include "core.h"
#include "graph.h"
#include "island.h"
#include "joint.h"
#include "pool.h"
#include "shape.h"
#include "solver_data.h"
#include "stack_allocator.h"

#include "box2d/aabb.h"
#include "box2d/box2d.h"
#include "box2d/constants.h"
#include "box2d/debug_draw.h"
#include "box2d/distance.h"
#include "box2d/timer.h"

#include <float.h>
#include <stdio.h>
#include <string.h>

b2World b2_worlds[b2_maxWorlds];
bool b2_parallel = true;

b2World* b2GetWorldFromId(b2WorldId id)
{
	B2_ASSERT(0 <= id.index && id.index < b2_maxWorlds);
	b2World* world = b2_worlds + id.index;
	B2_ASSERT(id.revision == world->revision);
	return world;
}

b2World* b2GetWorldFromIndex(int16_t index)
{
	B2_ASSERT(0 <= index && index < b2_maxWorlds);
	b2World* world = b2_worlds + index;
	B2_ASSERT(world->blockAllocator != NULL);
	return world;
}

static void* b2DefaultAddTaskFcn(b2TaskCallback* task, int32_t count, int32_t minRange, void* taskContext, void* userContext)
{
	B2_MAYBE_UNUSED(minRange);
	B2_MAYBE_UNUSED(userContext);
	task(0, count, 0, taskContext);
	return NULL;
}

static void b2DefaultFinishTaskFcn(void* userTask, void* userContext)
{
	B2_MAYBE_UNUSED(userTask);
	B2_MAYBE_UNUSED(userContext);
}

static void b2DefaultFinishAllTasksFcn(void* userContext)
{
	B2_MAYBE_UNUSED(userContext);
}

b2WorldId b2CreateWorld(const b2WorldDef* def)
{
	b2WorldId id = b2_nullWorldId;
	for (int16_t i = 0; i < b2_maxWorlds; ++i)
	{
		if (b2_worlds[i].blockAllocator == NULL)
		{
			id.index = i;
			break;
		}
	}

	if (id.index == b2_nullWorldId.index)
	{
		return id;
	}

	b2InitializeContactRegisters();

	b2World empty = {0};
	b2World* world = b2_worlds + id.index;
	*world = empty;

	world->index = id.index;

	world->blockAllocator = b2CreateBlockAllocator();
	world->stackAllocator = b2CreateStackAllocator(def->stackAllocatorCapacity);

	b2CreateBroadPhase(&world->broadPhase);
	b2CreateGraph(&world->graph, def->bodyCapacity, def->contactCapacity, def->jointCapacity);

	// pools
	world->bodyPool = b2CreatePool(sizeof(b2Body), B2_MAX(def->bodyCapacity, 1));
	world->bodies = (b2Body*)world->bodyPool.memory;

	world->shapePool = b2CreatePool(sizeof(b2Shape), B2_MAX(def->shapeCapacity, 1));
	world->shapes = (b2Shape*)world->shapePool.memory;

	world->chainPool = b2CreatePool(sizeof(b2ChainShape), 4);
	world->chains = (b2ChainShape*)world->chainPool.memory;

	world->contactPool = b2CreatePool(sizeof(b2Contact), B2_MAX(def->contactCapacity, 1));
	world->contacts = (b2Contact*)world->contactPool.memory;

	world->jointPool = b2CreatePool(sizeof(b2Joint), B2_MAX(def->jointCapacity, 1));
	world->joints = (b2Joint*)world->jointPool.memory;

	world->islandPool = b2CreatePool(sizeof(b2Island), B2_MAX(def->bodyCapacity, 1));
	world->islands = (b2Island*)world->islandPool.memory;

	world->awakeIslandArray = b2CreateArray(sizeof(int32_t), B2_MAX(def->bodyCapacity, 1));

	world->awakeContactArray = b2CreateArray(sizeof(int32_t), B2_MAX(def->contactCapacity, 1));
	world->contactAwakeIndexArray = b2CreateArray(sizeof(int32_t), world->contactPool.capacity);

	world->stepId = 0;

	// Globals start at 0. It should be fine for this to roll over.
	world->revision += 1;

	world->gravity = def->gravity;
	world->restitutionThreshold = def->restitutionThreshold;
	world->contactPushoutVelocity = def->contactPushoutVelocity;
	world->contactHertz = def->contactHertz;
	world->contactDampingRatio = def->contactDampingRatio;
	world->inv_dt0 = 0.0f;
	world->enableSleep = true;
	world->locked = false;
	world->enableWarmStarting = true;
	world->enableContinuous = true;
	world->profile = b2_emptyProfile;
	world->userTreeTask = NULL;
	world->splitIslandIndex = B2_NULL_INDEX;

	id.revision = world->revision;

	if (def->workerCount > 0 && def->enqueueTask != NULL && def->finishTask != NULL && def->finishAllTasks != NULL)
	{
		world->workerCount = B2_MIN(def->workerCount, b2_maxWorkers);
		world->enqueueTaskFcn = def->enqueueTask;
		world->finishTaskFcn = def->finishTask;
		world->finishAllTasksFcn = def->finishAllTasks;
		world->userTaskContext = def->userTaskContext;
	}
	else
	{
		world->workerCount = 1;
		world->enqueueTaskFcn = b2DefaultAddTaskFcn;
		world->finishTaskFcn = b2DefaultFinishTaskFcn;
		world->finishAllTasksFcn = b2DefaultFinishAllTasksFcn;
		world->userTaskContext = NULL;
	}

	world->taskContextArray = b2CreateArray(sizeof(b2TaskContext), world->workerCount);
	for (uint32_t i = 0; i < world->workerCount; ++i)
	{
		world->taskContextArray[i].contactStateBitSet = b2CreateBitSet(def->contactCapacity);
		world->taskContextArray[i].awakeContactBitSet = b2CreateBitSet(def->contactCapacity);
		world->taskContextArray[i].shapeBitSet = b2CreateBitSet(def->shapeCapacity);
		world->taskContextArray[i].awakeIslandBitSet = b2CreateBitSet(256);
	}

	return id;
}

void b2DestroyWorld(b2WorldId id)
{
	b2World* world = b2GetWorldFromId(id);

	for (uint32_t i = 0; i < world->workerCount; ++i)
	{
		b2DestroyBitSet(&world->taskContextArray[i].contactStateBitSet);
		b2DestroyBitSet(&world->taskContextArray[i].awakeContactBitSet);
		b2DestroyBitSet(&world->taskContextArray[i].shapeBitSet);
		b2DestroyBitSet(&world->taskContextArray[i].awakeIslandBitSet);
	}

	b2DestroyArray(world->taskContextArray, sizeof(b2TaskContext));
	b2DestroyArray(world->awakeContactArray, sizeof(int32_t));
	b2DestroyArray(world->awakeIslandArray, sizeof(int32_t));
	b2DestroyArray(world->contactAwakeIndexArray, sizeof(int32_t));

	b2DestroyPool(&world->islandPool);
	b2DestroyPool(&world->jointPool);
	b2DestroyPool(&world->contactPool);
	b2DestroyPool(&world->shapePool);

	int32_t chainCapacity = world->chainPool.capacity;
	for (int32_t i = 0; i < chainCapacity; ++i)
	{
		b2ChainShape* chain = world->chains + i;
		if (b2ObjectValid(&chain->object))
		{
			b2Free(chain->shapeIndices, chain->count * sizeof(int32_t));
		}
	}

	b2DestroyPool(&world->chainPool);
	b2DestroyPool(&world->bodyPool);

	b2DestroyGraph(&world->graph);
	b2DestroyBroadPhase(&world->broadPhase);

	b2DestroyBlockAllocator(world->blockAllocator);
	b2DestroyStackAllocator(world->stackAllocator);

	memset(world, 0, sizeof(b2World));
}

static void b2CollideTask(int32_t startIndex, int32_t endIndex, uint32_t threadIndex, void* context)
{
	b2TracyCZoneNC(collide_task, "Collide Task", b2_colorDodgerBlue1, true);

	b2World* world = context;
	B2_ASSERT(threadIndex < world->workerCount);
	b2TaskContext* taskContext = world->taskContextArray + threadIndex;
	b2Shape* shapes = world->shapes;
	b2Body* bodies = world->bodies;
	b2Contact* contacts = world->contacts;
	int32_t awakeCount = b2Array(world->awakeContactArray).count;
	int32_t* awakeContactArray = world->awakeContactArray;
	int32_t* contactAwakeIndexArray = world->contactAwakeIndexArray;

	B2_MAYBE_UNUSED(awakeCount);
	B2_ASSERT(startIndex < endIndex);
	B2_ASSERT(endIndex <= awakeCount);

	for (int32_t awakeIndex = startIndex; awakeIndex < endIndex; ++awakeIndex)
	{
		int32_t contactIndex = awakeContactArray[awakeIndex];
		if (contactIndex == B2_NULL_INDEX)
		{
			// Contact was destroyed
			continue;
		}

		B2_ASSERT(0 <= contactIndex && contactIndex < world->contactPool.capacity);
		b2Contact* contact = contacts + contactIndex;

		B2_ASSERT(contactAwakeIndexArray[contactIndex] == awakeIndex);
		B2_ASSERT(contact->object.index == contactIndex && contact->object.index == contact->object.next);

		// Reset contact awake index. Contacts must be added to the awake contact array
		// each time step in the island solver.
		contactAwakeIndexArray[contactIndex] = B2_NULL_INDEX;

		b2Shape* shapeA = shapes + contact->shapeIndexA;
		b2Shape* shapeB = shapes + contact->shapeIndexB;

		// Do proxies still overlap?
		bool overlap = b2AABB_Overlaps(shapeA->fatAABB, shapeB->fatAABB);
		if (overlap == false)
		{
			contact->flags |= b2_contactDisjoint;
			b2SetBit(&taskContext->contactStateBitSet, awakeIndex);
		}
		else
		{
			bool wasTouching = (contact->flags & b2_contactTouchingFlag);
			B2_ASSERT(wasTouching || contact->islandIndex == B2_NULL_INDEX);

			// Update contact respecting shape/body order (A,B)
			b2Body* bodyA = bodies + shapeA->bodyIndex;
			b2Body* bodyB = bodies + shapeB->bodyIndex;
			b2UpdateContact(world, contact, shapeA, bodyA, shapeB, bodyB);

			bool touching = (contact->flags & b2_contactTouchingFlag) != 0;

			// State changes that affect island connectivity
			if (touching == true && wasTouching == false)
			{
				contact->flags |= b2_contactStartedTouching;
				b2SetBit(&taskContext->contactStateBitSet, awakeIndex);
			}
			else if (touching == false && wasTouching == true)
			{
				contact->flags |= b2_contactStoppedTouching;
				b2SetBit(&taskContext->contactStateBitSet, awakeIndex);
			}
		}
	}

	b2TracyCZoneEnd(collide_task);
}

static void b2UpdateTreesTask(int32_t startIndex, int32_t endIndex, uint32_t threadIndex, void* context)
{
	B2_MAYBE_UNUSED(startIndex);
	B2_MAYBE_UNUSED(endIndex);
	B2_MAYBE_UNUSED(threadIndex);

	b2TracyCZoneNC(tree_task, "Rebuild Trees", b2_colorSnow1, true);

	b2World* world = context;
	b2BroadPhase_RebuildTrees(&world->broadPhase);

	b2TracyCZoneEnd(tree_task);
}

// Narrow-phase collision
static void b2Collide(b2World* world)
{
	B2_ASSERT(world->workerCount > 0);

	b2TracyCZoneNC(collide, "Collide", b2_colorDarkOrchid, true);

	// Tasks that can be done in parallel with the narrow-phase
	// - rebuild the collision tree for dynamic and kinematic bodies to keep their query performance good
	if (b2_parallel)
	{
		world->userTreeTask = world->enqueueTaskFcn(&b2UpdateTreesTask, 1, 1, world, world->userTaskContext);
	}
	else
	{
		b2UpdateTreesTask(0, 1, 0, world);
		world->userTreeTask = NULL;
	}

	int32_t awakeContactCount = b2Array(world->awakeContactArray).count;

	if (awakeContactCount == 0)
	{
		b2TracyCZoneEnd(collide);
		return;
	}

	for (uint32_t i = 0; i < world->workerCount; ++i)
	{
		b2SetBitCountAndClear(&world->taskContextArray[i].contactStateBitSet, awakeContactCount);
	}

	if (b2_parallel)
	{
		// Task should take at least 40us on a 4GHz CPU (10K cycles)
		int32_t minRange = 64;
		void* userCollideTask = world->enqueueTaskFcn(&b2CollideTask, awakeContactCount, minRange, world, world->userTaskContext);
		world->finishTaskFcn(userCollideTask, world->userTaskContext);
	}
	else
	{
		b2CollideTask(0, awakeContactCount, 0, world);
	}

	// Serially update contact state
	b2TracyCZoneNC(contact_state, "Contact State", b2_colorCoral, true);

	// Bitwise OR all contact bits
	b2BitSet* bitSet = &world->taskContextArray[0].contactStateBitSet;
	for (uint32_t i = 1; i < world->workerCount; ++i)
	{
		b2InPlaceUnion(bitSet, &world->taskContextArray[i].contactStateBitSet);
	}

	// Process contact state changes. Iterate over set bits
	uint64_t word;
	for (uint32_t k = 0; k < bitSet->wordCount; ++k)
	{
		word = bitSet->bits[k];
		while (word != 0)
		{
			uint32_t ctz = b2CTZ(word);
			uint32_t awakeIndex = 64 * k + ctz;
			B2_ASSERT(awakeIndex < (uint32_t)awakeContactCount);

			int32_t contactIndex = world->awakeContactArray[awakeIndex];
			B2_ASSERT(contactIndex != B2_NULL_INDEX);

			b2Contact* contact = world->contacts + contactIndex;

			if (contact->flags & b2_contactDisjoint)
			{
				// Bounding boxes no longer overlap
				b2DestroyContact(world, contact);
			}
			else if (contact->flags & b2_contactStartedTouching)
			{
				B2_ASSERT(contact->islandIndex == B2_NULL_INDEX);
				b2LinkContact(world, contact);
				b2AddContactToGraph(world, contact);
				contact->flags &= ~b2_contactStartedTouching;
			}
			else
			{
				B2_ASSERT(contact->flags & b2_contactStoppedTouching);
				if (contact->colorIndex == B2_NULL_INDEX)
				{
					contact->colorIndex = B2_NULL_INDEX;
				}

				b2UnlinkContact(world, contact);
				b2RemoveContactFromGraph(world, contact);
				contact->flags &= ~b2_contactStoppedTouching;
			}

			// Clear the smallest set bit
			word = word & (word - 1);
		}
	}

	b2TracyCZoneEnd(contact_state);

	b2TracyCZoneEnd(collide);
}

void b2World_Step(b2WorldId worldId, float timeStep, int32_t velocityIterations, int32_t relaxIterations)
{
	if (timeStep == 0.0f)
	{
		// TODO_ERIN would be useful to still process collision while paused
		return;
	}

	b2TracyCZoneNC(world_step, "Step", b2_colorChartreuse, true);

	b2World* world = b2GetWorldFromId(worldId);
	B2_ASSERT(world->locked == false);
	if (world->locked)
	{
		return;
	}

	world->profile = b2_emptyProfile;

	b2Timer stepTimer = b2CreateTimer();

	// Update collision pairs and create contacts
	{
		b2Timer timer = b2CreateTimer();
		b2UpdateBroadPhasePairs(world);
		world->profile.pairs = b2GetMilliseconds(&timer);
	}

	// TODO_ERIN atomic
	world->locked = true;

	b2StepContext context = {0};
	context.dt = timeStep;
	context.velocityIterations = velocityIterations;
	context.relaxIterations = relaxIterations;
	if (timeStep > 0.0f)
	{
		context.inv_dt = 1.0f / timeStep;
	}
	else
	{
		context.inv_dt = 0.0f;
	}

	context.dtRatio = world->inv_dt0 * timeStep;
	context.restitutionThreshold = world->restitutionThreshold;
	context.enableWarmStarting = world->enableWarmStarting;
	context.bodies = world->bodies;
	context.bodyCapacity = world->bodyPool.capacity;

	// Update contacts
	{
		b2Timer timer = b2CreateTimer();
		b2Collide(world);
		world->profile.collide = b2GetMilliseconds(&timer);
	}

	// Integrate velocities, solve velocity constraints, and integrate positions.
	if (context.dt > 0.0f)
	{
		b2Timer timer = b2CreateTimer();
		b2Solve(world, &context);
		world->profile.solve = b2GetMilliseconds(&timer);
	}

	if (context.dt > 0.0f)
	{
		world->inv_dt0 = context.inv_dt;
	}

	world->locked = false;

	world->profile.step = b2GetMilliseconds(&stepTimer);

	if (b2_parallel)
	{
		// This finishes tree rebuild and split island tasks
		world->finishAllTasksFcn(world->userTaskContext);
	}

	B2_ASSERT(b2GetStackAllocation(world->stackAllocator) == 0);

	// Ensure stack is large enough
	b2GrowStack(world->stackAllocator);

	b2TracyCZoneEnd(world_step);
}

static void b2DrawShape(b2DebugDraw* draw, b2Shape* shape, b2Transform xf, b2Color color)
{
	switch (shape->type)
	{
		case b2_capsuleShape:
		{
			b2Capsule* capsule = &shape->capsule;
			b2Vec2 p1 = b2TransformPoint(xf, capsule->point1);
			b2Vec2 p2 = b2TransformPoint(xf, capsule->point2);
			draw->DrawSolidCapsule(p1, p2, capsule->radius, color, draw->context);
		}
		break;

		case b2_circleShape:
		{
			b2Circle* circle = &shape->circle;
			b2Vec2 center = b2TransformPoint(xf, circle->point);
			b2Vec2 axis = b2RotateVector(xf.q, (b2Vec2){1.0f, 0.0f});
			draw->DrawSolidCircle(center, circle->radius, axis, color, draw->context);
		}
		break;

		case b2_polygonShape:
		{
			b2Color fillColor = {0.5f * color.r, 0.5f * color.g, 0.5f * color.b, 0.5f};

			b2Polygon* poly = &shape->polygon;
			int32_t count = poly->count;
			B2_ASSERT(count <= b2_maxPolygonVertices);
			b2Vec2 vertices[b2_maxPolygonVertices];

			for (int32_t i = 0; i < count; ++i)
			{
				vertices[i] = b2TransformPoint(xf, poly->vertices[i]);
			}

			if (poly->radius > 0.0f)
			{
				draw->DrawRoundedPolygon(vertices, count, poly->radius, fillColor, color, draw->context);
			}
			else
			{
				draw->DrawSolidPolygon(vertices, count, color, draw->context);
			}
		}
		break;

		case b2_segmentShape:
		{
			b2Segment* segment = &shape->segment;
			b2Vec2 p1 = b2TransformPoint(xf, segment->point1);
			b2Vec2 p2 = b2TransformPoint(xf, segment->point2);
			draw->DrawSegment(p1, p2, color, draw->context);
		}
		break;

		case b2_smoothSegmentShape:
		{
			b2Segment* segment = &shape->smoothSegment.segment;
			b2Vec2 p1 = b2TransformPoint(xf, segment->point1);
			b2Vec2 p2 = b2TransformPoint(xf, segment->point2);
			draw->DrawSegment(p1, p2, color, draw->context);
			draw->DrawPoint(p2, 4.0f, color, draw->context);
		}
		break;

		default:
			break;
	}
}

void b2World_Draw(b2WorldId worldId, b2DebugDraw* draw)
{
	b2World* world = b2GetWorldFromId(worldId);
	B2_ASSERT(world->locked == false);
	if (world->locked)
	{
		return;
	}

	if (draw->drawShapes)
	{
		int32_t count = world->bodyPool.capacity;
		for (int32_t i = 0; i < count; ++i)
		{
			b2Body* b = world->bodies + i;
			if (b->object.next != i)
			{
				continue;
			}

			bool isAwake = false;
			if (b->islandIndex != B2_NULL_INDEX)
			{
				isAwake = world->islands[b->islandIndex].awakeIndex != B2_NULL_INDEX;
			}

			b2Transform xf = b->transform;
			int32_t shapeIndex = b->shapeList;
			while (shapeIndex != B2_NULL_INDEX)
			{
				b2Shape* shape = world->shapes + shapeIndex;
				if (b->type == b2_dynamicBody && b->mass == 0.0f)
				{
					// Bad body
					b2DrawShape(draw, shape, xf, b2MakeColor(b2_colorRed, 0.5f));
				}
				else if (b->isEnabled == false)
				{
					b2DrawShape(draw, shape, xf, b2MakeColor(b2_colorSlateGray2, 0.5f));
				}
				else if (b->isSpeedCapped)
				{
					b2DrawShape(draw, shape, xf, b2MakeColor(b2_colorYellow, 1.0f));
				}
				else if (b->isFast)
				{
					b2DrawShape(draw, shape, xf, b2MakeColor(b2_colorSalmon, 1.0f));
				}
				else if (b->type == b2_staticBody)
				{
					b2DrawShape(draw, shape, xf, b2MakeColor(b2_colorPaleGreen, 1.0f));
				}
				else if (b->type == b2_kinematicBody)
				{
					b2DrawShape(draw, shape, xf, (b2Color){0.5f, 0.5f, 0.9f, 1.0f});
				}
				else if (isAwake)
				{
					b2DrawShape(draw, shape, xf, b2MakeColor(b2_colorPink3, 1.0f));
				}
				else
				{
					b2DrawShape(draw, shape, xf, (b2Color){0.6f, 0.6f, 0.6f, 1.0f});
				}

				shapeIndex = shape->nextShapeIndex;
			}
		}
	}

	if (draw->drawJoints)
	{
		int32_t count = world->jointPool.capacity;
		for (int32_t i = 0; i < count; ++i)
		{
			b2Joint* joint = world->joints + i;
			if (joint->object.next != i)
			{
				continue;
			}

			b2DrawJoint(draw, world, joint);
		}
	}

	// if (debugDraw->drawPi & b2Draw::e_pairBit)
	//{
	//		b2Color color(0.3f, 0.9f, 0.9f);
	//		for (b2Contact* c = m_contactManager.m_contactList; c; c = c->GetNext())
	//		{
	//		b2Shape* fixtureA = c->GetFixtureA();
	//		b2Shape* fixtureB = c->GetFixtureB();
	//		int32 indexA = c->GetChildIndexA();
	//		int32 indexB = c->GetChildIndexB();
	//		b2Vec2 cA = fixtureA->GetAABB(indexA).GetCenter();
	//		b2Vec2 cB = fixtureB->GetAABB(indexB).GetCenter();

	//		m_debugDraw->DrawSegment(cA, cB, color);
	//		}
	//}

	if (draw->drawAABBs)
	{
		b2Color color = {0.9f, 0.3f, 0.9f, 1.0f};

		int32_t count = world->bodyPool.capacity;
		for (int32_t i = 0; i < count; ++i)
		{
			b2Body* b = world->bodies + i;
			if (b->object.next != i)
			{
				continue;
			}

			char buffer[32];
			sprintf(buffer, "%d", b->object.index);
			draw->DrawString(b->position, buffer, draw->context);

			int32_t shapeIndex = b->shapeList;
			while (shapeIndex != B2_NULL_INDEX)
			{
				b2Shape* shape = world->shapes + shapeIndex;
				b2AABB aabb = shape->fatAABB;

				b2Vec2 vs[4] = {{aabb.lowerBound.x, aabb.lowerBound.y},
								{aabb.upperBound.x, aabb.lowerBound.y},
								{aabb.upperBound.x, aabb.upperBound.y},
								{aabb.lowerBound.x, aabb.upperBound.y}};

				draw->DrawPolygon(vs, 4, color, draw->context);

				shapeIndex = shape->nextShapeIndex;
			}
		}

		// for (b2Shape* f = b->GetFixtureList(); f; f = f->GetNext())
		//{
		//	for (int32 i = 0; i < f->m_proxyCount; ++i)
		//	{
		//		b2FixtureProxy* proxy = f->m_proxies + i;
		//		b2AABB aabb = bp->GetFatAABB(proxy->proxyId);
		//		b2Vec2 vs[4];
		//		vs[0].Set(aabb.lowerBound.x, aabb.lowerBound.y);
		//		vs[1].Set(aabb.upperBound.x, aabb.lowerBound.y);
		//		vs[2].Set(aabb.upperBound.x, aabb.upperBound.y);
		//		vs[3].Set(aabb.lowerBound.x, aabb.upperBound.y);

		//		m_debugDraw->DrawPolygon(vs, 4, color);
		//	}
		//}
	}

	if (draw->drawMass)
	{
		b2Vec2 offset = {0.1f, 0.1f};
		b2Body* bodies = world->bodies;
		int32_t bodyCapacity = world->bodyPool.capacity;
		for (int32_t i = 0; i < bodyCapacity; ++i)
		{
			b2Body* body = bodies + i;
			if (b2ObjectValid(&body->object) == false)
			{
				continue;
			}

			draw->DrawTransform(body->transform, draw->context);

			b2Vec2 p = b2TransformPoint(body->transform, offset);

			char buffer[32];
			sprintf(buffer, "%.1f", body->mass);
			draw->DrawString(p, buffer, draw->context);
		}
	}
}

void b2World_EnableSleeping(b2WorldId worldId, bool flag)
{
	b2World* world = b2GetWorldFromId(worldId);
	B2_ASSERT(world->locked == false);
	if (world->locked)
	{
		return;
	}

	if (flag == world->enableSleep)
	{
		return;
	}

	world->enableSleep = flag;
	if (flag == false)
	{
		int32_t count = world->islandPool.capacity;
		for (int32_t i = 0; i < count; ++i)
		{
			b2Island* island = world->islands + i;
			if (island->object.next != i)
			{
				continue;
			}

			b2WakeIsland(island);
		}
	}
}

void b2World_EnableWarmStarting(b2WorldId worldId, bool flag)
{
	b2World* world = b2GetWorldFromId(worldId);
	B2_ASSERT(world->locked == false);
	if (world->locked)
	{
		return;
	}

	world->enableWarmStarting = flag;
}

void b2World_EnableContinuous(b2WorldId worldId, bool flag)
{
	b2World* world = b2GetWorldFromId(worldId);
	B2_ASSERT(world->locked == false);
	if (world->locked)
	{
		return;
	}

	world->enableContinuous = flag;
}

void b2World_SetRestitutionThreshold(b2WorldId worldId, float value)
{
	b2World* world = b2GetWorldFromId(worldId);
	B2_ASSERT(world->locked == false);
	if (world->locked)
	{
		return;
	}

	world->restitutionThreshold = B2_CLAMP(value, 0.0f, FLT_MAX);
}

void b2World_SetContactTuning(b2WorldId worldId, float hertz, float dampingRatio, float pushOut)
{
	b2World* world = b2GetWorldFromId(worldId);
	B2_ASSERT(world->locked == false);
	if (world->locked)
	{
		return;
	}

	world->contactHertz = B2_CLAMP(hertz, 0.0f, FLT_MAX);
	world->contactDampingRatio = B2_CLAMP(dampingRatio, 0.0f, FLT_MAX);
	world->contactPushoutVelocity = B2_CLAMP(pushOut, 0.0f, FLT_MAX);
}

b2Profile b2World_GetProfile(b2WorldId worldId)
{
	b2World* world = b2GetWorldFromId(worldId);
	return world->profile;
}

b2Statistics b2World_GetStatistics(b2WorldId worldId)
{
	b2World* world = b2GetWorldFromId(worldId);
	b2Statistics s = {0};
	s.islandCount = world->islandPool.count;
	s.bodyCount = world->bodyPool.count;
	s.contactCount = world->contactPool.count;
	s.jointCount = world->jointPool.count;
	s.pairCount = world->broadPhase.pairSet.count;

	b2DynamicTree* tree = world->broadPhase.trees + b2_dynamicBody;
	s.proxyCount = tree->nodeCount;
	s.treeHeight = b2DynamicTree_GetHeight(tree);
	s.stackCapacity = b2GetStackCapacity(world->stackAllocator);
	s.stackUsed = b2GetMaxStackAllocation(world->stackAllocator);
	s.byteCount = b2GetByteCount();
	for (int32_t i = 0; i <= b2_graphColorCount; ++i)
	{
		s.colorCounts[i] = world->graph.occupancy[i];
	}
	return s;
}

typedef struct WorldQueryContext
{
	b2World* world;
	b2QueryResultFcn* fcn;
	b2QueryFilter filter;
	void* userContext;
} WorldQueryContext;

static bool TreeQueryCallback(int32_t proxyId, int32_t shapeIndex, void* context)
{
	B2_MAYBE_UNUSED(proxyId);

	WorldQueryContext* worldContext = context;
	b2World* world = worldContext->world;

	B2_ASSERT(0 <= shapeIndex && shapeIndex < world->shapePool.capacity);

	b2Shape* shape = world->shapes + shapeIndex;
	b2Filter shapeFilter = shape->filter;
	b2QueryFilter queryFilter = worldContext->filter;

	if ((shapeFilter.categoryBits & queryFilter.maskBits) == 0 || (shapeFilter.maskBits & queryFilter.categoryBits) == 0)
	{
		return true;
	}

	B2_ASSERT(shape->object.index == shape->object.next);

	b2ShapeId shapeId = {shape->object.index, world->index, shape->object.revision};
	bool result = worldContext->fcn(shapeId, worldContext->userContext);
	return result;
}

void b2World_QueryAABB(b2WorldId worldId, b2QueryResultFcn* fcn, b2AABB aabb, b2QueryFilter filter, void* context)
{
	b2World* world = b2GetWorldFromId(worldId);
	B2_ASSERT(world->locked == false);
	if (world->locked)
	{
		return;
	}

	WorldQueryContext worldContext = {world, fcn, filter, context};

	for (int32_t i = 0; i < b2_bodyTypeCount; ++i)
	{
		b2DynamicTree_Query(world->broadPhase.trees + i, aabb, TreeQueryCallback, &worldContext);
	}
}

typedef struct WorldRayCastContext
{
	b2World* world;
	b2RayResultFcn* fcn;
	b2QueryFilter filter;
	float fraction;
	void* userContext;
} WorldRayCastContext;

static float RayCastCallback(const b2RayCastInput* input, int32_t proxyId, int32_t shapeIndex, void* context)
{
	B2_MAYBE_UNUSED(proxyId);

	WorldRayCastContext* worldContext = context;
	b2World* world = worldContext->world;

	B2_ASSERT(0 <= shapeIndex && shapeIndex < world->shapePool.capacity);

	b2Shape* shape = world->shapes + shapeIndex;
	b2Filter shapeFilter = shape->filter;
	b2QueryFilter queryFilter = worldContext->filter;

	if ((shapeFilter.categoryBits & queryFilter.maskBits) == 0 || (shapeFilter.maskBits & queryFilter.categoryBits) == 0)
	{
		return input->maxFraction;
	}

	int32_t bodyIndex = shape->bodyIndex;
	B2_ASSERT(0 <= bodyIndex && bodyIndex < world->bodyPool.capacity);

	b2Body* body = world->bodies + bodyIndex;
	B2_ASSERT(b2ObjectValid(&body->object));

	b2RayCastOutput output = b2RayCastShape(input, shape, body->transform);

	if (output.hit)
	{
		b2ShapeId shapeId = {shapeIndex, world->index, shape->object.revision};
		float fraction = worldContext->fcn(shapeId, output.point, output.normal, output.fraction, worldContext->userContext);
		worldContext->fraction = fraction;
		return fraction;
	}

	return input->maxFraction;
}

void b2World_RayCast(b2WorldId worldId, b2RayResultFcn* fcn, b2Vec2 point1, b2Vec2 point2, b2QueryFilter filter,
					 void* context)
{
	b2World* world = b2GetWorldFromId(worldId);
	B2_ASSERT(world->locked == false);
	if (world->locked)
	{
		return;
	}

	b2RayCastInput input = {point1, point2, 0.0f, 1.0f};
	WorldRayCastContext worldContext = {world, fcn, filter, 1.0f, context};

	for (int32_t i = 0; i < b2_bodyTypeCount; ++i)
	{
		b2DynamicTree_RayCast(world->broadPhase.trees + i, &input, filter.maskBits, RayCastCallback, &worldContext);
		
		if (worldContext.fraction == 0.0f)
		{
			return;
		}

		input.maxFraction = worldContext.fraction;
	}
}

#if 0

void b2World_ShiftOrigin(b2WorldId worldId, b2Vec2 newOrigin)
{
	B2_ASSERT(m_locked == false);
	if (m_locked)
	{
		return;
	}

	for (b2Body* b = m_bodyList; b; b = b->m_next)
	{
		b->m_xf.p -= newOrigin;
		b->m_sweep.c0 -= newOrigin;
		b->m_sweep.c -= newOrigin;
	}

	for (b2Joint* j = m_jointList; j; j = j->m_next)
	{
		j->ShiftOrigin(newOrigin);
	}

	m_contactManager.m_broadPhase.ShiftOrigin(newOrigin);
}

void b2World_Dump()
{
	if (m_locked)
	{
		return;
	}

	b2OpenDump("box2d_dump.inl");

	b2Dump("b2Vec2 g(%.9g, %.9g);\n", m_gravity.x, m_gravity.y);
	b2Dump("m_world->SetGravity(g);\n");

	b2Dump("b2Body** bodies = (b2Body**)b2Alloc(%d * sizeof(b2Body*));\n", m_bodyCount);
	b2Dump("b2Joint** joints = (b2Joint**)b2Alloc(%d * sizeof(b2Joint*));\n", m_jointCount);

	int32 i = 0;
	for (b2Body* b = m_bodyList; b; b = b->m_next)
	{
		b->m_islandIndex = i;
		b->Dump();
		++i;
	}

	i = 0;
	for (b2Joint* j = m_jointList; j; j = j->m_next)
	{
		j->m_index = i;
		++i;
	}

	// First pass on joints, skip gear joints.
	for (b2Joint* j = m_jointList; j; j = j->m_next)
	{
		if (j->m_type == e_gearJoint)
		{
			continue;
		}

		b2Dump("{\n");
		j->Dump();
		b2Dump("}\n");
	}

	// Second pass on joints, only gear joints.
	for (b2Joint* j = m_jointList; j; j = j->m_next)
	{
		if (j->m_type != e_gearJoint)
		{
			continue;
		}

		b2Dump("{\n");
		j->Dump();
		b2Dump("}\n");
	}

	b2Dump("b2Free(joints);\n");
	b2Dump("b2Free(bodies);\n");
	b2Dump("joints = nullptr;\n");
	b2Dump("bodies = nullptr;\n");

	b2CloseDump();
}
#endif

bool b2IsBodyIdValid(b2World* world, b2BodyId id)
{
	if (id.world != world->index)
	{
		return false;
	}

	if (id.index >= world->bodyPool.capacity)
	{
		return false;
	}

	b2Body* body = world->bodies + id.index;
	if (body->object.index != body->object.next)
	{
		return false;
	}

	if (body->object.revision != id.revision)
	{
		return false;
	}

	return true;
}

void b2World_SetPreSolveCallback(b2WorldId worldId, b2PreSolveFcn* fcn, void* context)
{
	b2World* world = b2GetWorldFromId(worldId);
	world->preSolveFcn = fcn;
	world->preSolveContext = context;
}

void b2World_SetPostSolveCallback(b2WorldId worldId, b2PostSolveFcn* fcn, void* context)
{
	b2World* world = b2GetWorldFromId(worldId);
	world->postSolveFcn = fcn;
	world->postSolveContext = context;
}
