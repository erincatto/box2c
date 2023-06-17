// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "box2d/callbacks.h"
#include "box2d/timer.h"

#include "array.h"
#include "body.h"
#include "contact.h"
#include "contact_solver.h"
#include "island.h"
#include "joint.h"
#include "shape.h"
#include "solver_data.h"
#include "stack_allocator.h"
#include "world.h"

#include <assert.h>
#include <float.h>
#include <string.h>

/*
Position Correction Notes
=========================
I tried the several algorithms for position correction of the 2D revolute joint.
I looked at these systems:
- simple pendulum (1m diameter sphere on massless 5m stick) with initial angular velocity of 100 rad/s.
- suspension bridge with 30 1m long planks of length 1m.
- multi-link chain with 30 1m long links.

Here are the algorithms:

Baumgarte - A fraction of the position error is added to the velocity error. There is no
separate position solver.

Pseudo Velocities - After the velocity solver and position integration,
the position error, Jacobian, and effective mass are recomputed. Then
the velocity constraints are solved with pseudo velocities and a fraction
of the position error is added to the pseudo velocity error. The pseudo
velocities are initialized to zero and there is no warm-starting. After
the position solver, the pseudo velocities are added to the positions.
This is also called the First Order World method or the Position LCP method.

Modified Nonlinear Gauss-Seidel (NGS) - Like Pseudo Velocities except the
position error is re-computed for each constraint and the positions are updated
after the constraint is solved. The radius vectors (aka Jacobians) are
re-computed too (otherwise the algorithm has horrible instability). The pseudo
velocity states are not needed because they are effectively zero at the beginning
of each iteration. Since we have the current position error, we allow the
iterations to terminate early if the error becomes smaller than b2_linearSlop.

Full NGS or just NGS - Like Modified NGS except the effective mass are re-computed
each time a constraint is solved.

Here are the results:
Baumgarte - this is the cheapest algorithm but it has some stability problems,
especially with the bridge. The chain links separate easily close to the root
and they jitter as they struggle to pull together. This is one of the most common
methods in the field. The big drawback is that the position correction artificially
affects the momentum, thus leading to instabilities and false bounce. I used a
bias factor of 0.2. A larger bias factor makes the bridge less stable, a smaller
factor makes joints and contacts more spongy.

Pseudo Velocities - the is more stable than the Baumgarte method. The bridge is
stable. However, joints still separate with large angular velocities. Drag the
simple pendulum in a circle quickly and the joint will separate. The chain separates
easily and does not recover. I used a bias factor of 0.2. A larger value lead to
the bridge collapsing when a heavy cube drops on it.

Modified NGS - this algorithm is better in some ways than Baumgarte and Pseudo
Velocities, but in other ways it is worse. The bridge and chain are much more
stable, but the simple pendulum goes unstable at high angular velocities.

Full NGS - stable in all tests. The joints display good stiffness. The bridge
still sags, but this is better than infinite forces.

Recommendations
Pseudo Velocities are not really worthwhile because the bridge and chain cannot
recover from joint separation. In other cases the benefit over Baumgarte is small.

Modified NGS is not a robust method for the revolute joint due to the violent
instability seen in the simple pendulum. Perhaps it is viable with other constraint
types, especially scalar constraints where the effective mass is a scalar.

This leaves Baumgarte and Full NGS. Baumgarte has small, but manageable instabilities
and is very fast. I don't think we can escape Baumgarte, especially in highly
demanding cases where high constraint fidelity is not needed.

Full NGS is robust and easy on the eyes. I recommend this as an option for
higher fidelity simulation and certainly for suspension bridges and long chains.
Full NGS might be a good choice for ragdolls, especially motorized ragdolls where
joint separation can be problematic. The number of NGS iterations can be reduced
for better performance without harming robustness much.

Each joint in a can be handled differently in the position solver. So I recommend
a system where the user can select the algorithm on a per joint basis. I would
probably default to the slower Full NGS and let the user select the faster
Baumgarte method in performance critical scenarios.
*/

/*
Cache Performance

The Box2D solvers are dominated by cache misses. Data structures are designed
to increase the number of cache hits. Much of misses are due to random access
to body data. The constraint structures are iterated over linearly, which leads
to few cache misses.

The bodies are not accessed during iteration. Instead read only data, such as
the mass values are stored with the constraints. The mutable data are the constraint
impulses and the bodies velocities/positions. The impulses are held inside the
constraint structures. The body velocities/positions are held in compact, temporary
arrays to increase the number of cache hits. Linear and angular velocity are
stored in a single array since multiple arrays lead to multiple misses.
*/

/*
2D Rotation

R = [cos(theta) -sin(theta)]
	[sin(theta) cos(theta) ]

thetaDot = omega

Let q1 = cos(theta), q2 = sin(theta).
R = [q1 -q2]
	[q2  q1]

q1Dot = -thetaDot * q2
q2Dot = thetaDot * q1

q1_new = q1_old - dt * w * q2
q2_new = q2_old + dt * w * q1
then normalize.

This might be faster than computing sin+cos.
However, we can compute sin+cos of the same angle fast.
*/

void b2PrepareIsland(b2PersistentIsland* island, b2StepContext* stepContext)
{
	island->stepContext = stepContext;

	int32_t bodyCount = island->bodyCount;
	int32_t contactCount = island->contactCount;
	int32_t jointCount = island->jointCount;

	b2World* world = island->world;

	int32_t nextIndex = island->nextIsland;
	while (nextIndex != B2_NULL_INDEX)
	{
		b2PersistentIsland* nextIsland = world->islands + island->nextIsland;
		bodyCount += nextIsland->bodyCount;
		contactCount += nextIsland->contactCount;
		jointCount += nextIsland->jointCount;
	}

	b2StackAllocator* alloc = world->stackAllocator;

	island->bodyIndices = b2AllocateStackItem(alloc, bodyCount * sizeof(int32_t), "body indices");
	island->contactIndices = b2AllocateStackItem(alloc, contactCount * sizeof(int32_t), "contact indices");
	island->jointIndices = b2AllocateStackItem(alloc, jointCount * sizeof(int32_t), "joint indices");

	b2ContactSolverDef contactSolverDef;
	contactSolverDef.context = island->stepContext;
	contactSolverDef.world = world;
	contactSolverDef.contactIndices = island->contactIndices;
	contactSolverDef.count = island->contactCount;
	island->contactSolver = b2CreateContactSolver(&contactSolverDef);
}

#if 0
// This just allocates data doing the minimal amount of single threaded work
b2Island* b2CreateIsland(b2IslandBuilder* builder, int32_t islandIndex, struct b2World* world, b2StepContext* context)
{
	b2StackAllocator* alloc = world->stackAllocator;

	b2Island* island = b2AllocateStackItem(alloc, sizeof(b2Island), "island");

	if (islandIndex == 0)
	{
		island->bodyIndices = builder->bodyIslands;
		island->bodyCount = builder->bodyIslandEnds[0];

		if (builder->jointCount > 0)
		{
			island->jointIndices = builder->jointIslands;
			island->jointCount = builder->jointIslandEnds[0];
		}
		else
		{
			island->jointIndices = NULL;
			island->jointCount = 0;
		}

		if (builder->contactCount > 0)
		{
			island->contactIndices = builder->contactIslands;
			island->contactCount = builder->contactIslandEnds[0];
		}
		else
		{
			island->contactIndices = NULL;
			island->contactCount = 0;
		}
	}
	else
	{
		int32_t bodyStartIndex = builder->bodyIslandEnds[islandIndex - 1];
		island->bodyIndices = builder->bodyIslands + bodyStartIndex;
		island->bodyCount = builder->bodyIslandEnds[islandIndex] - bodyStartIndex;

		if (builder->jointCount > 0)
		{
			int32_t jointStartIndex = builder->jointIslandEnds[islandIndex - 1];
			island->jointIndices = builder->jointIslands + jointStartIndex;
			island->jointCount = builder->jointIslandEnds[islandIndex] - jointStartIndex;
		}
		else
		{
			island->jointIndices = NULL;
			island->jointCount = 0;
		}

		if (builder->contactCount > 0)
		{
			int32_t contactStartIndex = builder->contactIslandEnds[islandIndex - 1];
			island->contactIndices = builder->contactIslands + contactStartIndex;
			island->contactCount = builder->contactIslandEnds[islandIndex] - contactStartIndex;
		}
		else
		{
			island->contactIndices = NULL;
			island->contactCount = 0;
		}
	}

	island->world = world;
	island->context = context;

	b2ContactSolverDef contactSolverDef;
	contactSolverDef.context = island->context;
	contactSolverDef.world = world;
	contactSolverDef.contactIndices = island->contactIndices;
	contactSolverDef.count = island->contactCount;
	island->contactSolver = b2CreateContactSolver(alloc, &contactSolverDef);

	return island;
}

void b2DestroyIsland(b2Island* island)
{
	// Destroy in reverse order
	b2StackAllocator* alloc = island->world->stackAllocator;
	b2DestroyContactSolver(alloc, island->contactSolver);
	b2FreeStackItem(alloc, island);
}
#endif

#if 0
if (island->bodyCount > 16)
{
	int32_t k = 4;
	b2Vec2 clusterCenters[4] = { 0 };
	int32_t clusterCounts[4] = { 0 };
	int32_t m = island->bodyCount / k;

	// seed cluster positions
	for (int32_t i = 0; i < k; ++i)
	{
		int32_t j = (i * m) % island->bodyCount;
		clusterCenters[i] = island->bodies[j]->position;
	}

	for (int32_t i = 0; i < island->bodyCount; ++i)
	{
		b2Body* b = island->bodies[i];
		b2Vec2 p = b->position;
		float bestDist = b2DistanceSquared(clusterCenters[0], p);
		b->cluster = 0;

		for (int32_t j = 1; j < k; ++j)
		{
			float dist = b2DistanceSquared(clusterCenters[j], p);
			if (dist < bestDist)
			{
				bestDist = dist;
				b->cluster = j;
			}
		}
	}

	int32_t maxIter = 4;
	for (int32_t iter = 0; iter < maxIter; ++iter)
	{
		// reset clusters
		for (int32_t i = 0; i < k; ++i)
		{
			clusterCenters[i] = b2Vec2_zero;
			clusterCounts[i] = 0;
		}

		// computer new clusters
		for (int32_t i = 0; i < island->bodyCount; ++i)
		{
			b2Body* b = island->bodies[i];
			int32_t j = b->cluster;
			clusterCenters[j] = b2Add(clusterCenters[j], b->position);
			clusterCounts[j] += 1;
		}
	}
}
#endif

void b2ConnectIsland(b2PersistentIsland* island)
{
	b2World* world = island->world;
	b2Body* bodies = world->bodies;
	b2Contact* contacts = world->contacts;
	b2Joint* joints = world->joints;

	int32_t nextIndex = island->nextIsland;
	while (nextIndex != B2_NULL_INDEX)
	{
		b2PersistentIsland* nextIsland = world->islands + island->nextIsland;
		island->bodyCount += nextIsland->bodyCount;
		island->contactCount += nextIsland->contactCount;
		island->jointCount += nextIsland->jointCount;

		b2Body* tailBody = bodies + island->tailBody;
		b2Body* headBody = bodies + nextIsland->headBody;

		tailBody->islandNext = nextIsland->headBody;
		headBody->islandPrev = island->tailBody;

		island->tailBody = nextIsland->tailBody;

		// TODO_ERIN contacts can be null
		b2Contact* tailContact = contacts + island->tailContact;
		b2Contact* headContact = contacts + nextIsland->headContact;

		tailContact->islandNext = nextIsland->headContact;
		headContact->islandPrev = island->tailContact;

		island->tailContact = nextIsland->tailContact;

		// TODO_ERIN joints can be null
		b2Joint* tailJoint = joints + island->tailJoint;
		b2Joint* headJoint = joints + nextIsland->headJoint;

		tailJoint->islandNext = nextIsland->headJoint;
		headJoint->islandPrev = island->tailJoint;

		island->tailJoint = nextIsland->tailJoint;
	}

	// Fill body, contact, joint index arrays. Maybe check for contacts touching
	// Note: I'm not sure the arrays are worthwhile. maybe use the linked lists or arrays of pointers
}

// This must be thread safe
void b2SolveIsland(b2PersistentIsland* island)
{
	b2World* world = island->world;
	b2Body* bodies = world->bodies;
	b2StepContext* context = island->stepContext;
	b2Joint* joints = world->joints;

	b2Vec2 gravity = world->gravity;

	float h = context->dt;

	// Integrate velocities and apply damping. Initialize the body state.
	int32_t bodyCount = island->bodyCount;
	for (int32_t i = 0; i < bodyCount; ++i)
	{
		b2Body* b = bodies + island->bodyIndices[i];

		float invMass = b->invMass;
		float invI = b->invI;

		if (b->type == b2_dynamicBody)
		{
			b2Vec2 v = b->linearVelocity;
			float w = b->angularVelocity;

			// Integrate velocities
			v = b2Add(v, b2MulSV(h * invMass, b2MulAdd(b->force, b->gravityScale * b->mass, gravity)));
			w = w + h * invI * b->torque;

			// Apply damping.
			// ODE: dv/dt + c * v = 0
			// Solution: v(t) = v0 * exp(-c * t)
			// Time step: v(t + dt) = v0 * exp(-c * (t + dt)) = v0 * exp(-c * t) * exp(-c * dt) = v * exp(-c * dt)
			// v2 = exp(-c * dt) * v1
			// Pade approximation:
			// v2 = v1 * 1 / (1 + c * dt)
			v = b2MulSV(1.0f / (1.0f + h * b->linearDamping), v);
			w *= 1.0f / (1.0f + h * b->angularDamping);

			b->linearVelocity = v;
			b->angularVelocity = w;
		}
	}

	// Solver data
	b2ContactSolver_Initialize(island->contactSolver);
	
	for (int32_t i = 0; i < island->jointCount; ++i)
	{
		int32_t jointIndex = island->jointIndices[i];
		assert(0 <= jointIndex && jointIndex < context->activeJointCount);

		b2Joint* joint = joints[jointIndex];
		b2InitVelocityConstraints(joint, context);
	}

	b2TracyCZoneNC(velc, "Velocity Constraints", b2_colorCadetBlue, true);
	// Solve velocity constraints
	for (int32_t i = 0; i < context->velocityIterations; ++i)
	{
		for (int32_t j = 0; j < island->jointCount; ++j)
		{
			int32_t jointIndex = island->jointIndices[j];
			assert(0 <= jointIndex && jointIndex < context->activeJointCount);

			b2Joint* joint = joints[jointIndex];
			b2SolveVelocityConstraints(joint, context);
		}

		b2ContactSolver_SolveVelocityConstraints(island->contactSolver);
	}
	b2TracyCZoneEnd(velc);

	// Special handling for restitution
	b2ContactSolver_ApplyRestitution(island->contactSolver);

	// Store impulses for warm starting
	b2ContactSolver_StoreImpulses(island->contactSolver);

	// Integrate positions
	for (int32_t i = 0; i < island->bodyCount; ++i)
	{
		b2Body* b = bodies + island->bodyIndices[i];

		b2Vec2 c = b->position;
		float a = b->angle;
		b2Vec2 v = b->linearVelocity;
		float w = b->angularVelocity;

		// Check for large velocities
		b2Vec2 translation = b2MulSV(h, v);
		if (b2Dot(translation, translation) > b2_maxTranslationSquared)
		{
			float ratio = b2_maxTranslation / b2Length(translation);
			v = b2MulSV(ratio, v);
		}

		float rotation = h * w;
		if (rotation * rotation > b2_maxRotationSquared)
		{
			float ratio = b2_maxRotation / B2_ABS(rotation);
			w *= ratio;
		}

		// Integrate
		c = b2MulAdd(c, h, v);
		a += h * w;

		b->position = c;
		b->angle = a;
		b->linearVelocity = v;
		b->angularVelocity = w;
	}

	b2TracyCZoneNC(posc, "Position Constraints", b2_colorBurlywood, true);

	// Solve position constraints
	bool positionSolved = false;
	for (int32_t i = 0; i < context->positionIterations; ++i)
	{
		bool contactsOkay = b2ContactSolver_SolvePositionConstraintsBlock(island->contactSolver);

		bool jointsOkay = true;
		for (int32_t j = 0; j < island->jointCount; ++j)
		{
			int32_t jointIndex = island->jointIndices[j];
			assert(0 <= jointIndex && jointIndex < context->activeJointCount);

			b2Joint* joint = joints[jointIndex];

			bool jointOkay = b2SolvePositionConstraints(joint, context);
			jointsOkay = jointsOkay && jointOkay;
		}

		if (contactsOkay && jointsOkay)
		{
			// Exit early if the position errors are small.
			positionSolved = true;
			break;
		}
	}

	b2TracyCZoneEnd(posc);

	b2TracyCZoneNC(sleep, "Sleep", b2_colorSalmon2, true);

	// Reset body sleep and update transform
	for (int32_t i = 0; i < island->bodyCount; ++i)
	{
		b2Body* body = bodies + island->bodyIndices[i];
		if (body->type != b2_staticBody)
		{
			body->awakeIndex = B2_NULL_INDEX;

			body->transform.q = b2MakeRot(body->angle);
			body->transform.p = b2Sub(body->position, b2RotateVector(body->transform.q, body->localCenter));
		}
	}

	// Update sleep
	bool isIslandAwake = true;

	if (world->enableSleep)
	{
		float minSleepTime = FLT_MAX;

		const float linTolSqr = b2_linearSleepTolerance * b2_linearSleepTolerance;
		const float angTolSqr = b2_angularSleepTolerance * b2_angularSleepTolerance;

		for (int32_t i = 0; i < island->bodyCount; ++i)
		{
			b2Body* b = bodies + island->bodyIndices[i];
			if (b->type == b2_staticBody)
			{
				continue;
			}

			if (b->enableSleep == false ||
				b->angularVelocity * b->angularVelocity > angTolSqr ||
				b2Dot(b->linearVelocity, b->linearVelocity) > linTolSqr)
			{
				b->sleepTime = 0.0f;
				minSleepTime = 0.0f;
			}
			else
			{
				b->sleepTime += h;
				minSleepTime = B2_MIN(minSleepTime, b->sleepTime);
			}
		}

		if (minSleepTime >= b2_timeToSleep && positionSolved)
		{
			isIslandAwake = false;

			for (int32_t i = 0; i < island->bodyCount; ++i)
			{
				b2Body* b = bodies + island->bodyIndices[i];
				b->sleepTime = 0.0f;
				b->linearVelocity = b2Vec2_zero;
				b->angularVelocity = 0.0f;
				b->speculativePosition = b->position;
				b->speculativeAngle = b->angle;
				b->force = b2Vec2_zero;
				b->torque = 0.0f;
			}
		}
	}

	island->isAwake = isIslandAwake;
	
	// Speculative transform
	// TODO_ERIN using old forces? Should be at the beginning of the time step?
	if (isIslandAwake)
	{
		for (int32_t i = 0; i < island->bodyCount; ++i)
		{
			b2Body* b = bodies + island->bodyIndices[i];
			if (b->type == b2_staticBody)
			{
				continue;
			}

			b->force = b2Vec2_zero;
			b->torque = 0.0f;

			b2Vec2 v = b->linearVelocity;
			float w = b->angularVelocity;

			v = b2Add(v, b2MulSV(h * b->invMass, b2MulAdd(b->force, b->gravityScale * b->mass, gravity)));
			w = w + h * b->invI * b->torque;

			v = b2MulSV(1.0f / (1.0f + h * b->linearDamping), v);
			w *= 1.0f / (1.0f + h * b->angularDamping);

			// Stage 3 - predict new transforms
			b->speculativePosition = b2MulAdd(b->position, context->dt, v);
			b->speculativeAngle = b->angle + context->dt * w;

			// Update shapes AABBs
			int32_t shapeIndex = b->shapeIndex;
			while (shapeIndex != B2_NULL_INDEX)
			{
				b2Shape* shape = world->shapes + shapeIndex;
				for (int32_t j = 0; j < shape->proxyCount; ++j)
				{
					b2ShapeProxy* proxy = shape->proxies + j;

					// TODO_ERIN speculate
					proxy->aabb = b2Shape_ComputeAABB(shape, b->transform, proxy->childIndex);
				}

				shapeIndex = shape->nextShapeIndex;
			}
		}
	}

	b2TracyCZoneEnd(sleep);
}

// Single threaded work
void b2CompleteIsland(b2PersistentIsland* island)
{
	if (island->isAwake == false)
	{
		return;
	}

	b2World* world = island->world;
	b2Body* bodies = world->bodies;

	for (int32_t i = 0; i < island->bodyCount; ++i)
	{
		b2Body* b = bodies + island->bodyIndices[i];
		if (b->type == b2_staticBody)
		{
			continue;
		}

		assert(b->awakeIndex == B2_NULL_INDEX);
		b->awakeIndex = world->awakeCount;
		world->awakeBodies[b->awakeIndex] = b->object.index;
		world->awakeCount += 1;

		// Update shapes (for broad-phase).
		int32_t shapeIndex = b->shapeIndex;
		while (shapeIndex != B2_NULL_INDEX)
		{
			b2Shape* shape = world->shapes + shapeIndex;
			for (int32_t j = 0; j < shape->proxyCount; ++j)
			{
				b2ShapeProxy* proxy = shape->proxies + j;
				b2BroadPhase_MoveProxy(&world->broadPhase, proxy->proxyKey, proxy->aabb);
			}

			shapeIndex = shape->nextShapeIndex;
		}
	}

	// Report impulses
	b2Contact** contacts = world->contactArray;

	b2PostSolveFcn* postSolveFcn = island->world->postSolveFcn;
	if (postSolveFcn != NULL)
	{
		int16_t worldIndex = world->index;
		int32_t count = island->contactCount;
		const int32_t* contactIndices = island->contactIndices;
		const b2Shape* shapes = world->shapes;

		for (int32_t i = 0; i < count; ++i)
		{
			int32_t index = contactIndices[i];
			b2Array_Check(world->contactArray, index);
			const b2Contact* contact = contacts[index];

			const b2Shape* shapeA = shapes + contact->shapeIndexA;
			const b2Shape* shapeB = shapes + contact->shapeIndexB;

			b2ShapeId idA = { shapeA->object.index, worldIndex, shapeA->object.revision };
			b2ShapeId idB = { shapeB->object.index, worldIndex, shapeB->object.revision };
			postSolveFcn(idA, idB, &contact->manifold, world->postSolveContext);
		}
	}

	// Destroy in reverse order
	b2StackAllocator* alloc = island->world->stackAllocator;
	b2DestroyContactSolver(alloc, island->contactSolver);
	b2FreeStackItem(alloc, island);
}
