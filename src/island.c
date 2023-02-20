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
#include "world.h"

#include <assert.h>
#include <float.h>

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

b2Island b2CreateIsland(
	int32_t bodyCapacity,
	int32_t contactCapacity,
	int32_t jointCapacity,
	b2World* world)
{
	b2Island island = {0};
	island.bodyCapacity = bodyCapacity;
	island.contactCapacity = contactCapacity;
	island.jointCapacity = jointCapacity;

	island.world = world;

	b2StackAllocator* alloc = &world->stackAllocator;
	island.bodies = (b2Body**)b2AllocateStackItem(alloc, bodyCapacity * sizeof(b2Body*));
	island.contacts = (b2Contact**)b2AllocateStackItem(alloc, contactCapacity * sizeof(b2Contact*));
	island.joints = (b2Joint**)b2AllocateStackItem(alloc, jointCapacity * sizeof(b2Joint*));

	island.velocities = (b2Velocity*)b2AllocateStackItem(alloc, bodyCapacity * sizeof(b2Velocity));
	island.positions = (b2Position*)b2AllocateStackItem(alloc, bodyCapacity * sizeof(b2Position));

	return island;
}

void b2DestroyIsland(b2Island* island)
{
	b2StackAllocator* alloc = &island->world->stackAllocator;

	// Warning: the order should reverse the constructor order.
	b2FreeStackItem(alloc, island->positions);
	b2FreeStackItem(alloc, island->velocities);
	b2FreeStackItem(alloc, island->joints);
	b2FreeStackItem(alloc, island->contacts);
	b2FreeStackItem(alloc, island->bodies);
}

void b2Island_AddBody(b2Island* island, struct b2Body* body)
{
	assert(island->bodyCount < island->bodyCapacity);
	body->islandIndex = island->bodyCount;
	island->bodies[island->bodyCount] = body;
	++island->bodyCount;
}

void b2Island_AddContact(b2Island* island, b2Contact* contact)
{
	assert(island->contactCount < island->contactCapacity);
	island->contacts[island->contactCount++] = contact;
}

void b2Island_AddJoint(b2Island* island, b2Joint* joint)
{
	assert(island->jointCount < island->jointCapacity);
	island->joints[island->jointCount++] = joint;
}

void b2SolveIsland(b2Island* island, b2Profile* profile, const b2TimeStep* step, b2Vec2 gravity)
{
	b2Timer timer = b2CreateTimer();

	float h = step->dt;

	// Integrate velocities and apply damping. Initialize the body state.
	for (int32_t i = 0; i < island->bodyCount; ++i)
	{
		b2Body* b = island->bodies[i];

		b2Vec2 c = b->position;
		float a = b->angle;
		b2Vec2 v = b->linearVelocity;
		float w = b->angularVelocity;

		if (b->type == b2_dynamicBody)
		{
			// Integrate velocities
			v = b2Add(v, b2MulSV(h * b->invMass, b2MulAdd(b->force, b->gravityScale * b->mass, gravity)));
			w = w + h * b->invI * b->torque;

			// Apply damping.
			// ODE: dv/dt + c * v = 0
			// Solution: v(t) = v0 * exp(-c * t)
			// Time step: v(t + dt) = v0 * exp(-c * (t + dt)) = v0 * exp(-c * t) * exp(-c * dt) = v * exp(-c * dt)
			// v2 = exp(-c * dt) * v1
			// Pade approximation:
			// v2 = v1 * 1 / (1 + c * dt)
			v = b2MulSV(1.0f / (1.0f + h * b->linearDamping), v);
			w *= 1.0f / (1.0f + h * b->angularDamping);
		}

		island->positions[i].c = c;
		island->positions[i].a = a;
		island->velocities[i].v = v;
		island->velocities[i].w = w;
	}

	// Solver data
	b2SolverData solverData;
	solverData.step = *step;
	solverData.positions = island->positions;
	solverData.velocities = island->velocities;

	// Initialize velocity constraints.
	b2ContactSolverDef contactSolverDef;
	contactSolverDef.step = *step;
	contactSolverDef.world = island->world;
	contactSolverDef.contacts = island->contacts;
	contactSolverDef.count = island->contactCount;
	contactSolverDef.positions = island->positions;
	contactSolverDef.velocities = island->velocities;

	b2ContactSolver solver = b2CreateContactSolver(&contactSolverDef);

	b2ContactSolver_InitializeVelocityConstraints(&solver);

	// TODO_ERIN do this in init?
	if (step->warmStarting)
	{
		b2ContactSolver_WarmStart(&solver);
	}
	
	for (int32_t i = 0; i < island->jointCount; ++i)
	{
		b2InitVelocityConstraints(island->world, island->joints[i], &solverData);
	}

	profile->solveInit = b2GetMillisecondsAndReset(&timer);

	// Solve velocity constraints
	for (int32_t i = 0; i < step->velocityIterations; ++i)
	{
		for (int32_t j = 0; j < island->jointCount; ++j)
		{
			b2SolveVelocityConstraints(island->joints[j], &solverData);
		}

		b2ContactSolver_SolveVelocityConstraints(&solver);
	}

	// Special handling for restitution
	b2ContactSolver_ApplyRestitution(&solver);

	// Store impulses for warm starting
	b2ContactSolver_StoreImpulses(&solver);
	profile->solveVelocity = b2GetMillisecondsAndReset(&timer);

	// Integrate positions
	for (int32_t i = 0; i < island->bodyCount; ++i)
	{
		b2Vec2 c = island->positions[i].c;
		float a = island->positions[i].a;
		b2Vec2 v = island->velocities[i].v;
		float w = island->velocities[i].w;

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

		island->positions[i].c = c;
		island->positions[i].a = a;
		island->velocities[i].v = v;
		island->velocities[i].w = w;
	}

	// Solve position constraints
	b2GetMillisecondsAndReset(&timer);
	bool positionSolved = false;
	for (int32_t i = 0; i < step->positionIterations; ++i)
	{
		bool contactsOkay = b2ContactSolver_SolvePositionConstraints(&solver);

		bool jointsOkay = true;
		for (int32_t j = 0; j < island->jointCount; ++j)
		{
			bool jointOkay = b2SolvePositionConstraints(island->joints[j], &solverData);
			jointsOkay = jointsOkay && jointOkay;
		}

		if (contactsOkay && jointsOkay)
		{
			// Exit early if the position errors are small.
			positionSolved = true;
			break;
		}
	}

	// Copy state buffers back to the bodies
	for (int32_t i = 0; i < island->bodyCount; ++i)
	{
		b2Body* body = island->bodies[i];
		body->position = island->positions[i].c;
		body->angle = island->positions[i].a;
		body->linearVelocity = island->velocities[i].v;
		body->angularVelocity = island->velocities[i].w;

		// Update body transform
		body->transform.q = b2MakeRot(body->angle);
		body->transform.p = b2Sub(body->position, b2RotateVector(body->transform.q, body->localCenter));
	}

	profile->solvePosition = b2GetMillisecondsAndReset(&timer);

	// Report impulses
	b2PostSolveFcn* postSolveFcn = island->world->callbacks.postSolveFcn;
	if (postSolveFcn != NULL)
	{
		int16_t worldIndex = island->world->index;

		for (int32_t i = 0; i < island->contactCount; ++i)
		{
			const b2Contact* contact = island->contacts[i];

			b2ContactImpulse impulse = b2ContactSolver_GetImpulse(&solver, i);

			const b2Shape* shapeA = island->world->shapes + contact->shapeIndexA;
			const b2Shape* shapeB = island->world->shapes + contact->shapeIndexB;

			b2ShapeId idA = {shapeA->object.index, worldIndex, shapeA->object.revision};
			b2ShapeId idB = {shapeB->object.index, worldIndex, shapeB->object.revision};
			postSolveFcn(idA, idB, &impulse);
		}
	}

	// Update sleep
	bool isIslandAwake = true;

	b2World* world = island->world;
	if (world->enableSleep)
	{
		float minSleepTime = FLT_MAX;

		const float linTolSqr = b2_linearSleepTolerance * b2_linearSleepTolerance;
		const float angTolSqr = b2_angularSleepTolerance * b2_angularSleepTolerance;

		for (int32_t i = 0; i < island->bodyCount; ++i)
		{
			b2Body* b = island->bodies[i];
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
				b2Body* b = island->bodies[i];
				b->isAwake = false;
				b->sleepTime = 0.0f;
				b->linearVelocity = b2Vec2_zero;
				b->angularVelocity = 0.0f;
				b->speculativePosition = b->position;
				b->speculativeAngle = b->angle;
			}
		}
	}

	// Speculative transform
	// TODO_ERIN using old forces? Should be at the beginning of the time step?
	if (isIslandAwake)
	{
		for (int32_t i = 0; i < island->bodyCount; ++i)
		{
			b2Body* b = island->bodies[i];
			if (b->type == b2_staticBody)
			{
				continue;
			}

			assert(b->awakeIndex == B2_NULL_INDEX);
			b->awakeIndex = b2Array(world->awakeBodies).count;
			b2Array_Push(world->awakeBodies, b->object.index);

			b2Vec2 v = b->linearVelocity;
			float w = b->angularVelocity;

			v = b2Add(v, b2MulSV(h * b->invMass, b2MulAdd(b->force, b->gravityScale * b->mass, gravity)));
			w = w + h * b->invI * b->torque;

			v = b2MulSV(1.0f / (1.0f + h * b->linearDamping), v);
			w *= 1.0f / (1.0f + h * b->angularDamping);

			// Stage 3 - predict new transforms
			b->speculativePosition = b2MulAdd(b->position, step->dt, v);
			b->speculativeAngle = b->angle + step->dt * w;

			// Update shapes (for broad-phase).
			int32_t shapeIndex = b->shapeIndex;
			while (shapeIndex != B2_NULL_INDEX)
			{
				b2Shape* shape = world->shapes + shapeIndex;
				for (int32_t j = 0; j < shape->proxyCount; ++j)
				{
					b2ShapeProxy* proxy = shape->proxies + j;

					// TODO_ERIN speculate
					proxy->aabb = b2Shape_ComputeAABB(shape, b->transform, proxy->childIndex);

					b2BroadPhase_MoveProxy(&world->broadPhase, proxy->proxyKey, proxy->aabb);
				}

				shapeIndex = shape->nextShapeIndex;
			}
		}
	}

	b2DestroyContactSolver(&solver);
}
