// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "box2d/callbacks.h"
#include "box2d/timer.h"

#include "body.h"
#include "contact.h"
#include "island.h"
#include "shape.h"
#include "solver_data.h"
#include "stack_allocator.h"

//#include "contact_solver.h"
//#include "joint.h"

#include <assert.h>

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
	b2StackAllocator* allocator,
	b2WorldCallbacks* callbacks)
{
	b2Island island = {0};
	island.bodyCapacity = bodyCapacity;
	island.contactCapacity = contactCapacity;
	island.jointCapacity = jointCapacity;

	island.allocator = allocator;
	island.callbacks = callbacks;

	island.bodies = (b2Body**)b2AllocateStackItem(allocator, bodyCapacity * sizeof(b2Body*));
	island.contacts = (b2Contact**)b2AllocateStackItem(allocator, contactCapacity * sizeof(b2Contact*));
	//island.joints = (b2Joint**)b2AllocateStackItem(allocator, jointCapacity * sizeof(b2Joint*));

	island.velocities = (b2Velocity*)b2AllocateStackItem(allocator, bodyCapacity * sizeof(b2Velocity));
	island.positions = (b2Position*)b2AllocateStackItem(allocator, bodyCapacity * sizeof(b2Position));
}

void b2DestroyIsland(b2Island* island)
{
	// Warning: the order should reverse the constructor order.
	b2FreeStackItem(island->allocator, island->positions);
	b2FreeStackItem(island->allocator, island->velocities);
	b2FreeStackItem(island->allocator, island->joints);
	b2FreeStackItem(island->allocator, island->contacts);
	b2FreeStackItem(island->allocator, island->bodies);
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

//void b2Island_AddJoint(b2Joint* joint)
//{
//	assert(island->jointCount < island->jointCapacity);
//	island->joints[island->jointCount++] = joint;
//}


void b2SolveIsland(b2Island* island, b2Profile* profile, const b2TimeStep* step, b2Vec2 gravity, bool allowSleep)
{
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

	b2Timer timer = b2CreateTimer();

	// Solver data
	b2SolverData solverData;
	solverData.step = *step;
	solverData.positions = island->positions;
	solverData.velocities = island->velocities;

	// Initialize velocity constraints.
	b2ContactSolverDef contactSolverDef;
	contactSolverDef.step = step;
	contactSolverDef.contacts = island->contacts;
	contactSolverDef.count = island->contactCount;
	contactSolverDef.positions = island->positions;
	contactSolverDef.velocities = island->velocities;
	contactSolverDef.allocator = island->allocator;

	b2ContactSolver contactSolver(&contactSolverDef);
	contactSolver.InitializeVelocityConstraints();

	if (step.warmStarting)
	{
		contactSolver.WarmStart();
	}
	
	for (int32_t i = 0; i < island->jointCount; ++i)
	{
		island->joints[i]->InitVelocityConstraints(solverData);
	}

	profile->solveInit = timer.GetMilliseconds();

	// Solve velocity constraints
	timer.Reset();
	for (int32_t i = 0; i < step.velocityIterations; ++i)
	{
		for (int32_t j = 0; j < island->jointCount; ++j)
		{
			island->joints[j]->SolveVelocityConstraints(solverData);
		}

		contactSolver.SolveVelocityConstraints();
	}

	// Special handling for restitution
	contactSolver.ApplyRestitution();

	// Store impulses for warm starting
	contactSolver.StoreImpulses();
	profile->solveVelocity = timer.GetMilliseconds();

	// Integrate positions
	for (int32_t i = 0; i < island->bodyCount; ++i)
	{
		b2Vec2 c = island->positions[i].c;
		float a = island->positions[i].a;
		b2Vec2 v = island->velocities[i].v;
		float w = island->velocities[i].w;

		// Check for large velocities
		b2Vec2 translation = h * v;
		if (b2Dot(translation, translation) > b2_maxTranslationSquared)
		{
			float ratio = b2_maxTranslation / translation.Length();
			v *= ratio;
		}

		float rotation = h * w;
		if (rotation * rotation > b2_maxRotationSquared)
		{
			float ratio = b2_maxRotation / b2Abs(rotation);
			w *= ratio;
		}

		// Integrate
		c += h * v;
		a += h * w;

		island->positions[i].c = c;
		island->positions[i].a = a;
		island->velocities[i].v = v;
		island->velocities[i].w = w;
	}

	// Solve position constraints
	timer.Reset();
	bool positionSolved = false;
	for (int32_t i = 0; i < step.positionIterations; ++i)
	{
		bool contactsOkay = contactSolver.SolvePositionConstraints();

		bool jointsOkay = true;
		for (int32_t j = 0; j < island->jointCount; ++j)
		{
			bool jointOkay = island->joints[j]->SolvePositionConstraints(solverData);
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
		body->SynchronizeTransform();
	}

	profile->solvePosition = timer.GetMilliseconds();

	Report(contactSolver.island->velocityConstraints);

	if (allowSleep)
	{
		float minSleepTime = b2_maxFloat;

		const float linTolSqr = b2_linearSleepTolerance * b2_linearSleepTolerance;
		const float angTolSqr = b2_angularSleepTolerance * b2_angularSleepTolerance;

		for (int32_t i = 0; i < island->bodyCount; ++i)
		{
			b2Body* b = island->bodies[i];
			if (b->GetType() == b2_staticBody)
			{
				continue;
			}

			if ((b->flags & b2Body::e_autoSleepFlag) == 0 ||
				b->angularVelocity * b->angularVelocity > angTolSqr ||
				b2Dot(b->linearVelocity, b->linearVelocity) > linTolSqr)
			{
				b->sleepTime = 0.0f;
				minSleepTime = 0.0f;
			}
			else
			{
				b->sleepTime += h;
				minSleepTime = b2Min(minSleepTime, b->sleepTime);
			}
		}

		if (minSleepTime >= b2_timeToSleep && positionSolved)
		{
			for (int32_t i = 0; i < island->bodyCount; ++i)
			{
				b2Body* b = island->bodies[i];
				b->SetAwake(false);
			}
		}
	}

	for (int32_t i = 0; i < island->bodyCount; ++i)
	{
		b2Body* b = island->bodies[i];
		if (b->IsAwake() == false || b->IsEnabled() == false)
		{
			continue;
		}

		b2Vec2 v = b->linearVelocity;
		float w = b->angularVelocity;

		// Stage 1 - apply forces
		v += step.dt * b->invMass * (b->gravityScale * b->mass * gravity + b->force);
		w += step.dt * b->invI * b->torque;

		// Apply damping.
		// ODE: dv/dt + c * v = 0
		// Solution: v(t) = v0 * exp(-c * t)
		// Time step: v(t + dt) = v0 * exp(-c * (t + dt)) = v0 * exp(-c * t) * exp(-c * dt) = v * exp(-c * dt)
		// v2 = exp(-c * dt) * v1
		// Pade approximation:
		// v2 = v1 * 1 / (1 + c * dt)
		v *= 1.0f / (1.0f + step.dt * b->linearDamping);
		w *= 1.0f / (1.0f + step.dt * b->angularDamping);

		// Stage 3 - predict new transforms
		b->speculativePosition = b->position + step.dt * v;
		b->speculativeAngle = b->angle + step.dt * w;
	}
}

void b2Island::Report(const b2ContactVelocityConstraint* constraints)
{
	if (island->listener == nullptr)
	{
		return;
	}

	for (int32_t i = 0; i < island->contactCount; ++i)
	{
		b2Contact* c = island->contacts[i];

		const b2ContactVelocityConstraint* vc = constraints + i;
		
		b2ContactImpulse impulse;
		impulse.count = vc->pointCount;
		for (int32_t j = 0; j < vc->pointCount; ++j)
		{
			impulse.normalImpulses[j] = vc->points[j].normalImpulse;
			impulse.tangentImpulses[j] = vc->points[j].tangentImpulse;
		}

		island->listener->PostSolve(c, &impulse);
	}
}
