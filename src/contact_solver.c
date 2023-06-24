// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "array.h"
#include "body.h"
#include "contact.h"
#include "contact_solver.h"
#include "stack_allocator.h"
#include "world.h"

#include <assert.h>
#include <stdlib.h>

// Solver debugging is normally disabled because the block solver sometimes has to deal with a poorly conditioned
// effective mass matrix.
#define B2_DEBUG_SOLVER 0

typedef struct b2VelocityConstraintPoint
{
	b2Vec2 rA;
	b2Vec2 rB;
	float normalImpulse;
	float tangentImpulse;
	float normalMass;
	float tangentMass;
	float velocityBias;
	float relativeVelocity;
} b2VelocityConstraintPoint;

typedef struct b2ContactVelocityConstraint
{
	b2Contact* contact;
	b2VelocityConstraintPoint points[2];
	b2Vec2 normal;
	b2Mat22 normalMass;
	b2Mat22 K;
	float friction;
	float restitution;
	float tangentSpeed;
	int32_t pointCount;
} b2ContactVelocityConstraint;

typedef struct b2ContactPositionConstraint
{
	b2Contact* contact;
	b2Vec2 localAnchorsA[2];
	b2Vec2 localAnchorsB[2];
	float separations[2];
	float lambdas[2];
	b2Vec2 normal;
	int32_t pointCount;
} b2ContactPositionConstraint;

b2ContactSolver* b2CreateContactSolver(b2ContactSolverDef* def)
{
	b2StackAllocator* alloc = def->world->stackAllocator;

	b2ContactSolver* solver = b2AllocateStackItem(alloc, sizeof(b2ContactSolver), "contact solver");
	solver->context = def->context;
	solver->contactList = def->contactList;
	solver->contactCount = def->contactCount;

	// These are allocated conservatively because some island contacts may not have contact points
	solver->positionConstraints = b2AllocateStackItem(alloc, solver->contactCount * sizeof(b2ContactPositionConstraint), "position constraints");
	solver->velocityConstraints = b2AllocateStackItem(alloc, solver->contactCount * sizeof(b2ContactVelocityConstraint), "velocity constraints");

	solver->world = def->world;
	solver->constraintCount = 0;
	return solver;
}

void b2DestroyContactSolver(b2ContactSolver* solver)
{
	b2StackAllocator* alloc = solver->world->stackAllocator;

	b2FreeStackItem(alloc, solver->velocityConstraints);
	b2FreeStackItem(alloc, solver->positionConstraints);
	b2FreeStackItem(alloc, solver);
}

void b2ContactSolver_Initialize(b2ContactSolver* solver)
{
	b2World* world = solver->world;
	b2Contact* contacts = world->contacts;
	const b2StepContext* context = solver->context;
	b2Body* bodies = world->bodies;

	// Initialize position independent portions of the constraints.
	int32_t constraintCount = 0;
	int32_t contactIndex = solver->contactList;
	while (contactIndex != B2_NULL_INDEX)
	{
		b2Contact* contact = contacts + contactIndex;
		contactIndex = contact->islandNext;

		const b2Manifold* manifold = &contact->manifold;
		int32_t pointCount = manifold->pointCount;

		if (pointCount == 0)
		{
			continue;
		}

		int32_t indexA = contact->edges[0].bodyIndex;
		int32_t indexB = contact->edges[1].bodyIndex;
		b2Body* bodyA = bodies + indexA;
		b2Body* bodyB = bodies + indexB;

		b2ContactVelocityConstraint* vc = solver->velocityConstraints + constraintCount;
		vc->contact = contact;
		vc->normal = manifold->normal;
		vc->friction = contact->friction;
		vc->restitution = contact->restitution;
		vc->tangentSpeed = contact->tangentSpeed;
		vc->pointCount = pointCount;
		vc->K = b2Mat22_zero;
		vc->normalMass = b2Mat22_zero;

		b2ContactPositionConstraint* pc = solver->positionConstraints + constraintCount;
		pc->contact = contact;
		pc->normal = manifold->normal;
		pc->pointCount = pointCount;

		float mA = bodyA->invMass;
		float iA = bodyA->invI;
		float mB = bodyB->invMass;
		float iB = bodyB->invI;

		b2Rot qA = bodyA->transform.q;
		b2Vec2 cA = bodyA->position;
		b2Rot qB = bodyB->transform.q;
		b2Vec2 cB = bodyB->position;

		// TODO_ERIN testing
		//qA = b2MakeRot(bodyA->angle);
		//qB = b2MakeRot(bodyB->angle);

		b2Vec2 vA = bodyA->linearVelocity;
		float wA = bodyA->angularVelocity;
		b2Vec2 vB = bodyB->linearVelocity;
		float wB = bodyB->angularVelocity;

		for (int32_t j = 0; j < pointCount; ++j)
		{
			const b2ManifoldPoint* cp = manifold->points + j;
			b2VelocityConstraintPoint* vcp = vc->points + j;

			if (context->warmStarting)
			{
				vcp->normalImpulse = context->dtRatio * cp->normalImpulse;
				vcp->tangentImpulse = context->dtRatio * cp->tangentImpulse;
			}
			else
			{
				vcp->normalImpulse = 0.0f;
				vcp->tangentImpulse = 0.0f;
			}

			vcp->rA = b2Sub(cp->point, cA);
			vcp->rB = b2Sub(cp->point, cB);

			float rnA = b2Cross(vcp->rA, vc->normal);
			float rnB = b2Cross(vcp->rB, vc->normal);

			float kNormal = mA + mB + iA * rnA * rnA + iB * rnB * rnB;

			vcp->normalMass = kNormal > 0.0f ? 1.0f / kNormal : 0.0f;

			b2Vec2 tangent = b2CrossVS(vc->normal, 1.0f);

			float rtA = b2Cross(vcp->rA, tangent);
			float rtB = b2Cross(vcp->rB, tangent);

			float kTangent = mA + mB + iA * rtA * rtA + iB * rtB * rtB;

			vcp->tangentMass = kTangent > 0.0f ? 1.0f / kTangent : 0.0f;

			// Velocity bias for speculative collision
			vcp->velocityBias = -B2_MAX(0.0f, cp->separation * context->inv_dt);

			// Relative velocity
			b2Vec2 vrB = b2Add(vB, b2CrossSV(wB, vcp->rB));
			b2Vec2 vrA = b2Add(vA, b2CrossSV(wA, vcp->rA));
			vcp->relativeVelocity = b2Dot(vc->normal, b2Sub(vrB, vrA));

			pc->localAnchorsA[j] = b2InvRotateVector(qA, vcp->rA);
			pc->localAnchorsB[j] = b2InvRotateVector(qB, vcp->rB);
			pc->separations[j] = cp->separation;
			pc->lambdas[j] = 0.0f;
		}

		// If we have two points, then prepare the block solver.
		if (vc->pointCount == 2)
		{
			b2VelocityConstraintPoint* vcp1 = vc->points + 0;
			b2VelocityConstraintPoint* vcp2 = vc->points + 1;

			float rn1A = b2Cross(vcp1->rA, vc->normal);
			float rn1B = b2Cross(vcp1->rB, vc->normal);
			float rn2A = b2Cross(vcp2->rA, vc->normal);
			float rn2B = b2Cross(vcp2->rB, vc->normal);

			float k11 = mA + mB + iA * rn1A * rn1A + iB * rn1B * rn1B;
			float k22 = mA + mB + iA * rn2A * rn2A + iB * rn2B * rn2B;
			float k12 = mA + mB + iA * rn1A * rn2A + iB * rn1B * rn2B;

			// Ensure a reasonable condition number.
			const float k_maxConditionNumber = 1000.0f;
			if (k11 * k11 < k_maxConditionNumber * (k11 * k22 - k12 * k12))
			{
				// K is safe to invert.
				vc->K.cx = (b2Vec2){ k11, k12 };
				vc->K.cy = (b2Vec2){ k12, k22 };
				vc->normalMass = b2GetInverse22(vc->K);
			}
			else
			{
				// The constraints are redundant, just use one.
				// TODO_ERIN use deepest?
				vc->pointCount = 1;
			}
		}

		constraintCount += 1;
	}

	solver->constraintCount = constraintCount;

	// Warm start
	if (context->warmStarting)
	{
		for (int32_t i = 0; i < constraintCount; ++i)
		{
			b2ContactVelocityConstraint* vc = solver->velocityConstraints + i;

			const b2Contact* contact = vc->contact;

			int32_t indexA = contact->edges[0].bodyIndex;
			int32_t indexB = contact->edges[1].bodyIndex;
			b2Body* bodyA = bodies + indexA;
			b2Body* bodyB = bodies + indexB;
			float mA = bodyA->invMass;
			float iA = bodyA->invI;
			float mB = bodyB->invMass;
			float iB = bodyB->invI;
			int32_t pointCount = vc->pointCount;

			b2Vec2 vA = bodyA->linearVelocity;
			float wA = bodyA->angularVelocity;
			b2Vec2 vB = bodyB->linearVelocity;
			float wB = bodyB->angularVelocity;

			b2Vec2 normal = vc->normal;
			b2Vec2 tangent = b2CrossVS(normal, 1.0f);

			for (int32_t j = 0; j < pointCount; ++j)
			{
				b2VelocityConstraintPoint* vcp = vc->points + j;
				b2Vec2 P = b2Add(b2MulSV(vcp->normalImpulse, normal), b2MulSV(vcp->tangentImpulse, tangent));
				wA -= iA * b2Cross(vcp->rA, P);
				vA = b2MulAdd(vA, -mA, P);
				wB += iB * b2Cross(vcp->rB, P);
				vB = b2MulAdd(vB, mB, P);
			}

			bodyA->linearVelocity = vA;
			bodyA->angularVelocity = wA;
			bodyB->linearVelocity = vB;
			bodyB->angularVelocity = wB;
		}
	}
}

void b2ContactSolver_SolveVelocityConstraints(b2ContactSolver* solver)
{
	int32_t count = solver->constraintCount;

	b2World* world = solver->world;
	b2Body* bodies = world->bodies;

	for (int32_t i = 0; i < count; ++i)
	{
		b2ContactVelocityConstraint* vc = solver->velocityConstraints + i;

		const b2Contact* contact = vc->contact;

		int32_t indexA = contact->edges[0].bodyIndex;
		int32_t indexB = contact->edges[1].bodyIndex;
		b2Body* bodyA = bodies + indexA;
		b2Body* bodyB = bodies + indexB;

		float mA = bodyA->invMass;
		float iA = bodyA->invI;
		float mB = bodyB->invMass;
		float iB = bodyB->invI;
		int32_t pointCount = vc->pointCount;

		b2Vec2 vA = bodyA->linearVelocity;
		float wA = bodyA->angularVelocity;
		b2Vec2 vB = bodyB->linearVelocity;
		float wB = bodyB->angularVelocity;

		b2Vec2 normal = vc->normal;
		b2Vec2 tangent = b2CrossVS(normal, 1.0f);
		float friction = vc->friction;

		assert(pointCount == 1 || pointCount == 2);

		// Solve tangent constraints first because non-penetration is more important
		// than friction.
		for (int32_t j = 0; j < pointCount; ++j)
		{
			b2VelocityConstraintPoint* vcp = vc->points + j;

			// Relative velocity at contact
			b2Vec2 vrB = b2Add(vB, b2CrossSV(wB, vcp->rB));
			b2Vec2 vrA = b2Add(vA, b2CrossSV(wA, vcp->rA));
			b2Vec2 dv = b2Sub(vrB, vrA);

			// Compute tangent force
			float vt = b2Dot(dv, tangent) - vc->tangentSpeed;
			float lambda = vcp->tangentMass * (-vt);

			// Clamp the accumulated force
			float maxFriction = friction * vcp->normalImpulse;
			float newImpulse = B2_CLAMP(vcp->tangentImpulse + lambda, -maxFriction, maxFriction);
			lambda = newImpulse - vcp->tangentImpulse;
			vcp->tangentImpulse = newImpulse;

			// Apply contact impulse
			b2Vec2 P = b2MulSV(lambda, tangent);

			vA = b2MulSub(vA, mA, P);
			wA -= iA * b2Cross(vcp->rA, P);

			vB = b2MulAdd(vB, mB, P);
			wB += iB * b2Cross(vcp->rB, P);
		}

		// Solve normal constraints
		if (pointCount == 1)
		{
			for (int32_t j = 0; j < pointCount; ++j)
			{
				b2VelocityConstraintPoint* vcp = vc->points + j;

				// Relative velocity at contact
				b2Vec2 vrB = b2Add(vB, b2CrossSV(wB, vcp->rB));
				b2Vec2 vrA = b2Add(vA, b2CrossSV(wA, vcp->rA));
				b2Vec2 dv = b2Sub(vrB, vrA);

				// Compute normal impulse
				float vn = b2Dot(dv, normal);
				float lambda = -vcp->normalMass * (vn - vcp->velocityBias);

				// Clamp the accumulated impulse
				float newImpulse = B2_MAX(vcp->normalImpulse + lambda, 0.0f);
				lambda = newImpulse - vcp->normalImpulse;
				vcp->normalImpulse = newImpulse;

				// Apply contact impulse
				b2Vec2 P = b2MulSV(lambda, normal);
				vA = b2MulSub(vA, mA, P);
				wA -= iA * b2Cross(vcp->rA, P);

				vB = b2MulAdd(vB, mB, P);
				wB += iB * b2Cross(vcp->rB, P);
			}
		}
		else
		{
			// Block solver developed in collaboration with Dirk Gregorius (back in 01/07 on Box2D_Lite).
			// Build the mini LCP for this contact patch
			//
			// vn = A * x + b, vn >= 0, x >= 0 and vn_i * x_i = 0 with i = 1..2
			//
			// A = J * W * JT and J = ( -n, -r1 x n, n, r2 x n )
			// b = vn0 - velocityBias
			//
			// The system is solved using the "Total enumeration method" (s. Murty). The complementary constraint vn_i *
			// x_i implies that we must have in any solution either vn_i = 0 or x_i = 0. So for the 2D contact problem
			// the cases vn1 = 0 and vn2 = 0, x1 = 0 and x2 = 0, x1 = 0 and vn2 = 0, x2 = 0 and vn1 = 0 need to be
			// tested. The first valid solution that satisfies the problem is chosen.
			//
			// In order to account of the accumulated impulse 'a' (because of the iterative nature of the solver which
			// only requires that the accumulated impulse is clamped and not the incremental impulse) we change the
			// impulse variable (x_i).
			//
			// Substitute:
			//
			// x = a + d
			//
			// a := old total impulse
			// x := new total impulse
			// d := incremental impulse
			//
			// For the current iteration we extend the formula for the incremental impulse
			// to compute the new total impulse:
			//
			// vn = A * d + b
			//    = A * (x - a) + b
			//    = A * x + b - A * a
			//    = A * x + b'
			// b' = b - A * a;

			b2VelocityConstraintPoint* cp1 = vc->points + 0;
			b2VelocityConstraintPoint* cp2 = vc->points + 1;

			b2Vec2 a = {cp1->normalImpulse, cp2->normalImpulse};
			assert(a.x >= 0.0f && a.y >= 0.0f);

			// Relative velocity at contact
			b2Vec2 vrA, vrB;
			vrA = b2Add(vA, b2CrossSV(wA, cp1->rA));
			vrB = b2Add(vB, b2CrossSV(wB, cp1->rB));
			b2Vec2 dv1 = b2Sub(vrB, vrA);
			vrA = b2Add(vA, b2CrossSV(wA, cp2->rA));
			vrB = b2Add(vB, b2CrossSV(wB, cp2->rB));
			b2Vec2 dv2 = b2Sub(vrB, vrA);

			// Compute normal velocity
			float vn1 = b2Dot(dv1, normal);
			float vn2 = b2Dot(dv2, normal);

			b2Vec2 b = {vn1 - cp1->velocityBias, vn2 - cp2->velocityBias};

			// Compute b'
			b = b2Sub(b, b2MulMV(vc->K, a));

			const float k_errorTol = 1e-3f;
			B2_MAYBE_UNUSED(k_errorTol);

			for (;;)
			{
				//
				// Case 1: vn = 0
				//
				// 0 = A * x + b'
				//
				// Solve for x:
				//
				// x = - inv(A) * b'
				//
				b2Vec2 x = b2Neg(b2MulMV(vc->normalMass, b));

				if (x.x >= 0.0f && x.y >= 0.0f)
				{
					// Get the incremental impulse
					b2Vec2 d = b2Sub(x, a);

					// Apply incremental impulse
					b2Vec2 P1 = b2MulSV(d.x, normal);
					b2Vec2 P2 = b2MulSV(d.y, normal);
					vA = b2MulSub(vA, mA, b2Add(P1, P2));
					wA -= iA * (b2Cross(cp1->rA, P1) + b2Cross(cp2->rA, P2));

					vB = b2MulAdd(vB, mB, b2Add(P1, P2));
					wB += iB * (b2Cross(cp1->rB, P1) + b2Cross(cp2->rB, P2));

					// Accumulate
					cp1->normalImpulse = x.x;
					cp2->normalImpulse = x.y;

#if B2_DEBUG_SOLVER == 1
					// Postconditions
					dv1 = vB + b2Cross(wB, cp1->rB) - vA - b2Cross(wA, cp1->rA);
					dv2 = vB + b2Cross(wB, cp2->rB) - vA - b2Cross(wA, cp2->rA);

					// Compute normal velocity
					vn1 = b2Dot(dv1, normal);
					vn2 = b2Dot(dv2, normal);

					assert(b2Abs(vn1 - cp1->velocityBias) < k_errorTol);
					assert(b2Abs(vn2 - cp2->velocityBias) < k_errorTol);
#endif
					break;
				}

				//
				// Case 2: vn1 = 0 and x2 = 0
				//
				//   0 = a11 * x1 + a12 * 0 + b1'
				// vn2 = a21 * x1 + a22 * 0 + b2'
				//
				x.x = -cp1->normalMass * b.x;
				x.y = 0.0f;
				vn1 = 0.0f;
				vn2 = vc->K.cx.y * x.x + b.y;
				if (x.x >= 0.0f && vn2 >= 0.0f)
				{
					// Get the incremental impulse
					b2Vec2 d = b2Sub(x, a);

					// Apply incremental impulse
					b2Vec2 P1 = b2MulSV(d.x, normal);
					b2Vec2 P2 = b2MulSV(d.y, normal);

					vA = b2MulSub(vA, mA, b2Add(P1, P2));
					wA -= iA * (b2Cross(cp1->rA, P1) + b2Cross(cp2->rA, P2));

					vB = b2MulAdd(vB, mB, b2Add(P1, P2));
					wB += iB * (b2Cross(cp1->rB, P1) + b2Cross(cp2->rB, P2));

					// Accumulate
					cp1->normalImpulse = x.x;
					cp2->normalImpulse = x.y;

#if B2_DEBUG_SOLVER == 1
					// Postconditions
					dv1 = vB + b2Cross(wB, cp1->rB) - vA - b2Cross(wA, cp1->rA);

					// Compute normal velocity
					vn1 = b2Dot(dv1, normal);

					assert(b2Abs(vn1 - cp1->velocityBias) < k_errorTol);
#endif
					break;
				}

				//
				// Case 3: vn2 = 0 and x1 = 0
				//
				// vn1 = a11 * 0 + a12 * x2 + b1'
				//   0 = a21 * 0 + a22 * x2 + b2'
				//
				x.x = 0.0f;
				x.y = -cp2->normalMass * b.y;
				vn1 = vc->K.cy.x * x.y + b.x;
				vn2 = 0.0f;

				if (x.y >= 0.0f && vn1 >= 0.0f)
				{
					// Resubstitute for the incremental impulse
					b2Vec2 d = b2Sub(x, a);

					// Apply incremental impulse
					b2Vec2 P1 = b2MulSV(d.x, normal);
					b2Vec2 P2 = b2MulSV(d.y, normal);

					vA = b2MulSub(vA, mA, b2Add(P1, P2));
					wA -= iA * (b2Cross(cp1->rA, P1) + b2Cross(cp2->rA, P2));

					vB = b2MulAdd(vB, mB, b2Add(P1, P2));
					wB += iB * (b2Cross(cp1->rB, P1) + b2Cross(cp2->rB, P2));

					// Accumulate
					cp1->normalImpulse = x.x;
					cp2->normalImpulse = x.y;

#if B2_DEBUG_SOLVER == 1
					// Postconditions
					dv2 = vB + b2Cross(wB, cp2->rB) - vA - b2Cross(wA, cp2->rA);

					// Compute normal velocity
					vn2 = b2Dot(dv2, normal);

					assert(b2Abs(vn2 - cp2->velocityBias) < k_errorTol);
#endif
					break;
				}

				//
				// Case 4: x1 = 0 and x2 = 0
				//
				// vn1 = b1
				// vn2 = b2;
				x.x = 0.0f;
				x.y = 0.0f;
				vn1 = b.x;
				vn2 = b.y;

				if (vn1 >= 0.0f && vn2 >= 0.0f)
				{
					// Resubstitute for the incremental impulse
					b2Vec2 d = b2Sub(x, a);

					// Apply incremental impulse
					b2Vec2 P1 = b2MulSV(d.x, normal);
					b2Vec2 P2 = b2MulSV(d.y, normal);

					vA = b2MulSub(vA, mA, b2Add(P1, P2));
					wA -= iA * (b2Cross(cp1->rA, P1) + b2Cross(cp2->rA, P2));

					vB = b2MulAdd(vB, mB, b2Add(P1, P2));
					wB += iB * (b2Cross(cp1->rB, P1) + b2Cross(cp2->rB, P2));

					// Accumulate
					cp1->normalImpulse = x.x;
					cp2->normalImpulse = x.y;

					break;
				}

				// No solution, give up. This is hit sometimes, but it doesn't seem to matter.
				break;
			}
		}

		bodyA->linearVelocity = vA;
		bodyA->angularVelocity = wA;
		bodyB->linearVelocity = vB;
		bodyB->angularVelocity = wB;
	}
}

void b2ContactSolver_ApplyRestitution(b2ContactSolver* solver)
{
	int32_t count = solver->constraintCount;
	float threshold = solver->context->restitutionThreshold;

	b2World* world = solver->world;
	b2Body* bodies = world->bodies;

	for (int32_t i = 0; i < count; ++i)
	{
		b2ContactVelocityConstraint* vc = solver->velocityConstraints + i;
		const b2Contact* contact = vc->contact;

		int32_t indexA = contact->edges[0].bodyIndex;
		int32_t indexB = contact->edges[1].bodyIndex;
		b2Body* bodyA = bodies + indexA;
		b2Body* bodyB = bodies + indexB;

		if (vc->restitution == 0.0f)
		{
			continue;
		}

		float mA = bodyA->invMass;
		float iA = bodyA->invI;
		float mB = bodyB->invMass;
		float iB = bodyB->invI;
		int32_t pointCount = vc->pointCount;

		b2Vec2 vA = bodyA->linearVelocity;
		float wA = bodyA->angularVelocity;
		b2Vec2 vB = bodyB->linearVelocity;
		float wB = bodyB->angularVelocity;

		b2Vec2 normal = vc->normal;

		for (int32_t j = 0; j < pointCount; ++j)
		{
			b2VelocityConstraintPoint* vcp = vc->points + j;

			// if the normal impulse is zero then there was no collision
			if (vcp->relativeVelocity > -threshold || vcp->normalImpulse == 0.0f)
			{
				continue;
			}

			// Relative velocity at contact
			b2Vec2 vrB = b2Add(vB, b2CrossSV(wB, vcp->rB));
			b2Vec2 vrA = b2Add(vA, b2CrossSV(wA, vcp->rA));
			b2Vec2 dv = b2Sub(vrB, vrA);

			// Compute normal impulse
			float vn = b2Dot(dv, normal);
			float lambda = -vcp->normalMass * (vn + vc->restitution * vcp->relativeVelocity);

			// Apply contact impulse
			b2Vec2 P = b2MulSV(lambda, normal);
			vA = b2MulSub(vA, mA, P);
			wA -= iA * b2Cross(vcp->rA, P);

			vB = b2MulAdd(vB, mB, P);
			wB += iB * b2Cross(vcp->rB, P);
		}

		bodyA->linearVelocity = vA;
		bodyA->angularVelocity = wA;
		bodyB->linearVelocity = vB;
		bodyB->angularVelocity = wB;
	}
}

void b2ContactSolver_StoreImpulses(b2ContactSolver* solver)
{
	int32_t count = solver->constraintCount;

	for (int32_t i = 0; i < count; ++i)
	{
		b2ContactVelocityConstraint* vc = solver->velocityConstraints + i;
		b2Contact* contact = vc->contact;

		b2Manifold* manifold = &contact->manifold;

		for (int32_t j = 0; j < vc->pointCount; ++j)
		{
			manifold->points[j].normalImpulse = vc->points[j].normalImpulse;
			manifold->points[j].tangentImpulse = vc->points[j].tangentImpulse;
		}
	}
}

bool b2ContactSolver_SolvePositionConstraintsBlock(b2ContactSolver* solver)
{
	float minSeparation = 0.0f;
	int32_t count = solver->constraintCount;
	float slop = b2_linearSlop;

	b2World* world = solver->world;
	b2Body* bodies = world->bodies;

	for (int32_t i = 0; i < count; ++i)
	{
		b2ContactPositionConstraint* pc = solver->positionConstraints + i;
		const b2Contact* contact = pc->contact;

		int32_t indexA = contact->edges[0].bodyIndex;
		int32_t indexB = contact->edges[1].bodyIndex;
		b2Body* bodyA = bodies + indexA;
		b2Body* bodyB = bodies + indexB;

		float mA = bodyA->invMass;
		float iA = bodyA->invI;
		float mB = bodyB->invMass;
		float iB = bodyB->invI;

		int32_t pointCount = pc->pointCount;

		b2Vec2 cA = bodyA->position;
		float aA = bodyA->angle;
		b2Vec2 cB = bodyB->position;
		float aB = bodyB->angle;
		
		b2Vec2 normal = pc->normal;

		if (pointCount == 2)
		{
			b2Rot qA = b2MakeRot(aA);
			b2Rot qB = b2MakeRot(aB);

			b2Vec2 rA1 = b2RotateVector(qA, pc->localAnchorsA[0]);
			b2Vec2 rB1 = b2RotateVector(qB, pc->localAnchorsB[0]);
			b2Vec2 rA2 = b2RotateVector(qA, pc->localAnchorsA[1]);
			b2Vec2 rB2 = b2RotateVector(qB, pc->localAnchorsB[1]);

			// Current separation
			b2Vec2 d1 = b2Sub(b2Add(cB, rB1), b2Add(cA, rA1));
			float separation1 = b2Dot(d1, normal) + pc->separations[0];

			b2Vec2 d2 = b2Sub(b2Add(cB, rB2), b2Add(cA, rA2));
			float separation2 = b2Dot(d2, normal) + pc->separations[1];

			// Track max constraint error.
			minSeparation = B2_MIN(minSeparation, separation1);
			minSeparation = B2_MIN(minSeparation, separation2);

			float C1 = B2_CLAMP(b2_baumgarte * (separation1 + slop), -b2_maxLinearCorrection, 0.0f);
			float C2 = B2_CLAMP(b2_baumgarte * (separation2 + slop), -b2_maxLinearCorrection, 0.0f);

			b2Vec2 b = {C1, C2};

			float rn1A = b2Cross(rA1, normal);
			float rn1B = b2Cross(rB1, normal);
			float rn2A = b2Cross(rA2, normal);
			float rn2B = b2Cross(rB2, normal);

			float k11 = mA + mB + iA * rn1A * rn1A + iB * rn1B * rn1B;
			float k22 = mA + mB + iA * rn2A * rn2A + iB * rn2B * rn2B;
			float k12 = mA + mB + iA * rn1A * rn2A + iB * rn1B * rn2B;

			b2Mat22 K, invK;

			// Ensure a reasonable condition number.
			const float k_maxConditionNumber = 1000.0f;
			if (k11 * k11 < k_maxConditionNumber * (k11 * k22 - k12 * k12))
			{
				// K is safe to invert.
				K.cx = (b2Vec2){k11, k12};
				K.cy = (b2Vec2){k12, k22};
				invK = b2GetInverse22(K);
			}
			else
			{
				// The constraints are redundant, just use one.
				continue;
			}

			const float k_errorTol = 1e-3f;
			B2_MAYBE_UNUSED(k_errorTol);

			for (;;)
			{
				//
				// Case 1: vn = 0
				//
				// 0 = A * x + b'
				//
				// Solve for x:
				//
				// x = - inv(A) * b'
				//
				b2Vec2 x = b2Neg(b2MulMV(invK, b));

				if (x.x >= 0.0f && x.y >= 0.0f)
				{
					// Get the incremental impulse
					b2Vec2 d = x;

					// Apply incremental impulse
					b2Vec2 P1 = b2MulSV(d.x, normal);
					b2Vec2 P2 = b2MulSV(d.y, normal);

					cA = b2MulSub(cA, mA, b2Add(P1, P2));
					aA -= iA * (b2Cross(rA1, P1) + b2Cross(rA2, P2));

					cB = b2MulAdd(cB, mB, b2Add(P1, P2));
					aB += iB * (b2Cross(rB1, P1) + b2Cross(rB2, P2));
					break;
				}

				//
				// Case 2: vn1 = 0 and x2 = 0
				//
				//   0 = a11 * x1 + a12 * 0 + b1'
				// vn2 = a21 * x1 + a22 * 0 + b2'
				//
				x.x = -b.x / k11;
				x.y = 0.0f;
				float vn2 = K.cx.y * x.x + b.y;
				if (x.x >= 0.0f && vn2 >= 0.0f)
				{
					// Get the incremental impulse
					b2Vec2 d = x;

					// Apply incremental impulse
					b2Vec2 P1 = b2MulSV(d.x, normal);
					b2Vec2 P2 = b2MulSV(d.y, normal);

					cA = b2MulSub(cA, mA, b2Add(P1, P2));
					aA -= iA * (b2Cross(rA1, P1) + b2Cross(rA2, P2));

					cB = b2MulAdd(cB, mB, b2Add(P1, P2));
					aB += iB * (b2Cross(rB1, P1) + b2Cross(rB2, P2));
					break;
				}

				//
				// Case 3: vn2 = 0 and x1 = 0
				//
				// vn1 = a11 * 0 + a12 * x2 + b1'
				//   0 = a21 * 0 + a22 * x2 + b2'
				//
				x.x = 0.0f;
				x.y = -b.y / k22;
				float vn1 = K.cy.x * x.y + b.x;
				if (x.y >= 0.0f && vn1 >= 0.0f)
				{
					// Resubstitute for the incremental impulse
					b2Vec2 d = x;

					// Apply incremental impulse
					b2Vec2 P1 = b2MulSV(d.x, normal);
					b2Vec2 P2 = b2MulSV(d.y, normal);

					cA = b2MulSub(cA, mA, b2Add(P1, P2));
					aA -= iA * (b2Cross(rA1, P1) + b2Cross(rA2, P2));

					cB = b2MulAdd(cB, mB, b2Add(P1, P2));
					aB += iB * (b2Cross(rB1, P1) + b2Cross(rB2, P2));
					break;
				}
				break;
			}
		}
		else
		{
			for (int32_t j = 0; j < pointCount; ++j)
			{
				b2Rot qA = b2MakeRot(aA);
				b2Rot qB = b2MakeRot(aB);

				b2Vec2 rA = b2RotateVector(qA, pc->localAnchorsA[j]);
				b2Vec2 rB = b2RotateVector(qB, pc->localAnchorsB[j]);

				// Current separation
				b2Vec2 d = b2Sub(b2Add(cB, rB), b2Add(cA, rA));
				float separation = b2Dot(d, normal) + pc->separations[j];

				// Track max constraint error.
				minSeparation = B2_MIN(minSeparation, separation);

				// Prevent large corrections. Need to maintain a small overlap to avoid overshoot.
				// This improves stacking stability significantly.
				float C = B2_CLAMP(b2_baumgarte * (separation + slop), -b2_maxLinearCorrection, 0.0f);

				// Compute the effective mass.
				float rnA = b2Cross(rA, normal);
				float rnB = b2Cross(rB, normal);
				float K = mA + mB + iA * rnA * rnA + iB * rnB * rnB;

				// Compute normal impulse
				float impulse = K > 0.0f ? -C / K : 0.0f;

				b2Vec2 P = b2MulSV(impulse, normal);

				cA = b2MulSub(cA, mA, P);
				aA -= iA * b2Cross(rA, P);

				cB = b2MulAdd(cB, mB, P);
				aB += iB * b2Cross(rB, P);
			}
		}

		bodyA->position = cA;
		bodyA->angle = aA;
		bodyB->position = cB;
		bodyB->angle = aB;
	}

	// We can't expect minSpeparation >= -b2_linearSlop because we don't
	// push the separation above -b2_linearSlop.
	return minSeparation >= -3.0f * b2_linearSlop;
}
