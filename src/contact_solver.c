// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "contact_solver.h"
#include "body.h"
#include "contact.h"
#include "shape.h"
#include "world.h"

#include <assert.h>

// Solver debugging is normally disabled because the block solver sometimes has to deal with a poorly conditioned
// effective mass matrix.
#define B2_DEBUG_SOLVER 0

bool g_blockSolve = true;

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
	b2VelocityConstraintPoint points[2];
	b2Vec2 normal;
	b2Mat22 normalMass;
	b2Mat22 K;
	int32_t indexA;
	int32_t indexB;
	float invMassA, invMassB;
	float invIA, invIB;
	float friction;
	float restitution;
	float tangentSpeed;
	int32_t pointCount;
	int32_t contactIndex;
} b2ContactVelocityConstraint;

typedef struct b2ContactPositionConstraint
{
	b2Vec2 localPoints[b2_maxManifoldPoints];
	b2Vec2 localNormal;
	b2Vec2 localPoint;
	int32_t indexA;
	int32_t indexB;
	float invMassA, invMassB;
	b2Vec2 localCenterA, localCenterB;
	float invIA, invIB;
	b2ManifoldType type;
	float radiusA, radiusB;
	int32_t pointCount;
} b2ContactPositionConstraint;

b2ContactSolver b2CreateContactSolver(b2ContactSolverDef* def)
{
	b2StackAllocator* alloc = &def->world->stackAllocator;

	b2ContactSolver solver;
	solver.step = def->step;
	solver.world = def->world;
	solver.count = def->count;
	solver.positionConstraints =
		(b2ContactPositionConstraint*)b2AllocateStackItem(alloc, solver.count * sizeof(b2ContactPositionConstraint));
	solver.velocityConstraints =
		(b2ContactVelocityConstraint*)b2AllocateStackItem(alloc, solver.count * sizeof(b2ContactVelocityConstraint));
	solver.positions = def->positions;
	solver.velocities = def->velocities;
	solver.contacts = def->contacts;

	// Initialize position independent portions of the constraints.
	for (int32_t i = 0; i < solver.count; ++i)
	{
		b2Contact* contact = solver.contacts[i];

		b2Shape* shapeA = solver.world->shapes + contact->shapeIndexA;
		b2Shape* shapeB = solver.world->shapes + contact->shapeIndexB;
		b2Body* bodyA = solver.world->bodies + shapeA->bodyIndex;
		b2Body* bodyB = solver.world->bodies + shapeB->bodyIndex;
		b2Manifold* manifold = &contact->manifold;

		int32_t pointCount = manifold->pointCount;
		assert(pointCount > 0);

		b2ContactVelocityConstraint* vc = solver.velocityConstraints + i;
		vc->friction = contact->friction;
		vc->restitution = contact->restitution;
		vc->tangentSpeed = contact->tangentSpeed;
		vc->indexA = bodyA->islandIndex;
		vc->indexB = bodyB->islandIndex;
		vc->invMassA = bodyA->invMass;
		vc->invMassB = bodyB->invMass;
		vc->invIA = bodyA->invI;
		vc->invIB = bodyB->invI;
		vc->contactIndex = i;
		vc->pointCount = pointCount;
		vc->K = b2Mat22_zero;
		vc->normalMass = b2Mat22_zero;

		b2ContactPositionConstraint* pc = solver.positionConstraints + i;
		pc->indexA = bodyA->islandIndex;
		pc->indexB = bodyB->islandIndex;
		pc->invMassA = bodyA->invMass;
		pc->invMassB = bodyB->invMass;
		pc->localCenterA = bodyA->localCenter;
		pc->localCenterB = bodyB->localCenter;
		pc->invIA = bodyA->invI;
		pc->invIB = bodyB->invI;
		pc->localNormal = manifold->localNormal;
		pc->localPoint = manifold->localPoint;
		pc->pointCount = pointCount;
		pc->radiusA = b2Shape_GetRadius(shapeA);
		pc->radiusB = b2Shape_GetRadius(shapeB);
		pc->type = manifold->type;

		for (int32_t j = 0; j < pointCount; ++j)
		{
			b2ManifoldPoint* cp = manifold->points + j;
			b2VelocityConstraintPoint* vcp = vc->points + j;

			if (solver.step.warmStarting)
			{
				vcp->normalImpulse = solver.step.dtRatio * cp->normalImpulse;
				vcp->tangentImpulse = solver.step.dtRatio * cp->tangentImpulse;
			}
			else
			{
				vcp->normalImpulse = 0.0f;
				vcp->tangentImpulse = 0.0f;
			}

			vcp->rA = b2Vec2_zero;
			vcp->rB = b2Vec2_zero;
			vcp->normalMass = 0.0f;
			vcp->tangentMass = 0.0f;
			vcp->velocityBias = 0.0f;

			pc->localPoints[j] = cp->localPoint;
		}
	}

	return solver;
}

void b2DestroyContactSolver(b2ContactSolver* solver)
{
	b2StackAllocator* alloc = &solver->world->stackAllocator;
	b2FreeStackItem(alloc, solver->velocityConstraints);
	b2FreeStackItem(alloc, solver->positionConstraints);
	solver->velocityConstraints = NULL;
	solver->positionConstraints = NULL;
}

// Initialize position dependent portions of the velocity constraints.
void b2ContactSolver_InitializeVelocityConstraints(b2ContactSolver* solver)
{
	int32_t count = solver->count;
	for (int32_t i = 0; i < count; ++i)
	{
		b2ContactVelocityConstraint* vc = solver->velocityConstraints + i;
		b2ContactPositionConstraint* pc = solver->positionConstraints + i;

		float radiusA = pc->radiusA;
		float radiusB = pc->radiusB;
		b2Manifold* manifold = &solver->contacts[vc->contactIndex]->manifold;

		int32_t indexA = vc->indexA;
		int32_t indexB = vc->indexB;

		float mA = vc->invMassA;
		float mB = vc->invMassB;
		float iA = vc->invIA;
		float iB = vc->invIB;
		b2Vec2 localCenterA = pc->localCenterA;
		b2Vec2 localCenterB = pc->localCenterB;

		b2Vec2 cA = solver->positions[indexA].c;
		float aA = solver->positions[indexA].a;
		b2Vec2 vA = solver->velocities[indexA].v;
		float wA = solver->velocities[indexA].w;

		b2Vec2 cB = solver->positions[indexB].c;
		float aB = solver->positions[indexB].a;
		b2Vec2 vB = solver->velocities[indexB].v;
		float wB = solver->velocities[indexB].w;

		assert(manifold->pointCount > 0);

		b2Transform xfA, xfB;
		xfA.q = b2MakeRot(aA);
		xfB.q = b2MakeRot(aB);
		xfA.p = b2Sub(cA, b2RotateVector(xfA.q, localCenterA));
		xfB.p = b2Sub(cB, b2RotateVector(xfB.q, localCenterB));

		b2WorldManifold worldManifold = b2ComputeWorldManifold(manifold, xfA, radiusA, xfB, radiusB);
		vc->normal = worldManifold.normal;

		int32_t pointCount = vc->pointCount;
		for (int32_t j = 0; j < pointCount; ++j)
		{
			b2VelocityConstraintPoint* vcp = vc->points + j;

			vcp->rA = b2Sub(worldManifold.points[j], cA);
			vcp->rB = b2Sub(worldManifold.points[j], cB);

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
			vcp->velocityBias = -B2_MAX(0.0f, worldManifold.separations[j] * solver->step.inv_dt);

			// Relative velocity
			b2Vec2 vrB = b2Add(vB, b2CrossSV(wB, vcp->rB));
			b2Vec2 vrA = b2Add(vA, b2CrossSV(wA, vcp->rA));
			vcp->relativeVelocity = b2Dot(vc->normal, b2Sub(vrB, vrA));
		}

		// If we have two points, then prepare the block solver.
		if (vc->pointCount == 2 && g_blockSolve)
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
				vc->K.cx = (b2Vec2){k11, k12};
				vc->K.cy = (b2Vec2){k12, k22};
				vc->normalMass = b2GetInverse22(vc->K);
			}
			else
			{
				// The constraints are redundant, just use one.
				// TODO_ERIN use deepest?
				vc->pointCount = 1;
			}
		}
	}
}

void b2ContactSolver_WarmStart(b2ContactSolver* solver)
{
	int32_t count = solver->count;

	// Warm start.
	for (int32_t i = 0; i < count; ++i)
	{
		b2ContactVelocityConstraint* vc = solver->velocityConstraints + i;

		int32_t indexA = vc->indexA;
		int32_t indexB = vc->indexB;
		float mA = vc->invMassA;
		float iA = vc->invIA;
		float mB = vc->invMassB;
		float iB = vc->invIB;
		int32_t pointCount = vc->pointCount;

		b2Vec2 vA = solver->velocities[indexA].v;
		float wA = solver->velocities[indexA].w;
		b2Vec2 vB = solver->velocities[indexB].v;
		float wB = solver->velocities[indexB].w;

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

		solver->velocities[indexA].v = vA;
		solver->velocities[indexA].w = wA;
		solver->velocities[indexB].v = vB;
		solver->velocities[indexB].w = wB;
	}
}

void b2ContactSolver_SolveVelocityConstraints(b2ContactSolver* solver)
{
	int32_t count = solver->count;

	for (int32_t i = 0; i < count; ++i)
	{
		b2ContactVelocityConstraint* vc = solver->velocityConstraints + i;

		int32_t indexA = vc->indexA;
		int32_t indexB = vc->indexB;
		float mA = vc->invMassA;
		float iA = vc->invIA;
		float mB = vc->invMassB;
		float iB = vc->invIB;
		int32_t pointCount = vc->pointCount;

		b2Vec2 vA = solver->velocities[indexA].v;
		float wA = solver->velocities[indexA].w;
		b2Vec2 vB = solver->velocities[indexB].v;
		float wB = solver->velocities[indexB].w;

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

			// b2Clamp the accumulated force
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
		if (pointCount == 1 || g_blockSolve == false)
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

				// b2Clamp the accumulated impulse
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

		solver->velocities[indexA].v = vA;
		solver->velocities[indexA].w = wA;
		solver->velocities[indexB].v = vB;
		solver->velocities[indexB].w = wB;
	}
}

void b2ContactSolver_ApplyRestitution(b2ContactSolver* solver)
{
	int32_t count = solver->count;
	float threshold = solver->world->restitutionThreshold;

	for (int32_t i = 0; i < count; ++i)
	{
		b2ContactVelocityConstraint* vc = solver->velocityConstraints + i;

		if (vc->restitution == 0.0f)
		{
			continue;
		}

		int32_t indexA = vc->indexA;
		int32_t indexB = vc->indexB;
		float mA = vc->invMassA;
		float iA = vc->invIA;
		float mB = vc->invMassB;
		float iB = vc->invIB;
		int32_t pointCount = vc->pointCount;

		b2Vec2 vA = solver->velocities[indexA].v;
		float wA = solver->velocities[indexA].w;
		b2Vec2 vB = solver->velocities[indexB].v;
		float wB = solver->velocities[indexB].w;

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

		solver->velocities[indexA].v = vA;
		solver->velocities[indexA].w = wA;
		solver->velocities[indexB].v = vB;
		solver->velocities[indexB].w = wB;
	}
}

void b2ContactSolver_StoreImpulses(b2ContactSolver* solver)
{
	int32_t count = solver->count;

	for (int32_t i = 0; i < count; ++i)
	{
		b2ContactVelocityConstraint* vc = solver->velocityConstraints + i;
		b2Manifold* manifold = &solver->contacts[vc->contactIndex]->manifold;

		for (int32_t j = 0; j < vc->pointCount; ++j)
		{
			manifold->points[j].normalImpulse = vc->points[j].normalImpulse;
			manifold->points[j].tangentImpulse = vc->points[j].tangentImpulse;
		}
	}
}

typedef struct b2PositionSolverManifold
{
	b2Vec2 normal;
	b2Vec2 point;
	float separation;
} b2PositionSolverManifold;

b2PositionSolverManifold b2CreatePositionSolverManifold(b2ContactPositionConstraint* pc, b2Transform xfA,
														b2Transform xfB, int32_t index)
{
	assert(pc->pointCount > 0);

	b2PositionSolverManifold manifold = {0};

	switch (pc->type)
	{
		case b2_manifoldCircles:
		{
			b2Vec2 pointA = b2TransformPoint(xfA, pc->localPoint);
			b2Vec2 pointB = b2TransformPoint(xfB, pc->localPoints[0]);
			manifold.normal = b2Normalize(b2Sub(pointB, pointA));
			manifold.point = b2Lerp(pointA, pointB, 0.5f);
			manifold.separation = b2Dot(b2Sub(pointB, pointA), manifold.normal) - pc->radiusA - pc->radiusB;
		}
		break;

		case b2_manifoldFaceA:
		{
			manifold.normal = b2RotateVector(xfA.q, pc->localNormal);
			b2Vec2 planePoint = b2TransformPoint(xfA, pc->localPoint);

			b2Vec2 clipPoint = b2TransformPoint(xfB, pc->localPoints[index]);
			manifold.separation = b2Dot(b2Sub(clipPoint, planePoint), manifold.normal) - pc->radiusA - pc->radiusB;
			manifold.point = clipPoint;
		}
		break;

		case b2_manifoldFaceB:
		{
			manifold.normal = b2RotateVector(xfB.q, pc->localNormal);
			b2Vec2 planePoint = b2TransformPoint(xfB, pc->localPoint);

			b2Vec2 clipPoint = b2TransformPoint(xfA, pc->localPoints[index]);
			manifold.separation = b2Dot(b2Sub(clipPoint, planePoint), manifold.normal) - pc->radiusA - pc->radiusB;
			manifold.point = clipPoint;

			// Ensure normal points from A to B
			manifold.normal = b2Neg(manifold.normal);
		}
		break;

		default:
			assert(false);
			break;
	}

	return manifold;
}

// Sequential solver.
bool b2ContactSolver_SolvePositionConstraints(b2ContactSolver* solver)
{
	float minSeparation = 0.0f;
	int32_t count = solver->count;

	for (int32_t i = 0; i < count; ++i)
	{
		b2ContactPositionConstraint* pc = solver->positionConstraints + i;

		int32_t indexA = pc->indexA;
		int32_t indexB = pc->indexB;
		b2Vec2 localCenterA = pc->localCenterA;
		float mA = pc->invMassA;
		float iA = pc->invIA;
		b2Vec2 localCenterB = pc->localCenterB;
		float mB = pc->invMassB;
		float iB = pc->invIB;
		int32_t pointCount = pc->pointCount;

		b2Vec2 cA = solver->positions[indexA].c;
		float aA = solver->positions[indexA].a;

		b2Vec2 cB = solver->positions[indexB].c;
		float aB = solver->positions[indexB].a;

		// Solve normal constraints
		for (int32_t j = 0; j < pointCount; ++j)
		{
			b2Transform xfA, xfB;
			xfA.q = b2MakeRot(aA);
			xfB.q = b2MakeRot(aB);
			xfA.p = b2Sub(cA, b2RotateVector(xfA.q, localCenterA));
			xfB.p = b2Sub(cB, b2RotateVector(xfB.q, localCenterB));

			b2PositionSolverManifold psm = b2CreatePositionSolverManifold(pc, xfA, xfB, j);
			b2Vec2 normal = psm.normal;

			b2Vec2 point = psm.point;
			float separation = B2_MIN(0.0f, psm.separation);

			b2Vec2 rA = b2Sub(point, cA);
			b2Vec2 rB = b2Sub(point, cB);

			// Track max constraint error.
			minSeparation = B2_MIN(minSeparation, separation);

			// Prevent large corrections
			float C = B2_CLAMP(b2_baumgarte * separation, -b2_maxLinearCorrection, 0.0f);

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

		solver->positions[indexA].c = cA;
		solver->positions[indexA].a = aA;
		solver->positions[indexB].c = cB;
		solver->positions[indexB].a = aB;
	}

	// We can't expect minSpeparation >= -b2_linearSlop because we don't
	// push the separation above -b2_linearSlop.
	return minSeparation >= -3.0f * b2_linearSlop;
}

b2ContactImpulse b2ContactSolver_GetImpulse(b2ContactSolver* solver, int32_t index)
{
	const b2ContactVelocityConstraint* vc = solver->velocityConstraints + index;

	b2ContactImpulse impulse;
	impulse.count = vc->pointCount;
	for (int32_t i = 0; i < vc->pointCount; ++i)
	{
		impulse.normalImpulses[i] = vc->points[i].normalImpulse;
		impulse.tangentImpulses[i] = vc->points[i].tangentImpulse;
	}
	return impulse;
}
