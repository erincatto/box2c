// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "contact_solver.h"

#include "array.h"
#include "body.h"
#include "contact.h"
#include "core.h"
#include "graph.h"
#include "world.h"

#include <immintrin.h>

// Soft constraints with constraint error substepping. Includes a bias removal stage to help remove excess energy.
// http://mmacklin.com/smallsteps.pdf
// https://box2d.org/files/ErinCatto_SoftConstraints_GDC2011.pdf

void b2PrepareOverflowContacts(b2SolverTaskContext* context)
{
	b2TracyCZoneNC(prepare_contact, "Prepare Contact", b2_colorYellow, true);

	b2World* world = context->world;
	b2Graph* graph = context->graph;
	b2Contact* contacts = world->contacts;
	const int32_t* bodyMap = context->bodyToSolverMap;
	b2SolverBody* solverBodies = context->solverBodies;

	b2ContactConstraint* constraints = graph->overflow.contactConstraints;
	int32_t* contactIndices = graph->overflow.contactArray;
	int32_t contactCount = b2Array(graph->overflow.contactArray).count;

	// This is a dummy body to represent a static body because static bodies don't have a solver body.
	b2SolverBody dummyBody = {0};

	// 30 is a bit soft, 60 oscillates too much
	// const float contactHertz = 45.0f;
	// const float contactHertz = B2_MAX(15.0f, stepContext->inv_dt * stepContext->velocityIterations / 8.0f);
	const float contactHertz = world->contactHertz;

	float h = context->timeStep;
	bool enableWarmStarting = world->enableWarmStarting;

	for (int32_t i = 0; i < contactCount; ++i)
	{
		b2Contact* contact = contacts + contactIndices[i];

		const b2Manifold* manifold = &contact->manifold;
		int32_t pointCount = manifold->pointCount;

		B2_ASSERT(0 < pointCount && pointCount <= 2);

		int32_t indexA = bodyMap[contact->edges[0].bodyIndex];
		int32_t indexB = bodyMap[contact->edges[1].bodyIndex];

		b2ContactConstraint* constraint = constraints + i;
		constraint->contact = contact;
		constraint->indexA = indexA;
		constraint->indexB = indexB;
		constraint->normal = manifold->normal;
		constraint->friction = contact->friction;
		constraint->restitution = contact->restitution;
		constraint->pointCount = pointCount;

		b2SolverBody* solverBodyA = indexA == B2_NULL_INDEX ? &dummyBody : solverBodies + indexA;
		b2SolverBody* solverBodyB = indexB == B2_NULL_INDEX ? &dummyBody : solverBodies + indexB;

		float hertz = (indexA == B2_NULL_INDEX || indexB == B2_NULL_INDEX) ? 2.0f * contactHertz : contactHertz;
		b2Vec2 vA = solverBodyA->linearVelocity;
		float wA = solverBodyA->angularVelocity;
		float mA = solverBodyA->invMass;
		float iA = solverBodyA->invI;

		b2Vec2 vB = solverBodyB->linearVelocity;
		float wB = solverBodyB->angularVelocity;
		float mB = solverBodyB->invMass;
		float iB = solverBodyB->invI;

		// Stiffer for static contacts to avoid bodies getting pushed through the ground
		const float zeta = 1.0f;
		float omega = 2.0f * b2_pi * hertz;
		float c = h * omega * (2.0f * zeta + h * omega);
		constraint->impulseCoefficient = 1.0f / (1.0f + c);
		constraint->massCoefficient = c * constraint->impulseCoefficient;
		constraint->biasCoefficient = omega / (2.0f * zeta + h * omega);

		b2Vec2 normal = constraint->normal;
		b2Vec2 tangent = b2RightPerp(constraint->normal);

		for (int32_t j = 0; j < pointCount; ++j)
		{
			const b2ManifoldPoint* mp = manifold->points + j;
			b2ContactConstraintPoint* cp = constraint->points + j;

			cp->normalImpulse = mp->normalImpulse;
			cp->tangentImpulse = mp->tangentImpulse;

			cp->rA = mp->anchorA;
			cp->rB = mp->anchorB;

			float rnA = b2Cross(cp->rA, normal);
			float rnB = b2Cross(cp->rB, normal);
			float kNormal = mA + mB + iA * rnA * rnA + iB * rnB * rnB;

			float rtA = b2Cross(cp->rA, tangent);
			float rtB = b2Cross(cp->rB, tangent);
			float kTangent = mA + mB + iA * rtA * rtA + iB * rtB * rtB;

			cp->tangentMass = kTangent > 0.0f ? 1.0f / kTangent : 0.0f;
			cp->separation = mp->separation;
			cp->normalMass = kNormal > 0.0f ? 1.0f / kNormal : 0.0f;

			// Save relative velocity for restitution
			b2Vec2 vrA = b2Add(vA, b2CrossSV(wA, cp->rA));
			b2Vec2 vrB = b2Add(vB, b2CrossSV(wB, cp->rB));
			cp->relativeVelocity = b2Dot(normal, b2Sub(vrB, vrA));

			// Warm start
			if (enableWarmStarting)
			{
				b2Vec2 P = b2Add(b2MulSV(cp->normalImpulse, normal), b2MulSV(cp->tangentImpulse, tangent));
				wA -= iA * b2Cross(cp->rA, P);
				vA = b2MulAdd(vA, -mA, P);
				wB += iB * b2Cross(cp->rB, P);
				vB = b2MulAdd(vB, mB, P);
			}
		}

		solverBodyA->linearVelocity = vA;
		solverBodyA->angularVelocity = wA;
		solverBodyB->linearVelocity = vB;
		solverBodyB->angularVelocity = wB;
	}

	b2TracyCZoneEnd(prepare_contact);
}

void b2SolveOverflowContacts(b2SolverTaskContext* context, bool useBias)
{
	b2TracyCZoneNC(solve_contact, "Solve Contact", b2_colorAliceBlue, true);

	b2SolverBody* bodies = context->solverBodies;
	b2ContactConstraint* constraints = context->graph->overflow.contactConstraints;
	int32_t count = b2Array(context->graph->overflow.contactArray).count;
	float inv_dt = context->invTimeStep;
	const float pushout = context->world->maximumPushoutVelocity;

	// This is a dummy body to represent a static body since static bodies don't have a solver body.
	b2SolverBody dummyBody = {0};

	for (int32_t i = 0; i < count; ++i)
	{
		b2ContactConstraint* constraint = constraints + i;

		b2SolverBody* bodyA = constraint->indexA == B2_NULL_INDEX ? &dummyBody : bodies + constraint->indexA;
		b2Vec2 vA = bodyA->linearVelocity;
		float wA = bodyA->angularVelocity;
		b2Vec2 dpA = bodyA->deltaPosition;
		float daA = bodyA->deltaAngle;
		float mA = bodyA->invMass;
		float iA = bodyA->invI;

		b2SolverBody* bodyB = constraint->indexB == B2_NULL_INDEX ? &dummyBody : bodies + constraint->indexB;
		b2Vec2 vB = bodyB->linearVelocity;
		float wB = bodyB->angularVelocity;
		b2Vec2 dpB = bodyB->deltaPosition;
		float daB = bodyB->deltaAngle;
		float mB = bodyB->invMass;
		float iB = bodyB->invI;

		b2Vec2 normal = constraint->normal;
		b2Vec2 tangent = b2RightPerp(normal);
		float friction = constraint->friction;
		float biasCoefficient = constraint->biasCoefficient;
		float massCoefficient = constraint->massCoefficient;
		float impulseCoefficient = constraint->impulseCoefficient;

		int32_t pointCount = constraint->pointCount;

		for (int32_t j = 0; j < pointCount; ++j)
		{
			b2ContactConstraintPoint* cp = constraint->points + j;

			// Relative velocity at contact
			b2Vec2 vrB = b2Add(vB, b2CrossSV(wB, cp->rB));
			b2Vec2 vrA = b2Add(vA, b2CrossSV(wA, cp->rA));
			b2Vec2 dv = b2Sub(vrB, vrA);

			// Compute change in separation (small angle approximation of sin(angle) == angle)
			b2Vec2 prB = b2Add(dpB, b2CrossSV(daB, cp->rB));
			b2Vec2 prA = b2Add(dpA, b2CrossSV(daA, cp->rA));
			float ds = b2Dot(b2Sub(prB, prA), normal);
			float s = cp->separation + ds;
			float bias = 0.0f;
			float massScale = 1.0f;
			float impulseScale = 0.0f;
			if (s > 0.0f)
			{
				// TODO_ERIN what time to use?
				// Speculative (inverse of full time step)
				bias = s * inv_dt;
			}
			else if (useBias)
			{
				bias = B2_MAX(biasCoefficient * s, -pushout);
				// bias = cp->biasCoefficient * s;
				massScale = massCoefficient;
				impulseScale = impulseCoefficient;
			}

			// Compute normal impulse
			float vn = b2Dot(dv, normal);
			float impulse = -cp->normalMass * massScale * (vn + bias) - impulseScale * cp->normalImpulse;
			// float impulse = -cp->normalMass * (vn + bias + cp->gamma * cp->normalImpulse);

			// Clamp the accumulated impulse
			float newImpulse = B2_MAX(cp->normalImpulse + impulse, 0.0f);
			impulse = newImpulse - cp->normalImpulse;
			cp->normalImpulse = newImpulse;

			// Apply contact impulse
			b2Vec2 P = b2MulSV(impulse, normal);
			vA = b2MulSub(vA, mA, P);
			wA -= iA * b2Cross(cp->rA, P);

			vB = b2MulAdd(vB, mB, P);
			wB += iB * b2Cross(cp->rB, P);
		}

		for (int32_t j = 0; j < pointCount; ++j)
		{
			b2ContactConstraintPoint* cp = constraint->points + j;

			// Relative velocity at contact
			b2Vec2 vrB = b2Add(vB, b2CrossSV(wB, cp->rB));
			b2Vec2 vrA = b2Add(vA, b2CrossSV(wA, cp->rA));
			b2Vec2 dv = b2Sub(vrB, vrA);

			// Compute tangent force
			float vt = b2Dot(dv, tangent);
			float lambda = cp->tangentMass * (-vt);

			// Clamp the accumulated force
			float maxFriction = friction * cp->normalImpulse;
			float newImpulse = B2_CLAMP(cp->tangentImpulse + lambda, -maxFriction, maxFriction);
			lambda = newImpulse - cp->tangentImpulse;
			cp->tangentImpulse = newImpulse;

			// Apply contact impulse
			b2Vec2 P = b2MulSV(lambda, tangent);

			vA = b2MulSub(vA, mA, P);
			wA -= iA * b2Cross(cp->rA, P);

			vB = b2MulAdd(vB, mB, P);
			wB += iB * b2Cross(cp->rB, P);
		}

		bodyA->linearVelocity = vA;
		bodyA->angularVelocity = wA;
		bodyB->linearVelocity = vB;
		bodyB->angularVelocity = wB;
	}

	b2TracyCZoneEnd(solve_contact);
}

void b2ApplyOverflowRestitution(b2SolverTaskContext* context)
{
	b2TracyCZoneNC(overflow_resitution, "Overflow Restitution", b2_colorViolet, true);

	b2SolverBody* bodies = context->solverBodies;
	b2ContactConstraint* constraints = context->graph->overflow.contactConstraints;
	int32_t count = b2Array(context->graph->overflow.contactArray).count;
	float threshold = context->world->restitutionThreshold;

	// This is a dummy body to represent a static body since static bodies don't have a solver body.
	b2SolverBody dummyBody = {0};

	for (int32_t i = 0; i < count; ++i)
	{
		b2ContactConstraint* constraint = constraints + i;

		float restitution = constraint->restitution;
		if (restitution == 0.0f)
		{
			continue;
		}

		b2SolverBody* bodyA = constraint->indexA == B2_NULL_INDEX ? &dummyBody : bodies + constraint->indexA;
		b2Vec2 vA = bodyA->linearVelocity;
		float wA = bodyA->angularVelocity;
		float mA = bodyA->invMass;
		float iA = bodyA->invI;

		b2SolverBody* bodyB = constraint->indexB == B2_NULL_INDEX ? &dummyBody : bodies + constraint->indexB;
		b2Vec2 vB = bodyB->linearVelocity;
		float wB = bodyB->angularVelocity;
		float mB = bodyB->invMass;
		float iB = bodyB->invI;

		b2Vec2 normal = constraint->normal;
		int32_t pointCount = constraint->pointCount;

		for (int32_t j = 0; j < pointCount; ++j)
		{
			b2ContactConstraintPoint* cp = constraint->points + j;

			// if the normal impulse is zero then there was no collision
			if (cp->relativeVelocity > -threshold || cp->normalImpulse == 0.0f)
			{
				continue;
			}

			// Relative velocity at contact
			b2Vec2 vrB = b2Add(vB, b2CrossSV(wB, cp->rB));
			b2Vec2 vrA = b2Add(vA, b2CrossSV(wA, cp->rA));
			b2Vec2 dv = b2Sub(vrB, vrA);

			// Compute normal impulse
			float vn = b2Dot(dv, normal);
			float impulse = -cp->normalMass * (vn + restitution * cp->relativeVelocity);

			// Clamp the accumulated impulse
			float newImpulse = B2_MAX(cp->normalImpulse + impulse, 0.0f);
			impulse = newImpulse - cp->normalImpulse;
			cp->normalImpulse = newImpulse;

			// Apply contact impulse
			b2Vec2 P = b2MulSV(impulse, normal);
			vA = b2MulSub(vA, mA, P);
			wA -= iA * b2Cross(cp->rA, P);

			vB = b2MulAdd(vB, mB, P);
			wB += iB * b2Cross(cp->rB, P);
		}

		bodyA->linearVelocity = vA;
		bodyA->angularVelocity = wA;
		bodyB->linearVelocity = vB;
		bodyB->angularVelocity = wB;
	}

	b2TracyCZoneEnd(overflow_resitution);
}

void b2StoreOverflowImpulses(b2SolverTaskContext* context)
{
	b2TracyCZoneNC(store_impulses, "Store", b2_colorFirebrick, true);

	b2ContactConstraint* constraints = context->graph->overflow.contactConstraints;
	int32_t count = b2Array(context->graph->overflow.contactArray).count;

	for (int32_t i = 0; i < count; ++i)
	{
		b2ContactConstraint* constraint = constraints + i;
		b2Contact* contact = constraint->contact;
		b2Manifold* manifold = &contact->manifold;
		int32_t pointCount = manifold->pointCount;

		for (int32_t j = 0; j < pointCount; ++j)
		{
			manifold->points[j].normalImpulse = constraint->points[j].normalImpulse;
			manifold->points[j].tangentImpulse = constraint->points[j].tangentImpulse;
		}
	}

	b2TracyCZoneEnd(store_impulses);
}

// SIMD WIP
#define add(a, b) _mm256_add_ps((a), (b))
#define sub(a, b) _mm256_sub_ps((a), (b))
#define mul(a, b) _mm256_mul_ps((a), (b))
#define muladd(a, b, c) _mm256_add_ps((a), _mm256_mul_ps((b), (c)))
#define mulsub(a, b, c) _mm256_sub_ps((a), _mm256_mul_ps((b), (c)))

static inline __m256 b2CrossW(b2Vec2W a, b2Vec2W b)
{
	return sub(mul(a.X, b.Y), mul(a.Y, b.X));
}

typedef struct b2SimdBody
{
	b2Vec2W v;
	__m256 w;
	b2Vec2W dp;
	__m256 da;
	__m256 invM, invI;
} b2SimdBody;

// This is a load and 8x8 transpose
static b2SimdBody b2GatherBodies(const b2SolverBody* restrict bodies, int32_t* restrict indices)
{
	_Static_assert(sizeof(b2SolverBody) == 32, "b2SolverBody not 32 bytes");
	B2_ASSERT(((uintptr_t)bodies & 0x1F) == 0);
	__m256 zero = _mm256_setzero_ps();
	__m256 b0 = indices[0] == B2_NULL_INDEX ? zero : _mm256_load_ps((float*)(bodies + indices[0]));
	__m256 b1 = indices[1] == B2_NULL_INDEX ? zero : _mm256_load_ps((float*)(bodies + indices[1]));
	__m256 b2 = indices[2] == B2_NULL_INDEX ? zero : _mm256_load_ps((float*)(bodies + indices[2]));
	__m256 b3 = indices[3] == B2_NULL_INDEX ? zero : _mm256_load_ps((float*)(bodies + indices[3]));
	__m256 b4 = indices[4] == B2_NULL_INDEX ? zero : _mm256_load_ps((float*)(bodies + indices[4]));
	__m256 b5 = indices[5] == B2_NULL_INDEX ? zero : _mm256_load_ps((float*)(bodies + indices[5]));
	__m256 b6 = indices[6] == B2_NULL_INDEX ? zero : _mm256_load_ps((float*)(bodies + indices[6]));
	__m256 b7 = indices[7] == B2_NULL_INDEX ? zero : _mm256_load_ps((float*)(bodies + indices[7]));

	__m256 t0 = _mm256_unpacklo_ps(b0, b1);
	__m256 t1 = _mm256_unpackhi_ps(b0, b1);
	__m256 t2 = _mm256_unpacklo_ps(b2, b3);
	__m256 t3 = _mm256_unpackhi_ps(b2, b3);
	__m256 t4 = _mm256_unpacklo_ps(b4, b5);
	__m256 t5 = _mm256_unpackhi_ps(b4, b5);
	__m256 t6 = _mm256_unpacklo_ps(b6, b7);
	__m256 t7 = _mm256_unpackhi_ps(b6, b7);
	__m256 tt0 = _mm256_shuffle_ps(t0, t2, _MM_SHUFFLE(1, 0, 1, 0));
	__m256 tt1 = _mm256_shuffle_ps(t0, t2, _MM_SHUFFLE(3, 2, 3, 2));
	__m256 tt2 = _mm256_shuffle_ps(t1, t3, _MM_SHUFFLE(1, 0, 1, 0));
	__m256 tt3 = _mm256_shuffle_ps(t1, t3, _MM_SHUFFLE(3, 2, 3, 2));
	__m256 tt4 = _mm256_shuffle_ps(t4, t6, _MM_SHUFFLE(1, 0, 1, 0));
	__m256 tt5 = _mm256_shuffle_ps(t4, t6, _MM_SHUFFLE(3, 2, 3, 2));
	__m256 tt6 = _mm256_shuffle_ps(t5, t7, _MM_SHUFFLE(1, 0, 1, 0));
	__m256 tt7 = _mm256_shuffle_ps(t5, t7, _MM_SHUFFLE(3, 2, 3, 2));

	b2SimdBody simdBody;
	simdBody.v.X = _mm256_permute2f128_ps(tt0, tt4, 0x20);
	simdBody.v.Y = _mm256_permute2f128_ps(tt1, tt5, 0x20);
	simdBody.w = _mm256_permute2f128_ps(tt2, tt6, 0x20);
	simdBody.dp.X = _mm256_permute2f128_ps(tt3, tt7, 0x20);
	simdBody.dp.Y = _mm256_permute2f128_ps(tt0, tt4, 0x31);
	simdBody.da = _mm256_permute2f128_ps(tt1, tt5, 0x31);
	simdBody.invM = _mm256_permute2f128_ps(tt2, tt6, 0x31);
	simdBody.invI = _mm256_permute2f128_ps(tt3, tt7, 0x31);

	return simdBody;
}

// This writes everything back to the solver bodies but only the velocities change
static void b2ScatterBodies(b2SolverBody* restrict bodies, int32_t* restrict indices, const b2SimdBody* restrict simdBody)
{
	_Static_assert(sizeof(b2SolverBody) == 32, "b2SolverBody not 32 bytes");
	B2_ASSERT(((uintptr_t)bodies & 0x1F) == 0);
	__m256 t0 = _mm256_unpacklo_ps(simdBody->v.X, simdBody->v.Y);
	__m256 t1 = _mm256_unpackhi_ps(simdBody->v.X, simdBody->v.Y);
	__m256 t2 = _mm256_unpacklo_ps(simdBody->w, simdBody->dp.X);
	__m256 t3 = _mm256_unpackhi_ps(simdBody->w, simdBody->dp.X);
	__m256 t4 = _mm256_unpacklo_ps(simdBody->dp.Y, simdBody->da);
	__m256 t5 = _mm256_unpackhi_ps(simdBody->dp.Y, simdBody->da);
	__m256 t6 = _mm256_unpacklo_ps(simdBody->invM, simdBody->invI);
	__m256 t7 = _mm256_unpackhi_ps(simdBody->invM, simdBody->invI);
	__m256 tt0 = _mm256_shuffle_ps(t0, t2, _MM_SHUFFLE(1, 0, 1, 0));
	__m256 tt1 = _mm256_shuffle_ps(t0, t2, _MM_SHUFFLE(3, 2, 3, 2));
	__m256 tt2 = _mm256_shuffle_ps(t1, t3, _MM_SHUFFLE(1, 0, 1, 0));
	__m256 tt3 = _mm256_shuffle_ps(t1, t3, _MM_SHUFFLE(3, 2, 3, 2));
	__m256 tt4 = _mm256_shuffle_ps(t4, t6, _MM_SHUFFLE(1, 0, 1, 0));
	__m256 tt5 = _mm256_shuffle_ps(t4, t6, _MM_SHUFFLE(3, 2, 3, 2));
	__m256 tt6 = _mm256_shuffle_ps(t5, t7, _MM_SHUFFLE(1, 0, 1, 0));
	__m256 tt7 = _mm256_shuffle_ps(t5, t7, _MM_SHUFFLE(3, 2, 3, 2));

	// I don't use any dummy body in the body array because this will lead to multithreaded sharing and the
	// associated cache flushing.
	if (indices[0] != B2_NULL_INDEX)
		_mm256_store_ps((float*)(bodies + indices[0]), _mm256_permute2f128_ps(tt0, tt4, 0x20));
	if (indices[1] != B2_NULL_INDEX)
		_mm256_store_ps((float*)(bodies + indices[1]), _mm256_permute2f128_ps(tt1, tt5, 0x20));
	if (indices[2] != B2_NULL_INDEX)
		_mm256_store_ps((float*)(bodies + indices[2]), _mm256_permute2f128_ps(tt2, tt6, 0x20));
	if (indices[3] != B2_NULL_INDEX)
		_mm256_store_ps((float*)(bodies + indices[3]), _mm256_permute2f128_ps(tt3, tt7, 0x20));
	if (indices[4] != B2_NULL_INDEX)
		_mm256_store_ps((float*)(bodies + indices[4]), _mm256_permute2f128_ps(tt0, tt4, 0x31));
	if (indices[5] != B2_NULL_INDEX)
		_mm256_store_ps((float*)(bodies + indices[5]), _mm256_permute2f128_ps(tt1, tt5, 0x31));
	if (indices[6] != B2_NULL_INDEX)
		_mm256_store_ps((float*)(bodies + indices[6]), _mm256_permute2f128_ps(tt2, tt6, 0x31));
	if (indices[7] != B2_NULL_INDEX)
		_mm256_store_ps((float*)(bodies + indices[7]), _mm256_permute2f128_ps(tt3, tt7, 0x31));
}

void b2PrepareContactsSIMD(int32_t startIndex, int32_t endIndex, b2SolverTaskContext* context)
{
	b2TracyCZoneNC(prepare_contact, "Prepare Contact", b2_colorYellow, true);

	b2World* world = context->world;
	b2Contact* contacts = world->contacts;
	const int32_t* bodyMap = context->bodyToSolverMap;
	b2SolverBody* solverBodies = context->solverBodies;
	b2ContactConstraintSIMD* constraints = context->constraintAVXs;
	const int32_t* contactIndices = context->contactIndices;
	
	// This is a dummy body to represent a static body since static bodies don't have a solver body.
	b2SolverBody dummyBody = {0};

	// 30 is a bit soft, 60 oscillates too much
	// const float contactHertz = 45.0f;
	// const float contactHertz = B2_MAX(15.0f, stepContext->inv_dt * stepContext->velocityIterations / 8.0f);
	const float contactHertz = world->contactHertz;

	float h = context->timeStep;

	for (int32_t i = startIndex; i < endIndex; ++i)
	{
		b2ContactConstraintSIMD* constraint = constraints + i;

		for (int32_t j = 0; j < 8; ++j)
		{
			int32_t contactIndex = contactIndices[8 * i + j];

			if (contactIndex != B2_NULL_INDEX)
			{
				b2Contact* contact = contacts + contactIndex;

				const b2Manifold* manifold = &contact->manifold;
				int32_t indexA = bodyMap[contact->edges[0].bodyIndex];
				int32_t indexB = bodyMap[contact->edges[1].bodyIndex];

				constraint->indexA[j] = indexA;
				constraint->indexB[j] = indexB;

				b2SolverBody* solverBodyA = indexA == B2_NULL_INDEX ? &dummyBody : solverBodies + indexA;
				b2SolverBody* solverBodyB = indexB == B2_NULL_INDEX ? &dummyBody : solverBodies + indexB;
				float mA = solverBodyA->invMass;
				float iA = solverBodyA->invI;
				float mB = solverBodyB->invMass;
				float iB = solverBodyB->invI;

				float hertz = (indexA == B2_NULL_INDEX || indexB == B2_NULL_INDEX) ? 2.0f * contactHertz : contactHertz;

				// Stiffer for static contacts to avoid bodies getting pushed through the ground
				const float zeta = 1.0f;
				float omega = 2.0f * b2_pi * hertz;
				float d = (2.0f * zeta + h * omega);
				float c = h * omega * d;
				float impulseCoefficient = 1.0f / (1.0f + c);

				((float*)&constraint->friction)[j] = contact->friction;
				((float*)&constraint->restitution)[j] = contact->restitution;
				((float*)&constraint->impulseCoefficient)[j] = impulseCoefficient;
				((float*)&constraint->massCoefficient)[j] = c * impulseCoefficient;
				((float*)&constraint->biasCoefficient)[j] = omega / d;

				b2Vec2 normal = manifold->normal;
				((float*)&constraint->normal.X)[j] = normal.x;
				((float*)&constraint->normal.Y)[j] = normal.y;

				b2Vec2 tangent = b2RightPerp(normal);

				{
					const b2ManifoldPoint* mp = manifold->points + 0;
					((float*)&constraint->separation1)[j] = mp->separation;
					((float*)&constraint->normalImpulse1)[j] = mp->normalImpulse;
					((float*)&constraint->tangentImpulse1)[j] = mp->tangentImpulse;

					((float*)&constraint->rA1.X)[j] = mp->anchorA.x;
					((float*)&constraint->rA1.Y)[j] = mp->anchorA.y;
					((float*)&constraint->rB1.X)[j] = mp->anchorB.x;
					((float*)&constraint->rB1.Y)[j] = mp->anchorB.y;

					float rnA = b2Cross(mp->anchorA, normal);
					float rnB = b2Cross(mp->anchorB, normal);
					float kNormal = mA + mB + iA * rnA * rnA + iB * rnB * rnB;
					((float*)&constraint->normalMass1)[j] = kNormal > 0.0f ? 1.0f / kNormal : 0.0f;

					float rtA = b2Cross(mp->anchorA, tangent);
					float rtB = b2Cross(mp->anchorB, tangent);
					float kTangent = mA + mB + iA * rtA * rtA + iB * rtB * rtB;
					((float*)&constraint->tangentMass1)[j] = kTangent > 0.0f ? 1.0f / kTangent : 0.0f;

					// Save relative velocity for restitution
					b2Vec2 vrA = b2Add(solverBodyA->linearVelocity, b2CrossSV(solverBodyA->angularVelocity, mp->anchorA));
					b2Vec2 vrB = b2Add(solverBodyB->linearVelocity, b2CrossSV(solverBodyB->angularVelocity, mp->anchorB));
					((float*)&constraint->relativeVelocity1)[j] = b2Dot(normal, b2Sub(vrB, vrA));
				}

				int32_t pointCount = manifold->pointCount;
				B2_ASSERT(0 < pointCount && pointCount <= 2);

				if (pointCount == 2)
				{
					const b2ManifoldPoint* mp = manifold->points + 1;
					((float*)&constraint->separation2)[j] = mp->separation;
					((float*)&constraint->normalImpulse2)[j] = mp->normalImpulse;
					((float*)&constraint->tangentImpulse2)[j] = mp->tangentImpulse;

					((float*)&constraint->rA2.X)[j] = mp->anchorA.x;
					((float*)&constraint->rA2.Y)[j] = mp->anchorA.y;
					((float*)&constraint->rB2.X)[j] = mp->anchorB.x;
					((float*)&constraint->rB2.Y)[j] = mp->anchorB.y;

					float rnA = b2Cross(mp->anchorA, normal);
					float rnB = b2Cross(mp->anchorB, normal);
					float kNormal = mA + mB + iA * rnA * rnA + iB * rnB * rnB;
					((float*)&constraint->normalMass2)[j] = kNormal > 0.0f ? 1.0f / kNormal : 0.0f;

					float rtA = b2Cross(mp->anchorA, tangent);
					float rtB = b2Cross(mp->anchorB, tangent);
					float kTangent = mA + mB + iA * rtA * rtA + iB * rtB * rtB;
					((float*)&constraint->tangentMass2)[j] = kTangent > 0.0f ? 1.0f / kTangent : 0.0f;

					// Save relative velocity for restitution
					b2Vec2 vrA = b2Add(solverBodyA->linearVelocity, b2CrossSV(solverBodyA->angularVelocity, mp->anchorA));
					b2Vec2 vrB = b2Add(solverBodyB->linearVelocity, b2CrossSV(solverBodyB->angularVelocity, mp->anchorB));
					((float*)&constraint->relativeVelocity2)[j] = b2Dot(normal, b2Sub(vrB, vrA));
				}
				else
				{
					// dummy data that has no effect
					((float*)&constraint->separation2)[j] = 0.0f;
					((float*)&constraint->normalImpulse2)[j] = 0.0f;
					((float*)&constraint->tangentImpulse2)[j] = 0.0f;
					((float*)&constraint->rA2.X)[j] = 0.0f;
					((float*)&constraint->rA2.Y)[j] = 0.0f;
					((float*)&constraint->rB2.X)[j] = 0.0f;
					((float*)&constraint->rB2.Y)[j] = 0.0f;
					((float*)&constraint->normalMass2)[j] = 0.0f;
					((float*)&constraint->tangentMass2)[j] = 0.0f;
					((float*)&constraint->relativeVelocity2)[j] = 0.0f;
				}
			}
			else
			{
				// remainder
				constraint->indexA[j] = B2_NULL_INDEX;
				constraint->indexB[j] = B2_NULL_INDEX;
				((float*)&constraint->friction)[j] = 0.0f;
				((float*)&constraint->restitution)[j] = 0.0f;
				((float*)&constraint->impulseCoefficient)[j] = 0.0f;
				((float*)&constraint->massCoefficient)[j] = 0.0f;
				((float*)&constraint->biasCoefficient)[j] = 0.0f;
				((float*)&constraint->normal.X)[j] = 0.0f;
				((float*)&constraint->normal.Y)[j] = 0.0f;

				((float*)&constraint->separation1)[j] = 0.0f;
				((float*)&constraint->normalImpulse1)[j] = 0.0f;
				((float*)&constraint->tangentImpulse1)[j] = 0.0f;
				((float*)&constraint->rA1.X)[j] = 0.0f;
				((float*)&constraint->rA1.Y)[j] = 0.0f;
				((float*)&constraint->rB1.X)[j] = 0.0f;
				((float*)&constraint->rB1.Y)[j] = 0.0f;
				((float*)&constraint->normalMass1)[j] = 0.0f;
				((float*)&constraint->tangentMass1)[j] = 0.0f;
				((float*)&constraint->relativeVelocity1)[j] = 0.0f;

				((float*)&constraint->separation2)[j] = 0.0f;
				((float*)&constraint->normalImpulse2)[j] = 0.0f;
				((float*)&constraint->tangentImpulse2)[j] = 0.0f;
				((float*)&constraint->rA2.X)[j] = 0.0f;
				((float*)&constraint->rA2.Y)[j] = 0.0f;
				((float*)&constraint->rB2.X)[j] = 0.0f;
				((float*)&constraint->rB2.Y)[j] = 0.0f;
				((float*)&constraint->normalMass2)[j] = 0.0f;
				((float*)&constraint->tangentMass2)[j] = 0.0f;
				((float*)&constraint->relativeVelocity2)[j] = 0.0f;
			}
		}
	}

	b2TracyCZoneEnd(prepare_contact);
}

void b2WarmStartContactsSIMD(int32_t startIndex, int32_t endIndex, b2SolverTaskContext* context, int32_t colorIndex)
{
	b2TracyCZoneNC(warm_start_contact, "Warm Start", b2_colorGreen1, true);

	b2SolverBody* bodies = context->solverBodies;
	b2ContactConstraintSIMD* constraints = context->graph->colors[colorIndex].contactConstraints;

	for (int32_t i = startIndex; i < endIndex; ++i)
	{
		b2ContactConstraintSIMD* c = constraints + i;
		b2SimdBody bA = b2GatherBodies(bodies, c->indexA);
		b2SimdBody bB = b2GatherBodies(bodies, c->indexB);

		__m256 tangentX = c->normal.Y;
		__m256 tangentY = sub(_mm256_setzero_ps(), c->normal.X);

		{
			b2Vec2W P;
			P.X = add(mul(c->normalImpulse1, c->normal.X), mul(c->tangentImpulse1, tangentX));
			P.Y = add(mul(c->normalImpulse1, c->normal.Y), mul(c->tangentImpulse1, tangentY));
			bA.w = mulsub(bA.w, bA.invI, b2CrossW(c->rA1, P));
			bA.v.X = mulsub(bA.v.X, bA.invM, P.X);
			bA.v.Y = mulsub(bA.v.Y, bA.invM, P.Y);
			bB.w = muladd(bB.w, bB.invI, b2CrossW(c->rB1, P));
			bB.v.X = muladd(bB.v.X, bB.invM, P.X);
			bB.v.Y = muladd(bB.v.Y, bB.invM, P.Y);
		}

		{
			b2Vec2W P;
			P.X = add(mul(c->normalImpulse2, c->normal.X), mul(c->tangentImpulse2, tangentX));
			P.Y = add(mul(c->normalImpulse2, c->normal.Y), mul(c->tangentImpulse2, tangentY));
			bA.w = mulsub(bA.w, bA.invI, b2CrossW(c->rA2, P));
			bA.v.X = mulsub(bA.v.X, bA.invM, P.X);
			bA.v.Y = mulsub(bA.v.Y, bA.invM, P.Y);
			bB.w = muladd(bB.w, bB.invI, b2CrossW(c->rB2, P));
			bB.v.X = muladd(bB.v.X, bB.invM, P.X);
			bB.v.Y = muladd(bB.v.Y, bB.invM, P.Y);
		}

		b2ScatterBodies(bodies, c->indexA, &bA);
		b2ScatterBodies(bodies, c->indexB, &bB);
	}

	b2TracyCZoneEnd(warm_start_contact);
}

void b2SolveContactsSIMD(int32_t startIndex, int32_t endIndex, b2SolverTaskContext* context, int32_t colorIndex, bool useBias)
{
	b2TracyCZoneNC(solve_contact, "Solve Contact", b2_colorAliceBlue, true);

	b2SolverBody* bodies = context->solverBodies;
	b2ContactConstraintSIMD* constraints = context->graph->colors[colorIndex].contactConstraints;
	float inv_dt = context->invTimeStep;
	const float pushout = context->world->maximumPushoutVelocity;

	for (int32_t i = startIndex; i < endIndex; ++i)
	{
		b2ContactConstraintSIMD* c = constraints + i;

		b2SimdBody bA = b2GatherBodies(bodies, c->indexA);
		b2SimdBody bB = b2GatherBodies(bodies, c->indexB);

		__m256 biasCoeff, massCoeff, impulseCoeff;
		if (useBias)
		{
			biasCoeff = c->biasCoefficient;
			massCoeff = c->massCoefficient;
			impulseCoeff = c->impulseCoefficient;
		}
		else
		{
			biasCoeff = _mm256_setzero_ps();
			massCoeff = _mm256_set1_ps(1.0f);
			impulseCoeff = _mm256_setzero_ps();
		}

		__m256 invDtMul = _mm256_set1_ps(inv_dt);
		__m256 minBiasVel = _mm256_set1_ps(-pushout);

		// first point non-penetration constraint
		{
			// Compute change in separation (small angle approximation of sin(angle) == angle)
			__m256 prx = sub(sub(bB.dp.X, mul(bB.da, c->rB1.Y)), sub(bA.dp.X, mul(bA.da, c->rA1.Y)));
			__m256 pry = sub(add(bB.dp.Y, mul(bB.da, c->rB1.X)), add(bA.dp.Y, mul(bA.da, c->rA1.X)));
			__m256 ds = add(mul(prx, c->normal.X), mul(pry, c->normal.Y));

			__m256 s = add(c->separation1, ds);

			__m256 test = _mm256_cmp_ps(s, _mm256_setzero_ps(), _CMP_GT_OQ);
			__m256 specBias = mul(s, invDtMul);
			__m256 softBias = _mm256_max_ps(mul(biasCoeff, s), minBiasVel);
			__m256 bias = _mm256_blendv_ps(softBias, specBias, test);

			// Relative velocity at contact
			__m256 dvx = sub(sub(bB.v.X, mul(bB.w, c->rB1.Y)), sub(bA.v.X, mul(bA.w, c->rA1.Y)));
			__m256 dvy = sub(add(bB.v.Y, mul(bB.w, c->rB1.X)), add(bA.v.Y, mul(bA.w, c->rA1.X)));
			__m256 vn = add(mul(dvx, c->normal.X), mul(dvy, c->normal.Y));

			// Compute normal impulse
			__m256 negImpulse = add(mul(c->normalMass1, mul(massCoeff, add(vn, bias))), mul(impulseCoeff, c->normalImpulse1));

			// Clamp the accumulated impulse
			__m256 newImpulse = _mm256_max_ps(sub(c->normalImpulse1, negImpulse), _mm256_setzero_ps());
			__m256 impulse = sub(newImpulse, c->normalImpulse1);
			c->normalImpulse1 = newImpulse;

			// Apply contact impulse
			__m256 Px = mul(impulse, c->normal.X);
			__m256 Py = mul(impulse, c->normal.Y);

			bA.v.X = sub(bA.v.X, mul(bA.invM, Px));
			bA.v.Y = sub(bA.v.Y, mul(bA.invM, Py));
			bA.w = sub(bA.w, mul(bA.invI, sub(mul(c->rA1.X, Py), mul(c->rA1.Y, Px))));

			bB.v.X = add(bB.v.X, mul(bB.invM, Px));
			bB.v.Y = add(bB.v.Y, mul(bB.invM, Py));
			bB.w = add(bB.w, mul(bB.invI, sub(mul(c->rB1.X, Py), mul(c->rB1.Y, Px))));
		}

		// second point non-penetration constraint
		{
			// Compute change in separation (small angle approximation of sin(angle) == angle)
			__m256 prx = sub(sub(bB.dp.X, mul(bB.da, c->rB2.Y)), sub(bA.dp.X, mul(bA.da, c->rA2.Y)));
			__m256 pry = sub(add(bB.dp.Y, mul(bB.da, c->rB2.X)), add(bA.dp.Y, mul(bA.da, c->rA2.X)));
			__m256 ds = add(mul(prx, c->normal.X), mul(pry, c->normal.Y));

			__m256 s = add(c->separation2, ds);

			__m256 test = _mm256_cmp_ps(s, _mm256_setzero_ps(), _CMP_GT_OQ);
			__m256 specBias = mul(s, invDtMul);
			__m256 softBias = _mm256_max_ps(mul(biasCoeff, s), minBiasVel);
			__m256 bias = _mm256_blendv_ps(softBias, specBias, test);

			// Relative velocity at contact
			__m256 dvx = sub(sub(bB.v.X, mul(bB.w, c->rB2.Y)), sub(bA.v.X, mul(bA.w, c->rA2.Y)));
			__m256 dvy = sub(add(bB.v.Y, mul(bB.w, c->rB2.X)), add(bA.v.Y, mul(bA.w, c->rA2.X)));
			__m256 vn = add(mul(dvx, c->normal.X), mul(dvy, c->normal.Y));

			// Compute normal impulse
			__m256 negImpulse = add(mul(c->normalMass2, mul(massCoeff, add(vn, bias))), mul(impulseCoeff, c->normalImpulse2));

			// Clamp the accumulated impulse
			__m256 newImpulse = _mm256_max_ps(sub(c->normalImpulse2, negImpulse), _mm256_setzero_ps());
			__m256 impulse = sub(newImpulse, c->normalImpulse2);
			c->normalImpulse2 = newImpulse;

			// Apply contact impulse
			__m256 Px = mul(impulse, c->normal.X);
			__m256 Py = mul(impulse, c->normal.Y);

			bA.v.X = sub(bA.v.X, mul(bA.invM, Px));
			bA.v.Y = sub(bA.v.Y, mul(bA.invM, Py));
			bA.w = sub(bA.w, mul(bA.invI, sub(mul(c->rA2.X, Py), mul(c->rA2.Y, Px))));

			bB.v.X = add(bB.v.X, mul(bB.invM, Px));
			bB.v.Y = add(bB.v.Y, mul(bB.invM, Py));
			bB.w = add(bB.w, mul(bB.invI, sub(mul(c->rB2.X, Py), mul(c->rB2.Y, Px))));
		}

		__m256 tangentX = c->normal.Y;
		__m256 tangentY = sub(_mm256_setzero_ps(), c->normal.X);
		// float friction = constraint->friction;

		// first point friction constraint
		{
			// Relative velocity at contact
			__m256 dvx = sub(sub(bB.v.X, mul(bB.w, c->rB1.Y)), sub(bA.v.X, mul(bA.w, c->rA1.Y)));
			__m256 dvy = sub(add(bB.v.Y, mul(bB.w, c->rB1.X)), add(bA.v.Y, mul(bA.w, c->rA1.X)));
			__m256 vt = add(mul(dvx, tangentX), mul(dvy, tangentY));

			// Compute tangent force
			__m256 negImpulse = mul(c->tangentMass1, vt);

			// Clamp the accumulated force
			__m256 maxFriction = mul(c->friction, c->normalImpulse1);
			__m256 newImpulse = sub(c->tangentImpulse1, negImpulse);
			newImpulse = _mm256_max_ps(sub(_mm256_setzero_ps(), maxFriction), _mm256_min_ps(newImpulse, maxFriction));
			__m256 impulse = sub(newImpulse, c->tangentImpulse1);
			c->tangentImpulse1 = newImpulse;

			// Apply contact impulse
			__m256 Px = mul(impulse, tangentX);
			__m256 Py = mul(impulse, tangentY);

			bA.v.X = sub(bA.v.X, mul(bA.invM, Px));
			bA.v.Y = sub(bA.v.Y, mul(bA.invM, Py));
			bA.w = sub(bA.w, mul(bA.invI, sub(mul(c->rA1.X, Py), mul(c->rA1.Y, Px))));

			bB.v.X = add(bB.v.X, mul(bB.invM, Px));
			bB.v.Y = add(bB.v.Y, mul(bB.invM, Py));
			bB.w = add(bB.w, mul(bB.invI, sub(mul(c->rB1.X, Py), mul(c->rB1.Y, Px))));
		}

		// second point friction constraint
		{
			// Relative velocity at contact
			__m256 dvx = sub(sub(bB.v.X, mul(bB.w, c->rB2.Y)), sub(bA.v.X, mul(bA.w, c->rA2.Y)));
			__m256 dvy = sub(add(bB.v.Y, mul(bB.w, c->rB2.X)), add(bA.v.Y, mul(bA.w, c->rA2.X)));
			__m256 vt = add(mul(dvx, tangentX), mul(dvy, tangentY));

			// Compute tangent force
			__m256 negImpulse = mul(c->tangentMass2, vt);

			// Clamp the accumulated force
			__m256 maxFriction = mul(c->friction, c->normalImpulse2);
			__m256 newImpulse = sub(c->tangentImpulse2, negImpulse);
			newImpulse = _mm256_max_ps(sub(_mm256_setzero_ps(), maxFriction), _mm256_min_ps(newImpulse, maxFriction));
			__m256 impulse = sub(newImpulse, c->tangentImpulse2);
			c->tangentImpulse2 = newImpulse;

			// Apply contact impulse
			__m256 Px = mul(impulse, tangentX);
			__m256 Py = mul(impulse, tangentY);

			bA.v.X = sub(bA.v.X, mul(bA.invM, Px));
			bA.v.Y = sub(bA.v.Y, mul(bA.invM, Py));
			bA.w = sub(bA.w, mul(bA.invI, sub(mul(c->rA2.X, Py), mul(c->rA2.Y, Px))));

			bB.v.X = add(bB.v.X, mul(bB.invM, Px));
			bB.v.Y = add(bB.v.Y, mul(bB.invM, Py));
			bB.w = add(bB.w, mul(bB.invI, sub(mul(c->rB2.X, Py), mul(c->rB2.Y, Px))));
		}

		b2ScatterBodies(bodies, c->indexA, &bA);
		b2ScatterBodies(bodies, c->indexB, &bB);
	}

	b2TracyCZoneEnd(solve_contact);
}

void b2ApplyRestitutionSIMD(int32_t startIndex, int32_t endIndex, b2SolverTaskContext* context, int32_t colorIndex)
{
	b2TracyCZoneNC(restitution, "Restitution", b2_colorDodgerBlue, true);

	b2SolverBody* bodies = context->solverBodies;
	b2ContactConstraintSIMD* constraints = context->graph->colors[colorIndex].contactConstraints;
	b2FloatW threshold = _mm256_set1_ps(context->world->restitutionThreshold);
	b2FloatW zero = _mm256_setzero_ps();

	for (int32_t i = startIndex; i < endIndex; ++i)
	{
		b2ContactConstraintSIMD* c = constraints + i;

		b2SimdBody bA = b2GatherBodies(bodies, c->indexA);
		b2SimdBody bB = b2GatherBodies(bodies, c->indexB);

		// first point non-penetration constraint
		{
			// Set effective mass to zero if restitution should not be applied
			__m256 test1 = _mm256_cmp_ps(add(c->relativeVelocity1, threshold), zero, _CMP_GT_OQ);
			__m256 test2 = _mm256_cmp_ps(c->normalImpulse1, zero, _CMP_EQ_OQ);
			__m256 test = _mm256_or_ps(test1, test2);
			__m256 mass = _mm256_blendv_ps(c->normalMass1, zero, test);

			// Relative velocity at contact
			__m256 dvx = sub(sub(bB.v.X, mul(bB.w, c->rB1.Y)), sub(bA.v.X, mul(bA.w, c->rA1.Y)));
			__m256 dvy = sub(add(bB.v.Y, mul(bB.w, c->rB1.X)), add(bA.v.Y, mul(bA.w, c->rA1.X)));
			__m256 vn = add(mul(dvx, c->normal.X), mul(dvy, c->normal.Y));

			// Compute normal impulse
			__m256 negImpulse = mul(mass, add(vn, mul(c->restitution, c->relativeVelocity1)));

			// Clamp the accumulated impulse
			__m256 newImpulse = _mm256_max_ps(sub(c->normalImpulse1, negImpulse), _mm256_setzero_ps());
			__m256 impulse = sub(newImpulse, c->normalImpulse1);
			c->normalImpulse1 = newImpulse;

			// Apply contact impulse
			__m256 Px = mul(impulse, c->normal.X);
			__m256 Py = mul(impulse, c->normal.Y);

			bA.v.X = sub(bA.v.X, mul(bA.invM, Px));
			bA.v.Y = sub(bA.v.Y, mul(bA.invM, Py));
			bA.w = sub(bA.w, mul(bA.invI, sub(mul(c->rA1.X, Py), mul(c->rA1.Y, Px))));

			bB.v.X = add(bB.v.X, mul(bB.invM, Px));
			bB.v.Y = add(bB.v.Y, mul(bB.invM, Py));
			bB.w = add(bB.w, mul(bB.invI, sub(mul(c->rB1.X, Py), mul(c->rB1.Y, Px))));
		}

		// second point non-penetration constraint
		{
			// Set effective mass to zero if restitution should not be applied
			__m256 test1 = _mm256_cmp_ps(add(c->relativeVelocity2, threshold), zero, _CMP_GT_OQ);
			__m256 test2 = _mm256_cmp_ps(c->normalImpulse2, zero, _CMP_EQ_OQ);
			__m256 test = _mm256_or_ps(test1, test2);
			__m256 mass = _mm256_blendv_ps(zero, c->normalMass2, test);

			// Relative velocity at contact
			__m256 dvx = sub(sub(bB.v.X, mul(bB.w, c->rB2.Y)), sub(bA.v.X, mul(bA.w, c->rA2.Y)));
			__m256 dvy = sub(add(bB.v.Y, mul(bB.w, c->rB2.X)), add(bA.v.Y, mul(bA.w, c->rA2.X)));
			__m256 vn = add(mul(dvx, c->normal.X), mul(dvy, c->normal.Y));

			// Compute normal impulse
			__m256 negImpulse = mul(mass, add(vn, mul(c->restitution, c->relativeVelocity2)));

			// Clamp the accumulated impulse
			__m256 newImpulse = _mm256_max_ps(sub(c->normalImpulse2, negImpulse), _mm256_setzero_ps());
			__m256 impulse = sub(newImpulse, c->normalImpulse2);
			c->normalImpulse2 = newImpulse;

			// Apply contact impulse
			__m256 Px = mul(impulse, c->normal.X);
			__m256 Py = mul(impulse, c->normal.Y);

			bA.v.X = sub(bA.v.X, mul(bA.invM, Px));
			bA.v.Y = sub(bA.v.Y, mul(bA.invM, Py));
			bA.w = sub(bA.w, mul(bA.invI, sub(mul(c->rA2.X, Py), mul(c->rA2.Y, Px))));

			bB.v.X = add(bB.v.X, mul(bB.invM, Px));
			bB.v.Y = add(bB.v.Y, mul(bB.invM, Py));
			bB.w = add(bB.w, mul(bB.invI, sub(mul(c->rB2.X, Py), mul(c->rB2.Y, Px))));
		}

		b2ScatterBodies(bodies, c->indexA, &bA);
		b2ScatterBodies(bodies, c->indexB, &bB);
	}

	b2TracyCZoneEnd(restitution);
}

void b2StoreImpulsesSIMD(int32_t startIndex, int32_t endIndex, b2SolverTaskContext* context)
{
	b2TracyCZoneNC(store_impulses, "Store", b2_colorFirebrick, true);

	b2Contact* contacts = context->world->contacts;
	const b2ContactConstraintSIMD* constraints = context->constraintAVXs;
	const int32_t* indices = context->contactIndices;

	b2Manifold dummy = {0};

	for (int32_t i = startIndex; i < endIndex; ++i)
	{
		const b2ContactConstraintSIMD* c = constraints + i;
		const float* normalImpulse1 = (float*)&c->normalImpulse1;
		const float* normalImpulse2 = (float*)&c->normalImpulse2;
		const float* tangentImpulse1 = (float*)&c->tangentImpulse1;
		const float* tangentImpulse2 = (float*)&c->tangentImpulse2;

		const int32_t* base = indices + 8 * i;
		int32_t index0 = base[0];
		int32_t index1 = base[1];
		int32_t index2 = base[2];
		int32_t index3 = base[3];
		int32_t index4 = base[4];
		int32_t index5 = base[5];
		int32_t index6 = base[6];
		int32_t index7 = base[7];

		b2Manifold* m0 = index0 == B2_NULL_INDEX ? &dummy : &contacts[index0].manifold;
		b2Manifold* m1 = index1 == B2_NULL_INDEX ? &dummy : &contacts[index1].manifold;
		b2Manifold* m2 = index2 == B2_NULL_INDEX ? &dummy : &contacts[index2].manifold;
		b2Manifold* m3 = index3 == B2_NULL_INDEX ? &dummy : &contacts[index3].manifold;
		b2Manifold* m4 = index4 == B2_NULL_INDEX ? &dummy : &contacts[index4].manifold;
		b2Manifold* m5 = index5 == B2_NULL_INDEX ? &dummy : &contacts[index5].manifold;
		b2Manifold* m6 = index6 == B2_NULL_INDEX ? &dummy : &contacts[index6].manifold;
		b2Manifold* m7 = index7 == B2_NULL_INDEX ? &dummy : &contacts[index7].manifold;

		m0->points[0].normalImpulse = normalImpulse1[0];
		m0->points[0].tangentImpulse = tangentImpulse1[0];
		m0->points[1].normalImpulse = normalImpulse2[0];
		m0->points[1].tangentImpulse = tangentImpulse2[0];

		m1->points[0].normalImpulse = normalImpulse1[1];
		m1->points[0].tangentImpulse = tangentImpulse1[1];
		m1->points[1].normalImpulse = normalImpulse2[1];
		m1->points[1].tangentImpulse = tangentImpulse2[1];

		m2->points[0].normalImpulse = normalImpulse1[2];
		m2->points[0].tangentImpulse = tangentImpulse1[2];
		m2->points[1].normalImpulse = normalImpulse2[2];
		m2->points[1].tangentImpulse = tangentImpulse2[2];

		m3->points[0].normalImpulse = normalImpulse1[3];
		m3->points[0].tangentImpulse = tangentImpulse1[3];
		m3->points[1].normalImpulse = normalImpulse2[3];
		m3->points[1].tangentImpulse = tangentImpulse2[3];

		m4->points[0].normalImpulse = normalImpulse1[4];
		m4->points[0].tangentImpulse = tangentImpulse1[4];
		m4->points[1].normalImpulse = normalImpulse2[4];
		m4->points[1].tangentImpulse = tangentImpulse2[4];

		m5->points[0].normalImpulse = normalImpulse1[5];
		m5->points[0].tangentImpulse = tangentImpulse1[5];
		m5->points[1].normalImpulse = normalImpulse2[5];
		m5->points[1].tangentImpulse = tangentImpulse2[5];

		m6->points[0].normalImpulse = normalImpulse1[6];
		m6->points[0].tangentImpulse = tangentImpulse1[6];
		m6->points[1].normalImpulse = normalImpulse2[6];
		m6->points[1].tangentImpulse = tangentImpulse2[6];

		m7->points[0].normalImpulse = normalImpulse1[7];
		m7->points[0].tangentImpulse = tangentImpulse1[7];
		m7->points[1].normalImpulse = normalImpulse2[7];
		m7->points[1].tangentImpulse = tangentImpulse2[7];
	}

	b2TracyCZoneEnd(store_impulses);
}
