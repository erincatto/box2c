// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "block_array.h"

#include "box2d/constants.h"

#include <stdatomic.h>
#include <stdbool.h>
#include <stdint.h>

typedef struct b2Body b2Body;
typedef struct b2BodyState b2BodyState;
typedef struct b2Contact b2Contact;
typedef struct b2Joint b2Joint;
typedef struct b2World b2World;

// This holds solver set data. The following sets are used:
// - static set for all static bodies (no contacts or joints)
// - active set for all active bodies with body states (no contacts or joints)
// - disabled set for disabled bodies and their joints
// - all further sets are sleeping island sets along with their contacts and joints
typedef struct b2SolverSet
{
	b2BodyArray bodies;

	// Body state only exists for active set
	b2BodyState* states;

	int bodyCount;
	int bodyCapacity;

	// This holds sleeping/disabled joints. Empty for static/active set.
	b2JointArray joints;

	// This holds sleeping contacts for sleeping sets.
	// For the awake set this holds the non-touching awake contacts.
	// For the
	b2ContactArray contacts;
} b2SolverSet;

typedef struct b2Softness
{
	float biasRate;
	float massScale;
	float impulseScale;
} b2Softness;

typedef enum b2SolverStageType
{
	b2_stagePrepareJoints,
	b2_stagePrepareContacts,
	b2_stageIntegrateVelocities,
	b2_stageWarmStart,
	b2_stageSolve,
	b2_stageIntegratePositions,
	b2_stageRelax,
	b2_stageRestitution,
	b2_stageStoreImpulses
} b2SolverStageType;

typedef enum b2SolverBlockType
{
	b2_bodyBlock,
	b2_jointBlock,
	b2_contactBlock,
	b2_graphJointBlock,
	b2_graphContactBlock
} b2SolverBlockType;

// Each block of work has a sync index that gets incremented when a worker claims the block. This ensures only a single worker
// claims a block, yet lets work be distributed dynamically across multiple workers (work stealing). This also reduces contention
// on a single block index atomic. For non-iterative stages the sync index is simply set to one. For iterative stages (solver
// iteration) the same block of work is executed once per iteration and the atomic sync index is shared across iterations, so it
// increases monotonically.
typedef struct b2SolverBlock
{
	int32_t startIndex;
	int16_t count;
	int16_t blockType; // b2SolverBlockType
	_Atomic int syncIndex;
} b2SolverBlock;

// Each stage must be completed before going to the next stage.
// Non-iterative stages use a stage instance once while iterative stages re-use the same instance each iteration.
typedef struct b2SolverStage
{
	b2SolverStageType type;
	b2SolverBlock* blocks;
	int32_t blockCount;
	int32_t colorIndex;
	_Atomic int completionCount;
} b2SolverStage;

typedef struct b2ContactSubset
{
	b2Contact* contacts;
	int count;
	int colorIndex;
} b2ContactSubset;

typedef struct b2JointSubset
{
	b2Joint* joints;
	int count;
	int colorIndex;
} b2JointSubset;

// Context for a time step. Recreated each time step.
typedef struct b2StepContext
{
	// time step
	float dt;

	// inverse time step (0 if dt == 0).
	float inv_dt;

	// sub-step
	float h;
	float inv_h;

	int32_t subStepCount;

	b2Softness jointSoftness;
	b2Softness contactSoftness;
	b2Softness staticSoftness;

	float restitutionThreshold;
	float maxBiasVelocity;

	b2BodyState* bodyStates;

	struct b2World* world;
	struct b2ConstraintGraph* graph;

	// Transient array of contact arrays organized for easier parallel-for.
	// Pointers copied from graph colors and non-touching contacts.
	b2ContactSubset contactSubsets[b2_graphColorCount + 2];
	int contactSubsetCount;

	// Transient array of joint arrays organize for easier parallel-for.
	// Pointers copied from graph colors.
	b2JointSubset jointSubsets[b2_graphColorCount + 1];
	int jointSubsetCount;

	int32_t* jointIndices;
	int32_t* contactIndices;

	struct b2ContactConstraintSIMD* contactConstraints;
	int32_t activeColorCount;
	int32_t workerCount;

	b2SolverStage* stages;
	int32_t stageCount;

	// sync index (16-bits) | stage type (16-bits)
	_Atomic unsigned int syncBits;

	bool enableWarmStarting;
} b2StepContext;

static inline b2Softness b2MakeSoft(float hertz, float zeta, float h)
{
	if (hertz == 0.0f)
	{
		return (b2Softness){0.0f, 1.0f, 0.0f};
	}

	float omega = 2.0f * b2_pi * hertz;
	float a1 = 2.0f * zeta + h * omega;
	float a2 = h * omega * a1;
	float a3 = 1.0f / (1.0f + a2);
	return (b2Softness){omega / a1, a2 * a3, a3};
}

void b2Solve(b2World* world, b2StepContext* stepContext);
