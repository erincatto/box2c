// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "box2d/id.h"
#include "box2d/types.h"

#include "pool.h"

#include <stdint.h>

typedef struct b2SolverData b2SolverData;
typedef struct b2World b2World;

typedef enum b2JointType
{
	b2_unknownJoint,
	b2_revoluteJoint,
	b2_prismaticJoint,
	b2_distanceJoint,
	b2_pulleyJoint,
	b2_mouseJoint,
	b2_gearJoint,
	b2_wheelJoint,
    b2_weldJoint,
	b2_frictionJoint,
	b2_motorJoint
} b2JointType;

//struct b2Jacobian
//{
//	b2Vec2 linear;
//	float angularA;
//	float angularB;
//};

/// A joint edge is used to connect bodies and joints together
/// in a joint graph where each body is a node and each joint
/// is an edge. A joint edge belongs to a doubly linked list
/// maintained in each attached body. Each joint has two joint
/// nodes, one for each attached body.
typedef struct b2JointEdge
{
	int32_t bodyIndex;
	int32_t nextJointIndex;
} b2JointEdge;

/// Utility to compute linear stiffness values from frequency and damping ratio
//void b2LinearStiffness(float& stiffness, float& damping,
//	float frequencyHertz, float dampingRatio,
//	const b2Body* bodyA, const b2Body* bodyB);
//
///// Utility to compute rotational stiffness values frequency and damping ratio
//void b2AngularStiffness(float& stiffness, float& damping,
//	float frequencyHertz, float dampingRatio,
//	const b2Body* bodyA, const b2Body* bodyB);

typedef struct b2MouseJoint
{
	b2Vec2 localAnchorB;
	b2Vec2 targetA;
	float stiffness;
	float damping;
	float beta;

	// Solver shared
	b2Vec2 impulse;
	float maxForce;
	float gamma;

	// Solver temp
	int32_t indexA;
	int32_t indexB;
	b2Vec2 rB;
	b2Vec2 localCenterB;
	float invMassB;
	float invIB;
	b2Mat22 mass;
	b2Vec2 C;
} b2MouseJoint;

/// The base joint class. Joints are used to constraint two bodies together in
/// various fashions. Some joints also feature limits and motors.
typedef struct b2Joint
{
	b2Object object;

	b2JointType type;

	b2JointEdge edgeA;
	b2JointEdge edgeB;

	uint64_t islandId;

	union
	{
		b2MouseJoint mouseJoint;
	};

	bool collideConnected;
} b2Joint;

void b2InitVelocityConstraints(b2World* world, b2Joint* joint, b2SolverData* data);
void b2SolveVelocityConstraints(b2Joint* joint, b2SolverData* data);

// This returns true if the position errors are within tolerance.
bool b2SolvePositionConstraints(b2Joint* joint, b2SolverData* data);
