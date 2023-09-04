// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "box2d/id.h"
#include "box2d/types.h"

#include "pool.h"

#include <stdint.h>

typedef struct b2DebugDraw b2DebugDraw;
typedef struct b2StepContext b2StepContext;
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

/// A joint edge is used to connect bodies and joints together
/// in a joint graph where each body is a node and each joint
/// is an edge. A joint edge belongs to a doubly linked list
/// maintained in each attached body. Each joint has two joint
/// nodes, one for each attached body.
typedef struct b2JointEdge
{
	int32_t bodyIndex;
	int32_t prevKey;
	int32_t nextKey;
} b2JointEdge;

typedef struct b2MouseJoint
{
	b2Vec2 targetA;
	float stiffness;
	float damping;
	float beta;

	// Solver shared
	b2Vec2 impulse;
	float maxForce;
	float gamma;

	// Solver temp
	b2Vec2 rB;
	b2Vec2 localCenterB;
	float invMassB;
	float invIB;
	b2Mat22 mass;
	b2Vec2 C;
} b2MouseJoint;

typedef struct b2RevoluteJoint
{
	// Solver shared
	b2Vec2 impulse;
	float motorImpulse;
	float lowerImpulse;
	float upperImpulse;
	bool enableMotor;
	float maxMotorTorque;
	float motorSpeed;
	bool enableLimit;
	float referenceAngle;
	float lowerAngle;
	float upperAngle;

	// Solver temp
	b2Vec2 rA;
	b2Vec2 rB;
	b2Vec2 localCenterA;
	b2Vec2 localCenterB;
	float invMassA;
	float invMassB;
	float invIA;
	float invIB;
	b2Mat22 K;
	b2Vec2 separation;
	float biasCoefficient;
	float massCoefficient;
	float impulseCoefficient;
	float angle;
	float axialMass;
} b2RevoluteJoint;

typedef struct b2WeldJoint
{
	// Solver shared
	float referenceAngle;
	float linearHertz;
	float linearDampingRatio;
	float angularHertz;
	float angularDampingRatio;
	float linearBiasCoefficient;
	float linearMassCoefficient;
	float linearImpulseCoefficient;
	float angularBiasCoefficient;
	float angularMassCoefficient;
	float angularImpulseCoefficient;
	b2Vec3 impulse;

	// Solver temp
	b2Vec2 localCenterA;
	b2Vec2 localCenterB;
	float invMassA;
	float invMassB;
	float invIA;
	float invIB;
} b2WeldJoint;

/// The base joint class. Joints are used to constraint two bodies together in
/// various fashions. Some joints also feature limits and motors.
typedef struct b2Joint
{
	b2Object object;

	b2JointType type;

	b2JointEdge edges[2];

	int32_t islandIndex;
	int32_t islandPrev;
	int32_t islandNext;

	b2Vec2 localAnchorA;
	b2Vec2 localAnchorB;

	union
	{
		b2MouseJoint mouseJoint;
		b2RevoluteJoint revoluteJoint;
		b2WeldJoint weldJoint;
	};

	bool isMarked;
	bool collideConnected;
} b2Joint;

void b2PrepareJoint(b2Joint* joint, const b2StepContext* context);
void b2SolveJointVelocity(b2Joint* joint, const b2StepContext* context);
void b2SolveJointVelocitySoft(b2Joint* joint, const b2StepContext* context, bool removeOverlap);

// This returns true if the position errors are within tolerance.
bool b2SolveJointPosition(b2Joint* joint, const b2StepContext* context);

void b2DrawJoint(b2DebugDraw* draw, b2World* world, b2Joint* joint);
