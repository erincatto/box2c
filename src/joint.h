// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "pool.h"

#include "box2d/types.h"

#include <stdint.h>

typedef struct b2DebugDraw b2DebugDraw;
typedef struct b2SolverTaskContext b2SolverTaskContext;
typedef struct b2StepContext b2StepContext;
typedef struct b2World b2World;

typedef enum b2JointType
{
	b2_distanceJoint,
	b2_frictionJoint,
	b2_gearJoint,
	b2_motorJoint,
	b2_mouseJoint,
	b2_prismaticJoint,
	b2_pulleyJoint,
	b2_revoluteJoint,
	b2_weldJoint,
	b2_wheelJoint,
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

typedef struct b2DistanceJoint
{
	float hertz;
	float dampingRatio;
	float length;
	float minLength;
	float maxLength;

	// Solver shared
	float impulse;
	float lowerImpulse;
	float upperImpulse;

	// Solver temp
	int32_t indexA;
	int32_t indexB;
	b2Vec2 rA;
	b2Vec2 rB;
	b2Vec2 separation;
	float springBiasCoefficient;
	float springMassCoefficient;
	float springImpulseCoefficient;
	float limitBiasCoefficient;
	float limitMassCoefficient;
	float limitImpulseCoefficient;
	float axialMass;
} b2DistanceJoint;

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
	int32_t indexB;
	b2Vec2 positionB;
	b2Vec2 rB;
	b2Vec2 localCenterB;
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
	int32_t indexA;
	int32_t indexB;
	float angleA;
	float angleB;
	b2Vec2 rA;
	b2Vec2 rB;
	b2Vec2 separation;
	b2Mat22 pivotMass;
	float biasCoefficient;
	float massCoefficient;
	float impulseCoefficient;
	float axialMass;
} b2RevoluteJoint;

typedef struct b2PrismaticJoint
{
	// Solver shared
	b2Vec2 localAxisA;
	b2Vec2 impulse;
	float motorImpulse;
	float lowerImpulse;
	float upperImpulse;
	bool enableMotor;
	float maxMotorForce;
	float motorSpeed;
	bool enableLimit;
	float referenceAngle;
	float lowerTranslation;
	float upperTranslation;

	// Solver temp
	int32_t indexA;
	int32_t indexB;
	b2Vec2 positionA;
	b2Vec2 positionB;
	float angleA;
	float angleB;
	b2Vec2 localCenterA;
	b2Vec2 localCenterB;
	float biasCoefficient;
	float massCoefficient;
	float impulseCoefficient;
	float axialMass;
} b2PrismaticJoint;

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
	b2Vec2 pivotImpulse;
	float axialImpulse;

	// Solver temp
	int32_t indexA;
	int32_t indexB;
	b2Vec2 rA;
	b2Vec2 rB;
	b2Vec2 linearSeparation;
	float angularSeparation;
	b2Mat22 pivotMass;
	float axialMass;
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

	// The color of this constraint in the graph coloring
	int32_t colorIndex;

	// Index of joint within color
	int32_t colorSubIndex;

	b2Vec2 localAnchorA;
	b2Vec2 localAnchorB;

	union
	{
		b2DistanceJoint distanceJoint;
		b2MouseJoint mouseJoint;
		b2RevoluteJoint revoluteJoint;
		b2PrismaticJoint prismaticJoint;
		b2WeldJoint weldJoint;
	};

	bool isMarked;
	bool collideConnected;
} b2Joint;

void b2PrepareJoint(b2Joint* joint, b2StepContext* context);
void b2WarmStartJoint(b2Joint* joint, b2StepContext* context);
void b2SolveJointVelocity(b2Joint* joint, b2StepContext* context, bool useBias);

void b2PrepareAndWarmStartOverflowJoints(b2SolverTaskContext* context);
void b2SolveOverflowJoints(b2SolverTaskContext* context, bool useBias);

void b2DrawJoint(b2DebugDraw* draw, b2World* world, b2Joint* joint);

// Get joint from id with validation
b2Joint* b2GetJoint(b2JointId id, b2JointType type);
