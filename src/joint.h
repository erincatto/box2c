// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT
#pragma once

#include "pool.h"
#include "solver_data.h"

#include "box2d/joint_types.h"

#include <stdint.h>

typedef struct b2DebugDraw b2DebugDraw;
typedef struct b2StepContext b2StepContext;
typedef struct b2World b2World;

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

	float impulse;
	float lowerImpulse;
	float upperImpulse;

	int32_t indexA;
	int32_t indexB;
	b2Vec2 anchorA;
	b2Vec2 anchorB;
	b2Vec2 deltaCenter;
	b2Softness distanceSoftness;
	float axialMass;
} b2DistanceJoint;

typedef struct b2MotorJoint
{
	b2Vec2 linearOffset;
	float angularOffset;
	b2Vec2 linearImpulse;
	float angularImpulse;
	float maxForce;
	float maxTorque;
	float correctionFactor;

	int32_t indexA;
	int32_t indexB;
	b2Vec2 anchorA;
	b2Vec2 anchorB;
	b2Vec2 deltaCenter;
	float deltaAngle;
	b2Mat22 linearMass;
	float angularMass;
} b2MotorJoint;

typedef struct b2MouseJoint
{
	b2Vec2 targetA;
	float hertz;
	float dampingRatio;

	b2Vec2 linearImpulse;
	float angularImpulse;

	b2Softness linearSoftness;
	b2Softness angularSoftness;
	int32_t indexB;
	b2Vec2 anchorB;
	b2Vec2 deltaCenter;
	b2Mat22 linearMass;
} b2MouseJoint;

typedef struct b2PrismaticJoint
{
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

	int32_t indexA;
	int32_t indexB;
	b2Vec2 anchorA;
	b2Vec2 anchorB;
	b2Vec2 axisA;
	b2Vec2 deltaCenter;
	float deltaAngle;
	float axialMass;
} b2PrismaticJoint;

typedef struct b2RevoluteJoint
{
	b2Vec2 linearImpulse;
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

	int32_t indexA;
	int32_t indexB;
	b2Vec2 anchorA;
	b2Vec2 anchorB;
	b2Vec2 deltaCenter;
	float deltaAngle;
	float axialMass;
} b2RevoluteJoint;

typedef struct b2WeldJoint
{
	float referenceAngle;
	float linearHertz;
	float linearDampingRatio;
	float angularHertz;
	float angularDampingRatio;

	b2Softness linearSoftness;
	b2Softness angularSoftness;
	b2Vec2 linearImpulse;
	float angularImpulse;

	int32_t indexA;
	int32_t indexB;
	b2Vec2 anchorA;
	b2Vec2 anchorB;
	b2Vec2 deltaCenter;
	float deltaAngle;
	float axialMass;
} b2WeldJoint;

typedef struct b2WheelJoint
{
	// Solver shared
	b2Vec2 localAxisA;
	float perpImpulse;
	float motorImpulse;
	float springImpulse;
	float lowerImpulse;
	float upperImpulse;
	float maxMotorTorque;
	float motorSpeed;
	float lowerTranslation;
	float upperTranslation;
	float hertz;
	float dampingRatio;
	bool enableMotor;
	bool enableLimit;

	// Solver temp
	int32_t indexA;
	int32_t indexB;
	b2Vec2 anchorA;
	b2Vec2 anchorB;
	b2Vec2 axisA;
	b2Vec2 deltaCenter;
	float perpMass;
	float motorMass;
	float axialMass;
	b2Softness springSoftness;
} b2WheelJoint;

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

	// Anchors relative to body origin
	b2Vec2 localOriginAnchorA;
	b2Vec2 localOriginAnchorB;

	float invMassA, invMassB;
	float invIA, invIB;

	union
	{
		b2DistanceJoint distanceJoint;
		b2MotorJoint motorJoint;
		b2MouseJoint mouseJoint;
		b2RevoluteJoint revoluteJoint;
		b2PrismaticJoint prismaticJoint;
		b2WeldJoint weldJoint;
		b2WheelJoint wheelJoint;
	};

	float drawSize;
	bool isMarked;
	bool collideConnected;
} b2Joint;

b2Joint* b2GetJoint(b2World* world, b2JointId jointId);

// todo remove this
b2Joint* b2GetJointCheckType(b2JointId id, b2JointType type);

void b2PrepareJoint(b2Joint* joint, b2StepContext* context);
void b2WarmStartJoint(b2Joint* joint, b2StepContext* context);
void b2SolveJoint(b2Joint* joint, b2StepContext* context, bool useBias);

void b2PrepareOverflowJoints(b2StepContext* context);
void b2WarmStartOverflowJoints(b2StepContext* context);
void b2SolveOverflowJoints(b2StepContext* context, bool useBias);

void b2DrawJoint(b2DebugDraw* draw, b2World* world, b2Joint* joint);
