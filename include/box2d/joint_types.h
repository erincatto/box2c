// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "box2d/types.h"

/// A mouse joint is used to make a point on a body track a
/// specified world point. This a soft constraint with a maximum
/// force. This allows the constraint to stretch without
/// applying huge forces.
/// NOTE: this joint is not documented in the manual because it was
/// developed to be used in samples. If you want to learn how to
/// use the mouse joint, look at the samples app.
typedef struct b2MouseJointDef
{
	/// The first attached body.
	b2BodyId bodyIdA;

	/// The second attached body.
	b2BodyId bodyIdB;

	/// The initial target point in world space
	b2Vec2 target;

	/// The maximum constraint force that can be exerted
	/// to move the candidate body. Usually you will express
	/// as some multiple of the weight (multiplier * mass * gravity).
	float maxForce;

	/// The linear stiffness in N/m
	float stiffness;

	/// The linear damping in N*s/m
	float damping;
} b2MouseJointDef;

static inline struct b2MouseJointDef b2DefaultMouseJointDef(void)
{
	b2MouseJointDef def = {0};
	def.bodyIdA = b2_nullBodyId;
	def.bodyIdB = b2_nullBodyId;
	def.target = B2_LITERAL(b2Vec2){0.0f, 0.0f};
	def.maxForce = 0.0f;
	def.stiffness = 0.0f;
	def.damping = 0.0f;
	return def;
}

/// Revolute joint definition. This requires defining an anchor point where the
/// bodies are joined. The definition uses local anchor points so that the
/// initial configuration can violate the constraint slightly. You also need to
/// specify the initial relative angle for joint limits. This helps when saving
/// and loading a game.
/// The local anchor points are measured from the body's origin
/// rather than the center of mass because:
/// 1. you might not know where the center of mass will be.
/// 2. if you add/remove shapes from a body and recompute the mass,
///    the joints will be broken.
typedef struct b2RevoluteJointDef
{
	/// The first attached body.
	b2BodyId bodyIdA;

	/// The second attached body.
	b2BodyId bodyIdB;

	/// The local anchor point relative to bodyA's origin.
	b2Vec2 localAnchorA;

	/// The local anchor point relative to bodyB's origin.
	b2Vec2 localAnchorB;

	/// The bodyB angle minus bodyA angle in the reference state (radians).
	/// This defines the zero angle for the joint limit.
	float referenceAngle;

	/// A flag to enable joint limits.
	bool enableLimit;

	/// The lower angle for the joint limit (radians).
	float lowerAngle;

	/// The upper angle for the joint limit (radians).
	float upperAngle;

	/// A flag to enable the joint motor.
	bool enableMotor;

	/// The desired motor speed. Usually in radians per second.
	float motorSpeed;

	/// The maximum motor torque used to achieve the desired motor speed.
	/// Usually in N-m.
	float maxMotorTorque;

	/// Set this flag to true if the attached bodies should collide.
	bool collideConnected;
} b2RevoluteJointDef;

static inline struct b2RevoluteJointDef b2DefaultRevoluteJointDef(void)
{
	b2RevoluteJointDef def = {0};
	def.bodyIdA = b2_nullBodyId;
	def.bodyIdB = b2_nullBodyId;
	def.localAnchorA = B2_LITERAL(b2Vec2){0.0f, 0.0f};
	def.localAnchorB = B2_LITERAL(b2Vec2){0.0f, 0.0f};
	def.referenceAngle = 0.0f;
	def.lowerAngle = 0.0f;
	def.upperAngle = 0.0f;
	def.maxMotorTorque = 0.0f;
	def.motorSpeed = 0.0f;
	def.enableLimit = false;
	def.enableMotor = false;
	def.collideConnected = false;
	return def;
}
