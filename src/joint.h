// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "box2d/id.h"
#include "box2d/types.h"

#include "pool.h"

#include <stdint.h>

typedef struct b2SolverData b2SolverData;

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
	int32_t prevJointIndex;
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

#if 0
	/// Get the anchor point on bodyA in world coordinates.
	virtual b2Vec2 GetAnchorA() const = 0;

	/// Get the anchor point on bodyB in world coordinates.
	virtual b2Vec2 GetAnchorB() const = 0;

	/// Get the reaction force on bodyB at the joint anchor in Newtons.
	virtual b2Vec2 GetReactionForce(float inv_dt) const = 0;

	/// Get the reaction torque on bodyB in N*m.
	virtual float GetReactionTorque(float inv_dt) const = 0;

	/// Get the next joint the world joint list.
	b2Joint* GetNext();
	const b2Joint* GetNext() const;

	/// Get the user data pointer.
	b2JointUserData& GetUserData();
	const b2JointUserData& GetUserData() const;

	/// Short-cut function to determine if either body is enabled.
	bool IsEnabled() const;

	/// Get collide connected.
	/// Note: modifying the collide connect flag won't work correctly because
	/// the flag is only checked when fixture AABBs begin to overlap.
	bool GetCollideConnected() const;

	/// Dump this joint to the log file.
	virtual void Dump() { b2Dump("// Dump is not supported for this joint type.\n"); }

	/// Shift the origin for any points stored in world coordinates.
	virtual void ShiftOrigin(const b2Vec2& newOrigin) { B2_NOT_USED(newOrigin);  }

	/// Debug draw this joint
	virtual void Draw(b2Draw* draw) const;

	static b2Joint* Create(const b2JointDef* def, b2BlockAllocator* allocator);
	static void Destroy(b2Joint* joint, b2BlockAllocator* allocator);

	b2Joint(const b2JointDef* def);
	virtual ~b2Joint() {}

	#endif

	b2JointType type;

	b2JointEdge edgeA;
	b2JointEdge edgeB;

	int32_t islandId;

	union
	{
		b2MouseJoint mouseJoint;
	};

	bool collideConnected;
} b2Joint;

void b2InitVelocityConstraints(b2Joint* joint, const b2SolverData* data);
void b2SolveVelocityConstraints(b2Joint* joint, const b2SolverData* data);

// This returns true if the position errors are within tolerance.
bool b2SolvePositionConstraints(b2Joint* joint, const b2SolverData* data);
