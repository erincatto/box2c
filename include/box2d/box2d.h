// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "box2d/api.h"
#include "box2d/callbacks.h"
#include "box2d/event_types.h"
#include "box2d/geometry.h"
#include "box2d/id.h"
#include "box2d/joint_types.h"
#include "box2d/types.h"

typedef struct b2Capsule b2Capsule;
typedef struct b2Circle b2Circle;
typedef struct b2Polygon b2Polygon;
typedef struct b2DebugDraw b2DebugDraw;
typedef struct b2Segment b2Segment;

/**
 * \defgroup WorldAPI Worlds
 * These functions allow you to create a simulation world. You can then add bodies and
 * joints to the world and run the simulation. You can get contact information to get contact points
 * and normals as well as events. You can query to world, checking for overlaps and casting rays or shapes.
 * There is also debugging information such as debug draw, timing information, and counters.
 * @{
 */

/// Create a world for rigid body simulation. This contains all the bodies, shapes, and constraints.
B2_API b2WorldId b2CreateWorld(const b2WorldDef* def);

/// Destroy a world.
B2_API void b2DestroyWorld(b2WorldId worldId);

/// Take a time step. This performs collision detection, integration,
/// and constraint solution.
/// @param timeStep the amount of time to simulate, this should not vary.
/// @param velocityIterations for the velocity constraint solver.
/// @param relaxIterations for reducing constraint bounce solver.
B2_API void b2World_Step(b2WorldId worldId, float timeStep, int32_t velocityIterations, int32_t relaxIterations);

/// Call this to draw shapes and other debug draw data. This is intentionally non-const.
B2_API void b2World_Draw(b2WorldId worldId, b2DebugDraw* debugDraw);

/// Get sensor events for the current time step. The event data is transient. Do not store a reference to this data.
B2_API b2SensorEvents b2World_GetSensorEvents(b2WorldId worldId);

/// Get contact events for this current time step. The event data is transient. Do not store a reference to this data.
B2_API b2ContactEvents b2World_GetContactEvents(b2WorldId worldId);

/// Query the world for all shapes that potentially overlap the provided AABB.
B2_API void b2World_QueryAABB(b2WorldId worldId, b2QueryResultFcn* fcn, b2AABB aabb, b2QueryFilter filter, void* context);

/// Query the world for all shapes that overlap the provided circle.
B2_API void b2World_OverlapCircle(b2WorldId worldId, b2QueryResultFcn* fcn, const b2Circle* circle, b2Transform transform,
									 b2QueryFilter filter, void* context);

/// Query the world for all shapes that overlap the provided capsule.
B2_API void b2World_OverlapCapsule(b2WorldId worldId, b2QueryResultFcn* fcn, const b2Capsule* capsule, b2Transform transform,
									  b2QueryFilter filter, void* context);

/// Query the world for all shapes that overlap the provided polygon.
B2_API void b2World_OverlapPolygon(b2WorldId worldId, b2QueryResultFcn* fcn, const b2Polygon* polygon, b2Transform transform,
									  b2QueryFilter filter, void* context);

/// Ray-cast the world for all shapes in the path of the ray. Your callback
/// controls whether you get the closest point, any point, or n-points.
/// The ray-cast ignores shapes that contain the starting point.
/// @param callback a user implemented callback class.
/// @param point1 the ray starting point
/// @param point2 the ray ending point
B2_API void b2World_RayCast(b2WorldId worldId, b2Vec2 origin, b2Vec2 translation, b2QueryFilter filter, b2RayResultFcn* fcn,
							   void* context);

/// Ray-cast closest hit. Convenience function. This is less general than b2World_RayCast and does not allow for custom filtering.
B2_API b2RayResult b2World_RayCastClosest(b2WorldId worldId, b2Vec2 origin, b2Vec2 translation, b2QueryFilter filter);

/// Cast a circle through the world. Similar to a ray-cast except that a circle is cast instead of a point.
B2_API void b2World_CircleCast(b2WorldId worldId, const b2Circle* circle, b2Transform originTransform, b2Vec2 translation,
								  b2QueryFilter filter, b2RayResultFcn* fcn, void* context);

/// Cast a capsule through the world. Similar to a ray-cast except that a capsule is cast instead of a point.
B2_API void b2World_CapsuleCast(b2WorldId worldId, const b2Capsule* capsule, b2Transform originTransform, b2Vec2 translation,
								   b2QueryFilter filter, b2RayResultFcn* fcn, void* context);

/// Cast a capsule through the world. Similar to a ray-cast except that a polygon is cast instead of a point.
B2_API void b2World_PolygonCast(b2WorldId worldId, const b2Polygon* polygon, b2Transform originTransform, b2Vec2 translation,
								   b2QueryFilter filter, b2RayResultFcn* fcn, void* context);

/// Enable/disable sleep. Advanced feature for testing.
B2_API void b2World_EnableSleeping(b2WorldId worldId, bool flag);

/// Enable/disable constraint warm starting. Advanced feature for testing.
B2_API void b2World_EnableWarmStarting(b2WorldId worldId, bool flag);

/// Enable/disable continuous collision. Advanced feature for testing.
B2_API void b2World_EnableContinuous(b2WorldId worldId, bool flag);

/// Adjust the restitution threshold. Advanced feature for testing.
B2_API void b2World_SetRestitutionThreshold(b2WorldId worldId, float value);

/// Adjust contact tuning parameters:
/// - hertz is the contact stiffness (cycles per second)
/// - damping ratio is the contact bounciness with 1 being critical damping (non-dimensional)
/// - push velocity is the maximum contact constraint push out velocity (meters per second)
///	Advanced feature
B2_API void b2World_SetContactTuning(b2WorldId worldId, float hertz, float dampingRatio, float pushVelocity);

/// Get the current profile
B2_API b2Profile b2World_GetProfile(b2WorldId worldId);

/// Get counters and sizes
B2_API b2Counters b2World_GetCounters(b2WorldId worldId);

/** @} */

/**
 * \defgroup BodyAPI Bodies
 * This is the body API.
 * @{
 */

/// Create a rigid body given a definition. No reference to the definition is retained.
/// @warning This function is locked during callbacks.
B2_API b2BodyId b2CreateBody(b2WorldId worldId, const b2BodyDef* def);

/// Destroy a rigid body given an id.
/// @warning This function is locked during callbacks.
B2_API void b2DestroyBody(b2BodyId bodyId);

/// Get the type of a body
B2_API b2BodyType b2Body_GetType(b2BodyId bodyId);

/// Set the type of a body. This has a similar cost to re-creating the body.
B2_API void b2Body_SetType(b2BodyId bodyId, b2BodyType type);

/// Get the user data stored in a body
B2_API void* b2Body_GetUserData(b2BodyId bodyId);

/// Get the world position of a body. This is the location of the body origin.
B2_API b2Vec2 b2Body_GetPosition(b2BodyId bodyId);

/// Get the world angle of a body in radians.
B2_API float b2Body_GetAngle(b2BodyId bodyId);

/// Get the world transform of a body.
B2_API b2Transform b2Body_GetTransform(b2BodyId bodyId);

/// Set the world transform of a body. This acts as a teleport and is fairly expensive.
B2_API void b2Body_SetTransform(b2BodyId bodyId, b2Vec2 position, float angle);

/// Get a local point on a body given a world point
B2_API b2Vec2 b2Body_GetLocalPoint(b2BodyId bodyId, b2Vec2 worldPoint);

/// Get a world point on a body given a local point
B2_API b2Vec2 b2Body_GetWorldPoint(b2BodyId bodyId, b2Vec2 localPoint);

/// Get a local vector on a body given a world vector
B2_API b2Vec2 b2Body_GetLocalVector(b2BodyId bodyId, b2Vec2 worldVector);

/// Get a world vector on a body given a local vector
B2_API b2Vec2 b2Body_GetWorldVector(b2BodyId bodyId, b2Vec2 localVector);

/// Get the linear velocity of a body's center of mass
B2_API b2Vec2 b2Body_GetLinearVelocity(b2BodyId bodyId);

/// Get the angular velocity of a body in radians per second
B2_API float b2Body_GetAngularVelocity(b2BodyId bodyId);

/// Set the linear velocity of a body
B2_API void b2Body_SetLinearVelocity(b2BodyId bodyId, b2Vec2 linearVelocity);

/// Set the angular velocity of a body in radians per second
B2_API void b2Body_SetAngularVelocity(b2BodyId bodyId, float angularVelocity);

/// Apply a force at a world point. If the force is not
/// applied at the center of mass, it will generate a torque and
/// affect the angular velocity. This wakes up the body.
/// @param force the world force vector, usually in Newtons (N).
/// @param point the world position of the point of application.
/// @param wake also wake up the body
B2_API void b2Body_ApplyForce(b2BodyId bodyId, b2Vec2 force, b2Vec2 point, bool wake);

/// Apply a force to the center of mass. This wakes up the body.
/// @param force the world force vector, usually in Newtons (N).
/// @param wake also wake up the body
B2_API void b2Body_ApplyForceToCenter(b2BodyId bodyId, b2Vec2 force, bool wake);

/// Apply a torque. This affects the angular velocity
/// without affecting the linear velocity of the center of mass.
/// @param torque about the z-axis (out of the screen), usually in N-m.
/// @param wake also wake up the body
B2_API void b2Body_ApplyTorque(b2BodyId bodyId, float torque, bool wake);

/// Apply an impulse at a point. This immediately modifies the velocity.
/// It also modifies the angular velocity if the point of application
/// is not at the center of mass. This wakes up the body.
/// @param impulse the world impulse vector, usually in N-seconds or kg-m/s.
/// @param point the world position of the point of application.
/// @param wake also wake up the body
B2_API void b2Body_ApplyLinearImpulse(b2BodyId bodyId, b2Vec2 impulse, b2Vec2 point, bool wake);

/// Apply an impulse to the center of mass. This immediately modifies the velocity.
/// @param impulse the world impulse vector, usually in N-seconds or kg-m/s.
/// @param wake also wake up the body
B2_API void b2Body_ApplyLinearImpulseToCenter(b2BodyId bodyId, b2Vec2 impulse, bool wake);

/// Apply an angular impulse.
/// @param impulse the angular impulse in units of kg*m*m/s
/// @param wake also wake up the body
B2_API void b2Body_ApplyAngularImpulse(b2BodyId bodyId, float impulse, bool wake);

/// Get the mass of the body (kilograms)
B2_API float b2Body_GetMass(b2BodyId bodyId);

/// Get the inertia tensor of the body. In 2D this is a single number. (kilograms * meters^2)
B2_API float b2Body_GetInertiaTensor(b2BodyId bodyId);

/// Get the center of mass position of the body in local space.
B2_API b2Vec2 b2Body_GetLocalCenterOfMass(b2BodyId bodyId);

/// Get the center of mass position of the body in world space.
B2_API b2Vec2 b2Body_GetWorldCenterOfMass(b2BodyId bodyId);

/// Override the body's mass properties. Normally this is computed automatically using the
///	shape geometry and density. This information is lost if a shape is added or removed or if the
///	body type changes.
B2_API void b2Body_SetMassData(b2BodyId bodyId, b2MassData massData);

/// Get the mass data for a body.
B2_API b2MassData b2Body_GetMassData(b2BodyId bodyId);

/// Adjust the linear damping. Normally this is set in b2BodyDef before creation.
B2_API void b2Body_SetLinearDamping(b2BodyId bodyId, float linearDamping);

/// Adjust the angular damping. Normally this is set in b2BodyDef before creation.
B2_API void b2Body_SetAngularDamping(b2BodyId bodyId, float angularDamping);

/// Adjust the gravity scale. Normally this is set in b2BodyDef before creation.
B2_API void b2Body_SetGravityScale(b2BodyId bodyId, float gravityScale);

/// Is this body awake?
B2_API bool b2Body_IsAwake(b2BodyId bodyId);

/// Wake a body from sleep. This wakes the entire island the body is touching.
B2_API void b2Body_Wake(b2BodyId bodyId);

/// Is this body enabled?
B2_API bool b2Body_IsEnabled(b2BodyId bodyId);

/// Disable a body by removing it completely from the simulation
B2_API void b2Body_Disable(b2BodyId bodyId);

/// Enable a body by adding it to the simulation
B2_API void b2Body_Enable(b2BodyId bodyId);

/// Iterate over shapes on a body
B2_API b2ShapeId b2Body_GetFirstShape(b2BodyId bodyId);
B2_API b2ShapeId b2Body_GetNextShape(b2ShapeId shapeId);

/// Get the maximum capacity required for retrieving all the touching contacts on a body
B2_API int32_t b2Body_GetContactCapacity(b2BodyId bodyId);

/// Get the touching contact data for a body
B2_API int32_t b2Body_GetContactData(b2BodyId bodyId, b2ContactData* contactData, int32_t capacity);

/** @} */

/**
 * \defgroup ShapeAPI Shapes
 * This is the shape API.
 * @{
 */

/// Create a circle shape and attach it to a body. The shape defintion and geometry are fully cloned.
/// Contacts are not created until the next time step.
///	@return the shape id for accessing the shape
B2_API b2ShapeId b2CreateCircleShape(b2BodyId bodyId, const b2ShapeDef* def, const b2Circle* circle);

/// Create a line segment shape and attach it to a body. The shape defintion and geometry are fully cloned.
/// Contacts are not created until the next time step.
///	@return the shape id for accessing the shape
B2_API b2ShapeId b2CreateSegmentShape(b2BodyId bodyId, const b2ShapeDef* def, const b2Segment* segment);

/// Create a capsule shape and attach it to a body. The shape defintion and geometry are fully cloned.
/// Contacts are not created until the next time step.
///	@return the shape id for accessing the shape
B2_API b2ShapeId b2CreateCapsuleShape(b2BodyId bodyId, const b2ShapeDef* def, const b2Capsule* capsule);

/// Create a polygon shape and attach it to a body. The shape defintion and geometry are fully cloned.
/// Contacts are not created until the next time step.
///	@return the shape id for accessing the shape
B2_API b2ShapeId b2CreatePolygonShape(b2BodyId bodyId, const b2ShapeDef* def, const b2Polygon* polygon);

/// Destroy any shape type
B2_API void b2DestroyShape(b2ShapeId shapeId);

/// Create a chain shape
///	@see b2ChainDef for details
B2_API b2ChainId b2CreateChain(b2BodyId bodyId, const b2ChainDef* def);

/// Destroy a chain shape
B2_API void b2DestroyChain(b2ChainId chainId);

/// Get the type of a shape.
B2_API b2ShapeType b2Shape_GetType(b2ShapeId shapeId);

/// Get the body that a shape is attached to
B2_API b2BodyId b2Shape_GetBody(b2ShapeId shapeId);

/// Get the type of a shape.
B2_API bool b2Shape_IsSensor(b2ShapeId shapeId);

/// Get the user data for a shape. This is useful when you get a shape id
///	from an event or query
B2_API void* b2Shape_GetUserData(b2ShapeId shapeId);

/// Set the density on a shape. Normally this is specified in b2ShapeDef.
///	This will recompute the mass properties on the parent body.
B2_API void b2Shape_SetDensity(b2ShapeId shapeId, float density);

/// Get the density on a shape.
B2_API float b2Shape_GetDensity(b2ShapeId shapeId);

/// Set the friction on a shape. Normally this is specified in b2ShapeDef.
B2_API void b2Shape_SetFriction(b2ShapeId shapeId, float friction);

/// Get the friction on a shape.
B2_API float b2Shape_GetFriction(b2ShapeId shapeId);

/// Set the restitution (bounciness) on a shape. Normally this is specified in b2ShapeDef.
B2_API void b2Shape_SetRestitution(b2ShapeId shapeId, float restitution);

/// Get the restitution on a shape.
B2_API float b2Shape_GetRestitution(b2ShapeId shapeId);

/// Get the current filter
B2_API b2Filter b2Shape_GetFilter(b2ShapeId shapeId);

/// Set the current filter. This is almost as expensive as recreating the shape.
B2_API void b2Shape_SetFilter(b2ShapeId shapeId, b2Filter filter);

/// Test a point for overlap with a shape
B2_API bool b2Shape_TestPoint(b2ShapeId shapeId, b2Vec2 point);

/// Access the circle geometry of a shape.
B2_API const b2Circle* b2Shape_GetCircle(b2ShapeId shapeId);

/// Access the line segment geometry of a shape.
B2_API const b2Segment* b2Shape_GetSegment(b2ShapeId shapeId);

/// Access the smooth line segment geometry of a shape. These come from chain shapes.
B2_API const b2SmoothSegment* b2Shape_GetSmoothSegment(b2ShapeId shapeId);

/// Access the capsule geometry of a shape.
B2_API const b2Capsule* b2Shape_GetCapsule(b2ShapeId shapeId);

/// Access the convex polygon geometry of a shape.
B2_API const b2Polygon* b2Shape_GetPolygon(b2ShapeId shapeId);

/// If the type is b2_smoothSegmentShape then you can get the parent chain id.
/// If the shape is not a smooth segment then this will return b2_nullChainId.
B2_API b2ChainId b2Shape_GetParentChain(b2ShapeId shapeId);

/// Set the friction of a chain. Normally this is set in b2ChainDef.
B2_API void b2Chain_SetFriction(b2ChainId chainId, float friction);

/// Set the restitution (bounciness) on a chain. Normally this is specified in b2ChainDef.
B2_API void b2Chain_SetRestitution(b2ChainId chainId, float restitution);

/// Get the maximum capacity required for retrieving all the touching contacts on a shape
B2_API int32_t b2Shape_GetContactCapacity(b2ShapeId shapeId);

/// Get the touching contact data for a shape. The provided shapeId will be either shapeIdA or shapeIdB on the contact data.
B2_API int32_t b2Shape_GetContactData(b2ShapeId shapeId, b2ContactData* contactData, int32_t capacity);

/** @} */

/**
 * \defgroup JointAPI Joints
 * This is the joint API.
 * @{
 */

/// Create a distance joint
///	@see b2DistanceJointDef for details
B2_API b2JointId b2CreateDistanceJoint(b2WorldId worldId, const b2DistanceJointDef* def);

/// Create a motor joint
///	@see b2MotorJointDef for details
B2_API b2JointId b2CreateMotorJoint(b2WorldId worldId, const b2MotorJointDef* def);

/// Create a mouse joint
///	@see b2MouseJointDef for details
B2_API b2JointId b2CreateMouseJoint(b2WorldId worldId, const b2MouseJointDef* def);

/// Create a prismatic (slider) joint
///	@see b2PrismaticJointDef for details
B2_API b2JointId b2CreatePrismaticJoint(b2WorldId worldId, const b2PrismaticJointDef* def);

/// Create a revolute (hinge) joint
///	@see b2RevoluteJointDef for details
B2_API b2JointId b2CreateRevoluteJoint(b2WorldId worldId, const b2RevoluteJointDef* def);

/// Create a weld joint
///	@see b2WeldJointDef for details
B2_API b2JointId b2CreateWeldJoint(b2WorldId worldId, const b2WeldJointDef* def);

/// Create a wheel joint
///	@see b2WheelJointDef for details
B2_API b2JointId b2CreateWheelJoint(b2WorldId worldId, const b2WheelJointDef* def);

/// Destroy any joint type
B2_API void b2DestroyJoint(b2JointId jointId);

/// Get the joint type
B2_API b2JointType b2Joint_GetType(b2JointId jointId);

	/// Get body A on a joint
B2_API b2BodyId b2Joint_GetBodyA(b2JointId jointId);

/// Get body B on a joint
B2_API b2BodyId b2Joint_GetBodyB(b2JointId jointId);

/// Get the constraint force on a distance joint
B2_API float b2DistanceJoint_GetConstraintForce(b2JointId jointId, float timeStep);

/// Set the length parameters of a distance joint
///	@see b2DistanceJointDef for details
B2_API void b2DistanceJoint_SetLength(b2JointId jointId, float length, float minLength, float maxLength);

/// Get the current length of a distance joint
B2_API float b2DistanceJoint_GetCurrentLength(b2JointId jointId);

/// Adjust the softness of a distance joint
///	@see b2DistanceJointDef for details
B2_API void b2DistanceJoint_SetTuning(b2JointId jointId, float hertz, float dampingRatio);

/// Set the linear offset target for a motor joint
B2_API void b2MotorJoint_SetLinearOffset(b2JointId jointId, b2Vec2 linearOffset);

/// Set the angular offset target for a motor joint in radians
B2_API void b2MotorJoint_SetAngularOffset(b2JointId jointId, float angularOffset);

/// Set the maximum force for a motor joint
B2_API void b2MotorJoint_SetMaxForce(b2JointId jointId, float maxForce);

/// Set the maximum torque for a motor joint
B2_API void b2MotorJoint_SetMaxTorque(b2JointId jointId, float maxTorque);

/// Set the correction factor for a motor joint
B2_API void b2MotorJoint_SetCorrectionFactor(b2JointId jointId, float correctionFactor);

/// Get the current constraint force for a motor joint
B2_API b2Vec2 b2MotorJoint_GetConstraintForce(b2JointId jointId, float inverseTimeStep);

/// Get the current constraint torque for a motor joint
B2_API float b2MotorJoint_GetConstraintTorque(b2JointId jointId, float inverseTimeStep);

/// Set the target for a mouse joint
B2_API void b2MouseJoint_SetTarget(b2JointId jointId, b2Vec2 target);

/// Enable/disable a prismatic joint limit
B2_API void b2PrismaticJoint_EnableLimit(b2JointId jointId, bool enableLimit);

/// Enable/disable a prismatic joint motor
B2_API void b2PrismaticJoint_EnableMotor(b2JointId jointId, bool enableMotor);

/// Set the motor speed for a prismatic joint
B2_API void b2PrismaticJoint_SetMotorSpeed(b2JointId jointId, float motorSpeed);

/// Get the current motor force for a prismatic joint
B2_API float b2PrismaticJoint_GetMotorForce(b2JointId jointId, float inverseTimeStep);

/// Set the maximum force for a pristmatic joint motor
B2_API void b2PrismaticJoint_SetMaxMotorForce(b2JointId jointId, float force);

/// Get the current constraint force for a prismatic joint
B2_API b2Vec2 b2PrismaticJoint_GetConstraintForce(b2JointId jointId, float inverseTimeStep);

/// Get the current constraint torque for a prismatic joint
B2_API float b2PrismaticJoint_GetConstraintTorque(b2JointId jointId, float inverseTimeStep);

/// Enable/disable a revolute joint limit
B2_API void b2RevoluteJoint_EnableLimit(b2JointId jointId, bool enableLimit);

/// Enable/disable a revolute joint motor
B2_API void b2RevoluteJoint_EnableMotor(b2JointId jointId, bool enableMotor);

/// Set the motor speed for a revolute joint in radians per second
B2_API void b2RevoluteJoint_SetMotorSpeed(b2JointId jointId, float motorSpeed);

/// Get the current motor torque for a revolute joint
B2_API float b2RevoluteJoint_GetMotorTorque(b2JointId jointId, float inverseTimeStep);

/// Set the maximum torque for a revolute joint motor
B2_API void b2RevoluteJoint_SetMaxMotorTorque(b2JointId jointId, float torque);

/// Get the current constraint force for a revolute joint
B2_API b2Vec2 b2RevoluteJoint_GetConstraintForce(b2JointId jointId, float inverseTimeStep);

/// Get the current constraint torque for a revolute joint
B2_API float b2RevoluteJoint_GetConstraintTorque(b2JointId jointId, float inverseTimeStep);

/// Set the wheel joint stiffness
B2_API void b2WheelJoint_SetStiffness(b2JointId jointId, float stiffness);

/// Set the wheel joint damping
B2_API void b2WheelJoint_SetDamping(b2JointId jointId, float damping);

/// Enable/disable the wheel joint limit
B2_API void b2WheelJoint_EnableLimit(b2JointId jointId, bool enableLimit);

/// Enable/disable the wheel joint motor
B2_API void b2WheelJoint_EnableMotor(b2JointId jointId, bool enableMotor);

/// Set the wheel joint motor speed in radians per second
B2_API void b2WheelJoint_SetMotorSpeed(b2JointId jointId, float motorSpeed);

/// Get the wheel joint current motor torque
B2_API float b2WheelJoint_GetMotorTorque(b2JointId jointId, float inverseTimeStep);

/// Set the wheel joint maximum motor torque
B2_API void b2WheelJoint_SetMaxMotorTorque(b2JointId jointId, float torque);

/// Get the current wheel joint constraint force
B2_API b2Vec2 b2WheelJoint_GetConstraintForce(b2JointId jointId, float inverseTimeStep);

/// Get the current wheel joint constraint torque
B2_API float b2WheelJoint_GetConstraintTorque(b2JointId jointId, float inverseTimeStep);

/** @} */
