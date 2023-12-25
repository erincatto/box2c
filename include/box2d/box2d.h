// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "box2d/api.h"
#include "box2d/callbacks.h"
#include "box2d/event_types.h"
#include "box2d/geometry.h"
#include "box2d/id.h"
#include "box2d/joint_types.h"
#include "box2d/timer.h"
#include "box2d/types.h"

typedef struct b2Capsule b2Capsule;
typedef struct b2Circle b2Circle;
typedef struct b2Polygon b2Polygon;
typedef struct b2DebugDraw b2DebugDraw;
typedef struct b2Segment b2Segment;

/**
 * \defgroup WorldAPI Worlds
 * This is the main Box2D API. With this API you can create a simulation world. You can then add bodies and
 * joints to the world and run the simulation. You can get contact information to get contact points
 * and normals as well as events. You can query to world, checking for overlaps and casting rays or shapes.
 * There is also debugging information such as debug draw, timing information, and counters.
 * @{
 */

/// Create a world for rigid body simulation. This contains all the bodies, shapes, and constraints.
BOX2D_API b2WorldId b2CreateWorld(const b2WorldDef* def);

/// Destroy a world.
BOX2D_API void b2DestroyWorld(b2WorldId worldId);

/// Take a time step. This performs collision detection, integration,
/// and constraint solution.
/// @param timeStep the amount of time to simulate, this should not vary.
/// @param velocityIterations for the velocity constraint solver.
/// @param relaxIterations for reducing constraint bounce solver.
BOX2D_API void b2World_Step(b2WorldId worldId, float timeStep, int32_t velocityIterations, int32_t relaxIterations);

/// Call this to draw shapes and other debug draw data. This is intentionally non-const.
BOX2D_API void b2World_Draw(b2WorldId worldId, b2DebugDraw* debugDraw);

/// Create a rigid body given a definition. No reference to the definition is retained.
/// @warning This function is locked during callbacks.
BOX2D_API b2BodyId b2World_CreateBody(b2WorldId worldId, const b2BodyDef* def);

/// Destroy a rigid body given an id.
/// @warning This function is locked during callbacks.
BOX2D_API void b2World_DestroyBody(b2BodyId bodyId);

/// Create a joint
BOX2D_API b2JointId b2World_CreateDistanceJoint(b2WorldId worldId, const b2DistanceJointDef* def);
BOX2D_API b2JointId b2World_CreateMotorJoint(b2WorldId worldId, const b2MotorJointDef* def);
BOX2D_API b2JointId b2World_CreateMouseJoint(b2WorldId worldId, const b2MouseJointDef* def);
BOX2D_API b2JointId b2World_CreatePrismaticJoint(b2WorldId worldId, const b2PrismaticJointDef* def);
BOX2D_API b2JointId b2World_CreateRevoluteJoint(b2WorldId worldId, const b2RevoluteJointDef* def);
BOX2D_API b2JointId b2World_CreateWeldJoint(b2WorldId worldId, const b2WeldJointDef* def);
BOX2D_API b2JointId b2World_CreateWheelJoint(b2WorldId worldId, const b2WheelJointDef* def);

/// Destroy a joint
BOX2D_API void b2World_DestroyJoint(b2JointId jointId);

/// Get sensor events for the current time step. Do not store a reference to this data.
BOX2D_API b2SensorEvents b2World_GetSensorEvents(b2WorldId worldId);
BOX2D_API b2ContactEvents b2World_GetContactEvents(b2WorldId worldId);

/// Query the world for all shapes that potentially overlap the provided AABB.
BOX2D_API void b2World_QueryAABB(b2WorldId worldId, b2QueryResultFcn* fcn, b2AABB aabb, b2QueryFilter filter, void* context);

/// Query the world for all shapes that overlap the provided circle.
BOX2D_API void b2World_OverlapCircle(b2WorldId worldId, b2QueryResultFcn* fcn, const b2Circle* circle, b2Transform transform,
									 b2QueryFilter filter, void* context);

/// Query the world for all shapes that overlap the provided capsule.
BOX2D_API void b2World_OverlapCapsule(b2WorldId worldId, b2QueryResultFcn* fcn, const b2Capsule* capsule, b2Transform transform,
									  b2QueryFilter filter, void* context);

/// Query the world for all shapes that overlap the provided polygon.
BOX2D_API void b2World_OverlapPolygon(b2WorldId worldId, b2QueryResultFcn* fcn, const b2Polygon* polygon, b2Transform transform,
									  b2QueryFilter filter, void* context);

/// Ray-cast the world for all shapes in the path of the ray. Your callback
/// controls whether you get the closest point, any point, or n-points.
/// The ray-cast ignores shapes that contain the starting point.
/// @param callback a user implemented callback class.
/// @param point1 the ray starting point
/// @param point2 the ray ending point
BOX2D_API void b2World_RayCast(b2WorldId worldId, b2Vec2 origin, b2Vec2 translation, b2QueryFilter filter, b2RayResultFcn* fcn,
							   void* context);

// Ray-cast closest hit. Convenience function. This is less general than b2World_RayCast and does not allow for custom filtering.
BOX2D_API b2RayResult b2World_RayCastClosest(b2WorldId worldId, b2Vec2 origin, b2Vec2 translation, b2QueryFilter filter);

BOX2D_API void b2World_CircleCast(b2WorldId worldId, const b2Circle* circle, b2Transform originTransform, b2Vec2 translation,
								  b2QueryFilter filter, b2RayResultFcn* fcn, void* context);

BOX2D_API void b2World_CapsuleCast(b2WorldId worldId, const b2Capsule* capsule, b2Transform originTransform, b2Vec2 translation,
								   b2QueryFilter filter, b2RayResultFcn* fcn, void* context);

BOX2D_API void b2World_PolygonCast(b2WorldId worldId, const b2Polygon* polygon, b2Transform originTransform, b2Vec2 translation,
								   b2QueryFilter filter, b2RayResultFcn* fcn, void* context);

/// Enable/disable sleep. Advanced feature for testing.
BOX2D_API void b2World_EnableSleeping(b2WorldId worldId, bool flag);

/// Enable/disable constraint warm starting. Advanced feature for testing.
BOX2D_API void b2World_EnableWarmStarting(b2WorldId worldId, bool flag);

/// Enable/disable continuous collision. Advanced feature for testing.
BOX2D_API void b2World_EnableContinuous(b2WorldId worldId, bool flag);

/// Adjust the restitution threshold. Advanced feature for testing.
BOX2D_API void b2World_SetRestitutionThreshold(b2WorldId worldId, float value);

/// Adjust contact tuning parameters:
/// - hertz is the contact stiffness (cycles per second)
/// - damping ratio is the contact bounciness with 1 being critical damping (non-dimensional)
/// - push velocity is the maximum contact constraint push out velocity (meters per second)
///	Advanced feature
BOX2D_API void b2World_SetContactTuning(b2WorldId worldId, float hertz, float dampingRatio, float pushVelocity);

/// Get the current profile
BOX2D_API struct b2Profile b2World_GetProfile(b2WorldId worldId);

/// Get counters and sizes
BOX2D_API struct b2Statistics b2World_GetStatistics(b2WorldId worldId);

/** @} */

/**
 * \defgroup BodyAPI Bodies
 * This is the body API.
 * @{
 */

BOX2D_API b2BodyType b2Body_GetType(b2BodyId bodyId);
BOX2D_API void b2Body_SetType(b2BodyId bodyId, b2BodyType type);

/// Get the user data stored in a body
BOX2D_API void* b2Body_GetUserData(b2BodyId bodyId);

BOX2D_API b2Vec2 b2Body_GetPosition(b2BodyId bodyId);
BOX2D_API float b2Body_GetAngle(b2BodyId bodyId);
BOX2D_API b2Transform b2Body_GetTransform(b2BodyId bodyId);
BOX2D_API void b2Body_SetTransform(b2BodyId bodyId, b2Vec2 position, float angle);

BOX2D_API b2Vec2 b2Body_GetLocalPoint(b2BodyId bodyId, b2Vec2 globalPoint);
BOX2D_API b2Vec2 b2Body_GetWorldPoint(b2BodyId bodyId, b2Vec2 localPoint);

BOX2D_API b2Vec2 b2Body_GetLocalVector(b2BodyId bodyId, b2Vec2 globalVector);
BOX2D_API b2Vec2 b2Body_GetWorldVector(b2BodyId bodyId, b2Vec2 localVector);

BOX2D_API b2Vec2 b2Body_GetLinearVelocity(b2BodyId bodyId);
BOX2D_API float b2Body_GetAngularVelocity(b2BodyId bodyId);
BOX2D_API void b2Body_SetLinearVelocity(b2BodyId bodyId, b2Vec2 linearVelocity);
BOX2D_API void b2Body_SetAngularVelocity(b2BodyId bodyId, float angularVelocity);

/// Apply a force at a world point. If the force is not
/// applied at the center of mass, it will generate a torque and
/// affect the angular velocity. This wakes up the body.
/// @param force the world force vector, usually in Newtons (N).
/// @param point the world position of the point of application.
/// @param wake also wake up the body
BOX2D_API void b2Body_ApplyForce(b2BodyId bodyId, b2Vec2 force, b2Vec2 point, bool wake);

/// Apply a force to the center of mass. This wakes up the body.
/// @param force the world force vector, usually in Newtons (N).
/// @param wake also wake up the body
BOX2D_API void b2Body_ApplyForceToCenter(b2BodyId bodyId, b2Vec2 force, bool wake);

/// Apply a torque. This affects the angular velocity
/// without affecting the linear velocity of the center of mass.
/// @param torque about the z-axis (out of the screen), usually in N-m.
/// @param wake also wake up the body
BOX2D_API void b2Body_ApplyTorque(b2BodyId bodyId, float torque, bool wake);

/// Apply an impulse at a point. This immediately modifies the velocity.
/// It also modifies the angular velocity if the point of application
/// is not at the center of mass. This wakes up the body.
/// @param impulse the world impulse vector, usually in N-seconds or kg-m/s.
/// @param point the world position of the point of application.
/// @param wake also wake up the body
BOX2D_API void b2Body_ApplyLinearImpulse(b2BodyId bodyId, b2Vec2 impulse, b2Vec2 point, bool wake);

/// Apply an impulse to the center of mass. This immediately modifies the velocity.
/// @param impulse the world impulse vector, usually in N-seconds or kg-m/s.
/// @param wake also wake up the body
BOX2D_API void b2Body_ApplyLinearImpulseToCenter(b2BodyId bodyId, b2Vec2 impulse, bool wake);

/// Apply an angular impulse.
/// @param impulse the angular impulse in units of kg*m*m/s
/// @param wake also wake up the body
BOX2D_API void b2Body_ApplyAngularImpulse(b2BodyId bodyId, float impulse, bool wake);

/// Get the mass of the body (kilograms)
BOX2D_API float b2Body_GetMass(b2BodyId bodyId);

/// Get the inertia tensor of the body. In 2D this is a single number. (kilograms * meters^2)
BOX2D_API float b2Body_GetInertiaTensor(b2BodyId bodyId);

/// Get the center of mass position of the body in local space.
BOX2D_API b2Vec2 b2Body_GetLocalCenterOfMass(b2BodyId bodyId);

/// Get the center of mass position of the body in world space.
BOX2D_API b2Vec2 b2Body_GetWorldCenterOfMass(b2BodyId bodyId);

/// Override the body's mass properties. Normally this is computed automatically using the
///	shape geometry and density. This information is lost if a shape is added or removed or if the
///	body type changes.
BOX2D_API void b2Body_SetMassData(b2BodyId bodyId, b2MassData massData);

/// Is this body awake?
BOX2D_API bool b2Body_IsAwake(b2BodyId bodyId);

/// Wake a body from sleep. This wakes the entire island the body is touching.
BOX2D_API void b2Body_Wake(b2BodyId bodyId);

/// Is this body enabled?
BOX2D_API bool b2Body_IsEnabled(b2BodyId bodyId);

/// Disable a body by removing it completely from the simulation
BOX2D_API void b2Body_Disable(b2BodyId bodyId);

/// Enable a body by adding it to the simulation
BOX2D_API void b2Body_Enable(b2BodyId bodyId);

/// Create a shape and attach it to a body. The shape defintion and geometry are fully cloned.
/// Contacts are not created until the next time step.
/// @warning This function is locked during callbacks.
///	@return the shape id for accessing the shape
BOX2D_API b2ShapeId b2Body_CreateCircle(b2BodyId bodyId, const b2ShapeDef* def, const b2Circle* circle);
BOX2D_API b2ShapeId b2Body_CreateSegment(b2BodyId bodyId, const b2ShapeDef* def, const b2Segment* segment);
BOX2D_API b2ShapeId b2Body_CreateCapsule(b2BodyId bodyId, const b2ShapeDef* def, const b2Capsule* capsule);
BOX2D_API b2ShapeId b2Body_CreatePolygon(b2BodyId bodyId, const b2ShapeDef* def, const b2Polygon* polygon);
BOX2D_API void b2Body_DestroyShape(b2ShapeId shapeId);

BOX2D_API b2ChainId b2Body_CreateChain(b2BodyId bodyId, const b2ChainDef* def);
BOX2D_API void b2Body_DestroyChain(b2ChainId chainId);

/// Iterate over shapes on a body
BOX2D_API b2ShapeId b2Body_GetFirstShape(b2BodyId bodyId);
BOX2D_API b2ShapeId b2Body_GetNextShape(b2ShapeId shapeId);

/// Get the maximum capacity required for retrieving all the touching contacts on a body
BOX2D_API int32_t b2Body_GetContactCapacity(b2BodyId bodyId);

/// Get the touching contact data for a body
BOX2D_API int32_t b2Body_GetContactData(b2BodyId bodyId, b2ContactData* contactData, int32_t capacity);

/** @} */

/**
 * \defgroup ShapeAPI Shapes
 * This is the shape API.
 * @{
 */

BOX2D_API b2BodyId b2Shape_GetBody(b2ShapeId shapeId);
BOX2D_API void* b2Shape_GetUserData(b2ShapeId shapeId);
BOX2D_API bool b2Shape_TestPoint(b2ShapeId shapeId, b2Vec2 point);
BOX2D_API void b2Shape_SetFriction(b2ShapeId shapeId, float friction);
BOX2D_API void b2Shape_SetRestitution(b2ShapeId shapeId, float restitution);

BOX2D_API b2ShapeType b2Shape_GetType(b2ShapeId shapeId);
BOX2D_API const b2Circle* b2Shape_GetCircle(b2ShapeId shapeId);
BOX2D_API const b2Segment* b2Shape_GetSegment(b2ShapeId shapeId);
BOX2D_API const b2Polygon* b2Shape_GetSmoothSegment(b2ShapeId shapeId);
BOX2D_API const b2Capsule* b2Shape_GetCapsule(b2ShapeId shapeId);
BOX2D_API const b2Polygon* b2Shape_GetPolygon(b2ShapeId shapeId);

BOX2D_API void b2Chain_SetFriction(b2ChainId chainId, float friction);
BOX2D_API void b2Chain_SetRestitution(b2ChainId chainId, float restitution);

/// Get the maximum capacity required for retrieving all the touching contacts on a shape
BOX2D_API int32_t b2Shape_GetContactCapacity(b2ShapeId shapeId);

/// Get the touching contact data for a shape. The provided shapeId will be either shapeIdA or shapeIdB on the contact data.
BOX2D_API int32_t b2Shape_GetContactData(b2ShapeId shapeId, b2ContactData* contactData, int32_t capacity);

/** @} */

/**
 * \defgroup JointAPI Joints
 * This is the joint API.
 * @{
 */

/// Generic joint access
BOX2D_API b2BodyId b2Joint_GetBodyA(b2JointId jointId);
BOX2D_API b2BodyId b2Joint_GetBodyB(b2JointId jointId);

/// Distance joint access
BOX2D_API float b2DistanceJoint_GetConstraintForce(b2JointId jointId, float timeStep);
BOX2D_API void b2DistanceJoint_SetLength(b2JointId jointId, float length, float minLength, float maxLength);
BOX2D_API float b2DistanceJoint_GetCurrentLength(b2JointId jointId);
BOX2D_API void b2DistanceJoint_SetTuning(b2JointId jointId, float hertz, float dampingRatio);

/// Motor joint access
BOX2D_API void b2MotorJoint_SetLinearOffset(b2JointId jointId, b2Vec2 linearOffset);
BOX2D_API void b2MotorJoint_SetAngularOffset(b2JointId jointId, float angularOffset);
BOX2D_API void b2MotorJoint_SetMaxForce(b2JointId jointId, float maxForce);
BOX2D_API void b2MotorJoint_SetMaxTorque(b2JointId jointId, float maxTorque);
BOX2D_API void b2MotorJoint_SetCorrectionFactor(b2JointId jointId, float correctionFactor);
BOX2D_API b2Vec2 b2MotorJoint_GetConstraintForce(b2JointId jointId, float inverseTimeStep);
BOX2D_API float b2MotorJoint_GetConstraintTorque(b2JointId jointId, float inverseTimeStep);

/// Mouse joint access
BOX2D_API void b2MouseJoint_SetTarget(b2JointId jointId, b2Vec2 target);

// Prismatic joint access
BOX2D_API void b2PrismaticJoint_EnableLimit(b2JointId jointId, bool enableLimit);
BOX2D_API void b2PrismaticJoint_EnableMotor(b2JointId jointId, bool enableMotor);
BOX2D_API void b2PrismaticJoint_SetMotorSpeed(b2JointId jointId, float motorSpeed);
BOX2D_API float b2PrismaticJoint_GetMotorForce(b2JointId jointId, float inverseTimeStep);
BOX2D_API void b2PrismaticJoint_SetMaxMotorForce(b2JointId jointId, float force);
BOX2D_API b2Vec2 b2PrismaticJoint_GetConstraintForce(b2JointId jointId, float inverseTimeStep);
BOX2D_API float b2PrismaticJoint_GetConstraintTorque(b2JointId jointId, float inverseTimeStep);

// Revolute joint access
BOX2D_API void b2RevoluteJoint_EnableLimit(b2JointId jointId, bool enableLimit);
BOX2D_API void b2RevoluteJoint_EnableMotor(b2JointId jointId, bool enableMotor);
BOX2D_API void b2RevoluteJoint_SetMotorSpeed(b2JointId jointId, float motorSpeed);
BOX2D_API float b2RevoluteJoint_GetMotorTorque(b2JointId jointId, float inverseTimeStep);
BOX2D_API void b2RevoluteJoint_SetMaxMotorTorque(b2JointId jointId, float torque);
BOX2D_API b2Vec2 b2RevoluteJoint_GetConstraintForce(b2JointId jointId, float inverseTimeStep);
BOX2D_API float b2RevoluteJoint_GetConstraintTorque(b2JointId jointId, float inverseTimeStep);

// Wheel joint access
BOX2D_API void b2WheelJoint_SetStiffness(b2JointId jointId, float stiffness);
BOX2D_API void b2WheelJoint_SetDamping(b2JointId jointId, float damping);
BOX2D_API void b2WheelJoint_EnableLimit(b2JointId jointId, bool enableLimit);
BOX2D_API void b2WheelJoint_EnableMotor(b2JointId jointId, bool enableMotor);
BOX2D_API void b2WheelJoint_SetMotorSpeed(b2JointId jointId, float motorSpeed);
BOX2D_API float b2WheelJoint_GetMotorTorque(b2JointId jointId, float inverseTimeStep);
BOX2D_API void b2WheelJoint_SetMaxMotorTorque(b2JointId jointId, float torque);
BOX2D_API b2Vec2 b2WheelJoint_GetConstraintForce(b2JointId jointId, float inverseTimeStep);
BOX2D_API float b2WheelJoint_GetConstraintTorque(b2JointId jointId, float inverseTimeStep);

/** @} */
