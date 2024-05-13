// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "api.h"
#include "callbacks.h"
#include "event_types.h"
#include "geometry.h"
#include "id.h"
#include "joint_types.h"
#include "types.h"

#include <stdbool.h>

typedef struct b2Capsule b2Capsule;
typedef struct b2Circle b2Circle;
typedef struct b2DebugDraw b2DebugDraw;
typedef struct b2Polygon b2Polygon;
typedef struct b2Segment b2Segment;

/**
 * @defgroup world World
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

/// World identifier validation. Provides validation for up to 64K allocations.
B2_API bool b2World_IsValid(b2WorldId id);

/// Simulate a world for one time step. This performs collision detection, integration, and constraint solution.
/// @param worldId the world to simulate
/// @param timeStep the amount of time to simulate, this should be a fixed number. Typically 1/60.
/// @param subStepCount the number of sub-steps, increasing the sub-step count can increase accuracy. Typically 4.
B2_API void b2World_Step(b2WorldId worldId, float timeStep, int subStepCount);

/// Call this to draw shapes and other debug draw data. This is intentionally non-const.
B2_API void b2World_Draw(b2WorldId worldId, b2DebugDraw* draw);

/// Get the body events for the current time step. The event data is transient. Do not store a reference to this data.
B2_API b2BodyEvents b2World_GetBodyEvents(b2WorldId worldId);

/// Get sensor events for the current time step. The event data is transient. Do not store a reference to this data.
B2_API b2SensorEvents b2World_GetSensorEvents(b2WorldId worldId);

/// Get contact events for this current time step. The event data is transient. Do not store a reference to this data.
B2_API b2ContactEvents b2World_GetContactEvents(b2WorldId worldId);

/// Overlap test for all shapes that *potentially* overlap the provided AABB.
B2_API void b2World_OverlapAABB(b2WorldId worldId, b2AABB aabb, b2QueryFilter filter, b2OverlapResultFcn* fcn, void* context);

/// Overlap test for for all shapes that overlap the provided circle.
B2_API void b2World_OverlapCircle(b2WorldId worldId, const b2Circle* circle, b2Transform transform, b2QueryFilter filter,
								  b2OverlapResultFcn* fcn, void* context);

/// Overlap test for all shapes that overlap the provided capsule.
B2_API void b2World_OverlapCapsule(b2WorldId worldId, const b2Capsule* capsule, b2Transform transform, b2QueryFilter filter,
								   b2OverlapResultFcn* fcn, void* context);

/// Overlap test for all shapes that overlap the provided polygon.
B2_API void b2World_OverlapPolygon(b2WorldId worldId, const b2Polygon* polygon, b2Transform transform, b2QueryFilter filter,
								   b2OverlapResultFcn* fcn, void* context);

/// Ray-cast the world for all shapes in the path of the ray. Your callback
/// controls whether you get the closest point, any point, or n-points.
/// The ray-cast ignores shapes that contain the starting point.
/// @param callback a user implemented callback class.
/// @param point1 the ray starting point
/// @param point2 the ray ending point
B2_API void b2World_RayCast(b2WorldId worldId, b2Vec2 origin, b2Vec2 translation, b2QueryFilter filter, b2CastResultFcn* fcn,
							   void* context);

/// Ray-cast closest hit. Convenience function. This is less general than b2World_RayCast and does not allow for custom filtering.
B2_API b2RayResult b2World_RayCastClosest(b2WorldId worldId, b2Vec2 origin, b2Vec2 translation, b2QueryFilter filter);

/// Cast a circle through the world. Similar to a ray-cast except that a circle is cast instead of a point.
B2_API void b2World_CircleCast(b2WorldId worldId, const b2Circle* circle, b2Transform originTransform, b2Vec2 translation,
								  b2QueryFilter filter, b2CastResultFcn* fcn, void* context);

/// Cast a capsule through the world. Similar to a ray-cast except that a capsule is cast instead of a point.
B2_API void b2World_CapsuleCast(b2WorldId worldId, const b2Capsule* capsule, b2Transform originTransform, b2Vec2 translation,
								   b2QueryFilter filter, b2CastResultFcn* fcn, void* context);

/// Cast a capsule through the world. Similar to a ray-cast except that a polygon is cast instead of a point.
B2_API void b2World_PolygonCast(b2WorldId worldId, const b2Polygon* polygon, b2Transform originTransform, b2Vec2 translation,
								   b2QueryFilter filter, b2CastResultFcn* fcn, void* context);

/// Enable/disable sleep. Advanced feature for testing.
B2_API void b2World_EnableSleeping(b2WorldId worldId, bool flag);

/// Enable/disable constraint warm starting. Advanced feature for testing.
B2_API void b2World_EnableWarmStarting(b2WorldId worldId, bool flag);

/// Enable/disable continuous collision. Advanced feature for testing.
B2_API void b2World_EnableContinuous(b2WorldId worldId, bool flag);

/// Adjust the restitution threshold. Advanced feature for testing.
B2_API void b2World_SetRestitutionThreshold(b2WorldId worldId, float value);

/// Adjust the hit event threshold. Advanced feature for testing.
B2_API void b2World_SetHitEventThreshold(b2WorldId worldId, float value);

/// Register the pre-solve callback. This is optional.
B2_API void b2World_SetPreSolveCallback(b2WorldId worldId, b2PreSolveFcn* fcn, void* context);

/// Set the gravity vector for the entire world. Typically in m/s^2
B2_API void b2World_SetGravity(b2WorldId worldId, b2Vec2 gravity);

/// @return the gravity vector
B2_API b2Vec2 b2World_GetGravity(b2WorldId worldId);

/// Apply explosion
B2_API void b2World_Explode(b2WorldId worldId, b2Vec2 position, float radius, float impulse);

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

/// Dump memory stats to box2d_memory.txt
B2_API void b2World_DumpMemoryStats(b2WorldId worldId);

/** @} */

/**
 * @defgroup body Body
 * This is the body API.
 * @{
 */

/// Create a rigid body given a definition. No reference to the definition is retained.
/// @warning This function is locked during callbacks.
B2_API b2BodyId b2CreateBody(b2WorldId worldId, const b2BodyDef* def);

/// Destroy a rigid body given an id. This destroys all shapes and joints attached to the body.
///	Do not keep references to the associated shapes and joints.
/// @warning This function is locked during callbacks.
B2_API void b2DestroyBody(b2BodyId bodyId);

/// Body identifier validation. Provides validation for up to 64K allocations.
B2_API bool b2Body_IsValid(b2BodyId id);

/// Get the body type: static, kinematic, or dynamic
B2_API b2BodyType b2Body_GetType(b2BodyId bodyId);

/// Change the body type. This is an expensive operation. This automatically updates the mass
///	properties regardless of the automatic mass setting.
B2_API void b2Body_SetType(b2BodyId bodyId, b2BodyType type);

/// Set the user data for a body
B2_API void b2Body_SetUserData(b2BodyId bodyId, void* userData);

/// Get the user data stored in a body
B2_API void* b2Body_GetUserData(b2BodyId bodyId);

/// Get the world position of a body. This is the location of the body origin.
B2_API b2Vec2 b2Body_GetPosition(b2BodyId bodyId);

/// Get the world rotation of a body as a sine/cosine pair.
B2_API b2Rot b2Body_GetRotation(b2BodyId bodyId);

/// Get the body angle in radians in the range [-pi, pi]
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
/// This should be used for one-shot impulses. If you need a steady force,
/// use a force instead, which will work better with the sub-stepping solver.
/// @param impulse the world impulse vector, usually in N-seconds or kg-m/s.
/// @param point the world position of the point of application.
/// @param wake also wake up the body
B2_API void b2Body_ApplyLinearImpulse(b2BodyId bodyId, b2Vec2 impulse, b2Vec2 point, bool wake);

/// Apply an impulse to the center of mass. This immediately modifies the velocity.
/// This should be used for one-shot impulses. If you need a steady force,
/// use a force instead, which will work better with the sub-stepping solver.
/// @param impulse the world impulse vector, usually in N-seconds or kg-m/s.
/// @param wake also wake up the body
B2_API void b2Body_ApplyLinearImpulseToCenter(b2BodyId bodyId, b2Vec2 impulse, bool wake);

/// Apply an angular impulse.
/// This should be used for one-shot impulses. If you need a steady force,
/// use a force instead, which will work better with the sub-stepping solver.
/// @param impulse the angular impulse in units of
/// kg*m*m/s
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

/// This resets the mass properties to the sum of the mass properties of the shapes.
/// This normally does not need to be called unless you called SetMassData to override
/// the mass and you later want to reset the mass.
///	You may also use this when automatic mass computation has been disabled.
///	You should call this regardless of body type.
B2_API void b2Body_ApplyMassFromShapes(b2BodyId bodyId);

/// Set the automatic mass setting.
B2_API void b2Body_SetAutomaticMass(b2BodyId bodyId, bool automaticMass);

/// Get the automatic mass setting.
B2_API bool b2Body_GetAutomaticMass(b2BodyId bodyId);

/// Adjust the linear damping. Normally this is set in b2BodyDef before creation.
B2_API void b2Body_SetLinearDamping(b2BodyId bodyId, float linearDamping);

/// Get the current linear damping.
B2_API float b2Body_GetLinearDamping(b2BodyId bodyId);

/// Adjust the angular damping. Normally this is set in b2BodyDef before creation.
B2_API void b2Body_SetAngularDamping(b2BodyId bodyId, float angularDamping);

/// Get the current angular damping.
B2_API float b2Body_GetAngularDamping(b2BodyId bodyId);

/// Adjust the gravity scale. Normally this is set in b2BodyDef before creation.
B2_API void b2Body_SetGravityScale(b2BodyId bodyId, float gravityScale);

/// Get the current gravity scale.
B2_API float b2Body_GetGravityScale(b2BodyId bodyId);

/// Is this body awake?
B2_API bool b2Body_IsAwake(b2BodyId bodyId);

/// Wake a body from sleep. This wakes the entire island the body is touching.
///	Putting a body to sleep will put the entire island of bodies touching this body to sleep,
///	which can be expensive.
B2_API void b2Body_SetAwake(b2BodyId bodyId, bool awake);

/// Enable or disable sleeping this body. If sleeping is disabled the body will wake.
B2_API void b2Body_EnableSleep(b2BodyId bodyId, bool enableSleep);

/// @return is sleeping enabled for this body?
B2_API bool b2Body_IsSleepEnabled(b2BodyId bodyId);

/// Set the sleep threshold. Normally in meters per second.
B2_API void b2Body_SetSleepThreshold(b2BodyId bodyId, float sleepVelocity);

/// Get the sleep threshold. Normally in meters per second.
B2_API float b2Body_GetSleepThreshold(b2BodyId bodyId);

/// Is this body enabled?
B2_API bool b2Body_IsEnabled(b2BodyId bodyId);

/// Disable a body by removing it completely from the simulation
B2_API void b2Body_Disable(b2BodyId bodyId);

/// Enable a body by adding it to the simulation
B2_API void b2Body_Enable(b2BodyId bodyId);

/// Set this body to have fixed rotation. This causes the mass to be reset in all cases.
B2_API void b2Body_SetFixedRotation(b2BodyId bodyId, bool flag);

/// Does this body have fixed rotation?
B2_API bool b2Body_IsFixedRotation(b2BodyId bodyId);

/// Set this body to be a bullet. A bullet does continuous collision detection
/// against dynamic bodies (but not other bullets).
B2_API void b2Body_SetBullet(b2BodyId bodyId, bool flag);

/// Is this body a bullet?
B2_API bool b2Body_IsBullet(b2BodyId bodyId);

/// Enable/disable hit events on all shapes.
B2_API void b2Body_EnableHitEvents(b2BodyId bodyId, bool enableHitEvents);

/// Get the number of shapes on this body
B2_API int b2Body_GetShapeCount(b2BodyId bodyId);

/// Get the shape ids for all shapes on this body, up to the provided capacity.
///	@returns the number of shape ids stored in the user array
B2_API int b2Body_GetShapes(b2BodyId bodyId, b2ShapeId* shapeArray, int capacity);

/// Get the number of joints on this body
B2_API int b2Body_GetJointCount(b2BodyId bodyId);

/// Get the joint ids for all joints on this body, up to the provided capacity
///	@returns the number of joint ids stored in the user array
B2_API int b2Body_GetJoints(b2BodyId bodyId, b2JointId* jointArray, int capacity);

/// Get the maximum capacity required for retrieving all the touching contacts on a body
B2_API int b2Body_GetContactCapacity(b2BodyId bodyId);

/// Get the touching contact data for a body
B2_API int b2Body_GetContactData(b2BodyId bodyId, b2ContactData* contactData, int capacity);

/// Get the current world AABB that contains all the attached shapes. Note that this may not emcompass the body origin.
///	If there are no shapes attached then the returned AABB is empty and centered on the body origin.
B2_API b2AABB b2Body_ComputeAABB(b2BodyId bodyId);

/** @} */

/**
 * @defgroup shape Shape
 * This is the shape API. Shapes bind raw geometry to bodies and hold material properties including
 * friction and restitution.
 * @{
 */

/// Create a circle shape and attach it to a body. The shape definition and geometry are fully cloned.
/// Contacts are not created until the next time step.
///	@return the shape id for accessing the shape
B2_API b2ShapeId b2CreateCircleShape(b2BodyId bodyId, const b2ShapeDef* def, const b2Circle* circle);

/// Create a line segment shape and attach it to a body. The shape definition and geometry are fully cloned.
/// Contacts are not created until the next time step.
///	@return the shape id for accessing the shape
B2_API b2ShapeId b2CreateSegmentShape(b2BodyId bodyId, const b2ShapeDef* def, const b2Segment* segment);

/// Create a capsule shape and attach it to a body. The shape definition and geometry are fully cloned.
/// Contacts are not created until the next time step.
///	@return the shape id for accessing the shape
B2_API b2ShapeId b2CreateCapsuleShape(b2BodyId bodyId, const b2ShapeDef* def, const b2Capsule* capsule);

/// Create a polygon shape and attach it to a body. The shape definition and geometry are fully cloned.
/// Contacts are not created until the next time step.
///	@return the shape id for accessing the shape
B2_API b2ShapeId b2CreatePolygonShape(b2BodyId bodyId, const b2ShapeDef* def, const b2Polygon* polygon);

/// Destroy any shape type
B2_API void b2DestroyShape(b2ShapeId shapeId);

/// Shape identifier validation. Provides validation for up to 64K allocations.
B2_API bool b2Shape_IsValid(b2ShapeId id);

/// Get the type of a shape.
B2_API b2ShapeType b2Shape_GetType(b2ShapeId shapeId);

/// Get the body that a shape is attached to
B2_API b2BodyId b2Shape_GetBody(b2ShapeId shapeId);

/// Is this shape a sensor? See b2ShapeDef.
B2_API bool b2Shape_IsSensor(b2ShapeId shapeId);

/// Set the user data for a shape.
B2_API void b2Shape_SetUserData(b2ShapeId shapeId, void* userData);

/// Get the user data for a shape. This is useful when you get a shape id
///	from an event or query.
B2_API void* b2Shape_GetUserData(b2ShapeId shapeId);

/// Set the density on a shape. Normally this is specified in b2ShapeDef.
///	This will not update the mass properties on the parent body until you
/// call b2Body_ResetMassData.
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

/// Enable sensor events for this shape. Only applies to kinematic and dynamic bodies. Ignored for sensors.
B2_API void b2Shape_EnableSensorEvents(b2ShapeId shapeId, bool flag);

/// @return are sensor events enabled?
B2_API bool b2Shape_AreSensorEventsEnabled(b2ShapeId shapeId);

/// Enable contact events for this shape. Only applies to kinematic and dynamic bodies. Ignored for sensors.
B2_API void b2Shape_EnableContactEvents(b2ShapeId shapeId, bool flag);

/// @return are contact events enabled?
B2_API bool b2Shape_AreContactEventsEnabled(b2ShapeId shapeId);

/// Enable pre-solve contact events for this shape. Only applies to dynamic bodies. These are expensive
///	and must be carefully handled due to multi-threading. Ignored for sensors.
B2_API void b2Shape_EnablePreSolveEvents(b2ShapeId shapeId, bool flag);

/// @return are pre-solve events enabled?
B2_API bool b2Shape_ArePreSolveEventsEnabled(b2ShapeId shapeId);

/// Enable contact hit events for this shape. Ignored for sensors.
///	@see b2WorldDef.hitEventThreshold
B2_API void b2Shape_EnableHitEvents(b2ShapeId shapeId, bool flag);

/// @return are hit events enabled?
B2_API bool b2Shape_AreHitEventsEnabled(b2ShapeId shapeId);

/// Test a point for overlap with a shape
B2_API bool b2Shape_TestPoint(b2ShapeId shapeId, b2Vec2 point);

/// Ray cast a shape directly
B2_API b2CastOutput b2Shape_RayCast(b2ShapeId shapeId, b2Vec2 origin, b2Vec2 translation);

/// Access the circle geometry of a shape. Asserts the type is correct.
B2_API b2Circle b2Shape_GetCircle(b2ShapeId shapeId);

/// Access the line segment geometry of a shape. Asserts the type is correct.
B2_API b2Segment b2Shape_GetSegment(b2ShapeId shapeId);

/// Access the smooth line segment geometry of a shape. These come from chain shapes.
/// Asserts the type is correct.
B2_API b2SmoothSegment b2Shape_GetSmoothSegment(b2ShapeId shapeId);

/// Access the capsule geometry of a shape. Asserts the type is correct.
B2_API b2Capsule b2Shape_GetCapsule(b2ShapeId shapeId);

/// Access the convex polygon geometry of a shape. Asserts the type is correct.
B2_API b2Polygon b2Shape_GetPolygon(b2ShapeId shapeId);

/// Allows you to change a shape to be a circle or update the current circle.
/// This does not modify the mass properties.
B2_API void b2Shape_SetCircle(b2ShapeId shapeId, const b2Circle* circle);

/// Allows you to change a shape to be a capsule or update the current capsule.
/// This does not modify the mass properties.
B2_API void b2Shape_SetCapsule(b2ShapeId shapeId, const b2Capsule* capsule);

/// Allows you to change a shape to be a segment or update the current segment.
/// This does not modify the mass properties.
B2_API void b2Shape_SetSegment(b2ShapeId shapeId, const b2Segment* segment);

/// Allows you to change a shape to be a segment or update the current segment.
/// This does not modify the mass properties.
B2_API void b2Shape_SetPolygon(b2ShapeId shapeId, const b2Polygon* polygon);

/// If the type is b2_smoothSegmentShape then you can get the parent chain id.
/// If the shape is not a smooth segment then this will return b2_nullChainId.
B2_API b2ChainId b2Shape_GetParentChain(b2ShapeId shapeId);

/// Get the maximum capacity required for retrieving all the touching contacts on a shape
B2_API int b2Shape_GetContactCapacity(b2ShapeId shapeId);

/// Get the touching contact data for a shape. The provided shapeId will be either shapeIdA or shapeIdB on the contact data.
B2_API int b2Shape_GetContactData(b2ShapeId shapeId, b2ContactData* contactData, int capacity);

/// Get the current world AABB
B2_API b2AABB b2Shape_GetAABB(b2ShapeId shapeId);

/// Get the closest point on a shape to a target point. Target and result are in world space.
B2_API b2Vec2 b2Shape_GetClosestPoint(b2ShapeId shapeId, b2Vec2 target);

/// Chain Shape

/// Create a chain shape
///	@see b2ChainDef for details
B2_API b2ChainId b2CreateChain(b2BodyId bodyId, const b2ChainDef* def);

/// Destroy a chain shape
B2_API void b2DestroyChain(b2ChainId chainId);

/// Set the friction of a chain. Normally this is set in b2ChainDef.
B2_API void b2Chain_SetFriction(b2ChainId chainId, float friction);

/// Set the restitution (bounciness) on a chain. Normally this is specified in b2ChainDef.
B2_API void b2Chain_SetRestitution(b2ChainId chainId, float restitution);

/// Chain identifier validation. Provides validation for up to 64K allocations.
B2_API bool b2Chain_IsValid(b2ChainId id);

/** @} */

/**
 * @defgroup joint Joint
 * @brief Joints allow you to connect rigid bodies together while allowing various forms of relative motions.
 * @{
 */

/// Destroy any joint type
B2_API void b2DestroyJoint(b2JointId jointId);

/// Joint identifier validation. Provides validation for up to 64K allocations.
B2_API bool b2Joint_IsValid(b2JointId id);

/// Get the joint type
B2_API b2JointType b2Joint_GetType(b2JointId jointId);

/// Get body A on a joint
B2_API b2BodyId b2Joint_GetBodyA(b2JointId jointId);

/// Get body B on a joint
B2_API b2BodyId b2Joint_GetBodyB(b2JointId jointId);

/// Get local anchor on bodyA
B2_API b2Vec2 b2Joint_GetLocalAnchorA(b2JointId jointId);

/// Get local anchor on bodyB
B2_API b2Vec2 b2Joint_GetLocalAnchorB(b2JointId jointId);

/// Toggle collision between connected bodies
B2_API void b2Joint_SetCollideConnected(b2JointId jointId, bool shouldCollide);

/// Is collision allowed between connected bodies?
B2_API bool b2Joint_GetCollideConnected(b2JointId jointId);

/// Set the user data on a joint
B2_API void b2Joint_SetUserData(b2JointId jointId, void* userData);

/// Get the user data on a joint
B2_API void* b2Joint_GetUserData(b2JointId jointId);

/// Wake the bodies connect to this joint
B2_API void b2Joint_WakeBodies(b2JointId jointId);

/**
 * @defgroup distance_joint Distance Joint
 * @brief Functions for the distance joint.
 * @{
 */

/// Create a distance joint
///	@see b2DistanceJointDef for details
B2_API b2JointId b2CreateDistanceJoint(b2WorldId worldId, const b2DistanceJointDef* def);

/// @return The constraint force on a distance joint.
B2_API float b2DistanceJoint_GetConstraintForce(b2JointId jointId, float timeStep);

/// @brief Set the rest length of a distance joint.
/// @param[in] jointId The id for a distance joint.
/// @param[in] length The new distance joint length.
B2_API void b2DistanceJoint_SetLength(b2JointId jointId, float length);

/// Get the rest length of a distance joint
B2_API float b2DistanceJoint_GetLength(b2JointId jointId);

/// Enable/disable the distance joint spring. When disabled the distance joint is rigid.
B2_API void b2DistanceJoint_EnableSpring(b2JointId jointId, bool enableSpring);

B2_API bool b2DistanceJoint_IsSpringEnabled(b2JointId jointId);

/// Set the spring stiffness in Hertz
B2_API void b2DistanceJoint_SetSpringHertz(b2JointId jointId, float hertz);

/// Set the spring damping ratio, non-dimensional
B2_API void b2DistanceJoint_SetSpringDampingRatio(b2JointId jointId, float dampingRatio);

/// @return the spring Hertz
B2_API float b2DistanceJoint_GetHertz(b2JointId jointId);

/// @return the spring damping ratio
B2_API float b2DistanceJoint_GetDampingRatio(b2JointId jointId);

/// Enable joint limit. The limit only works if the joint spring is enabled. Otherwise the joint is rigid
///	and the limit has no effect.
B2_API void b2DistanceJoint_EnableLimit(b2JointId jointId, bool enableLimit);

B2_API bool b2DistanceJoint_IsLimitEnabled(b2JointId jointId);

/// Set the minimum and maximum length parameters of a distance joint
B2_API void b2DistanceJoint_SetLengthRange(b2JointId jointId, float minLength, float maxLength);

/// @return the minimum distance joint length
B2_API float b2DistanceJoint_GetMinLength(b2JointId jointId);

/// Get the maximum distance joint length
B2_API float b2DistanceJoint_GetMaxLength(b2JointId jointId);

/// Get the current length of a distance joint
B2_API float b2DistanceJoint_GetCurrentLength(b2JointId jointId);

B2_API void b2DistanceJoint_EnableMotor(b2JointId jointId, bool enableMotor);

B2_API bool b2DistanceJoint_IsMotorEnabled(b2JointId jointId);

B2_API void b2DistanceJoint_SetMotorSpeed(b2JointId jointId, float motorSpeed);

B2_API float b2DistanceJoint_GetMotorSpeed(b2JointId jointId);

B2_API float b2DistanceJoint_GetMotorForce(b2JointId jointId);

B2_API void b2DistanceJoint_SetMaxMotorForce(b2JointId jointId, float force);

B2_API float b2DistanceJoint_GetMaxMotorForce(b2JointId jointId);

/** @} */

/**
 * @defgroup motor_joint Motor Joint
 * @brief Functions for the motor joint.
 * @{
 */

/// Create a motor joint
///	@see b2MotorJointDef for details
B2_API b2JointId b2CreateMotorJoint(b2WorldId worldId, const b2MotorJointDef* def);

/// Set/Get the linear offset target for a motor joint
B2_API void b2MotorJoint_SetLinearOffset(b2JointId jointId, b2Vec2 linearOffset);

/// @return the linear offset target for a motor joint
B2_API b2Vec2 b2MotorJoint_GetLinearOffset(b2JointId jointId);

/// Set the angular offset target for a motor joint in radians
B2_API void b2MotorJoint_SetAngularOffset(b2JointId jointId, float angularOffset);

/// @return the angular offset target for a motor joint in radians
B2_API float b2MotorJoint_GetAngularOffset(b2JointId jointId);

/// Set the maximum force for a motor joint
B2_API void b2MotorJoint_SetMaxForce(b2JointId jointId, float maxForce);

/// @return the maximum force for a motor joint
B2_API float b2MotorJoint_GetMaxForce(b2JointId jointId);

/// Set the maximum torque for a motor joint
B2_API void b2MotorJoint_SetMaxTorque(b2JointId jointId, float maxTorque);

/// @return the maximum torque for a motor joint
B2_API float b2MotorJoint_GetMaxTorque(b2JointId jointId);

/// Set the correction factor for a motor joint
B2_API void b2MotorJoint_SetCorrectionFactor(b2JointId jointId, float correctionFactor);

/// @return the correction factor for a motor joint
B2_API float b2MotorJoint_GetCorrectionFactor(b2JointId jointId);

/// Get the current constraint force for a motor joint
B2_API b2Vec2 b2MotorJoint_GetConstraintForce(b2JointId jointId);

/// Get the current constraint torque for a motor joint
B2_API float b2MotorJoint_GetConstraintTorque(b2JointId jointId);

/** @} */


/**
 * @defgroup mouse_joint Mouse Joint
 * @brief Functions for the mouse joint.
 * 
 * The mouse joint is designed for use in the samples application, but you may find it useful in applications where
 * the user moves a rigid body with a cursor.
 * @{
 */

/// Create a mouse joint
///	@see b2MouseJointDef for details
B2_API b2JointId b2CreateMouseJoint(b2WorldId worldId, const b2MouseJointDef* def);

/// Set the target for a mouse joint
B2_API void b2MouseJoint_SetTarget(b2JointId jointId, b2Vec2 target);

/// @return the target for a mouse joint
B2_API b2Vec2 b2MouseJoint_GetTarget(b2JointId jointId);

/// Set the spring stiffness in Hertz
B2_API void b2MouseJoint_SetSpringHertz(b2JointId jointId, float hertz);

/// Set the spring damping ratio (non-dimensional)
B2_API void b2MouseJoint_SetSpringDampingRatio(b2JointId jointId, float dampingRatio);

/// Get the Hertz of a mouse joint
B2_API float b2MouseJoint_GetHertz(b2JointId jointId);

/// Get the damping ratio of a mouse joint
B2_API float b2MouseJoint_GetDampingRatio(b2JointId jointId);

/** @} */

/**
 * @defgroup prismatic_joint Prismatic Joint
 * @brief A prismatic joint allows for translation along a single axis with no rotation.
 * 
 * The prismatic joint is useful for things like pistons and moving platforms, where you want a body to translate
 * along an axis and have no rotation. Also called a *slider* joint.
 * @{
 */

/// Create a prismatic (slider) joint.
///	@see b2PrismaticJointDef for details
B2_API b2JointId b2CreatePrismaticJoint(b2WorldId worldId, const b2PrismaticJointDef* def);

/// Enable/disable the joint spring.
B2_API void b2PrismaticJoint_EnableSpring(b2JointId jointId, bool enableSpring);

/// @brief Is the spring enabled or not?
/// @param jointId 
/// @return true if the spring is enable, other-wise false.
B2_API bool b2PrismaticJoint_IsSpringEnabled(b2JointId jointId);

/// Set the joint stiffness in Hertz.
/// This should usually be less than a quarter of the simulation rate. For example, if the simulation
/// runs at 60Hz then the joint stiffness should be 15Hz or less.
B2_API void b2PrismaticJoint_SetSpringHertz(b2JointId jointId, float hertz);

/// Get the joint stiffness in Hertz
B2_API float b2PrismaticJoint_GetSpringHertz(b2JointId jointId);

/// Set the joint damping ratio (non-dimensional)
B2_API void b2PrismaticJoint_SetSpringDampingRatio(b2JointId jointId, float dampingRatio);

/// Get the spring damping ratio (non-dimensional)
B2_API float b2PrismaticJoint_GetSpringDampingRatio(b2JointId jointId);

/// Enable/disable a prismatic joint limit
B2_API void b2PrismaticJoint_EnableLimit(b2JointId jointId, bool enableLimit);

/// @return is the prismatic joint limit enabled
B2_API bool b2PrismaticJoint_IsLimitEnabled(b2JointId jointId);

/// Get the lower joint limit in length units (meters).
B2_API float b2PrismaticJoint_GetLowerLimit(b2JointId jointId);

/// Get the upper joint limit in length units (meters).
B2_API float b2PrismaticJoint_GetUpperLimit(b2JointId jointId);

/// Set the joint limits in length units (meters).
B2_API void b2PrismaticJoint_SetLimits(b2JointId jointId, float lower, float upper);

/// Enable/disable a prismatic joint motor
B2_API void b2PrismaticJoint_EnableMotor(b2JointId jointId, bool enableMotor);

/// @return is the prismatic joint motor enabled
B2_API bool b2PrismaticJoint_IsMotorEnabled(b2JointId jointId);

/// Set the motor speed for a prismatic joint
B2_API void b2PrismaticJoint_SetMotorSpeed(b2JointId jointId, float motorSpeed);

/// @return the motor speed for a prismatic joint
B2_API float b2PrismaticJoint_GetMotorSpeed(b2JointId jointId);

/// Get the current motor force for a prismatic joint
B2_API float b2PrismaticJoint_GetMotorForce(b2JointId jointId);

/// Set the maximum force for a prismatic joint motor
B2_API void b2PrismaticJoint_SetMaxMotorForce(b2JointId jointId, float force);

/// @return the maximum force for a prismatic joint motor
B2_API float b2PrismaticJoint_GetMaxMotorForce(b2JointId jointId);

/// Get the current constraint force for a prismatic joint
B2_API b2Vec2 b2PrismaticJoint_GetConstraintForce(b2JointId jointId);

/// Get the current constraint torque for a prismatic joint
B2_API float b2PrismaticJoint_GetConstraintTorque(b2JointId jointId);

/** @} */

/**
 * @defgroup revolute_joint Revolute Joint
 * @brief A revolute joint allows for rotation in the 2D plane with no relative translation.
 * 
 * The revolute joint is probably the most common joint. It can be used for ragdolls and chains. Also called a *hinge* joint.
 * @{
 */

/// Create a revolute (hinge) joint
///	@see b2RevoluteJointDef for details
B2_API b2JointId b2CreateRevoluteJoint(b2WorldId worldId, const b2RevoluteJointDef* def);

/// Enable/disable the joint spring.
B2_API void b2RevoluteJoint_EnableSpring(b2JointId jointId, bool enableSpring);

B2_API bool b2RevoluteJoint_IsLimitEnabled(b2JointId jointId);

/// Set the joint stiffness in Hertz
B2_API void b2RevoluteJoint_SetSpringHertz(b2JointId jointId, float hertz);

/// @return the joint stiffness in Hertz
B2_API float b2RevoluteJoint_GetSpringHertz(b2JointId jointId);

/// Set the joint damping ratio (non-dimensional)
B2_API void b2RevoluteJoint_SetSpringDampingRatio(b2JointId jointId, float dampingRatio);

/// @return the  joint damping ratio (non-dimensional)
B2_API float b2RevoluteJoint_GetSpringDampingRatio(b2JointId jointId);

/// Get the current joint angle in radians relative to the reference angle
B2_API float b2RevoluteJoint_GetAngle(b2JointId jointId);

/// Enable/disable a revolute joint limit.
B2_API void b2RevoluteJoint_EnableLimit(b2JointId jointId, bool enableLimit);

/// @return is the revolute joint limit enabled
B2_API bool b2RevoluteJoint_IsLimitEnabled(b2JointId jointId);

/// Get the lower joint limit in radians.
B2_API float b2RevoluteJoint_GetLowerLimit(b2JointId jointId);

/// Get the upper joint limit in radians.
B2_API float b2RevoluteJoint_GetUpperLimit(b2JointId jointId);

/// Set the joint limits in radians.
B2_API void b2RevoluteJoint_SetLimits(b2JointId jointId, float lower, float upper);

/// Enable/disable a revolute joint motor.
B2_API void b2RevoluteJoint_EnableMotor(b2JointId jointId, bool enableMotor);

/// @return is the revolute joint motor enabled
B2_API bool b2RevoluteJoint_IsMotorEnabled(b2JointId jointId);

/// Set the motor speed for a revolute joint in radians per second
B2_API void b2RevoluteJoint_SetMotorSpeed(b2JointId jointId, float motorSpeed);

/// @return the motor speed for a revolute joint in radians per second
B2_API float b2RevoluteJoint_GetMotorSpeed(b2JointId jointId);

/// Get the current motor torque for a revolute joint
B2_API float b2RevoluteJoint_GetMotorTorque(b2JointId jointId);

/// Set the maximum torque for a revolute joint motor
B2_API void b2RevoluteJoint_SetMaxMotorTorque(b2JointId jointId, float torque);

/// @return the maximum torque for a revolute joint motor
B2_API float b2RevoluteJoint_GetMaxMotorTorque(b2JointId jointId);

/// Get the current constraint force for a revolute joint
B2_API b2Vec2 b2RevoluteJoint_GetConstraintForce(b2JointId jointId);

/// Get the current constraint torque for a revolute joint
B2_API float b2RevoluteJoint_GetConstraintTorque(b2JointId jointId);

/** @} */

/**
 * @defgroup weld_joint Weld Joint
 * @brief todo
 * 
 * todo
 * @{
 */

/// Create a weld joint
///	@see b2WeldJointDef for details
B2_API b2JointId b2CreateWeldJoint(b2WorldId worldId, const b2WeldJointDef* def);

/// Set weld joint linear stiffness in Hertz. 0 is rigid.
B2_API void b2WeldJoint_SetLinearHertz(b2JointId jointId, float hertz);

/// @return the weld joint linear stiffness in Hertz.
B2_API float b2WeldJoint_GetLinearHertz(b2JointId jointId);

/// Set weld joint linear damping ratio (non-dimensional)
B2_API void b2WeldJoint_SetLinearDampingRatio(b2JointId jointId, float dampingRatio);

/// @return the weld joint linear damping ratio (non-dimensional)
B2_API float b2WeldJoint_GetLinearDampingRatio(b2JointId jointId);

/// Set weld joint angular stiffness in Hertz. 0 is rigid.
B2_API void b2WeldJoint_SetAngularHertz(b2JointId jointId, float hertz);

/// @return the weld joint angular stiffness in Hertz.
B2_API float b2WeldJoint_GetAngularHertz(b2JointId jointId);

/// Set weld joint angular damping ratio (non-dimensional)
B2_API void b2WeldJoint_SetAngularDampingRatio(b2JointId jointId, float dampingRatio);

/// @return the weld joint angular damping ratio (non-dimensional)
B2_API float b2WeldJoint_GetAngularDampingRatio(b2JointId jointId);

/** @} */

/**
 * @defgroup wheel_joint Wheel Joint
 * @brief todo
 * 
 * todo
 * @{
 */

/// Create a wheel joint
///	@see b2WheelJointDef for details
B2_API b2JointId b2CreateWheelJoint(b2WorldId worldId, const b2WheelJointDef* def);

B2_API void b2WheelJoint_EnableSpring(b2JointId jointId, bool enableSpring);

B2_API bool b2WheelJoint_IsSpringEnabled(b2JointId jointId);

/// Set the wheel joint stiffness in Hertz
B2_API void b2WheelJoint_SetSpringHertz(b2JointId jointId, float hertz);

/// @return the wheel joint stiffness in Hertz
B2_API float b2WheelJoint_GetSpringHertz(b2JointId jointId);

/// Set the wheel joint damping ratio (non-dimensional)
B2_API void b2WheelJoint_SetSpringDampingRatio(b2JointId jointId, float dampingRatio);

/// @return the wheel joint damping ratio (non-dimensional)
B2_API float b2WheelJoint_GetSpringDampingRatio(b2JointId jointId);

/// Enable/disable the wheel joint limit.
B2_API void b2WheelJoint_EnableLimit(b2JointId jointId, bool enableLimit);

/// @return is the wheel joint limit enabled
B2_API bool b2WheelJoint_IsLimitEnabled(b2JointId jointId);

/// Get the lower joint limit in length units (meters).
B2_API float b2WheelJoint_GetLowerLimit(b2JointId jointId);

/// Get the upper joint limit in length units (meters).
B2_API float b2WheelJoint_GetUpperLimit(b2JointId jointId);

/// Set the joint limits in length units (meters).
B2_API void b2WheelJoint_SetLimits(b2JointId jointId, float lower, float upper);

/// Enable/disable the wheel joint motor
B2_API void b2WheelJoint_EnableMotor(b2JointId jointId, bool enableMotor);

/// @return is the wheel joint motor enabled
B2_API bool b2WheelJoint_IsMotorEnabled(b2JointId jointId);

/// Set the wheel joint motor speed in radians per second
B2_API void b2WheelJoint_SetMotorSpeed(b2JointId jointId, float motorSpeed);

/// @return the wheel joint motor speed in radians per second
B2_API float b2WheelJoint_GetMotorSpeed(b2JointId jointId);

/// Get the wheel joint current motor torque
B2_API float b2WheelJoint_GetMotorTorque(b2JointId jointId);

/// Set the wheel joint maximum motor torque
B2_API void b2WheelJoint_SetMaxMotorTorque(b2JointId jointId, float torque);

/// @return the wheel joint maximum motor torque
B2_API float b2WheelJoint_GetMaxMotorTorque(b2JointId jointId);

/// Get the current wheel joint constraint force
B2_API b2Vec2 b2WheelJoint_GetConstraintForce(b2JointId jointId);

/// Get the current wheel joint constraint torque
B2_API float b2WheelJoint_GetConstraintTorque(b2JointId jointId);

/** @} */

/** @} */
