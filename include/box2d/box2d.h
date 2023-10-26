// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "box2d/api.h"
#include "box2d/id.h"
#include "box2d/joint_types.h"
#include "box2d/timer.h"
#include "box2d/types.h"

typedef struct b2Capsule b2Capsule;
typedef struct b2Circle b2Circle;
typedef struct b2Polygon b2Polygon;
typedef struct b2DebugDraw b2DebugDraw;
typedef struct b2Segment b2Segment;

/// Create a world for rigid body simulation. This contains all the bodies, shapes, and constraints.
BOX2D_API b2WorldId b2CreateWorld(const b2WorldDef* def);

/// Destroy a world.
BOX2D_API void b2DestroyWorld(b2WorldId worldId);

/// Take a time step. This performs collision detection, integration,
/// and constraint solution.
/// @param timeStep the amount of time to simulate, this should not vary.
/// @param velocityIterations for the velocity constraint solver.
/// @param positionIterations for the position constraint solver.
BOX2D_API void b2World_Step(b2WorldId worldId, float timeStep, int32_t velocityIterations, int32_t positionIterations);

/// Call this to draw shapes and other debug draw data. This is intentionally non-const.
BOX2D_API void b2World_Draw(b2WorldId worldId, b2DebugDraw* debugDraw);

/// Create a rigid body given a definition. No reference to the definition is retained.
/// @warning This function is locked during callbacks.
BOX2D_API b2BodyId b2World_CreateBody(b2WorldId worldId, const b2BodyDef* def);

/// Destroy a rigid body given an id.
/// @warning This function is locked during callbacks.
BOX2D_API void b2World_DestroyBody(b2BodyId bodyId);

BOX2D_API b2Vec2 b2Body_GetPosition(b2BodyId bodyId);
BOX2D_API float b2Body_GetAngle(b2BodyId bodyId);
BOX2D_API b2Vec2 b2Body_GetLocalPoint(b2BodyId bodyId, b2Vec2 globalPoint);

BOX2D_API void b2Body_SetTransform(b2BodyId bodyId, b2Vec2 position, float angle);
BOX2D_API void b2Body_SetLinearVelocity(b2BodyId bodyId, b2Vec2 linearVelocity);
BOX2D_API void b2Body_SetAngularVelocity(b2BodyId bodyId, float angularVelocity);

BOX2D_API b2BodyType b2Body_GetType(b2BodyId bodyId);
BOX2D_API float b2Body_GetMass(b2BodyId bodyId);
BOX2D_API void b2Body_Wake(b2BodyId bodyId);

/// Create a shape and attach it to a body. Contacts are not created until the next time step.
/// @warning This function is locked during callbacks.
BOX2D_API b2ShapeId b2Body_CreateCircle(b2BodyId bodyId, const b2ShapeDef* def, const b2Circle* circle);
BOX2D_API b2ShapeId b2Body_CreateSegment(b2BodyId bodyId, const b2ShapeDef* def, const b2Segment* segment);
BOX2D_API b2ShapeId b2Body_CreateCapsule(b2BodyId bodyId, const b2ShapeDef* def, const b2Capsule* capsule);
BOX2D_API b2ShapeId b2Body_CreatePolygon(b2BodyId bodyId, const b2ShapeDef* def, const b2Polygon* polygon);

BOX2D_API b2BodyId b2Shape_GetBody(b2ShapeId shapeId);
BOX2D_API bool b2Shape_TestPoint(b2ShapeId shapeId, b2Vec2 point);

BOX2D_API b2JointId b2World_CreateMouseJoint(b2WorldId worldId, const b2MouseJointDef* def);
BOX2D_API b2JointId b2World_CreateRevoluteJoint(b2WorldId worldId, const b2RevoluteJointDef* def);
BOX2D_API b2JointId b2World_CreateWeldJoint(b2WorldId worldId, const b2WeldJointDef* def);
BOX2D_API void b2World_DestroyJoint(b2JointId jointId);

BOX2D_API b2BodyId b2Joint_GetBodyA(b2JointId jointId);
BOX2D_API b2BodyId b2Joint_GetBodyB(b2JointId jointId);

BOX2D_API void b2MouseJoint_SetTarget(b2JointId jointId, b2Vec2 target);

BOX2D_API void b2RevoluteJoint_EnableLimit(b2JointId jointId, bool enableLimit);
BOX2D_API void b2RevoluteJoint_EnableMotor(b2JointId jointId, bool enableMotor);
BOX2D_API void b2RevoluteJoint_SetMotorSpeed(b2JointId jointId, float motorSpeed);
BOX2D_API float b2RevoluteJoint_GetMotorTorque(b2JointId jointId, float inverseTimeStep);
BOX2D_API void b2RevoluteJoint_SetMaxMotorTorque(b2JointId jointId, float torque);
BOX2D_API b2Vec2 b2RevoluteJoint_GetConstraintForce(b2JointId jointId);

	/// This function receives shapes found in the AABB query.
/// @return true if the query should continue
typedef bool b2QueryCallbackFcn(b2ShapeId shapeId, void* context);

/// Query the world for all shapse that potentially overlap the provided AABB.
/// @param callback a user implemented callback function.
/// @param aabb the query box.
BOX2D_API void b2World_QueryAABB(b2WorldId worldId, b2AABB aabb, b2QueryCallbackFcn* fcn, void* context);


/// Advanced API for testing and special cases

/// Enable/disable sleep.
BOX2D_API void b2World_EnableSleeping(b2WorldId worldId, bool flag);

/// Enable/disable contact warm starting. Improves stacking stability.
BOX2D_API void b2World_EnableWarmStarting(b2WorldId worldId, bool flag);

/// Enable/disable continuous collision.
BOX2D_API void b2World_EnableContinuous(b2WorldId worldId, bool flag);

/// Adjust the restitution threshold
BOX2D_API void b2World_SetRestitutionThreshold(b2WorldId worldId, float value);

/// Adjust the maximum contact constraint push out velocity
BOX2D_API void b2World_SetMaximumPushoutVelocity(b2WorldId worldId, float value);

/// Adjust the contact stiffness in cycles per second.
BOX2D_API void b2World_SetContactHertz(b2WorldId worldId, float value);

/// Get the current profile
BOX2D_API struct b2Profile b2World_GetProfile(b2WorldId worldId);

/// Get counters and sizes
BOX2D_API struct b2Statistics b2World_GetStatistics(b2WorldId worldId);
