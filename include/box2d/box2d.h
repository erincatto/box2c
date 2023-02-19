// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "box2d/api.h"
#include "box2d/id.h"
#include "box2d/types.h"

typedef struct b2Polygon b2Polygon;
typedef struct b2DebugDraw b2DebugDraw;
typedef struct b2DebugDraw b2DebugDraw;

/// Create a world for rigid body simulation. This contains all the bodies, shapes, and constraints.
BOX2D_API b2WorldId b2CreateWorld(const b2WorldDef* def);

/// Destroy a world.
BOX2D_API void b2DestroyWorld(b2WorldId worldId);

/// Create a rigid body given a definition. No reference to the definition is retained.
/// @warning This function is locked during callbacks.
BOX2D_API b2BodyId b2World_CreateBody(b2WorldId worldId, const b2BodyDef* def);

/// Destroy a rigid body given an id.
/// @warning This function is locked during callbacks.
BOX2D_API void b2World_DestroyBody(b2BodyId bodyId);

/// Take a time step. This performs collision detection, integration,
/// and constraint solution.
/// @param timeStep the amount of time to simulate, this should not vary.
/// @param velocityIterations for the velocity constraint solver.
/// @param positionIterations for the position constraint solver.
BOX2D_API void b2World_Step(b2WorldId worldId, float timeStep, int32_t velocityIterations, int32_t positionIterations);

/// Call this to draw shapes and other debug draw data. This is intentionally non-const.
BOX2D_API void b2World_Draw(b2WorldId worldId, b2DebugDraw* debugDraw);

/// Enable/disable sleep.
BOX2D_API void b2World_EnableSleeping(b2WorldId worldId, bool flag);

/// Get the current profile.
BOX2D_API struct b2Profile* b2World_GetProfile(b2WorldId worldId);

/// Create a shape and attach it to a body. Contacts are not created until the next time step.
/// @warning This function is locked during callbacks.
BOX2D_API b2ShapeId b2Body_CreatePolygon(b2BodyId bodyId, const b2ShapeDef* def, const b2Polygon* polygon);

BOX2D_API b2Vec2 b2Body_GetPosition(b2BodyId bodyId);
BOX2D_API float b2Body_GetAngle(b2BodyId bodyId);


BOX2D_API b2JointId b2World_CreateMouseJoint(b2WorldId worldId, const b2MouseJointDef* def);
BOX2D_API void b2MouseJoint_SetTarget(b2JointId jointId, b2Vec2 target);


#if 0
	/// Register a destruction listener. The listener is owned by you and must
/// remain in scope.
void SetDestructionListener(b2DestructionListener* listener);

/// Register a contact filter to provide specific control over collision.
/// Otherwise the default filter is used (b2_defaultFilter). The listener is
/// owned by you and must remain in scope.
void SetContactFilter(b2ContactFilter* filter);

/// Register a contact event listener. The listener is owned by you and must
/// remain in scope.
void SetContactListener(b2ContactListener* listener);

/// Create a joint to constrain bodies together. No reference to the definition
/// is retained. This may cause the connected bodies to cease colliding.
/// @warning This function is locked during callbacks.
b2Joint* CreateJoint(const b2JointDef* def);

/// Destroy a joint. This may cause the connected bodies to begin colliding.
/// @warning This function is locked during callbacks.
void DestroyJoint(b2Joint* joint);

/// Manually clear the force buffer on all bodies. By default, forces are cleared automatically
/// after each call to Step. The default behavior is modified by calling SetAutoClearForces.
/// The purpose of this function is to support sub-stepping. Sub-stepping is often used to maintain
/// a fixed sized time step under a variable frame-rate.
/// When you perform sub-stepping you will disable auto clearing of forces and instead call
/// ClearForces after all sub-steps are complete in one pass of your game loop.
/// @see SetAutoClearForces
void ClearForces();

/// Query the world for all fixtures that potentially overlap the
/// provided AABB.
/// @param callback a user implemented callback class.
/// @param aabb the query box.
void QueryAABB(b2QueryCallback* callback, const b2AABB& aabb) const;

/// Ray-cast the world for all fixtures in the path of the ray. Your callback
/// controls whether you get the closest point, any point, or n-points.
/// The ray-cast ignores shapes that contain the starting point.
/// @param callback a user implemented callback class.
/// @param point1 the ray starting point
/// @param point2 the ray ending point
void RayCast(b2RayCastCallback* callback, const b2Vec2& point1, const b2Vec2& point2) const;

/// Get the world body list. With the returned body, use b2Body::GetNext to get
/// the next body in the world list. A nullptr body indicates the end of the list.
/// @return the head of the world body list.
b2Body* GetBodyList();
const b2Body* GetBodyList() const;

/// Get the world joint list. With the returned joint, use b2Joint::GetNext to get
/// the next joint in the world list. A nullptr joint indicates the end of the list.
/// @return the head of the world joint list.
b2Joint* GetJointList();
const b2Joint* GetJointList() const;

/// Get the world contact list. With the returned contact, use b2Contact::GetNext to get
/// the next contact in the world list. A nullptr contact indicates the end of the list.
/// @return the head of the world contact list.
/// @warning contacts are created and destroyed in the middle of a time step.
/// Use b2ContactListener to avoid missing contacts.
b2Contact* GetContactList();
const b2Contact* GetContactList() const;


/// Enable/disable warm starting. For testing.
void SetWarmStarting(bool flag)
{
	m_warmStarting = flag;
}
bool GetWarmStarting() const
{
	return m_warmStarting;
}

	/// Get the number of broad-phase proxies.
	int32 GetProxyCount() const;

	/// Get the number of bodies.
	int32 GetBodyCount() const;

	/// Get the number of joints.
	int32 GetJointCount() const;

	/// Get the number of contacts (each may have 0 or more contact points).
	int32 GetContactCount() const;

	/// Get the height of the dynamic tree.
	int32 GetTreeHeight() const;

	/// Get the balance of the dynamic tree.
	int32 GetTreeBalance() const;

	/// Get the quality metric of the dynamic tree. The smaller the better.
	/// The minimum is 1.
	float GetTreeQuality() const;

	/// Change the global gravity vector.
	void SetGravity(const b2Vec2& gravity);

	/// Get the global gravity vector.
	b2Vec2 GetGravity() const;

	/// Is the world locked (in the middle of a time step).
	bool IsLocked() const;

	/// Set flag to control automatic clearing of forces after each time step.
	void SetAutoClearForces(bool flag);

	/// Get the flag that controls automatic clearing of forces after each time step.
	bool GetAutoClearForces() const;

	/// Shift the world origin. Useful for large worlds.
	/// The body shift formula is: position -= newOrigin
	/// @param newOrigin the new origin with respect to the old origin
	void ShiftOrigin(const b2Vec2& newOrigin);

	/// Get the contact manager for testing.
	const b2ContactManager& GetContactManager() const;


	/// Dump the world into the log file.
	/// @warning this should be called outside of a time step.
	void Dump();
#endif
