// SPDX-FileCopyrightText: 2022 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "box2d/types.h"
#include "box2d/manifold.h"

typedef struct b2Shape b2Shape;
typedef struct b2World b2World;

// A contact edge is used to connect bodies and contacts together
// in a contact graph where each body is a node and each contact
// is an edge. A contact edge belongs to a doubly linked list
// maintained in each attached body. Each contact has two contact
// nodes, one for each attached body.
typedef struct b2ContactEdge
{
	int32_t otherShapeIndex;
	struct b2Contact* contact;
	struct b2ContactEdge* prev;
	struct b2ContactEdge* next;
} b2ContactEdge;

// Flags stored in b2Contact::flags
enum b2ContactFlags
{
	// Used when crawling contact graph when forming islands.
	b2_contactIslandFlag = 0x0001,

	// Set when the shapes are touching.
	b2_contactTouchingFlag = 0x0002,

	// This contact can be disabled (by user)
	b2_contactEnabledFlag = 0x0004,

	// This contact needs filtering because a fixture filter was changed.
	b2_contactFilterFlag = 0x0008,
};

/// The class manages contact between two shapes. A contact exists for each overlapping
/// AABB in the broad-phase (except if filtered). Therefore a contact object may exist
/// that has no contact points.
typedef struct b2Contact
{

#if 0
	/// Get the contact manifold. Do not modify the manifold unless you understand the
	/// internals of Box2D.
	b2Manifold* GetManifold();
	const b2Manifold* GetManifold() const;

	/// Get the world manifold.
	void GetWorldManifold(b2WorldManifold* worldManifold) const;

	/// Is this contact touching?
	bool IsTouching() const;

	/// Enable/disable this contact. This can be used inside the pre-solve
	/// contact listener. The contact is only disabled for the current
	/// time step (or sub-step in continuous collisions).
	void SetEnabled(bool flag);

	/// Has this contact been disabled?
	bool IsEnabled() const;

	/// Get the next contact in the world's contact list.
	b2Contact* GetNext();
	const b2Contact* GetNext() const;

	/// Get fixture A in this contact.
	b2Fixture* GetFixtureA();
	const b2Fixture* GetFixtureA() const;

	/// Get the child primitive index for fixture A.
	int32 GetChildIndexA() const;

	/// Get fixture B in this contact.
	b2Fixture* GetFixtureB();
	const b2Fixture* GetFixtureB() const;

	/// Get the child primitive index for fixture B.
	int32 GetChildIndexB() const;

	/// Override the default friction mixture. You can call this in b2ContactListener::PreSolve.
	/// This value persists until set or reset.
	void SetFriction(float friction);

	/// Get the friction.
	float GetFriction() const;

	/// Reset the friction mixture to the default value.
	void ResetFriction();

	/// Override the default restitution mixture. You can call this in b2ContactListener::PreSolve.
	/// The value persists until you set or reset.
	void SetRestitution(float restitution);

	/// Get the restitution.
	float GetRestitution() const;

	/// Reset the restitution to the default value.
	void ResetRestitution();

	/// Override the default restitution velocity threshold mixture. You can call this in b2ContactListener::PreSolve.
	/// The value persists until you set or reset.
	void SetRestitutionThreshold(float threshold);

	/// Get the restitution threshold.
	float GetRestitutionThreshold() const;

	/// Reset the restitution threshold to the default value.
	void ResetRestitutionThreshold();

	/// Set the desired tangent speed for a conveyor belt behavior. In meters per second.
	void SetTangentSpeed(float speed);

	/// Get the desired tangent speed. In meters per second.
	float GetTangentSpeed() const;

	/// Evaluate this contact with your own manifold and transforms.
	virtual void Evaluate(b2Manifold* manifold, const b2Transform& xfA, const b2Transform& xfB) = 0;

protected:
	friend class b2ContactManager;
	friend class b2World;
	friend class b2ContactSolver;
	friend class b2Body;
	friend class b2Fixture;
#endif


	#if 0
	/// Flag this contact for filtering. Filtering will occur the next time step.
	void FlagForFiltering();

	static void AddType(b2ContactCreateFcn* createFcn, b2ContactDestroyFcn* destroyFcn,
						b2Shape::Type typeA, b2Shape::Type typeB);
	static void InitializeRegisters();
	static b2Contact* Create(b2Fixture* fixtureA, int32 indexA, b2Fixture* fixtureB, int32 indexB, b2BlockAllocator* allocator);
	static void Destroy(b2Contact* contact, b2Shape::Type typeA, b2Shape::Type typeB, b2BlockAllocator* allocator);
	static void Destroy(b2Contact* contact, b2BlockAllocator* allocator);

	b2Contact() : m_fixtureA(nullptr), m_fixtureB(nullptr) {}
	b2Contact(b2Fixture* fixtureA, int32 indexA, b2Fixture* fixtureB, int32 indexB);
	virtual ~b2Contact() {}

	void Update(b2ContactListener* listener);
	#endif


	uint32_t flags;

	struct b2Contact* prev;
	struct b2Contact* next;

	// Edges for connecting shapes (and thus bodies). These are the edges in the body-contact graph.
	b2ContactEdge edgeA;
	b2ContactEdge edgeB;

	int32_t shapeIndexA;
	int32_t shapeIndexB;

	int32_t childA;
	int32_t childB;

	b2Manifold manifold;

	// Mixed friction and restitution
	float friction;
	float restitution;

	// For conveyor belts
	float tangentSpeed;
} b2Contact;

void b2CreateContact(b2World* world, b2Shape* shapeA, int32_t childA, b2Shape* shapeB, int32_t childB);
void b2DestroyContact(b2World* world, b2Contact* contact);

#if 0
static inline void b2WorldManifold b2Contact_GetWorldManifold(b2Contact* contact)
{
	b2WorldManifold worldManifold;
	const b2Body* bodyA = m_fixtureA->GetBody();
	const b2Body* bodyB = m_fixtureB->GetBody();
	const b2Shape* shapeA = m_fixtureA->GetShape();
	const b2Shape* shapeB = m_fixtureB->GetShape();

	worldManifold->Initialize(&m_manifold, bodyA->GetTransform(), shapeA->m_radius, bodyB->GetTransform(), shapeB->m_radius);
}

inline void b2Contact::SetEnabled(bool flag)
{
	if (flag)
	{
		m_flags |= e_enabledFlag;
	}
	else
	{
		m_flags &= ~e_enabledFlag;
	}
}

inline bool b2Contact::IsEnabled() const
{
	return (m_flags & e_enabledFlag) == e_enabledFlag;
}

inline bool b2Contact::IsTouching() const
{
	return (m_flags & e_touchingFlag) == e_touchingFlag;
}

inline b2Contact* b2Contact::GetNext()
{
	return m_next;
}

inline const b2Contact* b2Contact::GetNext() const
{
	return m_next;
}

inline b2Fixture* b2Contact::GetFixtureA()
{
	return m_fixtureA;
}

inline const b2Fixture* b2Contact::GetFixtureA() const
{
	return m_fixtureA;
}

inline b2Fixture* b2Contact::GetFixtureB()
{
	return m_fixtureB;
}

inline int32 b2Contact::GetChildIndexA() const
{
	return m_indexA;
}

inline const b2Fixture* b2Contact::GetFixtureB() const
{
	return m_fixtureB;
}

inline int32 b2Contact::GetChildIndexB() const
{
	return m_indexB;
}

inline void b2Contact::FlagForFiltering()
{
	m_flags |= e_filterFlag;
}

inline void b2Contact::SetFriction(float friction)
{
	m_friction = friction;
}

inline float b2Contact::GetFriction() const
{
	return m_friction;
}

inline void b2Contact::ResetFriction()
{
	m_friction = b2MixFriction(m_fixtureA->m_friction, m_fixtureB->m_friction);
}

inline void b2Contact::SetRestitution(float restitution)
{
	m_restitution = restitution;
}

inline float b2Contact::GetRestitution() const
{
	return m_restitution;
}

inline void b2Contact::ResetRestitution()
{
	m_restitution = b2MixRestitution(m_fixtureA->m_restitution, m_fixtureB->m_restitution);
}

inline void b2Contact::SetRestitutionThreshold(float threshold)
{
	m_restitutionThreshold = threshold;
}

inline float b2Contact::GetRestitutionThreshold() const
{
	return m_restitutionThreshold;
}

inline void b2Contact::ResetRestitutionThreshold()
{
	m_restitutionThreshold = b2MixRestitutionThreshold(m_fixtureA->m_restitutionThreshold, m_fixtureB->m_restitutionThreshold);
}

inline void b2Contact::SetTangentSpeed(float speed)
{
	m_tangentSpeed = speed;
}

inline float b2Contact::GetTangentSpeed() const
{
	return m_tangentSpeed;
}

#endif
