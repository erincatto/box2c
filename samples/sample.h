// SPDX-FileCopyrightText: 2022 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "box2d/types.h"
#include "draw.h"

#include <stdlib.h>

struct Settings;
class Test;

#define	RAND_LIMIT 32767

/// Random number in range [-1,1]
inline float RandomFloat()
{
	float r = (float)(rand() & (RAND_LIMIT));
	r /= RAND_LIMIT;
	r = 2.0f * r - 1.0f;
	return r;
}

/// Random floating point number in range [lo, hi]
inline float RandomFloat(float lo, float hi)
{
	float r = (float)(rand() & (RAND_LIMIT));
	r /= RAND_LIMIT;
	r = (hi - lo) * r + lo;
	return r;
}

#if 0
// This is called when a joint in the world is implicitly destroyed
// because an attached body is destroyed. This gives us a chance to
// nullify the mouse joint.
class DestructionListener : public b2DestructionListener
{
public:
	void SayGoodbye(b2Fixture* fixture) override { B2_MAYBE_UNUSED(fixture); }
	void SayGoodbye(b2Joint* joint) override;

	Test* test;
};

const int32_t k_maxContactPoints = 2048;

struct ContactPoint
{
	b2Fixture* fixtureA;
	b2Fixture* fixtureB;
	b2Vec2 normal;
	b2Vec2 position;
	b2PointState state;
	float normalImpulse;
	float tangentImpulse;
	float separation;
};
#endif

struct b2World;
struct b2Body;
struct b2Contact;
struct b2MouseJoint;

class Sample
{
public:

	Sample();
	virtual ~Sample();

	void DrawTitle(const char* string);
	virtual void Step(Settings& settings);
	virtual void UpdateUI() {}
	virtual void Keyboard(int) { }
	virtual void KeyboardUp(int) { }
	virtual void MouseDown(b2Vec2 p, int button, int mod);
	virtual void MouseUp(b2Vec2 p, int button);
	virtual void MouseMove(b2Vec2 p);

	void ShiftOrigin(b2Vec2 newOrigin);

protected:
	friend class DestructionListener;
	friend class BoundaryListener;
	friend class ContactListener;

	b2Body* m_groundBody;
	b2AABB m_worldAABB;
	//ContactPoint m_points[k_maxContactPoints];
	int32_t m_pointCount;
	//DestructionListener m_destructionListener;
	int32_t m_textLine;
	b2World* m_world;
	b2MouseJoint* m_mouseJoint;
	int32_t m_stepCount;
	int32_t m_textIncrement;
	//b2Profile m_maxProfile;
	//b2Profile m_totalProfile;
};

typedef Sample* SampleCreateFcn();

int RegisterSample(const char* category, const char* name, SampleCreateFcn* fcn);

struct SampleEntry
{
	const char* category;
	const char* name;
	SampleCreateFcn* createFcn;
};

#define MAX_SAMPLES 256
extern SampleEntry g_sampleEntries[MAX_SAMPLES];
extern int g_sampleCount;
