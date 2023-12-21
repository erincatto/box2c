// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "human.h"

#include "box2d/box2d.h"
#include "box2d/math.h"

#include <assert.h>

Human::Human()
{
	for (int i = 0; i < Bone::e_count; ++i)
	{
		m_bones[i].bodyId = b2_nullBodyId;
		m_bones[i].jointId = b2_nullJointId;
		m_bones[i].parentIndex = -1;
	}
}

void Human::Spawn(b2WorldId worldId, b2Vec2 position, int groupIndex)
{
	m_worldId = worldId;

	for (int i = 0; i < Bone::e_count; ++i)
	{
		assert(B2_IS_NULL(m_bones[i].bodyId));
		assert(B2_IS_NULL(m_bones[i].jointId));
	}

	b2BodyDef bodyDef = b2_defaultBodyDef;
	bodyDef.type = b2_dynamicBody;

	b2ShapeDef shapeDef = b2_defaultShapeDef;
	shapeDef.filter.groupIndex = -groupIndex;

	// hip
	{
		Bone* bone = m_bones + Bone::e_hip;
		bone->parentIndex = -1;
		
		bodyDef.position = b2Add({0.0f, 0.95f}, position);
		bone->bodyId = b2World_CreateBody(m_worldId, &bodyDef);

		b2Capsule capsule = {{0.0f, -0.02f}, {0.0f, 0.02f}, 0.1f};
		b2Body_CreateCapsule(bone->bodyId, &shapeDef, &capsule);
	}

	// torso
	{
		Bone* bone = m_bones + Bone::e_torso;
		bone->parentIndex = Bone::e_hip;
		
		bodyDef.position = b2Add({0.0f, 1.2f}, position);
		//bodyDef.type = b2_staticBody;
		bone->bodyId = b2World_CreateBody(m_worldId, &bodyDef);

		bodyDef.type = b2_dynamicBody;

		b2Capsule capsule = {{0.0f, -0.135f}, {0.0f, 0.135f}, 0.09f};
		b2Body_CreateCapsule(bone->bodyId, &shapeDef, &capsule);

		b2Vec2 pivot = b2Add({0.0f, 1.0f}, position);
		b2RevoluteJointDef jointDef = b2DefaultRevoluteJointDef();
		jointDef.bodyIdA = m_bones[bone->parentIndex].bodyId;
		jointDef.bodyIdB = bone->bodyId;
		jointDef.localAnchorA = b2Body_GetLocalPoint(jointDef.bodyIdA, pivot);
		jointDef.localAnchorB = b2Body_GetLocalPoint(jointDef.bodyIdB, pivot);
		jointDef.enableLimit = true;
		jointDef.lowerAngle = -0.25f * b2_pi;
		jointDef.upperAngle = 0.0f;
		jointDef.enableMotor = true;
		jointDef.maxMotorTorque = 0.1f;
		jointDef.drawScale = 0.2f;

		bone->jointId = b2World_CreateRevoluteJoint(m_worldId, &jointDef);
	}

	// head
	{
		Bone* bone = m_bones + Bone::e_head;
		bone->parentIndex = Bone::e_torso;
		
		bodyDef.position = b2Add({0.02f, 1.5f}, position);
		bone->bodyId = b2World_CreateBody(m_worldId, &bodyDef);

		b2Capsule capsule = {{0.0f, -0.03f}, {0.0f, 0.03f}, 0.09f};
		b2Body_CreateCapsule(bone->bodyId, &shapeDef, &capsule);

		b2Vec2 pivot = b2Add({0.0f, 1.4f}, position);
		b2RevoluteJointDef jointDef = b2DefaultRevoluteJointDef();
		jointDef.bodyIdA = m_bones[bone->parentIndex].bodyId;
		jointDef.bodyIdB = bone->bodyId;
		jointDef.localAnchorA = b2Body_GetLocalPoint(jointDef.bodyIdA, pivot);
		jointDef.localAnchorB = b2Body_GetLocalPoint(jointDef.bodyIdB, pivot);
		jointDef.enableLimit = true;
		jointDef.lowerAngle = -0.25f * b2_pi;
		jointDef.upperAngle = 0.1f * b2_pi;
		jointDef.enableMotor = true;
		jointDef.maxMotorTorque = 0.1f;
		jointDef.drawScale = 0.2f;

		bone->jointId = b2World_CreateRevoluteJoint(m_worldId, &jointDef);
	}

	// upper left leg
	{
		Bone* bone = m_bones + Bone::e_upperLeftLeg;
		bone->parentIndex = Bone::e_hip;
		
		bodyDef.position = b2Add({0.0f, 0.775f}, position);
		bone->bodyId = b2World_CreateBody(m_worldId, &bodyDef);

		b2Capsule capsule = {{0.0f, -0.125f}, {0.0f, 0.125f}, 0.06f};
		b2Body_CreateCapsule(bone->bodyId, &shapeDef, &capsule);

		b2Vec2 pivot = b2Add({0.0f, 0.9f}, position);
		b2RevoluteJointDef jointDef = b2DefaultRevoluteJointDef();
		jointDef.bodyIdA = m_bones[bone->parentIndex].bodyId;
		jointDef.bodyIdB = bone->bodyId;
		jointDef.localAnchorA = b2Body_GetLocalPoint(jointDef.bodyIdA, pivot);
		jointDef.localAnchorB = b2Body_GetLocalPoint(jointDef.bodyIdB, pivot);
		jointDef.enableLimit = true;
		jointDef.lowerAngle = -0.05f * b2_pi;
		jointDef.upperAngle = 0.4f * b2_pi;
		jointDef.enableMotor = true;
		jointDef.maxMotorTorque = 0.1f;
		jointDef.drawScale = 0.2f;

		bone->jointId = b2World_CreateRevoluteJoint(m_worldId, &jointDef);
	}

	// lower left leg
	{
		Bone* bone = m_bones + Bone::e_lowerLeftLeg;
		bone->parentIndex = Bone::e_upperLeftLeg;
		
		bodyDef.position = b2Add({0.0f, 0.475f}, position);
		bone->bodyId = b2World_CreateBody(m_worldId, &bodyDef);

		b2Capsule capsule = {{0.0f, -0.125f}, {0.0f, 0.125f}, 0.05f};
		b2Body_CreateCapsule(bone->bodyId, &shapeDef, &capsule);

		b2Polygon box = b2MakeOffsetBox(0.1f, 0.03f, {0.05f, -0.175f}, 0.0f);
		b2Body_CreatePolygon(bone->bodyId, &shapeDef, &box);

		b2Vec2 pivot = b2Add({0.0f, 0.625f}, position);
		b2RevoluteJointDef jointDef = b2DefaultRevoluteJointDef();
		jointDef.bodyIdA = m_bones[bone->parentIndex].bodyId;
		jointDef.bodyIdB = bone->bodyId;
		jointDef.localAnchorA = b2Body_GetLocalPoint(jointDef.bodyIdA, pivot);
		jointDef.localAnchorB = b2Body_GetLocalPoint(jointDef.bodyIdB, pivot);
		jointDef.enableLimit = true;
		jointDef.lowerAngle = -0.5f * b2_pi;
		jointDef.upperAngle = -0.02f * b2_pi;
		jointDef.enableMotor = true;
		jointDef.maxMotorTorque = 0.1f;
		jointDef.drawScale = 0.2f;

		bone->jointId = b2World_CreateRevoluteJoint(m_worldId, &jointDef);
	}

	// upper left arm
	{
		Bone* bone = m_bones + Bone::e_upperLeftArm;
		bone->parentIndex = Bone::e_torso;

		bodyDef.position = b2Add({0.0f, 1.225f}, position);
		bone->bodyId = b2World_CreateBody(m_worldId, &bodyDef);

		b2Capsule capsule = {{0.0f, -0.125f}, {0.0f, 0.125f}, 0.035f};
		b2Body_CreateCapsule(bone->bodyId, &shapeDef, &capsule);

		b2Vec2 pivot = b2Add({0.0f, 1.4f}, position);
		b2RevoluteJointDef jointDef = b2DefaultRevoluteJointDef();
		jointDef.bodyIdA = m_bones[bone->parentIndex].bodyId;
		jointDef.bodyIdB = bone->bodyId;
		jointDef.localAnchorA = b2Body_GetLocalPoint(jointDef.bodyIdA, pivot);
		jointDef.localAnchorB = b2Body_GetLocalPoint(jointDef.bodyIdB, pivot);
		jointDef.enableLimit = true;
		jointDef.lowerAngle = -0.05f * b2_pi;
		jointDef.upperAngle = 0.25f * b2_pi;
		jointDef.enableMotor = true;
		jointDef.maxMotorTorque = 0.1f;
		jointDef.drawScale = 0.2f;

		bone->jointId = b2World_CreateRevoluteJoint(m_worldId, &jointDef);
	}

	// lower left arm
	{
		Bone* bone = m_bones + Bone::e_lowerLeftArm;
		bone->parentIndex = Bone::e_upperLeftArm;

		bodyDef.position = b2Add({0.0f, 0.975f}, position);
		bone->bodyId = b2World_CreateBody(m_worldId, &bodyDef);

		b2Capsule capsule = {{0.0f, -0.125f}, {0.0f, 0.125f}, 0.03f};
		b2Body_CreateCapsule(bone->bodyId, &shapeDef, &capsule);

		b2Vec2 pivot = b2Add({0.0f, 1.1f}, position);
		b2RevoluteJointDef jointDef = b2DefaultRevoluteJointDef();
		jointDef.bodyIdA = m_bones[bone->parentIndex].bodyId;
		jointDef.bodyIdB = bone->bodyId;
		jointDef.localAnchorA = b2Body_GetLocalPoint(jointDef.bodyIdA, pivot);
		jointDef.localAnchorB = b2Body_GetLocalPoint(jointDef.bodyIdB, pivot);
		jointDef.enableLimit = true;
		jointDef.lowerAngle = 0.01f * b2_pi;
		jointDef.upperAngle = 0.5f * b2_pi;
		jointDef.enableMotor = true;
		jointDef.maxMotorTorque = 0.1f;
		jointDef.drawScale = 0.2f;

		bone->jointId = b2World_CreateRevoluteJoint(m_worldId, &jointDef);
	}
}

void Human::Despawn()
{
	for (int i = 0; i < Bone::e_count; ++i)
	{
		if (B2_IS_NULL(m_bones[i].jointId))
		{
			continue;
		}

		b2World_DestroyJoint(m_bones[i].jointId);
		m_bones[i].jointId = b2_nullJointId;
	}

	for (int i = 0; i < Bone::e_count; ++i)
	{
		if (B2_IS_NULL(m_bones[i].bodyId))
		{
			continue;
		}

		b2World_DestroyBody(m_bones[i].bodyId);
		m_bones[i].bodyId = b2_nullBodyId;
	}
}
