// SPDX-FileCopyrightText: 2022 Erin Catto
// SPDX-License-Identifier: MIT

#include "draw.h"
#include "human.h"
#include "sample.h"
#include "settings.h"

#include "box2d/box2d.h"
#include "box2d/math_functions.h"

#include <GLFW/glfw3.h>
#include <imgui.h>
#include <vector>

class DemoBase : public Sample
{
public:
	explicit DemoBase( Settings& settings )
		: Sample( settings )
	{
	}

	// position is top left
	void CreateTextBodies( b2Vec2 position, float scale, const char* text )
	{
		ImGuiIO& io = ImGui::GetIO();
		if ( io.Fonts->Fonts.size() == 0 )
		{
			return;
		}

		const ImFont* font = io.Fonts->Fonts.back();
		const unsigned char* pixels = font->ContainerAtlas->TexPixelsAlpha8;
		int width = font->ContainerAtlas->TexWidth;
		int height = font->ContainerAtlas->TexHeight;
		// int fontSize = font->Ascent;

		b2BodyDef bodyDef = b2DefaultBodyDef();
		bodyDef.type = b2_dynamicBody;
		bodyDef.gravityScale = 0.2f;
		bodyDef.isAwake = false;

		b2ShapeDef shapeDef = b2DefaultShapeDef();
		shapeDef.customColor = b2_colorAzure;

		int n = (int)strlen( text );
		float zoom = 1.0f;

		float lower = FLT_MAX;
		float upper = -FLT_MAX;

		float x = position.x;
		for ( int k = 0; k < n; ++k )
		{
			const ImFontGlyph* glyph = font->FindGlyph( text[k] );
			float x1 = glyph->X0;
			float x2 = glyph->X1;
			float y1 = glyph->Y0;
			float y2 = glyph->Y1;
			float u1 = glyph->U0;
			float v1 = glyph->V0;
			float u2 = glyph->U1;
			float v2 = glyph->V1;

			float w = zoom * ( x2 - x1 );
			float h = zoom * ( y2 - y1 );

			int gridx = int( w );
			int gridy = int( h );
			for ( int i = 0; i < gridy; ++i )
			{
				float v = v1 + i / h * ( v2 - v1 );
				int iy = int( v * height );

				for ( int j = 0; j < gridx; ++j )
				{
					float u = u1 + j / w * ( u2 - u1 );
					int ix = int( u * width );

					unsigned char value = pixels[iy * width + ix];
					if ( value > 50 )
					{
						b2Polygon square = b2MakeSquare( 0.9f * scale * value / 255.0f );
						bodyDef.position = { x + 2.0f * ( zoom * x1 + j ) * scale,
											 position.y - 2.0f * ( zoom * y1 + i ) * scale };
						b2BodyId bodyId = b2CreateBody( m_worldId, &bodyDef );
						b2CreatePolygonShape( bodyId, &shapeDef, &square );

						lower = b2MinFloat( lower, bodyDef.position.x );
						upper = b2MaxFloat( upper, bodyDef.position.x );
					}
				}
			}

			x += 2.0f * zoom * scale * glyph->AdvanceX;
		}

		printf( "lower = %g, upper = %g\n", lower, upper );
	}
};

#if 0
class Demo01 : public DemoBase
{
public:
	explicit Demo01( Settings& settings )
		: DemoBase( settings )
	{
		if ( settings.restart == false )
		{
			g_camera.m_center = { 0.0f, 270.0 };
			g_camera.m_zoom = 80.0f;
			g_draw.m_showUI = false;
			m_finished = false;
		}
		else
		{
			m_finished = true;
		}

		m_bridgeCenterJointId = b2_nullJointId;
		m_time = 0.0f;
		m_delay = 0.5f;

		CreateTextBodies( b2Vec2{ -73.0f, 300.0f }, 0.6f, "Box2D" );
		CreateTextBodies( b2Vec2{ -123.0f, 200.0f }, 0.6f, "Version 3.0" );
		CreateTextBodies( b2Vec2{ -105.0f, 100.0f }, 0.6f, "Released!" );
		CreateBridge();
		CreateBalloons();
	}

	void CreateBridge()
	{
		float h = 2.0f;
		b2Capsule capsule = { { -h, 0.0f }, { h, 0.0f }, 0.75f };

		b2ShapeDef shapeDef = b2DefaultShapeDef();
		shapeDef.density = 5.0f;
		shapeDef.customColor = b2_colorBlanchedAlmond;

		b2RevoluteJointDef jointDef = b2DefaultRevoluteJointDef();
		float frictionTorque = 100.0f;

		float xbase = -200.0f;
		float y = -100.0f;

		constexpr int count = 100;

		b2BodyId groundId1 = b2_nullBodyId;
		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			bodyDef.position = { -200.0f, y };
			groundId1 = b2CreateBody( m_worldId, &bodyDef );
		}

		b2BodyId groundId2 = b2_nullBodyId;
		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			bodyDef.position = { 200.0f, y };
			groundId2 = b2CreateBody( m_worldId, &bodyDef );
		}

		b2BodyId bodyIds[count];
		b2BodyDef bodyDef = b2DefaultBodyDef();
		bodyDef.type = b2_dynamicBody;
		// bodyDef.linearDamping = 0.1f;
		// bodyDef.angularDamping = 0.1f;
		// bodyDef.gravityScale = 0.5f;

		b2BodyId prevBodyId = groundId1;
		for ( int i = 0; i < count; ++i )
		{
			bodyDef.position = { xbase + ( 1.0f + 2.0f * i ) * h, y };
			bodyIds[i] = b2CreateBody( m_worldId, &bodyDef );
			b2CreateCapsuleShape( bodyIds[i], &shapeDef, &capsule );

			b2Vec2 pivot = { xbase + 2.0f * h * i, y };
			jointDef.bodyIdA = prevBodyId;
			jointDef.bodyIdB = bodyIds[i];
			jointDef.localAnchorA = b2Body_GetLocalPoint( jointDef.bodyIdA, pivot );
			jointDef.localAnchorB = b2Body_GetLocalPoint( jointDef.bodyIdB, pivot );
			jointDef.enableMotor = true;
			jointDef.maxMotorTorque = frictionTorque;
			b2JointId jointId = b2CreateRevoluteJoint( m_worldId, &jointDef );

			if ( i == count >> 1 )
			{
				m_bridgeCenterJointId = jointId;
			}

			prevBodyId = bodyIds[i];
		}

		b2Vec2 pivot = { xbase + 2.0f * h * count, y };
		jointDef.bodyIdA = prevBodyId;
		jointDef.bodyIdB = groundId2;
		jointDef.localAnchorA = b2Body_GetLocalPoint( jointDef.bodyIdA, pivot );
		jointDef.localAnchorB = b2Body_GetLocalPoint( jointDef.bodyIdB, pivot );
		jointDef.enableMotor = true;
		jointDef.maxMotorTorque = frictionTorque;
		b2CreateRevoluteJoint( m_worldId, &jointDef );
	}

	void CreateBalloons()
	{
		constexpr int count = 16;

		float radius = 2.0f;
		b2BodyDef bodyDef = b2DefaultBodyDef();
		bodyDef.type = b2_dynamicBody;
		// bodyDef.isAwake = false;

		b2ShapeDef shapeDef = b2DefaultShapeDef();
		shapeDef.friction = 0.1f;
		shapeDef.restitution = 1.0f;

		float h = 1.0f;
		b2Capsule mainCapsule = { { 0.0f, -0.1f }, { 0.0f, 0.1f }, radius };
		b2Capsule stringCapsule = { { 0.0f, -h }, { 0.0f, h }, 0.1f };

		b2WeldJointDef weldDef = b2DefaultWeldJointDef();
		weldDef.angularHertz = 2.0f;
		weldDef.angularDampingRatio = 0.7f;

		b2HexColor colors[4] = { b2_colorRed, b2_colorGreen, b2_colorYellow, b2_colorBlue };

		float y = -110.0f;
		for ( int j = 0; j < 20; ++j )
		{
			float x = -2.0f * radius * ( count - 1.0f );
			for ( int i = 0; i < count; ++i )
			{
				shapeDef.filter = b2DefaultFilter();
				shapeDef.customColor = colors[i % 4];
				bodyDef.gravityScale = -1.0f;
				bodyDef.linearDamping = 0.0f;
				bodyDef.position = { x, y };
				b2BodyId mainId = b2CreateBody( m_worldId, &bodyDef );
				b2CreateCapsuleShape( mainId, &shapeDef, &mainCapsule );

				shapeDef.filter = { 2, 1, 0 };
				shapeDef.customColor = b2_colorWhite;
				bodyDef.gravityScale = 0.5f;
				bodyDef.linearDamping = 0.5f;
				bodyDef.position = { x, y - radius - h };
				b2BodyId stringId1 = b2CreateBody( m_worldId, &bodyDef );
				b2CreateCapsuleShape( stringId1, &shapeDef, &stringCapsule );

				bodyDef.position = { x, y - radius - 3.0f * h };
				b2BodyId stringId2 = b2CreateBody( m_worldId, &bodyDef );
				b2CreateCapsuleShape( stringId2, &shapeDef, &stringCapsule );

				bodyDef.position = { x, y - radius - 5.0f * h };
				b2BodyId stringId3 = b2CreateBody( m_worldId, &bodyDef );
				b2CreateCapsuleShape( stringId3, &shapeDef, &stringCapsule );

				bodyDef.position = { x, y - radius - 7.0f * h };
				b2BodyId stringId4 = b2CreateBody( m_worldId, &bodyDef );
				b2CreateCapsuleShape( stringId4, &shapeDef, &stringCapsule );

				bodyDef.position = { x, y - radius - 9.0f * h };
				b2BodyId stringId5 = b2CreateBody( m_worldId, &bodyDef );
				b2CreateCapsuleShape( stringId5, &shapeDef, &stringCapsule );

				bodyDef.position = { x, y - radius - 11.0f * h };
				b2BodyId stringId6 = b2CreateBody( m_worldId, &bodyDef );
				b2CreateCapsuleShape( stringId6, &shapeDef, &stringCapsule );

				weldDef.bodyIdA = mainId;
				weldDef.bodyIdB = stringId1;
				weldDef.localAnchorA = { 0.0f, -radius };
				weldDef.localAnchorB = { 0.0f, h };
				b2CreateWeldJoint( m_worldId, &weldDef );

				weldDef.bodyIdA = stringId1;
				weldDef.bodyIdB = stringId2;
				weldDef.localAnchorA = { 0.0f, -h };
				weldDef.localAnchorB = { 0.0f, h };
				b2CreateWeldJoint( m_worldId, &weldDef );

				weldDef.bodyIdA = stringId2;
				weldDef.bodyIdB = stringId3;
				weldDef.localAnchorA = { 0.0f, -h };
				weldDef.localAnchorB = { 0.0f, h };
				b2CreateWeldJoint( m_worldId, &weldDef );

				weldDef.bodyIdA = stringId3;
				weldDef.bodyIdB = stringId4;
				weldDef.localAnchorA = { 0.0f, -h };
				weldDef.localAnchorB = { 0.0f, h };
				b2CreateWeldJoint( m_worldId, &weldDef );

				weldDef.bodyIdA = stringId4;
				weldDef.bodyIdB = stringId5;
				weldDef.localAnchorA = { 0.0f, -h };
				weldDef.localAnchorB = { 0.0f, h };
				b2CreateWeldJoint( m_worldId, &weldDef );

				weldDef.bodyIdA = stringId5;
				weldDef.bodyIdB = stringId6;
				weldDef.localAnchorA = { 0.0f, -h };
				weldDef.localAnchorB = { 0.0f, h };
				b2CreateWeldJoint( m_worldId, &weldDef );

				x += 4.0f * radius;
			}

			y -= 10.0f;
		}
	}

	// void UpdateUI() override
	//{
	//	float height = 100.0f;
	//	ImGui::SetNextWindowPos( ImVec2( 10.0f, g_camera.m_height - height - 50.0f ), ImGuiCond_Once );
	//	ImGui::SetNextWindowSize( ImVec2( 180.0f, height ) );

	//	ImGui::Begin( "Dj|^", nullptr, ImGuiWindowFlags_NoResize );
	//	ImGui::PushItemWidth( 100.0f );

	//	ImGui::PopItemWidth();
	//	ImGui::End();
	//}

	void Step( Settings& settings ) override
	{
		Sample::Step( settings );

		// g_draw.DrawCircle( b2Vec2{ -20.0f, 20.0f }, 0.5f, b2_colorRosyBrown );

		float timeStep = settings.hertz > 0.0f ? 1.0f / settings.hertz : 0.0f;
		m_time += timeStep;

		if ( m_time > 1.0f && m_finished == false)
		{
			if ( g_camera.m_center.y > 80.0f )
			{
				g_camera.m_center.y -= 40.0f * timeStep;
			}
			else if (m_delay > 0.0f)
			{
				m_delay -= timeStep;
			}
			else if ( g_camera.m_zoom < 300.0f )
			{
				g_camera.m_zoom += 40.0f * timeStep;
			}
			else if ( B2_IS_NON_NULL( m_bridgeCenterJointId ) )
			{
				b2DestroyJoint( m_bridgeCenterJointId );
				m_bridgeCenterJointId = b2_nullJointId;
			}
			else
			{
				m_finished = true;
			}
		}

		if ( glfwGetKey( g_mainWindow, GLFW_KEY_A ) == GLFW_PRESS && B2_IS_NON_NULL( m_bridgeCenterJointId ) )
		{
			b2DestroyJoint( m_bridgeCenterJointId );
			m_bridgeCenterJointId = b2_nullJointId;
		}
	}

	static Sample* Create( Settings& settings )
	{
		return new Demo01( settings );
	}

	b2JointId m_bridgeCenterJointId;
	float m_time;
	float m_delay;
	bool m_finished;
};

static int sampleDemo01 = RegisterSample( "Demo", "01", Demo01::Create );
#endif
