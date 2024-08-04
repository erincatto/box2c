// SPDX-FileCopyrightText: 2022 Erin Catto
// SPDX-License-Identifier: MIT

#include "draw.h"
#include "donut.h"
#include "human.h"
#include "sample.h"
#include "settings.h"

#include "box2d/box2d.h"
#include "box2d/math_functions.h"

#include <GLFW/glfw3.h>
#include <imgui.h>
#include <stdio.h>

static inline float Square( float x )
{
	return x * x;
}

static inline float EaseInOutQuad( float x )
{
	return x < 0.5f ? 2.0f * x * x : 1.0f - 0.5f * Square( -2.0f * x + 2.0f );
}

class DemoBase : public Sample
{
public:
	explicit DemoBase( Settings& settings )
		: Sample( settings )
	{
	}

	// position is top left
	void CreateTextBodies( b2Vec2 position, float scale, float gravityScale, const char* text, b2HexColor color )
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
		bodyDef.gravityScale = gravityScale;
		bodyDef.isAwake = false;
		// bodyDef.isBullet = true;

		b2ShapeDef shapeDef = b2DefaultShapeDef();
		shapeDef.customColor = color;

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

		//float newX = position.x - 0.5f * ( upper - lower ) - lower;
		//printf( "lower = %g, upper = %g, new x = %g\n", lower, upper, newX );
	}

	b2BodyId CreateTextBody( b2Vec2 position, float scale, float gravityScale, const char* text, b2HexColor color )
	{
		ImGuiIO& io = ImGui::GetIO();
		if ( io.Fonts->Fonts.size() == 0 )
		{
			return b2_nullBodyId;
		}

		const ImFont* font = io.Fonts->Fonts.front();
		const unsigned char* pixels = font->ContainerAtlas->TexPixelsAlpha8;
		int width = font->ContainerAtlas->TexWidth;
		int height = font->ContainerAtlas->TexHeight;

		b2BodyDef bodyDef = b2DefaultBodyDef();
		bodyDef.type = b2_dynamicBody;
		bodyDef.gravityScale = gravityScale;
		b2BodyId bodyId = b2CreateBody( m_worldId, &bodyDef );

		b2ShapeDef shapeDef = b2DefaultShapeDef();
		shapeDef.customColor = color;

		int n = (int)strlen( text );

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

			float w = x2 - x1;
			float h = y2 - y1;

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
						b2Vec2 offset = { x + 2.0f * ( x1 + j ) * scale, position.y - 2.0f * ( y1 + i ) * scale };
						float hx = 0.9f * scale * value / 255.0f;
						b2Polygon square = b2MakeOffsetBox( hx, hx, offset, 0.0f );
						b2CreatePolygonShape( bodyId, &shapeDef, &square );
					}
				}
			}

			x += 2.0f * scale * glyph->AdvanceX;
		}

		return bodyId;
	}

	b2BodyId CreateBalloon( b2Vec2 position, float scale, b2HexColor color )
	{
		b2ShapeDef shapeDef = b2DefaultShapeDef();
		shapeDef.friction = 0.1f;
		shapeDef.filter = { 0, 0, 0 };
		shapeDef.customColor = b2_colorWhite;

		float h = 1.0f * scale;
		b2Capsule stringCapsule = { { 0.0f, -h }, { 0.0f, h }, 0.05f * scale };

		float x = position.x;
		float y = position.y;

		b2BodyDef bodyDef = b2DefaultBodyDef();
		bodyDef.type = b2_dynamicBody;
		bodyDef.gravityScale = 0.5f;
		bodyDef.linearDamping = 0.5f;

		bodyDef.position = { x, y + h };
		b2BodyId stringId1 = b2CreateBody( m_worldId, &bodyDef );
		b2CreateCapsuleShape( stringId1, &shapeDef, &stringCapsule );

		bodyDef.position = { x, y + 3.0f * h };
		b2BodyId stringId2 = b2CreateBody( m_worldId, &bodyDef );
		b2CreateCapsuleShape( stringId2, &shapeDef, &stringCapsule );

		bodyDef.position = { x, y + 5.0f * h };
		b2BodyId stringId3 = b2CreateBody( m_worldId, &bodyDef );
		b2CreateCapsuleShape( stringId3, &shapeDef, &stringCapsule );

		bodyDef.position = { x, y + 7.0f * h };
		b2BodyId stringId4 = b2CreateBody( m_worldId, &bodyDef );
		b2CreateCapsuleShape( stringId4, &shapeDef, &stringCapsule );

		bodyDef.position = { x, y + 9.0f * h };
		b2BodyId stringId5 = b2CreateBody( m_worldId, &bodyDef );
		b2CreateCapsuleShape( stringId5, &shapeDef, &stringCapsule );

		bodyDef.position = { x, y + 11.0f * h };
		b2BodyId stringId6 = b2CreateBody( m_worldId, &bodyDef );
		b2CreateCapsuleShape( stringId6, &shapeDef, &stringCapsule );

		float radius = 2.0f * scale;
		b2Capsule mainCapsule = { { 0.0f, -0.1f * scale }, { 0.0f, 0.1f * scale }, radius };

		bodyDef.gravityScale = -1.0f;
		bodyDef.linearDamping = 0.0f;
		bodyDef.position = { x, y + 12.0f * h + radius };
		b2BodyId mainId = b2CreateBody( m_worldId, &bodyDef );

		shapeDef.restitution = 1.0f;
		shapeDef.filter = b2DefaultFilter();
		shapeDef.customColor = color;
		b2CreateCapsuleShape( mainId, &shapeDef, &mainCapsule );

		b2WeldJointDef weldDef = b2DefaultWeldJointDef();
		weldDef.angularHertz = 2.0f;
		weldDef.angularDampingRatio = 0.7f;

		weldDef.bodyIdA = stringId1;
		weldDef.bodyIdB = stringId2;
		weldDef.localAnchorA = { 0.0f, h };
		weldDef.localAnchorB = { 0.0f, -h };
		b2CreateWeldJoint( m_worldId, &weldDef );

		weldDef.bodyIdA = stringId2;
		weldDef.bodyIdB = stringId3;
		weldDef.localAnchorA = { 0.0f, h };
		weldDef.localAnchorB = { 0.0f, -h };
		b2CreateWeldJoint( m_worldId, &weldDef );

		weldDef.bodyIdA = stringId3;
		weldDef.bodyIdB = stringId4;
		weldDef.localAnchorA = { 0.0f, h };
		weldDef.localAnchorB = { 0.0f, -h };
		b2CreateWeldJoint( m_worldId, &weldDef );

		weldDef.bodyIdA = stringId4;
		weldDef.bodyIdB = stringId5;
		weldDef.localAnchorA = { 0.0f, h };
		weldDef.localAnchorB = { 0.0f, -h };
		b2CreateWeldJoint( m_worldId, &weldDef );

		weldDef.bodyIdA = stringId5;
		weldDef.bodyIdB = stringId6;
		weldDef.localAnchorA = { 0.0f, h };
		weldDef.localAnchorB = { 0.0f, -h };
		b2CreateWeldJoint( m_worldId, &weldDef );

		weldDef.bodyIdA = stringId6;
		weldDef.bodyIdB = mainId;
		weldDef.localAnchorA = { 0.0f, h };
		weldDef.localAnchorB = { 0.0f, -radius };
		b2CreateWeldJoint( m_worldId, &weldDef );

		return stringId1;
	}
};

#if 0

class Demo01 : public DemoBase
{
public:
	explicit Demo01( Settings& settings )
		: DemoBase( settings )
	{
		m_centerStart = { 0.0f, 307.5f };
		m_zoomStart = 3.5f;

		if ( settings.restart == false )
		{
			g_camera.m_center = m_centerStart;
			g_camera.m_zoom = m_zoomStart;
			g_draw.m_showUI = false;
			settings.drawJoints = false;
		}

		m_stage = 0;
		m_bridgeCenterJointId = b2_nullJointId;
		m_base = 0.0f;
		m_fraction = 0.0f;

		float scale = 0.6f;
		float gravityScale = 1.0f;
		CreateTextBodies( b2Vec2{ -72.7f, 340.0f }, scale, gravityScale, "Box2D", b2_colorAzure );
		CreateTextBodies( b2Vec2{ -122.9f, 220.0f }, scale, gravityScale, "Version 3.0", b2_colorAzure );
		CreateTextBodies( b2Vec2{ -106.1f, 100.0f }, scale, gravityScale, "Released!", b2_colorAzure );
	}

	void CreateBridge()
	{
		float h = 2.0f;
		b2Capsule capsule = { { -h, 0.0f }, { h, 0.0f }, 0.75f };

		b2ShapeDef shapeDef = b2DefaultShapeDef();
		shapeDef.density = 5.0f;
		shapeDef.customColor = b2_colorSlateGray;

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

	void Step( Settings& settings ) override
	{
		Sample::Step( settings );

		// g_draw.DrawCircle( b2Vec2{ -20.0f, 20.0f }, 0.5f, b2_colorRosyBrown );

		float timeStep = settings.hertz > 0.0f ? 1.0f / settings.hertz : 0.0f;

		switch ( m_stage )
		{
			case 0:
				break;

			case 1:
			{
				m_fraction += 0.2f * timeStep;
				m_fraction = b2MinFloat( m_fraction, 1.0f );
				g_camera.m_zoom = m_base + ( 80.0f - m_base ) * EaseInOutQuad( m_fraction );
				if ( m_fraction == 1.0f )
				{
					m_base = g_camera.m_center.y;
					m_fraction = 0.0f;
					m_stage += 1;
				}
			}
			break;

			case 2:
			{
				m_fraction += 0.2f * timeStep;
				m_fraction = b2MinFloat( m_fraction, 1.0f );
				g_camera.m_center.y = m_base + ( 75.0f - m_base ) * EaseInOutQuad( m_fraction );
				if ( m_fraction == 1.0f )
				{
					CreateBridge();
					CreateBalloons();
					m_base = g_camera.m_zoom;
					m_fraction = 0.0f;
					m_stage += 1;
				}
			}
			break;

			case 3:
			{
				m_fraction += 0.2f * timeStep;
				m_fraction = b2MinFloat( m_fraction, 1.0f );
				g_camera.m_zoom = m_base + ( 300.0f - m_base ) * EaseInOutQuad( m_fraction );
				if ( m_fraction == 1.0f )
				{
					m_base = 0.0f;
					m_fraction = 0.0f;
					m_stage += 1;
				}
			}
			break;

			case 4:
				if ( B2_IS_NON_NULL( m_bridgeCenterJointId ) )
				{
					b2DestroyJoint( m_bridgeCenterJointId );
					m_bridgeCenterJointId = b2_nullJointId;
				}
				m_stage += 1;
				break;

			default:
				break;
		}

		if ( glfwGetKey( g_mainWindow, GLFW_KEY_G ) == GLFW_PRESS )
		{
			g_camera.m_center = m_centerStart;
			g_camera.m_zoom = m_zoomStart;
			m_stage = 1;
			m_base = g_camera.m_zoom;
			m_fraction = 0.0f;
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

	b2Vec2 m_centerStart;
	float m_zoomStart;
	b2JointId m_bridgeCenterJointId;
	float m_fraction;
	float m_base;
	int m_stage;
};

static int sampleDemo01 = RegisterSample( "Demo", "01", Demo01::Create );

class Demo02 : public DemoBase
{
public:
	explicit Demo02( Settings& settings )
		: DemoBase( settings )
	{
		m_centerStart = { 0.0f, 50.0f };
		m_zoomStart = 4.0f;

		if ( settings.restart == false )
		{
			g_camera.m_center = m_centerStart;
			g_camera.m_zoom = m_zoomStart;
			g_draw.m_showUI = false;
			settings.drawJoints = false;
		}

		m_stage = 0;
		m_base = 0.0f;
		m_fraction = 0.0f;

		float scale = 0.6f;
		float gravityScale = 0.0f;
		CreateTextBodies( b2Vec2{ -96.2f, 160.0f }, scale, gravityScale, "Stronger", b2_colorAzure );
		CreateTextBodies( b2Vec2{ -69.5f, 80.0f }, scale, gravityScale, "Faster", b2_colorAzure );
		CreateTextBodies( b2Vec2{ -97.1f, 0.0f }, scale, gravityScale, "Rounder", b2_colorAzure );
	}

	void CreateRagdolls()
	{
		constexpr int count = 8;

		int index = 1;
		float y = 240.0f;
		for ( int j = 0; j < 4; ++j )
		{
			float x = -10.0f * ( count - 1.0f );
			for ( int i = 0; i < count; ++i )
			{
				Human human;
				human.Spawn( m_worldId, { x, y }, 20.0f, 0.01f, 0.5f, 0.1f, index, nullptr, true );
				human.ApplyRandomAngularImpulse( 1000000.0f );

				x += 20.0f;
				index += 1;
			}

			y += 80.0f;
		}
	}

	void Step( Settings& settings ) override
	{
		Sample::Step( settings );

		float timeStep = settings.hertz > 0.0f ? 1.0f / settings.hertz : 0.0f;

		// g_draw.DrawSegment( { 0.0f, 0.0f }, { -400.0f, 400.0f }, b2_colorRosyBrown );

		switch ( m_stage )
		{
			case 0:
				break;

			case 1:
			{
				m_fraction += 0.2f * timeStep;
				m_fraction = b2MinFloat( m_fraction, 1.0f );
				g_camera.m_zoom = m_base + ( 200.0f - m_base ) * EaseInOutQuad( m_fraction );
				if ( m_fraction == 1.0f )
				{
					m_base = 0.0f;
					m_fraction = 0.0f;
					m_stage += 1;
				}
			}
			break;

			case 2:
			{
				CreateRagdolls();
				m_stage += 1;
			}
			break;

			default:
				break;
		}

		if ( glfwGetKey( g_mainWindow, GLFW_KEY_G ) == GLFW_PRESS )
		{
			g_camera.m_center = m_centerStart;
			g_camera.m_zoom = m_zoomStart;
			m_stage = 1;
			m_base = g_camera.m_zoom;
			m_fraction = 0.0f;
		}
	}

	static Sample* Create( Settings& settings )
	{
		return new Demo02( settings );
	}

	b2Vec2 m_centerStart;
	float m_zoomStart;
	float m_fraction;
	float m_base;
	int m_stage;
};

static int sampleDemo02 = RegisterSample( "Demo", "02", Demo02::Create );

class Demo03 : public DemoBase
{
public:
	explicit Demo03( Settings& settings )
		: DemoBase( settings )
	{
		m_centerStart = { 0.0f, 50.0f };
		m_zoomStart = 200.0f;

		if ( settings.restart == false )
		{
			g_camera.m_center = m_centerStart;
			g_camera.m_zoom = m_zoomStart;
			g_draw.m_showUI = false;
			settings.drawJoints = false;
		}

		m_stage = 0;
		m_base = 0.0f;
		m_fraction = 0.0f;

		float scale = 0.6f;
		float gravityScale = 0.0f;
		CreateTextBodies( b2Vec2{ -166.3f, 100.0f }, scale, gravityScale, "Multithreading", b2_colorGray9 );
		CreateTextBodies( b2Vec2{ -17.7f, 50.0f }, scale, gravityScale, "&", b2_colorGray8 );
		CreateTextBodies( b2Vec2{ -58.7f, 0.0f }, scale, gravityScale, "SIMD", b2_colorGray9 );
	}

	void CreateBits()
	{
		constexpr int count = 20;

		b2BodyDef bodyDef = b2DefaultBodyDef();
		bodyDef.type = b2_dynamicBody;
		bodyDef.fixedRotation = true;
		bodyDef.gravityScale = 0.0f;
		bodyDef.linearDamping = 0.0f;

		float h = 1.0f;
		b2Polygon bit1 = b2MakeOffsetBox( h, h, { 0.0f, 0.0f }, 0.0f );
		b2Polygon bit2 = b2MakeOffsetBox( h, h, { 0.0f, 2.0f * h }, 0.0f );
		b2Polygon bit3 = b2MakeOffsetBox( h, h, { 0.0f, 4.0f * h }, 0.0f );
		b2Polygon bit4 = b2MakeOffsetBox( h, h, { 0.0f, 6.0f * h }, 0.0f );
		b2Polygon bit5 = b2MakeOffsetBox( h, h, { 0.0f, 8.0f * h }, 0.0f );
		b2Polygon bit6 = b2MakeOffsetBox( h, h, { 0.0f, 10.0f * h }, 0.0f );
		b2Polygon bit7 = b2MakeOffsetBox( h, h, { 0.0f, 12.0f * h }, 0.0f );
		b2Polygon bit8 = b2MakeOffsetBox( h, h, { 0.0f, 14.0f * h }, 0.0f );

		b2ShapeDef shapeDef = b2DefaultShapeDef();

		b2HexColor bitColors[5] = { b2_colorGreen, b2_colorLightGreen, b2_colorLightSeaGreen, b2_colorMintCream,
									b2_colorDarkSeaGreen };
		int colorCount = 5;

		int index = 0;
		float y = 260.0f;
		for ( int j = 0; j < 30; ++j )
		{
			float x = -RandomFloat( 10.0f, 20.0f ) * ( count - 1.0f );
			for ( int i = 0; i < count; ++i )
			{
				bodyDef.position.x = x;
				bodyDef.position.y = RandomFloat( y - 10.0f, y + 10.0f );
				bodyDef.linearVelocity.y = RandomFloat( -80.0f, -160.0f );

				b2BodyId bodyId = b2CreateBody( m_worldId, &bodyDef );

				shapeDef.customColor = bitColors[index % colorCount];
				b2CreatePolygonShape( bodyId, &shapeDef, &bit1 );
				index += 1;

				shapeDef.customColor = bitColors[index % colorCount];
				b2CreatePolygonShape( bodyId, &shapeDef, &bit2 );
				index += 1;

				shapeDef.customColor = bitColors[index % colorCount];
				b2CreatePolygonShape( bodyId, &shapeDef, &bit3 );
				index += 1;

				shapeDef.customColor = bitColors[index % colorCount];
				b2CreatePolygonShape( bodyId, &shapeDef, &bit4 );
				index += 1;

				shapeDef.customColor = bitColors[index % colorCount];
				b2CreatePolygonShape( bodyId, &shapeDef, &bit5 );
				index += 1;

				shapeDef.customColor = bitColors[index % colorCount];
				b2CreatePolygonShape( bodyId, &shapeDef, &bit6 );
				index += 1;

				shapeDef.customColor = bitColors[index % colorCount];
				b2CreatePolygonShape( bodyId, &shapeDef, &bit7 );
				index += 1;

				shapeDef.customColor = bitColors[index % colorCount];
				b2CreatePolygonShape( bodyId, &shapeDef, &bit8 );
				index += 1;

				x += RandomFloat( 30.0f, 40.0f );
			}

			y += 40.0f;
		}
	}

	void Step( Settings& settings ) override
	{
		Sample::Step( settings );

		float timeStep = settings.hertz > 0.0f ? 1.0f / settings.hertz : 0.0f;

		// g_draw.DrawSegment( { 0.0f, 0.0f }, { -400.0f, 400.0f }, b2_colorRosyBrown );

		switch ( m_stage )
		{
			case 0:
				break;

			case 1:
			case 2:
			case 3:
			case 4:
			case 5:
			case 6:
			case 7:
				// case 8:
				// case 9:
				// case 10:
				CreateBits();
				m_stage += 1;
				break;

			default:
				break;
		}

		if ( glfwGetKey( g_mainWindow, GLFW_KEY_G ) == GLFW_PRESS && m_stage == 0 )
		{
			g_camera.m_center = m_centerStart;
			g_camera.m_zoom = m_zoomStart;
			m_stage = 1;
		}
	}

	static Sample* Create( Settings& settings )
	{
		return new Demo03( settings );
	}

	b2Vec2 m_centerStart;
	float m_zoomStart;
	float m_fraction;
	float m_base;
	int m_stage;
};

static int sampleDemo03 = RegisterSample( "Demo", "03", Demo03::Create );

class Demo04 : public DemoBase
{
public:
	explicit Demo04( Settings& settings )
		: DemoBase( settings )
	{
		m_centerStart = { 0.0f, 15.0f };
		m_zoomStart = 60.0f;

		if ( settings.restart == false )
		{
			g_camera.m_center = m_centerStart;
			g_camera.m_zoom = m_zoomStart;
			g_draw.m_showUI = false;
			settings.drawJoints = false;
		}

		m_stage = 0;
		m_baseZoom = g_camera.m_zoom;
		m_baseY = g_camera.m_center.y;
		m_fraction = 0.0f;

		float scale = 0.3f;
		float gravityScale = 0.0f;
		CreateTextBodies( b2Vec2{ -39.4f, 60.0f }, scale, gravityScale, "Robust", b2_colorBox2DRed );
		CreateTextBodies( b2Vec2{ -46.1f, 30.0f }, scale, gravityScale, "Soft Step", b2_colorBox2DBlue );
		CreateTextBodies( b2Vec2{ -34.4f, 0.0f }, scale, gravityScale, "Solver", b2_colorBox2DGreen );
	}

	void CreateWreckingBall( b2Vec2 position, float scale, bool reflect )
	{
		b2BodyId groundId = b2_nullBodyId;
		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			bodyDef.position = position;
			groundId = b2CreateBody( m_worldId, &bodyDef );
		}

		float frictionTorque = 100.0f;
		float sign = reflect ? -1.0f : 1.0f;

		{
			float hx = 0.5f * scale;
			float radius = 0.125f * scale;
			b2Capsule capsule = { { -hx, 0.0f }, { hx, 0.0f }, radius };

			b2ShapeDef shapeDef = b2DefaultShapeDef();
			shapeDef.density = 2.0f;
			shapeDef.filter.maskBits = 0;
			shapeDef.customColor = b2_colorGray4;

			b2RevoluteJointDef jointDef = b2DefaultRevoluteJointDef();

			b2BodyId prevBodyId = groundId;
			int count = 30;

			for ( int i = 0; i < count; ++i )
			{
				b2BodyDef bodyDef = b2DefaultBodyDef();
				bodyDef.type = b2_dynamicBody;

				bodyDef.position = b2Vec2{ sign * ( 1.0f + 2.0f * i ) * hx, count * hx } + position;
				b2BodyId bodyId = b2CreateBody( m_worldId, &bodyDef );
				b2CreateCapsuleShape( bodyId, &shapeDef, &capsule );

				b2Vec2 pivot = b2Vec2{ sign * ( 2.0f * i ) * hx, count * hx } + position;
				jointDef.bodyIdA = prevBodyId;
				jointDef.bodyIdB = bodyId;
				jointDef.localAnchorA = b2Body_GetLocalPoint( jointDef.bodyIdA, pivot );
				jointDef.localAnchorB = b2Body_GetLocalPoint( jointDef.bodyIdB, pivot );
				// jointDef.enableMotor = true;
				jointDef.maxMotorTorque = frictionTorque;
				b2CreateRevoluteJoint( m_worldId, &jointDef );

				prevBodyId = bodyId;
			}

			b2Circle circle = { { 0.0f, 0.0f }, 7.0f * scale };

			b2BodyDef bodyDef = b2DefaultBodyDef();
			bodyDef.type = b2_dynamicBody;
			bodyDef.position = b2Vec2{ sign * ( ( 1.0f + 2.0f * count ) * hx + circle.radius - hx ), count * hx } + position;

			shapeDef.customColor = b2_colorSlateGray;
			shapeDef.restitution = 0.25f;
			shapeDef.friction = 0.2f;
			shapeDef.filter.maskBits = ~0u;

			b2BodyId bodyId = b2CreateBody( m_worldId, &bodyDef );
			b2CreateCircleShape( bodyId, &shapeDef, &circle );

			b2Vec2 pivot = b2Vec2{ sign * ( 2.0f * count ) * hx, count * hx } + position;
			jointDef.bodyIdA = prevBodyId;
			jointDef.bodyIdB = bodyId;
			jointDef.localAnchorA = b2Body_GetLocalPoint( jointDef.bodyIdA, pivot );
			jointDef.localAnchorB = b2Body_GetLocalPoint( jointDef.bodyIdB, pivot );
			jointDef.enableMotor = true;
			jointDef.maxMotorTorque = frictionTorque;
			b2CreateRevoluteJoint( m_worldId, &jointDef );
		}
	}

	void Step( Settings& settings ) override
	{
		Sample::Step( settings );

		float timeStep = settings.hertz > 0.0f ? 1.0f / settings.hertz : 0.0f;

		// g_draw.DrawSegment( { 0.0f, 0.0f }, { -400.0f, 400.0f }, b2_colorRosyBrown );

		switch ( m_stage )
		{
			case 0:
				break;

			case 1:
				CreateWreckingBall( { 20.0f, 130.0f }, 4.0f, false );
				CreateWreckingBall( { -20.0f, 130.0f }, 4.0f, true );
				m_stage += 1;
				break;

			case 2:
			{
				m_fraction += 0.2f * timeStep;
				m_fraction = b2MinFloat( m_fraction, 1.0f );
				g_camera.m_zoom = m_baseZoom + ( 140.0f - m_baseZoom ) * EaseInOutQuad( m_fraction );
				g_camera.m_center.y = m_baseY + ( 50.0f - m_baseY ) * EaseInOutQuad( m_fraction );
				if ( m_fraction == 1.0f )
				{
					m_baseZoom = g_camera.m_zoom;
					m_baseY = g_camera.m_center.y;
					m_fraction = 0.0f;
					m_stage += 1;
				}
			}
			break;

			default:
				break;
		}

		if ( glfwGetKey( g_mainWindow, GLFW_KEY_G ) == GLFW_PRESS && m_stage == 0 )
		{
			g_camera.m_center = m_centerStart;
			g_camera.m_zoom = m_zoomStart;
			m_baseZoom = g_camera.m_zoom;
			m_baseY = g_camera.m_center.y;
			m_fraction = 0.0f;
			m_stage = 1;
		}
	}

	static Sample* Create( Settings& settings )
	{
		return new Demo04( settings );
	}

	b2Vec2 m_centerStart;
	float m_zoomStart;
	float m_fraction;
	float m_baseZoom;
	float m_baseY;
	int m_stage;
};

static int sampleDemo04 = RegisterSample( "Demo", "04", Demo04::Create );

class Demo05 : public DemoBase
{
public:
	explicit Demo05( Settings& settings )
		: DemoBase( settings )
	{
		m_centerStart = { 0.0f, 0.0f };
		m_zoomStart = 100.0f;

		if ( settings.restart == false )
		{
			g_camera.m_center = m_centerStart;
			g_camera.m_zoom = m_zoomStart;
			g_draw.m_showUI = false;
			settings.drawJoints = false;
		}

		m_stage = 0;
		m_baseZoom = g_camera.m_zoom;
		m_baseY = g_camera.m_center.y;
		m_fraction = 0.0f;

		float scale = 0.3f;
		float gravityScale = 0.0f;
		CreateTextBodies( b2Vec2{ -49.5f, 60.0f }, scale, gravityScale, "Capsules", b2_colorLime );
		CreateTextBodies( b2Vec2{ -8.85f, 30.0f }, scale, gravityScale, "&", b2_colorYellow );
		CreateTextBodies( b2Vec2{ -49.9f, 0.0f }, scale, gravityScale, "Rounded", b2_colorFuchsia );
		CreateTextBodies( b2Vec2{ -50.3f, -30.0f }, scale, gravityScale, "Polygons", b2_colorFuchsia );
		CreateSpiral();
	}

	void CreateSpiral()
	{
		float scale = 8.0f;
		b2Capsule capsule = { { -0.25f * scale, 0.0f }, { 0.25f * scale, 0.0f }, 0.125f * scale };
		b2Polygon box = b2MakeRoundedBox( 0.1f * scale, 0.15f * scale, 0.15f * scale );

		b2ShapeDef shapeDef = b2DefaultShapeDef();
		shapeDef.friction = 0.2f;
		shapeDef.density = 1.0f;

		float distance = 10.0f * scale;
		float deltaAngle = scale / distance;
		float deltaDistance = 0.05f * scale;
		float angle = 0.0f;
		b2BodyDef bodyDef = b2DefaultBodyDef();
		bodyDef.type = b2_dynamicBody;
		bodyDef.gravityScale = 0.0f;

		for ( int i = 0; i < k_count; ++i )
		{
			bodyDef.position = { distance * cosf( angle ), distance * sinf( angle ) };
			bodyDef.angularVelocity = RandomFloat( -2.0f, 2.0f );

			// bodyDef.linearVelocity = {2.0f * distance * sinf(angle), -1.5f * distance * cosf(angle)};
			m_spiralIds[i] = b2CreateBody( m_worldId, &bodyDef );

			if ( ( i & 1 ) == 0 )
			{
				shapeDef.customColor = b2_colorGray9;
				b2CreateCapsuleShape( m_spiralIds[i], &shapeDef, &capsule );
			}
			else
			{
				shapeDef.customColor = b2_colorGray9;
				b2CreatePolygonShape( m_spiralIds[i], &shapeDef, &box );
			}

			angle += deltaAngle;
			distance += deltaDistance;
		}
	}

	void Step( Settings& settings ) override
	{
		Sample::Step( settings );

		// g_draw.DrawSegment( { 0.0f, 0.0f }, { -400.0f, 400.0f }, b2_colorRosyBrown );
		float timeStep = settings.hertz > 0.0f ? 1.0f / settings.hertz : 0.0f;

		switch ( m_stage )
		{
			case 0:
				break;

			case 1:
				m_fraction += 0.14f * timeStep;
				m_fraction = b2MinFloat( m_fraction, 1.0f );
				if (m_fraction == 1.0f)
				{
					m_fraction = 0.0f;
					b2World_Explode( m_worldId, b2Vec2_zero, 15.0f, 1000.0f );
					m_stage += 1;
				}
				else
				{
					float force = 200.0f;
					for ( int i = 0; i < k_count; ++i )
					{
						b2Vec2 p = b2Body_GetPosition( m_spiralIds[i] );
						float distance = b2Length( p );
						if ( distance < 0.1f )
						{
							continue;
						}

						float scale = force / distance;
						b2Vec2 f = { -scale * p.x, -scale * p.y };
						b2Body_ApplyForceToCenter( m_spiralIds[i], f, false );
					}
				}

				break;

			default:
				break;
		}


		if ( glfwGetKey( g_mainWindow, GLFW_KEY_G ) == GLFW_PRESS && m_stage == 0 )
		{
			g_camera.m_center = m_centerStart;
			g_camera.m_zoom = m_zoomStart;
			m_baseZoom = g_camera.m_zoom;
			m_baseY = g_camera.m_center.y;
			m_fraction = 0.0f;
			m_stage = 1;
		}
	}

	static Sample* Create( Settings& settings )
	{
		return new Demo05( settings );
	}

	static constexpr int k_count = 400;
	b2BodyId m_spiralIds[k_count];
	b2Vec2 m_centerStart;
	float m_zoomStart;
	float m_fraction;
	float m_baseZoom;
	float m_baseY;
	int m_stage;
};

static int sampleDemo05 = RegisterSample( "Demo", "05", Demo05::Create );

class Demo06 : public DemoBase
{
public:
	explicit Demo06( Settings& settings )
		: DemoBase( settings )
	{
		m_centerStart = { 0.0f, 65.0f };
		m_zoomStart = 75.0f;

		if ( settings.restart == false )
		{
			g_camera.m_center = m_centerStart;
			g_camera.m_zoom = m_zoomStart;
			g_draw.m_showUI = false;
			settings.drawJoints = true;
			settings.subStepCount = 8;
		}

		m_stage = 0;
		m_baseZoom = g_camera.m_zoom;
		m_baseY = g_camera.m_center.y;
		m_fraction = 0.0f;

		float scale = 0.3f;
		float gravityScale = 1.0f;
		CreateTextBodies( b2Vec2{ -47.2f, 120.0f }, scale, gravityScale, "Portable", b2_colorAqua );
		CreateTextBodies( b2Vec2{ -32.6f, 90.0f }, scale, gravityScale, "C  API", b2_colorPlum );

		CreateLift();
	}

	void CreateLift()
	{
		float scale = 10.0f;

		b2BodyId groundId;
		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			groundId = b2CreateBody( m_worldId, &bodyDef );

			b2ShapeDef shapeDef = b2DefaultShapeDef();
			b2Segment segment = { { -10.0f * scale, 0.0f }, { 10.0f * scale, 0.0f } };
			b2CreateSegmentShape( groundId, &shapeDef, &segment );
		}

		b2BodyDef bodyDef = b2DefaultBodyDef();
		bodyDef.type = b2_dynamicBody;
		bodyDef.enableSleep = false;

		b2ShapeDef shapeDef = b2DefaultShapeDef();
		shapeDef.customColor = b2_colorGray8;
		b2Capsule capsule = { { -2.5f * scale, 0.0f }, { 2.5f * scale, 0.0f }, 0.15f * scale };

		b2BodyId baseId1 = groundId;
		b2BodyId baseId2 = groundId;
		b2Vec2 baseAnchor1 = { -2.5f * scale, 0.2f * scale };
		b2Vec2 baseAnchor2 = { 2.5f * scale, 0.2f * scale };
		float y = 0.5f * scale;

		b2BodyId linkId1;
		int N = 3;

		for ( int i = 0; i < N; ++i )
		{
			bodyDef.position = { 0.0f, y };
			bodyDef.rotation = b2MakeRot( 0.15f );
			b2BodyId bodyId1 = b2CreateBody( m_worldId, &bodyDef );
			b2CreateCapsuleShape( bodyId1, &shapeDef, &capsule );

			bodyDef.position = { 0.0f, y };
			bodyDef.rotation = b2MakeRot( -0.15f );

			b2BodyId bodyId2 = b2CreateBody( m_worldId, &bodyDef );
			b2CreateCapsuleShape( bodyId2, &shapeDef, &capsule );

			if ( i == 1 )
			{
				linkId1 = bodyId2;
			}

			b2RevoluteJointDef revoluteDef = b2DefaultRevoluteJointDef();

			// left pin
			revoluteDef.bodyIdA = baseId1;
			revoluteDef.bodyIdB = bodyId1;
			revoluteDef.localAnchorA = baseAnchor1;
			revoluteDef.localAnchorB = { -2.5f * scale, 0.0f };
			revoluteDef.collideConnected = ( i == 0 ) ? true : false;

			b2CreateRevoluteJoint( m_worldId, &revoluteDef );

			// right pin
			if ( i == 0 )
			{
				b2WheelJointDef wheelDef = b2DefaultWheelJointDef();
				wheelDef.bodyIdA = baseId2;
				wheelDef.bodyIdB = bodyId2;
				wheelDef.localAxisA = { 1.0f, 0.0f };
				wheelDef.localAnchorA = baseAnchor2;
				wheelDef.localAnchorB = { 2.5f * scale, 0.0f };
				wheelDef.enableSpring = false;
				wheelDef.collideConnected = true;

				b2CreateWheelJoint( m_worldId, &wheelDef );
			}
			else
			{
				revoluteDef.bodyIdA = baseId2;
				revoluteDef.bodyIdB = bodyId2;
				revoluteDef.localAnchorA = baseAnchor2;
				revoluteDef.localAnchorB = { 2.5f * scale, 0.0f };

				b2CreateRevoluteJoint( m_worldId, &revoluteDef );
			}

			// middle pin
			revoluteDef.bodyIdA = bodyId1;
			revoluteDef.bodyIdB = bodyId2;
			revoluteDef.localAnchorA = { 0.0f, 0.0f };
			revoluteDef.localAnchorB = { 0.0f, 0.0f };

			b2CreateRevoluteJoint( m_worldId, &revoluteDef );

			baseId1 = bodyId2;
			baseId2 = bodyId1;
			baseAnchor1 = { -2.5f * scale, 0.0f };
			baseAnchor2 = { 2.5f * scale, 0.0f };
			y += 1.0f * scale;
		}

		bodyDef.position = { 0.0f, y };
		bodyDef.rotation = b2Rot_identity;
		b2BodyId platformId = b2CreateBody( m_worldId, &bodyDef );

		b2Polygon box = b2MakeBox( 3.0f * scale, 0.2f * scale );
		shapeDef.customColor = b2_colorYellow;
		b2CreatePolygonShape( platformId, &shapeDef, &box );

		// left pin
		b2RevoluteJointDef revoluteDef = b2DefaultRevoluteJointDef();
		revoluteDef.bodyIdA = platformId;
		revoluteDef.bodyIdB = baseId1;
		revoluteDef.localAnchorA = { -2.5f * scale, -0.4f * scale };
		revoluteDef.localAnchorB = baseAnchor1;
		revoluteDef.collideConnected = true;
		b2CreateRevoluteJoint( m_worldId, &revoluteDef );

		// right pin
		b2WheelJointDef wheelDef = b2DefaultWheelJointDef();
		wheelDef.bodyIdA = platformId;
		wheelDef.bodyIdB = baseId2;
		wheelDef.localAxisA = { 1.0f, 0.0f };
		wheelDef.localAnchorA = { 2.5f * scale, -0.4f * scale };
		wheelDef.localAnchorB = baseAnchor2;
		wheelDef.enableSpring = false;
		wheelDef.collideConnected = true;
		b2CreateWheelJoint( m_worldId, &wheelDef );

		b2DistanceJointDef distanceDef = b2DefaultDistanceJointDef();
		distanceDef.bodyIdA = groundId;
		distanceDef.bodyIdB = linkId1;
		distanceDef.localAnchorA = { -2.5f * scale, 0.2f * scale };
		distanceDef.localAnchorB = { 0.5f * scale, 0.0f };
		distanceDef.enableSpring = true;
		distanceDef.minLength = 0.2f * scale;
		distanceDef.maxLength = 5.5f * scale;
		distanceDef.enableLimit = true;
		distanceDef.enableMotor = false;
		distanceDef.motorSpeed = 0.0f;
		distanceDef.maxMotorForce = 2000000.0f * scale;
		m_jointId = b2CreateDistanceJoint( m_worldId, &distanceDef );
	}

	void Step( Settings& settings ) override
	{
		Sample::Step( settings );

		float timeStep = settings.hertz > 0.0f ? 1.0f / settings.hertz : 0.0f;

		// g_draw.DrawSegment( { 0.0f, 0.0f }, { -400.0f, 400.0f }, b2_colorRosyBrown );

		switch ( m_stage )
		{
			case 0:
				break;

			case 1:
				b2DistanceJoint_EnableMotor( m_jointId, true );
				m_stage += 1;
				break;

			case 2:
			{
				m_fraction += 0.2f * timeStep;
				m_fraction = b2MinFloat( m_fraction, 1.0f );
				float easedFraction = EaseInOutQuad( m_fraction );
				g_camera.m_zoom = m_baseZoom + ( 75.0f - m_baseZoom ) * easedFraction;
				g_camera.m_center.y = m_baseY + ( 65.0f - m_baseY ) * easedFraction;

				b2DistanceJoint_SetMotorSpeed( m_jointId, 2.0f * easedFraction );

				if ( m_fraction == 1.0f )
				{
					m_baseZoom = g_camera.m_zoom;
					m_baseY = g_camera.m_center.y;
					m_fraction = 0.0f;
					m_stage += 1;
				}
			}
			break;

			default:
				break;
		}

		if ( glfwGetKey( g_mainWindow, GLFW_KEY_G ) == GLFW_PRESS && m_stage == 0 )
		{
			g_camera.m_center = m_centerStart;
			g_camera.m_zoom = m_zoomStart;
			m_baseZoom = g_camera.m_zoom;
			m_baseY = g_camera.m_center.y;
			m_fraction = 0.0f;
			m_stage = 1;
		}
	}

	static Sample* Create( Settings& settings )
	{
		return new Demo06( settings );
	}

	b2Vec2 m_centerStart;
	b2JointId m_jointId;
	float m_zoomStart;
	float m_fraction;
	float m_baseZoom;
	float m_baseY;
	int m_stage;
};

static int sampleDemo06 = RegisterSample( "Demo", "06", Demo06::Create );

// skip this, hard to show a large world nicely in a video
class Demo07 : public DemoBase
{
public:
	explicit Demo07( Settings& settings )
		: DemoBase( settings )
	{
		m_period = 40.0f;
		float omega = 2.0 * b2_pi / m_period;
		m_cycleCount = g_sampleDebug ? 10 : 600;
		m_gridSize = 1.0f;
		m_gridCount = (int)( m_cycleCount * m_period / m_gridSize );

		float xStart = -0.5f * ( m_cycleCount * m_period );

		m_centerStart = { -11960.0f, 20.0f };
		m_zoomStart = 25.0f;

		if ( settings.restart == false )
		{
			g_camera.m_center = m_centerStart;
			g_camera.m_zoom = m_zoomStart;
			g_draw.m_showUI = false;
			settings.drawJoints = false;
		}

		m_stage = 0;
		m_baseX = g_camera.m_center.x;
		m_fraction = 0.0f;

		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			b2ShapeDef shapeDef = b2DefaultShapeDef();

			// Setting this to false significantly reduces the cost of creating
			// static bodies and shapes.
			shapeDef.forceContactCreation = false;

			float height = 4.0f;
			float xBody = xStart;
			float xShape = xStart;

			float textScale = 0.05f;
			CreateTextBodies( b2Vec2{xShape, 30.0f }, textScale, 1.0f, "Larger Worlds", b2_colorAqua );

			char buffer[32] = { 0 };
			for (int i = 0; i <= 24; ++i)
			{
				snprintf( buffer, 32, "%d km", i );
				CreateTextBodies( b2Vec2{xStart + 1000.0f * i, 25.0f }, textScale, 1.0f, buffer, b2_colorWhite );
			}

			b2BodyId groundId;
			for ( int i = 0; i < m_gridCount; ++i )
			{
				// Create a new body regularly so that shapes are not too far from the body origin.
				// Most algorithms in Box2D work in local coordinates, but contact points are computed
				// relative to the body origin.
				// This makes a noticeable improvement in stability far from the origin.
				if ( i % 10 == 0 )
				{
					bodyDef.position.x = xBody;
					groundId = b2CreateBody( m_worldId, &bodyDef );
					xShape = 0.0f;
				}

				float y = 0.0f;

				int ycount = (int)roundf( height * cosf( omega * xBody ) ) + 12;

				for ( int j = 0; j < ycount; ++j )
				{
					b2Polygon square = b2MakeOffsetBox( 0.4f * m_gridSize, 0.4f * m_gridSize, { xShape, y }, 0.0f );
					square.radius = 0.1f;
					b2CreatePolygonShape( groundId, &shapeDef, &square );

					y += m_gridSize;
				}

				xBody += m_gridSize;
				xShape += m_gridSize;
			}
		}

		int humanIndex = 0;
		int donutIndex = 0;
		for ( int cycleIndex = 0; cycleIndex < m_cycleCount; ++cycleIndex )
		{
			float xbase = ( 0.5f + cycleIndex ) * m_period + xStart;

			int remainder = cycleIndex % 3;
			if ( remainder == 0 )
			{
				b2BodyDef bodyDef = b2DefaultBodyDef();
				bodyDef.type = b2_dynamicBody;
				bodyDef.position = { xbase - 3.0f, 10.0f };

				b2ShapeDef shapeDef = b2DefaultShapeDef();
				b2Polygon box = b2MakeBox( 0.3f, 0.2f );

				for ( int i = 0; i < 10; ++i )
				{
					bodyDef.position.y = 10.0f;
					for ( int j = 0; j < 5; ++j )
					{
						b2BodyId bodyId = b2CreateBody( m_worldId, &bodyDef );
						b2CreatePolygonShape( bodyId, &shapeDef, &box );
						bodyDef.position.y += 0.5f;
					}
					bodyDef.position.x += 0.6f;
				}
			}
			else if ( remainder == 1 )
			{
				b2Vec2 position = { xbase - 2.0f, 10.0f };
				for ( int i = 0; i < 5; ++i )
				{
					Human human;
					human.Spawn( m_worldId, position, 1.5f, 0.02f, 2.0f, 0.2f, humanIndex + 1, NULL, false );
					humanIndex += 1;
					position.x += 1.0f;
				}
			}
			else
			{
				b2Vec2 position = { xbase - 4.0f, 12.0f };

				for ( int i = 0; i < 5; ++i )
				{
					Donut donut;
					donut.Spawn( m_worldId, position, 0.75f, 0, NULL );
					donutIndex += 1;
					position.x += 2.0f;
				}
			}
		}

		m_cycleIndex = 0;
	}

	void Step( Settings& settings ) override
	{
		float timeStep = settings.hertz > 0.0f ? 1.0f / settings.hertz : 0.0f;

		// g_draw.DrawSegment( { 0.0f, 0.0f }, { -400.0f, 400.0f }, b2_colorRosyBrown );

		switch ( m_stage )
		{
			case 0:
				break;

			case 1:
			{
				m_fraction += 0.01f * timeStep;
				m_fraction = b2MinFloat( m_fraction, 1.0f );
				float easedFraction = EaseInOutQuad( m_fraction );
				g_camera.m_center.x = m_baseX + ( m_gridCount * m_gridSize - 100.0f) * easedFraction;

				if ( m_fraction == 1.0f )
				{
					m_baseX = g_camera.m_center.x;
					m_fraction = 0.0f;
					m_stage += 1;
				}
			}
			break;

			default:
				break;
		}

		if ( glfwGetKey( g_mainWindow, GLFW_KEY_G ) == GLFW_PRESS && m_stage == 0 )
		{
			g_camera.m_center = m_centerStart;
			g_camera.m_zoom = m_zoomStart;
			m_baseX = g_camera.m_center.x;
			m_fraction = 0.0f;
			m_stage = 1;
		}

		Sample::Step( settings );
	}

	static Sample* Create( Settings& settings )
	{
		return new Demo07( settings );
	}

	float m_period;
	int m_cycleCount;
	int m_cycleIndex;
	float m_gridCount;
	float m_gridSize;

	b2Vec2 m_centerStart;
	float m_zoomStart;
	b2JointId m_jointId;
	float m_fraction;
	float m_baseX;
	int m_stage;
};

static int sampleDemo07 = RegisterSample( "Demo", "07", Demo07::Create );

class Demo08 : public DemoBase
{
public:
	explicit Demo08( Settings& settings )
		: DemoBase( settings )
	{
		m_centerStart = { 0.0f, 0.0f };
		m_zoomStart = 40.0f;

		if ( settings.restart == false )
		{
			g_camera.m_center = m_centerStart;
			g_camera.m_zoom = m_zoomStart;
			g_draw.m_showUI = false;
			settings.drawJoints = true;
			settings.subStepCount = 8;
		}

		m_stage = 0;
		m_baseZoom = g_camera.m_zoom;
		m_baseY = g_camera.m_center.y;
		m_fraction = 0.0f;

		float scale = 0.3f;
		float gravityScale = 1.0f;
		CreateTextBodies( b2Vec2{ -54.9f, 20.0f }, scale, gravityScale, "Have Fun!", b2_colorWhiteSmoke );
	}

	void CreateFloater( b2Vec2 position, int index )
	{
		float balloonScale = 1.0f;

		b2BodyId stringId1 = CreateBalloon( position - b2Vec2{ 0.2f, 0.0f }, 0.9f * balloonScale, b2_colorBox2DBlue );
		b2BodyId stringId2 = CreateBalloon( position, balloonScale, b2_colorBox2DRed );
		b2BodyId stringId3 = CreateBalloon( position + b2Vec2{ 0.2f, 0.0f }, 0.95f * balloonScale, b2_colorBox2DGreen );

		float humanScale = 6.0f;
		Human human;
		human.Spawn( m_worldId, { position.x - 0.5f, position.y - 5.5f }, humanScale, 0.02f, 1.0f, 0.1f, index, nullptr, true );

		b2RevoluteJointDef revoluteDef = b2DefaultRevoluteJointDef();
		revoluteDef.bodyIdA = human.m_bones[Bone::e_lowerLeftArm].bodyId;
		revoluteDef.localAnchorA = { 0.0f, -0.125f * humanScale };

		revoluteDef.bodyIdB = stringId1;
		revoluteDef.localAnchorB = { 0.0f, -balloonScale };
		b2CreateRevoluteJoint( m_worldId, &revoluteDef );

		revoluteDef.bodyIdB = stringId2;
		revoluteDef.localAnchorB = { 0.0f, -balloonScale };
		b2CreateRevoluteJoint( m_worldId, &revoluteDef );

		revoluteDef.bodyIdB = stringId3;
		revoluteDef.localAnchorB = { 0.0f, -balloonScale };
		b2CreateRevoluteJoint( m_worldId, &revoluteDef );
	}

	void Step( Settings& settings ) override
	{
		Sample::Step( settings );

		//float timeStep = settings.hertz > 0.0f ? 1.0f / settings.hertz : 0.0f;
		//g_draw.DrawSolidCircle( b2Transform_identity, b2Vec2{ 0.0f, 0.0f }, 2.0f, b2_colorChocolate );

		switch ( m_stage )
		{
			case 0:
				break;

			case 1:
				CreateFloater( { 0.0f, -60.0f }, 2 );
				CreateFloater( { -40.0f, -80.0f }, 3 );
				CreateFloater( { 30.0f, -90.0f }, 4 );
				m_stage += 1;
				break;

			default:
				break;
		}

		if ( glfwGetKey( g_mainWindow, GLFW_KEY_G ) == GLFW_PRESS && m_stage == 0 )
		{
			g_camera.m_center = m_centerStart;
			g_camera.m_zoom = m_zoomStart;
			m_baseZoom = g_camera.m_zoom;
			m_baseY = g_camera.m_center.y;
			m_fraction = 0.0f;
			m_stage = 1;
		}
	}

	static Sample* Create( Settings& settings )
	{
		return new Demo08( settings );
	}

	b2Vec2 m_centerStart;
	float m_zoomStart;
	float m_fraction;
	float m_baseZoom;
	float m_baseY;
	int m_stage;
};

static int sampleDemo08 = RegisterSample( "Demo", "08", Demo08::Create );

#endif
