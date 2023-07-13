// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "sample.h"
#include "box2d/dynamic_tree.h"
#include "box2d/math.h"

#include <GLFW/glfw3.h>
#include <imgui.h>

struct Proxy
{
	b2AABB box;
	b2Vec2 startPosition;
	int32_t proxyId;
	int32_t rayStamp;
	int32_t queryStamp;
};

static bool QueryCallback(int32_t proxyId, void* userData, void* context);
static float RayCallback(const b2RayCastInput* input, int32_t proxyId, void* userData, void* context);

// Tests the Box2D bounding volume hierarchy (BVH). The dynamic tree
// can be used independently for as a spatial data structure.
class DynamicTree : public Sample
{
public:
	DynamicTree(const Settings& settings)
		: Sample(settings)
	{
		m_fill = 1.0f;
		m_moveFraction = 0.0f;
		m_moveDelta = 0.1f;
		m_proxies = nullptr;
		m_mapArray = nullptr;
		m_proxyCount = 0;
		m_proxyCapacity = 0;
		m_wx = 0.5f;
		m_wy = 0.5f;

		m_rowCount = 1;
		m_columnCount = 2;
		memset(&m_tree, 0, sizeof(m_tree));
		BuildTree();
		m_timeStamp = 0;

		m_startPoint = {0.0f, 0.0f};
		m_endPoint = {0.0f, 0.0f};
		m_queryDrag = false;
		m_rayDrag = false;

		m_validate = true;
	}

	~DynamicTree()
	{
		free(m_proxies);
		free(m_mapArray);
		b2DynamicTree_Destroy(&m_tree);
	}

	void BuildTree()
	{
		b2DynamicTree_Destroy(&m_tree);
		free(m_proxies);
		free(m_mapArray);

		m_proxyCapacity = m_rowCount * m_columnCount;
		m_proxies = static_cast<Proxy*>(malloc(m_proxyCapacity * sizeof(Proxy)));
		m_mapArray = static_cast<struct b2ProxyMap*>(malloc(m_proxyCapacity * sizeof(struct b2ProxyMap)));
		m_proxyCount = 0;

		float y = -4.0f;

		m_tree = b2DynamicTree_Create();

		for (int i = 0; i < m_rowCount; ++i)
		{
			float x = -40.0f;

			for (int j = 0; j < m_columnCount; ++j)
			{
				float fillTest = RandomFloat(0.0f, 1.0f);
				if (fillTest <= m_fill)
				{
					assert(m_proxyCount <= m_proxyCapacity);
					Proxy* p = m_proxies + m_proxyCount;
					p->startPosition = {x, y};
					p->box.lowerBound = {x, y};
					p->box.upperBound = {x + m_wx, y + m_wy};
					p->proxyId = b2DynamicTree_CreateProxy(&m_tree, p->box, b2_defaultCategoryBits, p);
					p->rayStamp = -1;
					p->queryStamp = -1;
					++m_proxyCount;
				}

				x += m_wx;
			}

			y += m_wy;
		}
	}

	void UpdateUI() override
	{
		ImGui::SetNextWindowPos(ImVec2(10.0f, 100.0f));
		ImGui::SetNextWindowSize(ImVec2(250.0f, 220.0f));
		ImGui::Begin("Tree Controls", nullptr, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize);

		bool changed = false;
		if (ImGui::SliderInt("rows", &m_rowCount, 0, 20, "%d"))
		{
			changed = true;
		}

		if (ImGui::SliderInt("columns", &m_columnCount, 0, 20, "%d"))
		{
			changed = true;
		}

		if (ImGui::SliderFloat("fill", &m_fill, 0.0f, 1.0f, "%.2f"))
		{
			changed = true;
		}

		if (ImGui::SliderFloat("move", &m_moveFraction, 0.0f, 1.0f, "%.2f"))
		{
		}

		if (ImGui::SliderFloat("delta", &m_moveDelta, 0.0f, 1.0f, "%.2f"))
		{
		}

		if (ImGui::Checkbox("validate", &m_validate))
		{
		}

		if (ImGui::Button("Rebuild Top Down"))
		{
			assert(m_proxyCount == b2DynamicTree_GetProxyCount(&m_tree));
			b2DynamicTree_RebuildTopDownSAH(&m_tree, m_mapArray, m_proxyCount);
			for (int32_t i = 0; i < m_proxyCount; ++i)
			{
				Proxy* proxy = static_cast<Proxy*>(m_mapArray[i].userData);
				proxy->proxyId = m_mapArray[i].newIndex;
			}
		}

		ImGui::Separator();

		ImGui::Text("mouse button 1: ray cast");
		ImGui::Text("mouse button 1 + shift: query");

		ImGui::End();

		if (changed)
		{
			BuildTree();
		}
	}

	void MouseDown(b2Vec2 p, int button, int mods) override
	{
		if (button == GLFW_MOUSE_BUTTON_1)
		{
			if (mods == 0 && m_queryDrag == false)
			{
				m_rayDrag = true;
				m_startPoint = p;
				m_endPoint = p;
			}
			else if (mods = GLFW_MOD_SHIFT && m_rayDrag == false)
			{
				m_queryDrag = true;
				m_startPoint = p;
				m_endPoint = p;
			}
		}
	}

	void MouseUp(b2Vec2, int button) override
	{
		if (button == GLFW_MOUSE_BUTTON_1)
		{
			m_queryDrag = false;
			m_rayDrag = false;
		}
	}

	void MouseMove(b2Vec2 p) override
	{
		m_endPoint = p;
	}

	void Step(Settings&) override
	{
		if (m_queryDrag)
		{
			b2AABB box = {b2Min(m_startPoint, m_endPoint), b2Max(m_startPoint, m_endPoint)};
			b2DynamicTree_QueryFiltered(&m_tree, box, b2_defaultMaskBits, QueryCallback, this);

			g_draw.DrawAABB(box, {1.0f, 1.0f, 1.0f, 1.0f});
		}

		//m_startPoint = {-42.0f, -6.0f};
		//m_endPoint = {-38.0f, -2.0f};

		if (m_rayDrag)
		{
			b2RayCastInput input = {m_startPoint, m_endPoint, 0.0f, 1.0f};
			b2DynamicTree_RayCast(&m_tree, &input, b2_defaultMaskBits, RayCallback, this);

			g_draw.DrawSegment(m_startPoint, m_endPoint, {1.0f, 1.0f, 1.0f, 1.0f});
			g_draw.DrawPoint(m_startPoint, 5.0f, {0.0f, 1.0f, 0.0f, 1.0f});
			g_draw.DrawPoint(m_endPoint, 5.0f, {1.0f, 0.0f, 0.0f, 1.0f});
		}

		b2Color c = {0.3f, 0.3f, 0.8f, 0.7f};
		b2Color qc = {0.3, 0.8f, 0.3f, 1.0f};

		for (int i = 0; i < m_proxyCount; ++i)
		{
			Proxy* p = m_proxies + i;

			if (p->queryStamp == m_timeStamp || p->rayStamp == m_timeStamp)
			{
				g_draw.DrawAABB(p->box, qc);
			}
			else
			{
				g_draw.DrawAABB(p->box, c);
			}

			float moveTest = RandomFloat(0.0f, 1.0f);
			if (m_moveFraction > moveTest)
			{
				float dx = m_moveDelta * RandomFloat();
				float dy = m_moveDelta * RandomFloat();
				p->box.lowerBound.x = p->startPosition.x + dx;
				p->box.lowerBound.y = p->startPosition.y + dy;
				p->box.upperBound.x = p->startPosition.x + dx + m_wx;
				p->box.upperBound.y = p->startPosition.y + dy + m_wy;

				b2DynamicTree_MoveProxy(&m_tree, p->proxyId, p->box);
			}
		}

		int32_t height = b2DynamicTree_GetHeight(&m_tree);
		float areaRatio = b2DynamicTree_GetAreaRatio(&m_tree);

		int32_t hmin = (int32_t)(ceilf(logf((float)m_proxyCount) / logf(2.0f) - 1.0f));
		g_draw.DrawString(5, m_textLine, "proxies = %d, height = %d, hmin = %d, area ratio = %.1f", m_proxyCount, height, hmin, areaRatio);
		m_textLine += m_textIncrement;

		if (m_validate)
		{
			g_draw.DrawString(5, m_textLine, "validating");
			m_textLine += m_textIncrement;

			b2DynamicTree_Validate(&m_tree);
		}

		m_timeStamp += 1;
	}

	static Sample* Create(const Settings& settings)
	{
		return new DynamicTree(settings);
	}

	b2DynamicTree m_tree;
	int m_rowCount, m_columnCount;
	Proxy* m_proxies;
	struct b2ProxyMap* m_mapArray;
	int m_proxyCapacity;
	int m_proxyCount;
	int m_timeStamp;
	float m_fill;
	float m_moveFraction;
	float m_moveDelta;
	float m_wx, m_wy;

	b2Vec2 m_startPoint;
	b2Vec2 m_endPoint;

	bool m_rayDrag;
	bool m_queryDrag;
	bool m_validate;
};

static bool QueryCallback(int32_t proxyId, void* userData, void* context)
{
	DynamicTree* sample = static_cast<DynamicTree*>(context);
	Proxy* proxy = static_cast<Proxy*>(userData);
	assert(proxy->proxyId == proxyId);
	proxy->queryStamp = sample->m_timeStamp;
	return true;
}

static float RayCallback(const b2RayCastInput* input, int32_t proxyId, void* userData, void* context)
{
	DynamicTree* sample = static_cast<DynamicTree*>(context);
	Proxy* proxy = static_cast<Proxy*>(userData);
	assert(proxy->proxyId == proxyId);
	proxy->rayStamp = sample->m_timeStamp;
	return input->maxFraction;
}

static int sampleIndex = RegisterSample("Collision", "Dynamic Tree", DynamicTree::Create);
