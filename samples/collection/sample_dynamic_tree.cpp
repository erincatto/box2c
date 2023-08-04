// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "sample.h"

#include "box2d/aabb.h"
#include "box2d/dynamic_tree.h"
#include "box2d/math.h"

#include <GLFW/glfw3.h>
#include <imgui.h>

enum UpdateType
{
	Update_Incremental = 0,
	Update_FullRebuild = 1,
	Update_PartialRebuild = 2,
};

struct Proxy
{
	b2AABB box;
	b2AABB fatBox;
	b2Vec2 position;
	b2Vec2 width;
	int32_t proxyId;
	int32_t rayStamp;
	int32_t queryStamp;
	bool moved;
};

static bool QueryCallback(int32_t proxyId, int32_t userData, void* context);
static float RayCallback(const b2RayCastInput* input, int32_t proxyId, int32_t userData, void* context);

// Tests the Box2D bounding volume hierarchy (BVH). The dynamic tree
// can be used independently as a spatial data structure.
class DynamicTree : public Sample
{
  public:
	DynamicTree(const Settings& settings) : Sample(settings)
	{
		m_fill = 0.25f;
		m_moveFraction = 0.05f;
		m_moveDelta = 0.1f;
		m_proxies = nullptr;
		m_proxyCount = 0;
		m_proxyCapacity = 0;
		m_ratio = 5.0f;
		m_grid = 1.0f;

		m_moveBuffer = nullptr;
		m_moveCount = 0;

		m_rowCount = g_sampleDebug ? 100 : 1000;
		m_columnCount = g_sampleDebug ? 100 : 1000;
		memset(&m_tree, 0, sizeof(m_tree));
		BuildTree();
		m_timeStamp = 0;
		m_updateType = Update_Incremental;

		m_startPoint = {0.0f, 0.0f};
		m_endPoint = {0.0f, 0.0f};
		m_queryDrag = false;
		m_rayDrag = false;
		m_validate = true;
	}

	~DynamicTree()
	{
		free(m_proxies);
		free(m_moveBuffer);
		b2DynamicTree_Destroy(&m_tree);
	}

	void BuildTree()
	{
		b2DynamicTree_Destroy(&m_tree);
		free(m_proxies);
		free(m_moveBuffer);

		m_proxyCapacity = m_rowCount * m_columnCount;
		m_proxies = static_cast<Proxy*>(malloc(m_proxyCapacity * sizeof(Proxy)));
		m_proxyCount = 0;

		m_moveBuffer = static_cast<int*>(malloc(m_proxyCapacity * sizeof(int)));
		m_moveCount = 0;

		float y = -4.0f;

		bool isStatic = false;
		m_tree = b2DynamicTree_Create();

		const b2Vec2 aabbExtension = {b2_aabbMargin, b2_aabbMargin};

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
					p->position = {x, y};

					float ratio = RandomFloat(1.0f, m_ratio);
					float width = RandomFloat(0.1f, 0.5f);
					if (RandomFloat() > 0.0f)
					{
						p->width.x =  ratio * width;
						p->width.y =  width;
					}
					else
					{
						p->width.x =  width;
						p->width.y =  ratio * width;
					}

					p->box.lowerBound = {x, y};
					p->box.upperBound = {x + p->width.x, y + p->width.y};
					p->fatBox.lowerBound = b2Sub(p->box.lowerBound, aabbExtension);
					p->fatBox.upperBound = b2Add(p->box.upperBound, aabbExtension);

					p->proxyId = b2DynamicTree_CreateProxy(&m_tree, p->fatBox, b2_defaultCategoryBits, m_proxyCount);
					p->rayStamp = -1;
					p->queryStamp = -1;
					p->moved = false;
					++m_proxyCount;
				}

				x += m_grid;
			}

			y += m_grid;
		}
	}

	void UpdateUI() override
	{
		ImGui::SetNextWindowPos(ImVec2(10.0f, 100.0f));
		ImGui::SetNextWindowSize(ImVec2(240.0f, 340.0f));
		ImGui::Begin("Tree Controls", nullptr, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize);

		bool changed = false;
		if (ImGui::SliderInt("rows", &m_rowCount, 0, 1000, "%d"))
		{
			changed = true;
		}

		if (ImGui::SliderInt("columns", &m_columnCount, 0, 1000, "%d"))
		{
			changed = true;
		}

		if (ImGui::SliderFloat("fill", &m_fill, 0.0f, 1.0f, "%.2f"))
		{
			changed = true;
		}

		if (ImGui::SliderFloat("grid", &m_grid, 0.5f, 2.0f, "%.2f"))
		{
			changed = true;
		}

		if (ImGui::SliderFloat("ratio", &m_ratio, 1.0f, 10.0f, "%.2f"))
		{
			changed = true;
		}

		if (ImGui::SliderFloat("move", &m_moveFraction, 0.0f, 1.0f, "%.2f"))
		{
		}

		if (ImGui::SliderFloat("delta", &m_moveDelta, 0.0f, 1.0f, "%.2f"))
		{
		}

		if (ImGui::RadioButton("Incremental", m_updateType == Update_Incremental))
		{
			m_updateType = Update_Incremental;
			changed = true;
		}

		if (ImGui::RadioButton("Full Rebuild", m_updateType == Update_FullRebuild))
		{
			m_updateType = Update_FullRebuild;
			changed = true;
		}

		if (ImGui::RadioButton("Partial Rebuild", m_updateType == Update_PartialRebuild))
		{
			m_updateType = Update_PartialRebuild;
			changed = true;
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

		// m_startPoint = {-42.0f, -6.0f};
		// m_endPoint = {-38.0f, -2.0f};

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

		const b2Vec2 aabbExtension = {b2_aabbMargin, b2_aabbMargin};

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

				p->position.x += dx;
				p->position.y += dy;

				p->box.lowerBound.x = p->position.x + dx;
				p->box.lowerBound.y = p->position.y + dy;
				p->box.upperBound.x = p->position.x + dx + p->width.x;
				p->box.upperBound.y = p->position.y + dy + p->width.y;

				if (b2AABB_Contains(p->fatBox, p->box) == false)
				{
					p->fatBox.lowerBound = b2Sub(p->box.lowerBound, aabbExtension);
					p->fatBox.upperBound = b2Add(p->box.lowerBound, aabbExtension);
					p->moved = true;
				}
				else
				{
					p->moved = false;
				}
			}
			else
			{
				p->moved = false;
			}
		}

		switch (m_updateType)
		{
		case Update_Incremental:
		{
			b2Timer timer = b2CreateTimer();
			for (int i = 0; i < m_proxyCount; ++i)
			{
				Proxy* p = m_proxies + i;
				if (p->moved)
				{
					b2DynamicTree_MoveProxy(&m_tree, p->proxyId, p->fatBox);
				}
			}
			float ms = b2GetMilliseconds(&timer);
			g_draw.DrawString(5, m_textLine, "incremental : %.3f ms", ms);
			m_textLine += m_textIncrement;
		}
		break;

		case Update_FullRebuild:
		{
			for (int i = 0; i < m_proxyCount; ++i)
			{
				Proxy* p = m_proxies + i;
				if (p->moved)
				{
					b2DynamicTree_EnlargeProxy(&m_tree, p->proxyId, p->fatBox);
				}
			}

			b2Timer timer = b2CreateTimer();
			int32_t boxCount = b2DynamicTree_Rebuild(&m_tree, true);
			float ms = b2GetMilliseconds(&timer);
			g_draw.DrawString(5, m_textLine, "full build %d : %.3f ms", boxCount, ms);
			m_textLine += m_textIncrement;
		}
		break;

		case Update_PartialRebuild:
		{
			for (int i = 0; i < m_proxyCount; ++i)
			{
				Proxy* p = m_proxies + i;
				if (p->moved)
				{
					b2DynamicTree_EnlargeProxy(&m_tree, p->proxyId, p->fatBox);
				}
			}

			b2Timer timer = b2CreateTimer();
			int32_t boxCount = b2DynamicTree_Rebuild(&m_tree, false);
			float ms = b2GetMilliseconds(&timer);
			g_draw.DrawString(5, m_textLine, "partial rebuild %d : %.3f ms", boxCount, ms);
			m_textLine += m_textIncrement;
		}
		break;

		default:
			break;
		}

		int32_t height = b2DynamicTree_GetHeight(&m_tree);
		float areaRatio = b2DynamicTree_GetAreaRatio(&m_tree);

		int32_t hmin = (int32_t)(ceilf(logf((float)m_proxyCount) / logf(2.0f) - 1.0f));
		g_draw.DrawString(5, m_textLine, "proxies = %d, height = %d, hmin = %d, area ratio = %.1f", m_proxyCount, height, hmin, areaRatio);
		m_textLine += m_textIncrement;

		b2DynamicTree_Validate(&m_tree);

		m_timeStamp += 1;
	}

	static Sample* Create(const Settings& settings)
	{
		return new DynamicTree(settings);
	}

	b2DynamicTree m_tree;
	int m_rowCount, m_columnCount;
	Proxy* m_proxies;
	int* m_moveBuffer;
	int m_moveCount;
	int m_proxyCapacity;
	int m_proxyCount;
	int m_timeStamp;
	int m_updateType;
	float m_fill;
	float m_moveFraction;
	float m_moveDelta;
	float m_ratio;
	float m_grid;

	b2Vec2 m_startPoint;
	b2Vec2 m_endPoint;

	bool m_rayDrag;
	bool m_queryDrag;
	bool m_validate;
};

static bool QueryCallback(int32_t proxyId, int32_t userData, void* context)
{
	DynamicTree* sample = static_cast<DynamicTree*>(context);
	Proxy* proxy = sample->m_proxies + userData;
	assert(proxy->proxyId == proxyId);
	proxy->queryStamp = sample->m_timeStamp;
	return true;
}

static float RayCallback(const b2RayCastInput* input, int32_t proxyId, int32_t userData, void* context)
{
	DynamicTree* sample = static_cast<DynamicTree*>(context);
	Proxy* proxy = sample->m_proxies + userData;
	assert(proxy->proxyId == proxyId);
	proxy->rayStamp = sample->m_timeStamp;
	return input->maxFraction;
}

static int sampleIndex = RegisterSample("Collision", "Dynamic Tree", DynamicTree::Create);
