// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#define _CRT_SECURE_NO_WARNINGS
#define IMGUI_DISABLE_OBSOLETE_FUNCTIONS 1

#include <glad/glad.h>

#include <GLFW/glfw3.h>

#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>

#include "draw.h"
#include "sample.h"
#include "settings.h"

#include "box2d/allocate.h"
#include "box2d/constants.h"
#include "box2d/timer.h"
#include "box2d/math.h"

#include <stdio.h>
#include <stdlib.h>

#if defined(_WIN32)
#include <crtdbg.h>
#endif

GLFWwindow* g_mainWindow = nullptr;
static int32_t s_selection = 0;
static Sample* s_sample = nullptr;
static Settings s_settings;
static bool s_rightMouseDown = false;
static b2Vec2 s_clickPointWS = b2Vec2_zero;
static float s_displayScale = 1.0f;

void* AllocFcn(int32_t size)
{
	return malloc(size);
}

void FreeFcn(void* mem)
{
	free(mem);
}

void glfwErrorCallback(int error, const char* description)
{
	fprintf(stderr, "GLFW error occured. Code: %d. Description: %s\n", error, description);
}

static inline int CompareSamples(const void* a, const void* b)
{
	SampleEntry* sa = (SampleEntry*)a;
	SampleEntry* sb = (SampleEntry*)b;

	int result = strcmp(sa->category, sb->category);
	if (result == 0)
	{
		result = strcmp(sa->name, sb->name);
	}

	return result;
}

static void SortTests()
{
	qsort(g_sampleEntries, g_sampleCount, sizeof(SampleEntry), CompareSamples);
}

static void RestartTest()
{
	delete s_sample;
	s_sample = g_sampleEntries[s_settings.m_sampleIndex].createFcn();
}

static void CreateUI(GLFWwindow* window, const char* glslVersion)
{
	IMGUI_CHECKVERSION();
	ImGui::CreateContext();

	bool success;
	success = ImGui_ImplGlfw_InitForOpenGL(window, false);
	if (success == false)
	{
		printf("ImGui_ImplGlfw_InitForOpenGL failed\n");
		assert(false);
	}

	success = ImGui_ImplOpenGL3_Init(glslVersion);
	if (success == false)
	{
		printf("ImGui_ImplOpenGL3_Init failed\n");
		assert(false);
	}

	// Search for font file
	const char* fontPath1 = "data/droid_sans.ttf";
	const char* fontPath2 = "../data/droid_sans.ttf";
	const char* fontPath = nullptr;
	FILE* file1 = fopen(fontPath1, "rb");
	FILE* file2 = fopen(fontPath2, "rb");
	if (file1)
	{
		fontPath = fontPath1;
		fclose(file1);
	}

	if (file2)
	{
		fontPath = fontPath2;
		fclose(file2);
	}

	if (fontPath)
	{
		ImGui::GetIO().Fonts->AddFontFromFileTTF(fontPath, 14.0f * s_displayScale);
	}
}

static void ResizeWindowCallback(GLFWwindow*, int width, int height)
{
	g_camera.m_width = width;
	g_camera.m_height = height;
	s_settings.m_windowWidth = width;
	s_settings.m_windowHeight = height;
}

static void KeyCallback(GLFWwindow* window, int key, int scancode, int action, int mods)
{
	ImGui_ImplGlfw_KeyCallback(window, key, scancode, action, mods);
	if (ImGui::GetIO().WantCaptureKeyboard)
	{
		return;
	}

	if (action == GLFW_PRESS)
	{
		switch (key)
		{
		case GLFW_KEY_ESCAPE:
			// Quit
			glfwSetWindowShouldClose(g_mainWindow, GL_TRUE);
			break;

		case GLFW_KEY_LEFT:
			// Pan left
			if (mods == GLFW_MOD_CONTROL)
			{
				b2Vec2 newOrigin = {2.0f, 0.0f};
				s_sample->ShiftOrigin(newOrigin);
			}
			else
			{
				g_camera.m_center.x -= 0.5f;
			}
			break;

		case GLFW_KEY_RIGHT:
			// Pan right
			if (mods == GLFW_MOD_CONTROL)
			{
				b2Vec2 newOrigin = {-2.0f, 0.0f};
				s_sample->ShiftOrigin(newOrigin);
			}
			else
			{
				g_camera.m_center.x += 0.5f;
			}
			break;

		case GLFW_KEY_DOWN:
			// Pan down
			if (mods == GLFW_MOD_CONTROL)
			{
				b2Vec2 newOrigin = {0.0f, 2.0f};
				s_sample->ShiftOrigin(newOrigin);
			}
			else
			{
				g_camera.m_center.y -= 0.5f;
			}
			break;

		case GLFW_KEY_UP:
			// Pan up
			if (mods == GLFW_MOD_CONTROL)
			{
				b2Vec2 newOrigin = {0.0f, -2.0f};
				s_sample->ShiftOrigin(newOrigin);
			}
			else
			{
				g_camera.m_center.y += 0.5f;
			}
			break;

		case GLFW_KEY_HOME:
			g_camera.ResetView();
			break;

		case GLFW_KEY_Z:
			// Zoom out
			g_camera.m_zoom = B2_MIN(1.1f * g_camera.m_zoom, 20.0f);
			break;

		case GLFW_KEY_X:
			// Zoom in
			g_camera.m_zoom = B2_MAX(0.9f * g_camera.m_zoom, 0.02f);
			break;

		case GLFW_KEY_R:
			RestartTest();
			break;

		case GLFW_KEY_O:
			s_settings.m_singleStep = true;
			break;

		case GLFW_KEY_P:
			s_settings.m_pause = !s_settings.m_pause;
			break;

		case GLFW_KEY_LEFT_BRACKET:
			// Switch to previous test
			--s_selection;
			if (s_selection < 0)
			{
				s_selection = g_sampleCount - 1;
			}
			break;

		case GLFW_KEY_RIGHT_BRACKET:
			// Switch to next test
			++s_selection;
			if (s_selection == g_sampleCount)
			{
				s_selection = 0;
			}
			break;

		case GLFW_KEY_TAB:
			g_draw.m_showUI = !g_draw.m_showUI;

		default:
			if (s_sample)
			{
				s_sample->Keyboard(key);
			}
		}
	}
	else if (action == GLFW_RELEASE)
	{
		s_sample->KeyboardUp(key);
	}
	// else GLFW_REPEAT
}

static void CharCallback(GLFWwindow* window, unsigned int c)
{
	ImGui_ImplGlfw_CharCallback(window, c);
}

static void MouseButtonCallback(GLFWwindow* window, int32_t button, int32_t action, int32_t mods)
{
	ImGui_ImplGlfw_MouseButtonCallback(window, button, action, mods);

	if (ImGui::GetIO().WantCaptureMouse)
	{
		return;
	}

	double xd, yd;
	glfwGetCursorPos(g_mainWindow, &xd, &yd);
	b2Vec2 ps = {(float)xd, (float)yd};

	// Use the mouse to move things around.
	if (button == GLFW_MOUSE_BUTTON_1)
	{
		b2Vec2 pw = g_camera.ConvertScreenToWorld(ps);
		if (action == GLFW_PRESS)
		{
			s_sample->MouseDown(pw, button, mods);
		}

		if (action == GLFW_RELEASE)
		{
			s_sample->MouseUp(pw, button);
		}
	}
	else if (button == GLFW_MOUSE_BUTTON_2)
	{
		if (action == GLFW_PRESS)
		{
			s_clickPointWS = g_camera.ConvertScreenToWorld(ps);
			s_rightMouseDown = true;
		}

		if (action == GLFW_RELEASE)
		{
			s_rightMouseDown = false;
		}
	}
}

static void MouseMotionCallback(GLFWwindow*, double xd, double yd)
{
	b2Vec2 ps = {(float)xd, (float)yd};

	b2Vec2 pw = g_camera.ConvertScreenToWorld(ps);
	s_sample->MouseMove(pw);

	if (s_rightMouseDown)
	{
		b2Vec2 diff = b2Sub(pw, s_clickPointWS);
		g_camera.m_center.x -= diff.x;
		g_camera.m_center.y -= diff.y;
		s_clickPointWS = g_camera.ConvertScreenToWorld(ps);
	}
}

static void ScrollCallback(GLFWwindow* window, double dx, double dy)
{
	ImGui_ImplGlfw_ScrollCallback(window, dx, dy);
	if (ImGui::GetIO().WantCaptureMouse)
	{
		return;
	}

	if (dy > 0)
	{
		g_camera.m_zoom /= 1.1f;
	}
	else
	{
		g_camera.m_zoom *= 1.1f;
	}
}

static void UpdateUI()
{
	float menuWidth = 180.0f * s_displayScale;
	if (g_draw.m_showUI)
	{
		ImGui::SetNextWindowPos({g_camera.m_width - menuWidth - 10.0f, 10.0f});
		ImGui::SetNextWindowSize({menuWidth, g_camera.m_height - 20.0f});

		ImGui::Begin("Tools", &g_draw.m_showUI, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse);

		if (ImGui::BeginTabBar("ControlTabs", ImGuiTabBarFlags_None))
		{
			if (ImGui::BeginTabItem("Controls"))
			{
				ImGui::SliderInt("Vel Iters", &s_settings.m_velocityIterations, 0, 50);
				ImGui::SliderInt("Pos Iters", &s_settings.m_positionIterations, 0, 50);
				ImGui::SliderFloat("Hertz", &s_settings.m_hertz, 5.0f, 120.0f, "%.0f hz");

				ImGui::Separator();

				ImGui::Checkbox("Sleep", &s_settings.m_enableSleep);
				ImGui::Checkbox("Warm Starting", &s_settings.m_enableWarmStarting);
				ImGui::Checkbox("Continuous", &s_settings.m_enableContinuous);

				ImGui::Separator();

				ImGui::Checkbox("Shapes", &s_settings.m_drawShapes);
				ImGui::Checkbox("Joints", &s_settings.m_drawJoints);
				ImGui::Checkbox("AABBs", &s_settings.m_drawAABBs);
				ImGui::Checkbox("Contact Points", &s_settings.m_drawContactPoints);
				ImGui::Checkbox("Contact Normals", &s_settings.m_drawContactNormals);
				ImGui::Checkbox("Contact Impulses", &s_settings.m_drawContactImpulse);
				ImGui::Checkbox("Friction Impulses", &s_settings.m_drawFrictionImpulse);
				ImGui::Checkbox("Center of Masses", &s_settings.m_drawCOMs);
				ImGui::Checkbox("Statistics", &s_settings.m_drawStats);
				ImGui::Checkbox("Profile", &s_settings.m_drawProfile);

				ImVec2 button_sz = ImVec2(-1, 0);
				if (ImGui::Button("Pause (P)", button_sz))
				{
					s_settings.m_pause = !s_settings.m_pause;
				}

				if (ImGui::Button("Single Step (O)", button_sz))
				{
					s_settings.m_singleStep = !s_settings.m_singleStep;
				}

				if (ImGui::Button("Restart (R)", button_sz))
				{
					RestartTest();
				}

				if (ImGui::Button("Quit", button_sz))
				{
					glfwSetWindowShouldClose(g_mainWindow, GL_TRUE);
				}

				ImGui::EndTabItem();
			}

			ImGuiTreeNodeFlags leafNodeFlags = ImGuiTreeNodeFlags_OpenOnArrow | ImGuiTreeNodeFlags_OpenOnDoubleClick;
			leafNodeFlags |= ImGuiTreeNodeFlags_Leaf | ImGuiTreeNodeFlags_NoTreePushOnOpen;

			ImGuiTreeNodeFlags nodeFlags = ImGuiTreeNodeFlags_OpenOnArrow | ImGuiTreeNodeFlags_OpenOnDoubleClick;

			if (ImGui::BeginTabItem("Tests"))
			{
				int categoryIndex = 0;
				const char* category = g_sampleEntries[categoryIndex].category;
				int i = 0;
				while (i < g_sampleCount)
				{
					bool categorySelected = strcmp(category, g_sampleEntries[s_settings.m_sampleIndex].category) == 0;
					ImGuiTreeNodeFlags nodeSelectionFlags = categorySelected ? ImGuiTreeNodeFlags_Selected : 0;
					bool nodeOpen = ImGui::TreeNodeEx(category, nodeFlags | nodeSelectionFlags);

					if (nodeOpen)
					{
						while (i < g_sampleCount && strcmp(category, g_sampleEntries[i].category) == 0)
						{
							ImGuiTreeNodeFlags selectionFlags = 0;
							if (s_settings.m_sampleIndex == i)
							{
								selectionFlags = ImGuiTreeNodeFlags_Selected;
							}
							ImGui::TreeNodeEx((void*)(intptr_t)i, leafNodeFlags | selectionFlags, "%s", g_sampleEntries[i].name);
							if (ImGui::IsItemClicked())
							{
								s_selection = i;
							}
							++i;
						}
						ImGui::TreePop();
					}
					else
					{
						while (i < g_sampleCount && strcmp(category, g_sampleEntries[i].category) == 0)
						{
							++i;
						}
					}

					if (i < g_sampleCount)
					{
						category = g_sampleEntries[i].category;
						categoryIndex = i;
					}
				}
				ImGui::EndTabItem();
			}
			ImGui::EndTabBar();
		}

		ImGui::End();

		s_sample->UpdateUI();
	}
}

//
int main(int, char**)
{
#if defined(_WIN32)
	// Enable memory-leak reports
	_CrtSetDbgFlag(_CRTDBG_LEAK_CHECK_DF | _CrtSetDbgFlag(_CRTDBG_REPORT_FLAG));
#endif

	// Install memory hooks
	b2SetAllocator(AllocFcn, FreeFcn);

	char buffer[128];

	s_settings.Load();
	SortTests();

	glfwSetErrorCallback(glfwErrorCallback);

	g_camera.m_width = s_settings.m_windowWidth;
	g_camera.m_height = s_settings.m_windowHeight;

	if (glfwInit() == 0)
	{
		fprintf(stderr, "Failed to initialize GLFW\n");
		return -1;
	}

#if __APPLE__
	const char* glslVersion = "#version 150";
#else
	const char* glslVersion = NULL;
#endif

	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

	// MSAA
	glfwWindowHint(GLFW_SAMPLES, 4);

	sprintf(buffer, "Box2D Version %d.%d.%d", b2_version.major, b2_version.minor, b2_version.revision);

	bool fullscreen = false;
	if (fullscreen)
	{
		g_mainWindow = glfwCreateWindow(1920, 1080, buffer, glfwGetPrimaryMonitor(), NULL);
	}
	else
	{
		g_mainWindow = glfwCreateWindow(g_camera.m_width, g_camera.m_height, buffer, NULL, NULL);
	}

	if (g_mainWindow == NULL)
	{
		fprintf(stderr, "Failed to open GLFW g_mainWindow.\n");
		glfwTerminate();
		return -1;
	}

	glfwGetWindowContentScale(g_mainWindow, &s_displayScale, &s_displayScale);

	glfwMakeContextCurrent(g_mainWindow);

	// Load OpenGL functions using glad
	if (!gladLoadGL())
	{
		fprintf(stderr, "Failed to initialize glad\n");
		glfwTerminate();
		return -1;
	}

	printf("GL %d.%d\n", GLVersion.major, GLVersion.minor);
	printf("OpenGL %s, GLSL %s\n", glGetString(GL_VERSION), glGetString(GL_SHADING_LANGUAGE_VERSION));

	glfwSetWindowSizeCallback(g_mainWindow, ResizeWindowCallback);
	glfwSetKeyCallback(g_mainWindow, KeyCallback);
	glfwSetCharCallback(g_mainWindow, CharCallback);
	glfwSetMouseButtonCallback(g_mainWindow, MouseButtonCallback);
	glfwSetCursorPosCallback(g_mainWindow, MouseMotionCallback);
	glfwSetScrollCallback(g_mainWindow, ScrollCallback);

	g_draw.Create();
	CreateUI(g_mainWindow, glslVersion);

	s_settings.m_sampleIndex = B2_CLAMP(s_settings.m_sampleIndex, 0, g_sampleCount - 1);
	s_selection = s_settings.m_sampleIndex;
	s_sample = g_sampleEntries[s_settings.m_sampleIndex].createFcn();

	glClearColor(0.2f, 0.2f, 0.2f, 1.0f);

	float frameTime = 0.0;

	int32_t frame = 0;

	while (!glfwWindowShouldClose(g_mainWindow))
	{
		double time1 = glfwGetTime();

		glfwGetWindowSize(g_mainWindow, &g_camera.m_width, &g_camera.m_height);

		int bufferWidth, bufferHeight;
		glfwGetFramebufferSize(g_mainWindow, &bufferWidth, &bufferHeight);
		glViewport(0, 0, bufferWidth, bufferHeight);

		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		ImGui_ImplOpenGL3_NewFrame();
		ImGui_ImplGlfw_NewFrame();

		ImGui::NewFrame();

		if (g_draw.m_showUI)
		{
			ImGui::SetNextWindowPos(ImVec2(0.0f, 0.0f));
			ImGui::SetNextWindowSize(ImVec2(float(g_camera.m_width), float(g_camera.m_height)));
			ImGui::SetNextWindowBgAlpha(0.0f);
			ImGui::Begin("Overlay", nullptr,
			             ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoInputs | ImGuiWindowFlags_AlwaysAutoResize |
			                 ImGuiWindowFlags_NoScrollbar);
			ImGui::End();

			const SampleEntry& entry = g_sampleEntries[s_settings.m_sampleIndex];
			sprintf(buffer, "%s : %s", entry.category, entry.name);
			s_sample->DrawTitle(buffer);
		}

		s_sample->Step(s_settings);

		g_draw.Flush();

		UpdateUI();

		// ImGui::ShowDemoWindow();

		if (g_draw.m_showUI)
		{
			sprintf(buffer, "%.1f ms", 1000.0f * frameTime);
			g_draw.DrawString(5, g_camera.m_height - 20, buffer);
		}

		ImGui::Render();
		ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

		glfwSwapBuffers(g_mainWindow);

		if (s_selection != s_settings.m_sampleIndex)
		{
			s_settings.m_sampleIndex = s_selection;
			delete s_sample;
			s_sample = g_sampleEntries[s_settings.m_sampleIndex].createFcn();
			g_camera.ResetView();
		}

		glfwPollEvents();

		// Limit frame rate to 60Hz
		double time2 = glfwGetTime();
		double targetTime = time1 + 1.0f / 60.0f;
		int loopCount = 0;
		while (time2 < targetTime)
		{
			b2SleepMilliseconds(0.0f);
			time2 = glfwGetTime();
			++loopCount;
		}

		frameTime = (float)(time2 - time1);
		//if (frame % 17 == 0)
		//{
		//	printf("loop count = %d, frame time = %.1f\n", loopCount, 1000.0f * frameTime);
		//}
		++frame;
	}

	delete s_sample;
	s_sample = nullptr;

	g_draw.Destroy();
	ImGui_ImplOpenGL3_Shutdown();
	ImGui_ImplGlfw_Shutdown();
	glfwTerminate();

	s_settings.Save();

#if defined(_WIN32)
	_CrtDumpMemoryLeaks();
#endif

	return 0;
}
