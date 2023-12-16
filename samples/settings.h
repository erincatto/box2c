// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

// todo add camera and draw and remove globals
struct Settings
{
	void Save();
	void Load();

	int sampleIndex = 0;
	int windowWidth = 1920;
	int windowHeight = 1080;
	float hertz = 60.0f;
	int velocityIterations = 8;
	int relaxIterations = 3;
	int workerCount = 1;
	bool drawShapes = true;
	bool drawJoints = true;
	bool drawAABBs = false;
	bool drawContactPoints = false;
	bool drawContactNormals = false;
	bool drawContactImpulses = false;
	bool drawFrictionImpulses = false;
	bool drawMass = false;
	bool drawGraphColors = false;
	bool drawStats = false;
	bool drawProfile = false;
	bool enableWarmStarting = true;
	bool enableContinuous = true;
	bool enableSleep = false;
	bool pause = false;
	bool singleStep = false;
	bool restart = false;
};
