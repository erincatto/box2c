// SPDX-FileCopyrightText: 2022 Erin Catto
// SPDX-License-Identifier: MIT

#define _CRT_SECURE_NO_WARNINGS
#include "settings.h"
#include <sajson.h>
#include <stdio.h>

static const char* fileName = "settings.ini";

// Load a file. You must free the character array.
static bool sReadFile(char*& data, int& size, const char* filename)
{
	FILE* file = fopen(filename, "rb");
	if (file == nullptr)
	{
		return false;
	}

	fseek(file, 0, SEEK_END);
	size = ftell(file);
	fseek(file, 0, SEEK_SET);

	if (size == 0)
	{
		return false;
	}

	data = (char*)malloc(size + 1);
	fread(data, size, 1, file);
	fclose(file);
	data[size] = 0;

	return true;
}

void Settings::Save()
{
	FILE* file = fopen(fileName, "w");
	fprintf(file, "{\n");
	fprintf(file, "  \"testIndex\": %d,\n", m_sampleIndex);
	fprintf(file, "  \"drawShapes\": %s,\n", m_drawShapes ? "true" : "false");
	fprintf(file, "  \"drawJoints\": %s,\n", m_drawJoints ? "true" : "false");
	fprintf(file, "  \"drawAABBs\": %s,\n", m_drawAABBs ? "true" : "false");
	fprintf(file, "  \"drawContactPoints\": %s,\n", m_drawContactPoints ? "true" : "false");
	fprintf(file, "  \"drawContactNormals\": %s,\n", m_drawContactNormals ? "true" : "false");
	fprintf(file, "  \"drawContactImpulse\": %s,\n", m_drawContactImpulse ? "true" : "false");
	fprintf(file, "  \"drawFrictionImpulse\": %s,\n", m_drawFrictionImpulse ? "true" : "false");
	fprintf(file, "  \"drawCOMs\": %s,\n", m_drawCOMs ? "true" : "false");
	fprintf(file, "  \"drawStats\": %s,\n", m_drawStats ? "true" : "false");
	fprintf(file, "  \"drawProfile\": %s,\n", m_drawProfile ? "true" : "false");
	fprintf(file, "  \"enableWarmStarting\": %s,\n", m_enableWarmStarting ? "true" : "false");
	fprintf(file, "  \"enableContinuous\": %s,\n", m_enableContinuous ? "true" : "false");
	fprintf(file, "  \"enableSleep\": %s\n", m_enableSleep ? "true" : "false");
	fprintf(file, "}\n");
	fclose(file);
}

void Settings::Load()
{
	char* data = nullptr;
	int size = 0;
	bool found = sReadFile(data, size, fileName);
	if (found ==  false)
	{
		return;
	}

	const sajson::document& document = sajson::parse(sajson::dynamic_allocation(), sajson::mutable_string_view(size, data));
	if (document.is_valid() == false)
	{
		return;
	}

	sajson::value root = document.get_root();
	int fieldCount = int(root.get_length());
	for (int i = 0; i < fieldCount; ++i)
	{
		sajson::string fieldName = root.get_object_key(i);
		sajson::value fieldValue = root.get_object_value(i);

		if (strncmp(fieldName.data(), "testIndex", fieldName.length()) == 0)
		{
			if (fieldValue.get_type() == sajson::TYPE_INTEGER)
			{
				m_sampleIndex = fieldValue.get_integer_value();
			}
			continue;
		}

		if (strncmp(fieldName.data(), "drawShapes", fieldName.length()) == 0)
		{
			if (fieldValue.get_type() == sajson::TYPE_FALSE)
			{
				m_drawShapes = false;
			}
			else if (fieldValue.get_type() == sajson::TYPE_TRUE)
			{
				m_drawShapes = true;
			}
			continue;
		}
	}

	free(data);
}
