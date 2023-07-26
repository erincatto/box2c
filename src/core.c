// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "core.h"

#include <stdio.h>

int b2DefaultAssertFcn(const char* condition, const char* fileName, int lineNumber)
{
	printf("BOX2D ASSERTION: %s, %s, line %d\n", condition, fileName, lineNumber);
	return 1;
}

b2AssertFcn* Box2DAssertCallback = b2DefaultAssertFcn;
