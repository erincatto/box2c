// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "core.h"
#include "box2d/constants.h"

#include <stdio.h>

_Static_assert(b2_maxPolygonVertices > 2, "must be 3 or more");
_Static_assert(b2_maxWorlds > 0, "must be 1 or more");

int b2DefaultAssertFcn(const char* condition, const char* fileName, int lineNumber)
{
	printf("BOX2D ASSERTION: %s, %s, line %d\n", condition, fileName, lineNumber);
	return 1;
}

b2AssertFcn* Box2DAssertCallback = b2DefaultAssertFcn;
