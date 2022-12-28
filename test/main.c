// SPDX-FileCopyrightText: 2022 Erin Catto
// SPDX-License-Identifier: MIT

#include "test_macros.h"

extern int MathTest();
extern int CollisionTest();

int main(void)
{
	RUN_TEST(MathTest);
	RUN_TEST(CollisionTest);
	return 0;
}
