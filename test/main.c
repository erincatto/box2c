// SPDX-FileCopyrightText: 2022 Erin Catto
// SPDX-License-Identifier: MIT

#include "test_macros.h"

extern int MathTest();
extern int CollisionTest();
extern int DistanceTest();

int main(void)
{
	RUN_TEST(MathTest);
	RUN_TEST(CollisionTest);
	RUN_TEST(DistanceTest);
	return 0;
}
