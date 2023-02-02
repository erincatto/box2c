// SPDX-FileCopyrightText: 2022 Erin Catto
// SPDX-License-Identifier: MIT

#include "test_macros.h"

extern int MathTest();
extern int CollisionTest();
extern int DistanceTest();
extern int HelloWorld();
extern int ShapeTest();

int main(void)
{
	RUN_TEST(MathTest);
	RUN_TEST(CollisionTest);
	RUN_TEST(DistanceTest);
	RUN_TEST(HelloWorld);
	RUN_TEST(ShapeTest);

	printf("======================================\n");
	printf("All Box2D tests passed!\n");

	return 0;
}
