// SPDX-FileCopyrightText: 2022 Erin Catto
// SPDX-License-Identifier: MIT

#include "test_macros.h"

extern bool MathTest();

int main(void)
{
	RUN_TEST(MathTest);
	return 0;
}
