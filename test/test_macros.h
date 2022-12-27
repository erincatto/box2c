// SPDX-FileCopyrightText: 2022 Erin Catto
// SPDX-License-Identifier: MIT

#include <stdbool.h>
#include <stdio.h>
#include <assert.h>

#define RUN_TEST(T) \
	do { \
		bool result = T(); \
		if (result == false) \
		{ \
			printf("test failed: " #T "\n"); \
			return 1; \
		} \
	} while (false)

#define ENSURE(C) \
	do { \
		printf("huh?: " #C "\n"); \
		if ((C) == false) \
		{ \
			printf("condition false: " #C "\n"); \
			assert(false); \
			return false; \
		} \
	} while (false)
