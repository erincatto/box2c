// SPDX-FileCopyrightText: 2022 Erin Catto
// SPDX-License-Identifier: MIT

#include <stdbool.h>
#include <stdio.h>
#include <assert.h>

#define RUN_TEST(T) \
	do { \
		int result = T(); \
		if (result == 1) \
		{ \
			printf("test failed: " #T "\n"); \
			return 1; \
		} \
		else \
		{ \
			printf("test passed: " #T "\n"); \
		} \
	} while (false)

#define ENSURE(C) \
	do { \
		if ((C) == false) \
		{ \
			printf("condition false: " #C "\n"); \
			assert(false); \
			return 1; \
		} \
	} while (false)

#define ENSURE_SMALL(C, tol) \
	do { \
		if ((C) < -(tol) || (tol) < (C)) \
		{ \
			printf("condition false: abs(" #C ") < %g\n", tol); \
			assert(false); \
			return 1; \
		} \
	} while (false)
