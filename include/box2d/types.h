// SPDX-FileCopyrightText: 2022 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
    #define B2_LITERAL(T) T
#else
    #define B2_LITERAL(T) (T)
#endif

#define B2_ARRAY_COUNT(A) (sizeof(A) / sizeof(A[0]))

typedef struct b2Vec2
{
    float x, y;
} b2Vec2;
