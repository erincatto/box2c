// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

/// Returns the number of elements of an array
#define B2_ARRAY_COUNT(A) (int)(sizeof(A) / sizeof(A[0]))

/// Used to prevent the compiler from warning about unused variables
#define B2_MAYBE_UNUSED(x) ((void)(x))
