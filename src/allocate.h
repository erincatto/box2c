// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once
#include <stdint.h>

void* b2Alloc(int32_t size);
void b2Free(void* mem, int32_t size);
