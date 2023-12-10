// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "test_macros.h"
#include "table.h"
#include "box2d/timer.h"

#define SET_SPAN 317
#define ITEM_COUNT ((SET_SPAN * SET_SPAN - SET_SPAN) / 2)


int TableTest(void)
{
	const int32_t N = SET_SPAN;
	const uint32_t itemCount = ITEM_COUNT;
	bool removed[ITEM_COUNT] = {0};

	for (int32_t iter = 0; iter < 1; ++iter)
	{
		b2HashSet set = b2CreateSet(16);

		// Fill set
		for (int32_t i = 0; i < N; ++i)
		{
			for (int32_t j = i + 1; j < N; ++j)
			{
				uint64_t key = B2_SHAPE_PAIR_KEY(i, j);
				b2AddKey(&set, key);
			}
		}

		ENSURE(set.count == itemCount);

		// Remove a portion of the set
		int32_t k = 0;
		uint32_t removeCount = 0;
		for (int32_t i = 0; i < N; ++i)
		{
			for (int32_t j = i + 1; j < N; ++j)
			{
				if (j == i + 1)
				{
					uint64_t key = B2_SHAPE_PAIR_KEY(i, j);
					b2RemoveKey(&set, key);
					removed[k++] = true;
					removeCount += 1;
				}
				else
				{
					removed[k++] = false;
				}
			}
		}

		ENSURE(set.count == (itemCount - removeCount));

#if B2_DEBUG
		extern int32_t g_probeCount;
		g_probeCount = 0;
#endif

		// Test key search
		// ~5ns per search on an AMD 7950x
		b2Timer timer = b2CreateTimer();

		k = 0;
		for (int32_t i = 0; i < N; ++i)
		{
			for (int32_t j = i + 1; j < N; ++j)
			{
				uint64_t key = B2_SHAPE_PAIR_KEY(j, i);
				ENSURE(b2ContainsKey(&set, key) || removed[k]);
				k += 1;
			}
		}

		// uint64_t ticks = b2GetTicks(&timer);
		// printf("set ticks = %llu\n", ticks);

		float ms = b2GetMilliseconds(&timer);
		printf("set: count = %d, b2ContainsKey = %.5f ms, ave = %.5f us\n", itemCount, ms, 1000.0f * ms / itemCount);

#if B2_DEBUG
		float aveProbeCount = (float)g_probeCount / (float)itemCount;
		printf("item count = %d, probe count = %d, ave probe count %.2f\n", itemCount, g_probeCount, aveProbeCount);
#endif

		// Remove all keys from set
		for (int32_t i = 0; i < N; ++i)
		{
			for (int32_t j = i + 1; j < N; ++j)
			{
				uint64_t key = B2_SHAPE_PAIR_KEY(i, j);
				b2RemoveKey(&set, key);
			}
		}

		ENSURE(set.count == 0);

		b2DestroySet(&set);
	}

	return 0;
}
