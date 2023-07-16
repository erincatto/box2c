// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "test_macros.h"
#include "table.h"
#include "box2d/timer.h"

int TableTest()
{
	for (int32_t iter = 0; iter < 1; ++iter)
	{
		b2Set set = b2CreateSet(16);

		const int32_t N = 1024;
		const int32_t itemCount = (N * N - N) / 2;

		for (int32_t i = 0; i < N; ++i)
		{
			for (int32_t j = i + 1; j < N; ++j)
			{
				uint64_t key = B2_PROXY_PAIR_KEY(i, j);
				b2AddKey(&set, key);
			}
		}

		ENSURE(set.count == itemCount);

#ifdef _DEBUG
		extern int32_t g_probeCount;
		g_probeCount = 0;
#endif

		b2Timer timer = b2CreateTimer();

		for (int32_t i = 0; i < N; ++i)
		{
			for (int32_t j = i + 1; j < N; ++j)
			{
				uint64_t key = B2_PROXY_PAIR_KEY(j, i);
				ENSURE(b2ContainsKey(&set, key));
			}
		}

		// uint64_t ticks = b2GetTicks(&timer);
		// printf("set ticks = %llu\n", ticks);

		float ms = b2GetMilliseconds(&timer);
		printf("item count = %d, contains = %.5f ms, ave = %.5f us\n", itemCount, ms, 1000.0f * ms / itemCount);

#ifdef _DEBUG
		float aveProbeCount = (float)g_probeCount / (float)itemCount;
		printf("item count = %d, probe count = %d, ave probe count %.2f\n", itemCount, g_probeCount, aveProbeCount);
#endif

		for (int32_t i = 0; i < N; ++i)
		{
			for (int32_t j = i + 1; j < N; ++j)
			{
				uint64_t key = B2_PROXY_PAIR_KEY(i, j);
				b2RemoveKey(&set, key);
			}
		}

		ENSURE(set.count == 0);

		b2DestroySet(&set);
	}

	return 0;
}
