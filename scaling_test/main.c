// SPDX-FileCopyrightText: 2024 Erin Catto
// SPDX-License-Identifier: MIT

#include "TaskScheduler_c.h"
#include "timer.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#define MAYBE_UNUSED(x) ((void)(x))
#define THREAD_LIMIT 16

typedef struct Vec2
{
	float x, y;
} Vec2;

typedef struct Rot
{
	float s, c;
} Rot;

typedef struct State
{
	Vec2 linearVelocity;
	float angularVelocity;
	int flags;
	Vec2 deltaPosition;
	Rot deltaRotation;
} State;

inline Rot IntegrateRotation(Rot q1, float deltaAngle)
{
	Rot q2 = {q1.s + deltaAngle * q1.c, q1.c - deltaAngle * q1.s};
	float mag = sqrtf(q2.s * q2.s + q2.c * q2.c);
	float invMag = mag > 0.0 ? 1.0f / mag : 0.0f;
	Rot qn = {q2.s * invMag, q2.c * invMag};
	return qn;
}

typedef struct TaskData
{
	State* states;
	int startIndex;
	int endIndex;
	float h;
	int stepCount;
	int stepsTaken;
	char cacheLine[64];
} TaskData;

void ExecutePinnedTask(void* context)
{
	TaskData* data = context;
	float h = data->h;
	int startIndex = data->startIndex;
	int endIndex = data->endIndex;
	int stepCount = data->stepCount;

	for (int step = 0; step < stepCount; ++step)
	{
		for (int i = startIndex; i < endIndex; ++i)
		{
			State* state = data->states + i;
			state->deltaRotation = IntegrateRotation(state->deltaRotation, h * state->angularVelocity);
			state->deltaPosition.x += h * state->linearVelocity.x;
			state->deltaPosition.y += h * state->linearVelocity.y;
		}
	}

	data->stepsTaken += stepCount;
}

int main(int argc, char** argv)
{
	int maxThreadCount = 16;
	int runCount = 4;
	//int warmUpCount = 10;
	int loopCount = 1000;
	int stepCount = 8;
	int stateCount = 40000;
	State* states = malloc(stateCount * sizeof(State));
	assert(maxThreadCount <= THREAD_LIMIT);

	printf("Starting scaling test\n");
	printf("======================================\n");

	for (int threadCount = 1; threadCount <= maxThreadCount; ++threadCount)
	{
		printf("thread count: %d\n", threadCount);

		for (int runIndex = 0; runIndex < runCount; ++runIndex)
		{
			enkiTaskScheduler* scheduler = enkiNewTaskScheduler();
			struct enkiTaskSchedulerConfig config = enkiGetTaskSchedulerConfig(scheduler);
			config.numTaskThreadsToCreate = threadCount - 1;
			enkiInitTaskSchedulerWithConfig(scheduler, config);

			for (int i = 0; i < stateCount; ++i)
			{
				states[i] = (State){0};
				states[i].linearVelocity = (Vec2){1.0f, -2.0f};
				states[i].angularVelocity = 0.0f;
				states[i].deltaRotation = (Rot){0.0f, 1.0f};
			}

			TaskData taskData[THREAD_LIMIT];
			enkiPinnedTask* taskSets[THREAD_LIMIT];
			int blockSize = stateCount / threadCount;
			int baseIndex = 0;
			for (int i = 0; i < threadCount; ++i)
			{
				taskData[i].states = states;
				taskData[i].h = 1.0f / 6.0f;
				taskData[i].startIndex = baseIndex;
				taskData[i].endIndex = i == threadCount - 1 ? stateCount : baseIndex + blockSize;
				taskData[i].stepCount = stepCount;
				taskData[i].stepsTaken = 0;
				taskSets[i] = enkiCreatePinnedTask(scheduler, ExecutePinnedTask, i);
				enkiSetArgsPinnedTask(taskSets[i], taskData + i);
				
				baseIndex += blockSize;
			}

			// warm up
			//for (int step = 0; step < warmUpCount; ++step)
			//{
			//	for (int i = 0; i < threadCount; ++i)
			//	{
			//		enkiAddPinnedTask(scheduler, taskSets[i]);
			//	}

			//	enkiWaitForAll(scheduler);
			//}

			Timer timer = CreateTimer();

			for (int loop = 0; loop < loopCount; ++loop)
			{
				for (int i = 0; i < threadCount; ++i)
				{
					enkiAddPinnedTask(scheduler, taskSets[i]);
				}

				enkiWaitForAll(scheduler);
			}

			float ms = GetMilliseconds(&timer);
			float fps = 1000.0f * stepCount / ms;
			printf("run %d : %g (ms), %g (fps)\n", runIndex, ms, fps);

			for (int i = 0; i < threadCount; ++i)
			{
				//assert(taskData[i].stepCount == warmUpCount + stepCount);
				assert(taskData[i].stepsTaken == stepCount * loopCount);
				enkiDeletePinnedTask(scheduler, taskSets[i]);
				taskSets[i] = NULL;
			}

			enkiDeleteTaskScheduler(scheduler);
		}
	}

	printf("======================================\n");
	printf("Scaling test complete!\n");

	free(states);
	return 0;
}
