// SPDX-FileCopyrightText: 2024 Erin Catto
// SPDX-License-Identifier: MIT

#include "TaskScheduler_c.h"

#include "box2d/box2d.h"
#include "box2d/math.h"
#include "box2d/timer.h"

#include <assert.h>
#include <stdbool.h>
#include <stdio.h>

typedef b2WorldId CreateBenchmarkFcn(b2WorldDef* worldDef);
extern b2WorldId BenchmarkPyramids(b2WorldDef* worldDef);

typedef struct Benchmark
{
	const char* name;
	CreateBenchmarkFcn* createFcn;
} Benchmark;

#define MAX_TASKS 128

typedef struct TaskData
{
	b2TaskCallback* box2dTask;
	void* box2dContext;
} TaskData;

enkiTaskScheduler* scheduler;
enkiTaskSet* tasks[MAX_TASKS];
TaskData taskData[MAX_TASKS];
int taskCount;

void ExecuteRangeTask(uint32_t start, uint32_t end, uint32_t threadIndex, void* context)
{
	TaskData* data = context;
	data->box2dTask(start, end, threadIndex, data->box2dContext);
}

static void* EnqueueTask(b2TaskCallback* box2dTask, int itemCount, int minRange, void* box2dContext, void* userContext)
{
	B2_MAYBE_UNUSED(userContext);

	if (taskCount < MAX_TASKS)
	{
		enkiTaskSet* task = tasks[taskCount];
		TaskData* data = taskData + taskCount;
		data->box2dTask = box2dTask;
		data->box2dContext = box2dContext;

		struct enkiParamsTaskSet params;
		params.minRange = minRange;
		params.setSize = itemCount;
		params.pArgs = data;
		params.priority = 0;

		enkiSetParamsTaskSet(task, params);
		enkiAddTaskSet(scheduler, task);

		++taskCount;

		return task;
	}
	else
	{
		box2dTask(0, itemCount, 0, box2dContext);

		return NULL;
	}
}

static void FinishTask(void* userTask, void* userContext)
{
	B2_MAYBE_UNUSED(userContext);

	enkiTaskSet* task = userTask;
	enkiWaitForTaskSet(scheduler, task);
}

#define THREAD_LIMIT 16

int main(int argc, char** argv)
{
	int maxThreadCount = 16;
	int runCount = 4;
	int stepCount = 500;
	bool enableContinuous = true;

	assert(maxThreadCount <= THREAD_LIMIT);

	Benchmark benchmarks[] = {
		{"pyramids", BenchmarkPyramids},
	};

	int benchmarkCount = B2_ARRAY_COUNT(benchmarks);

	printf("Starting Box2D benchmarks\n");
	printf("======================================\n");

	for (int benchmarkIndex = 0; benchmarkIndex < benchmarkCount; ++benchmarkIndex)
	{
		printf("benchmark: %s\n", benchmarks[benchmarkIndex].name);

		float maxFps[THREAD_LIMIT] = {0};

		for (int threadCount = 1; threadCount <= maxThreadCount; ++threadCount)
		{
			printf("thread count: %d\n", threadCount);

			for (int runIndex = 0; runIndex < runCount; ++runIndex)
			{
				scheduler = enkiNewTaskScheduler();
				struct enkiTaskSchedulerConfig config = enkiGetTaskSchedulerConfig(scheduler);
				config.numTaskThreadsToCreate = threadCount - 1;
				enkiInitTaskSchedulerWithConfig(scheduler, config);

				for (int taskIndex = 0; taskIndex < MAX_TASKS; ++taskIndex)
				{
					tasks[taskIndex] = enkiCreateTaskSet(scheduler, ExecuteRangeTask);
				}

				b2WorldDef worldDef = b2DefaultWorldDef();
				worldDef.enableSleep = false;
				worldDef.enableContinous = enableContinuous;
				worldDef.enqueueTask = EnqueueTask;
				worldDef.finishTask = FinishTask;
				worldDef.workerCount = threadCount;

				b2WorldId worldId = benchmarks[benchmarkIndex].createFcn(&worldDef);

				float timeStep = 1.0f / 60.0f;
				int subStepCount = 4;

				// Initial step can be expensive and skew benchmark
				b2World_Step(worldId, timeStep, subStepCount);

				float totalTime = 0.0f;
				b2Timer timer = b2CreateTimer();

				for (int step = 0; step < stepCount; ++step)
				{
					b2World_Step(worldId, timeStep, subStepCount);
					taskCount = 0;
				}

				float ms = b2GetMilliseconds(&timer);
				float fps = 1000.0f * stepCount / ms;
				printf("run %d : %g (ms), %g (fps)\n", runIndex, ms, fps);

				maxFps[threadCount - 1] = B2_MAX(maxFps[threadCount - 1], fps);

				//b2Counters counters = b2World_GetCounters(worldId);
				//printf("body %d / shape %d / contact %d / joint %d / stack %d\n", counters.bodyCapacity, counters.shapeCapacity,
				//	   counters.contactCapacity, counters.jointCapacity, counters.stackUsed);

				b2DestroyWorld(worldId);

				for (int taskIndex = 0; taskIndex < MAX_TASKS; ++taskIndex)
				{
					enkiDeleteTaskSet(scheduler, tasks[taskIndex]);
				}

				enkiDeleteTaskScheduler(scheduler);
				scheduler = NULL;
			}
		}

		char fileName[64] = {0};
		snprintf(fileName, 64, "%s.csv", benchmarks[benchmarkIndex].name);
		FILE* file = NULL;
		errno_t status = fopen_s(&file, fileName, "w");
		if (status != 0 || file == NULL)
		{
			continue;
		}

		fprintf(file, "threads, fps\n");
		for (int threadCount = 1; threadCount <= maxThreadCount; ++threadCount)
		{
			fprintf(file, "%d, %g\n", threadCount, maxFps[threadCount - 1]);
		}

		fclose(file);
	}

	printf("======================================\n");
	printf("All Box2D benchmarks complete!\n");

	return 0;
}
