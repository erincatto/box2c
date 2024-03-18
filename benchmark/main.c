// SPDX-FileCopyrightText: 2024 Erin Catto
// SPDX-License-Identifier: MIT

#include "TaskScheduler_c.h"

#include "box2d/box2d.h"
#include "box2d/math.h"
#include "box2d/timer.h"

#include <assert.h>
#include <stdbool.h>
#include <stdio.h>

#define ARRAY_COUNT(A) (int)(sizeof(A) / sizeof(A[0]))
#define MAYBE_UNUSED(x) ((void)(x))

typedef b2WorldId CreateBenchmarkFcn(b2WorldDef* worldDef);
extern b2WorldId ManyPyramids(b2WorldDef* worldDef);
extern b2WorldId LargePyramid(b2WorldDef* worldDef);

typedef struct Benchmark
{
	const char* name;
	CreateBenchmarkFcn* createFcn;
	int stepCount;
} Benchmark;

#define MAX_TASKS 128
#define THREAD_LIMIT 32

typedef struct TaskData
{
	b2TaskCallback* box2dTask;
	void* box2dContext;
} TaskData;

typedef struct PinnedTaskData
{
	b2PinnedTaskFcn* box2dPinnedTask;
	void* box2dContext;
	int threadIndex;
	bool inUse;
} PinnedTaskData;

enkiTaskScheduler* scheduler;
enkiTaskSet* tasks[MAX_TASKS];
TaskData taskData[MAX_TASKS];
int taskCount;

enkiPinnedTask* pinnedTasks[THREAD_LIMIT];
PinnedTaskData pinnedTaskData[THREAD_LIMIT];

void ExecuteRangeTask(uint32_t start, uint32_t end, uint32_t threadIndex, void* context)
{
	TaskData* data = context;
	data->box2dTask(start, end, threadIndex, data->box2dContext);
}

static void* EnqueueTask(b2TaskCallback* box2dTask, int itemCount, int minRange, void* box2dContext, void* userContext)
{
	MAYBE_UNUSED(userContext);

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
		printf("MAX_TASKS exceeded!!!\n");
		box2dTask(0, itemCount, 0, box2dContext);
		return NULL;
	}
}

static void PinnedTaskFunc(void* args)
{
	PinnedTaskData* data = args;
	data->box2dPinnedTask(data->threadIndex, data->box2dContext);
}

static void* AddPinnedTask(b2PinnedTaskFcn* box2dTask, int32_t threadIndex, void* box2dContext, void* userContext)
{
	assert(threadIndex < THREAD_LIMIT);
	PinnedTaskData* data = pinnedTaskData + threadIndex;
	if (data->inUse == false)
	{
		enkiPinnedTask* task = pinnedTasks[threadIndex];
		data->box2dPinnedTask = box2dTask;
		data->box2dContext = box2dContext;
		data->threadIndex = threadIndex;
		data->inUse = true;

		enkiAddPinnedTaskArgs(scheduler, task, data);
		return task;
	}
	else
	{
		printf("MAX_TASKS exceeded!!!\n");
		box2dTask(threadIndex, box2dContext);
		return NULL;
	}
}

static void FinishTask(void* userTask, void* userContext)
{
	MAYBE_UNUSED(userContext);

	enkiTaskSet* task = userTask;
	enkiWaitForTaskSet(scheduler, task);
}

static void FinishPinnedTask(void* userTask, void* userContext)
{
	MAYBE_UNUSED(userContext);

	enkiPinnedTask* task = userTask;
	enkiWaitForPinnedTask(scheduler, task);

	for (int i = 0; i < THREAD_LIMIT; ++i)
	{
		if (task == pinnedTasks[i])
		{
			pinnedTaskData[i] = (PinnedTaskData){0};
			return;
		}
	}

	assert(false);
}

int main(int argc, char** argv)
{
	int maxThreadCount = 8;
	int runCount = 4;
	b2Counters counters = {0};
	bool countersAcquired = false;
	bool enableContinuous = true;

	assert(maxThreadCount <= THREAD_LIMIT);

	Benchmark benchmarks[] = {
		//{"many_pyramids", ManyPyramids, 200},
		{"large_pyramids", LargePyramid, 500},
	};

	int benchmarkCount = ARRAY_COUNT(benchmarks);

	printf("Starting Box2D benchmarks\n");
	printf("======================================\n");

	for (int benchmarkIndex = 0; benchmarkIndex < benchmarkCount; ++benchmarkIndex)
	{
#ifdef NDEBUG
		int stepCount = benchmarks[benchmarkIndex].stepCount;
#else
		int stepCount = 10;
#endif

		printf("benchmark: %s, steps = %d\n", benchmarks[benchmarkIndex].name, stepCount);

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

				for (int threadIndex = 0; threadIndex < threadCount; ++threadIndex)
				{
					pinnedTasks[threadIndex] = enkiCreatePinnedTask(scheduler, PinnedTaskFunc, threadIndex);
				}

				b2WorldDef worldDef = b2DefaultWorldDef();
				worldDef.enableSleep = false;
				worldDef.enableContinous = enableContinuous;
				worldDef.enqueueTask = EnqueueTask;
				worldDef.finishTask = FinishTask;
				worldDef.addPinnedTask = AddPinnedTask;
				worldDef.finishPinnedTask = FinishPinnedTask;
				worldDef.workerCount = threadCount;

				b2WorldId worldId = benchmarks[benchmarkIndex].createFcn(&worldDef);

				float timeStep = 1.0f / 60.0f;
				int subStepCount = 4;

				// Initial step can be expensive and skew benchmark
				b2World_Step(worldId, timeStep, subStepCount);

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

				if (countersAcquired == false)
				{
					counters = b2World_GetCounters(worldId);
					countersAcquired = true;
				}

				b2DestroyWorld(worldId);

				for (int taskIndex = 0; taskIndex < MAX_TASKS; ++taskIndex)
				{
					enkiDeleteTaskSet(scheduler, tasks[taskIndex]);
					tasks[taskIndex] = NULL;
					taskData[taskIndex] = (TaskData){0};
				}

				for (int threadIndex = 0; threadIndex < threadCount; ++threadIndex)
				{
					enkiDeletePinnedTask(scheduler, pinnedTasks[threadIndex]);
					pinnedTasks[threadIndex] = NULL;
					pinnedTaskData[threadIndex] = (PinnedTaskData){0};
				}

				enkiDeleteTaskScheduler(scheduler);
				scheduler = NULL;
			}
		}

		printf("body %d / shape %d / contact %d / joint %d / stack %d\n", counters.bodyCount, counters.shapeCount,
			   counters.contactCount, counters.jointCount, counters.stackUsed);

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
