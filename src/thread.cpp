// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "thread.h"

#include <mutex>

#ifdef BOX2D_PROFILE

#include <tracy/Tracy.hpp>
#include <string.h>

struct b2Mutex
{
	TracyLockable(std::mutex, mutex);
};

b2Mutex* b2CreateMutex(const char* name)
{
	b2Mutex* mutex = new b2Mutex;
	mutex->mutex.CustomName(name, strlen(name));
	return mutex;
}

#else

struct b2Mutex
{
	std::mutex mutex;
};

b2Mutex* b2CreateMutex(const char*)
{
	return new b2Mutex;
}
#endif

void b2DestroyMutex(b2Mutex* mutex)
{
	delete mutex;
}

void b2LockMutex(b2Mutex* mutex)
{
	mutex->mutex.lock();
}

void b2UnlockMutex(b2Mutex* mutex)
{
	mutex->mutex.unlock();
}
