// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "thread.h"

#include <mutex>
#include <new>

// This wraps threading primitives for C. Replace when Microsoft supports C11 threads.

void b2CreateMutex(b2Mutex* mutex)
{
	static_assert(sizeof(b2Mutex) >= sizeof(std::mutex));
	static_assert(alignof(b2Mutex) >= alignof(std::mutex));
	new (mutex->data) std::mutex;
}

void b2DestroyMutex(b2Mutex* mutex)
{
	std::mutex* s = reinterpret_cast<std::mutex*>(mutex);
	s->~mutex();
}

void b2LockMutex(b2Mutex* mutex)
{
	std::mutex* s = reinterpret_cast<std::mutex*>(mutex);
	s->lock();
}

void b2UnlockMutex(b2Mutex* mutex)
{
	std::mutex* s = reinterpret_cast<std::mutex*>(mutex);
	s->unlock();
}