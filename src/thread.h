#pragma once

typedef struct b2Mutex b2Mutex;

// TODO_ERIN temp until github updates MSVC
#if defined(_MSC_VER) && !defined(__clang__)
#define B2_ATOMIC
#else
#define B2_ATOMIC _Atomic
#endif

#ifdef __cplusplus
extern "C"
{
#endif

b2Mutex* b2CreateMutex(const char* name);
void b2DestroyMutex(b2Mutex* mutex);
void b2LockMutex(b2Mutex* mutex);
void b2UnlockMutex(b2Mutex* mutex);

#ifdef __cplusplus
}
#endif
