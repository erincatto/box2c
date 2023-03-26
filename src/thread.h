#pragma once

#include <stdalign.h>

typedef struct b2Mutex
{
	alignas(8) char data[80];
} b2Mutex;

#ifdef __cplusplus
extern "C"
{
#endif

void b2CreateMutex(b2Mutex* mutex);
void b2DestroyMutex(b2Mutex* mutex);
void b2LockMutex(b2Mutex* mutex);
void b2UnlockMutex(b2Mutex* mutex);

#ifdef __cplusplus
}
#endif
