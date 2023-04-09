#include <stdint.h>

#if defined(_MSC_VER) && !defined(__clang__)

#define WIN32_LEAN_AND_MEAN
#include <windows.h>

typedef int32_t atomic_int32_t;

static inline int32_t atomic_fetch_add_int32_t(volatile atomic_int32_t* obj, int32_t val)
{
	return InterlockedExchangeAdd(obj, val);
}

static inline int32_t atomic_store_int32_t(volatile atomic_int32_t* obj, int32_t val)
{
	return InterlockedExchange(obj, val);
}

static inline int32_t atomice_load_int32_t(volatile atomic_int32_t* obj)
{
	return InterlockedOr(obj, 0);
}

static inline bool atomic_compare_exchange_weak_int32_t(volatile atomic_int32_t* obj, int32_t* expected, int32_t desired)
{
	return InterlockedCompareExchange((volatile long*)obj, desired, *expected) == *(long*)expected;
}

#else

#include <stdatomic.h>

#endif
