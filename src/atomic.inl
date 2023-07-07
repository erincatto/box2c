#include <stdint.h>

#if 0 && defined(_MSC_VER) && !defined(__clang__)

#include <intrin.h>

// Returns the initial value
static inline long atomic_fetch_add_long(volatile long* obj, long val)
{
	return _InterlockedExchangeAdd(obj, val);
}

// Returns the initial value
static inline long atomic_fetch_sub_long(volatile long* obj, long val)
{
	return _InterlockedExchangeAdd(obj, -val);
}

// Returns the initial value
static inline long atomic_store_long(volatile long* obj, long val)
{
	return _InterlockedExchange(obj, val);
}

static inline long atomic_load_long(const volatile long* obj)
{
	return *obj;
}

static inline bool atomic_compare_exchange_strong_long(volatile long* obj, long* expected, int32_t desired)
{
	long current = _InterlockedCompareExchange(obj, desired, *expected);
	if (current == *expected)
	{
		return true;
	}

	*expected = current;
	return false;
}

static inline bool atomic_compare_exchange_weak_long(volatile long* obj, long* expected, int32_t desired)
{
	long current = _InterlockedCompareExchange(obj, desired, *expected);
	if (current == *expected)
	{
		return true;
	}

	*expected = current;
	return false;
}

#else

#include <stdatomic.h>

#define atomic_fetch_add_long atomic_fetch_add
#define atomic_fetch_sub_long atomic_fetch_sub
#define atomic_store_long atomic_store
#define atomic_load_long atomic_load
#define atomic_compare_exchange_strong_long atomic_compare_exchange_strong
#define atomic_compare_exchange_weak_long atomic_compare_exchange_weak

#endif

// Atomically store the minimum two values
// *ptr = min(*ptr, b)
static inline void b2AtomicStoreMin(B2_ATOMIC long* ptr, long b)
{
	long a = atomic_load_long(ptr);
	while (a > b)
	{
		bool success = atomic_compare_exchange_weak_long(ptr, &a, b);
		if (success)
		{
			return;
		}

		// otherwise `a` now holds the current value stored in `ptr`
	}
}
