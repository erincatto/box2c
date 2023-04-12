#include <stdint.h>

#if defined(_MSC_VER) && !defined(__clang__)

#define WIN32_LEAN_AND_MEAN
#include <windows.h>

typedef long atomic_long;

static inline int32_t atomic_fetch_add_long(volatile atomic_long* obj, int32_t val)
{
	return InterlockedExchangeAdd(obj, val);
}

static inline int32_t atomic_store_long(volatile atomic_long* obj, int32_t val)
{
	return InterlockedExchange(obj, val);
}

static inline int32_t atomic_load_long(const volatile atomic_long* obj)
{
	return *obj;
}

static inline bool atomic_compare_exchange_weak_long(volatile atomic_long* obj, int32_t* expected, int32_t desired)
{
	long current = InterlockedCompareExchange(obj, desired, *expected);
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
#define atomic_store_long atomic_store
#define atomic_load_long atomic_load
#define atomic_compare_exchange_weak_long atomic_compare_exchange_weak

#endif

// Atomically store the minimum two values
// *ptr = min(*ptr, b)
void b2AtomicStoreMin(atomic_long* ptr, int32_t b)
{
	int32_t a = atomic_load_long(ptr);
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
