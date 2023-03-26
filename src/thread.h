#pragma once

#ifndef THREAD_U64
#define THREAD_U64 unsigned long long
#endif

#define THREAD_STACK_SIZE_DEFAULT (0)
#define THREAD_SIGNAL_WAIT_INFINITE (-1)
#define THREAD_QUEUE_WAIT_INFINITE (-1)

typedef void* thread_id_t;
thread_id_t thread_current_thread_id(void);

typedef union thread_mutex_t thread_mutex_t;
void thread_mutex_init(thread_mutex_t* mutex);
void thread_mutex_term(thread_mutex_t* mutex);
void thread_mutex_lock(thread_mutex_t* mutex);
void thread_mutex_unlock(thread_mutex_t* mutex);

typedef union thread_atomic_int_t thread_atomic_int_t;
int thread_atomic_int_load(thread_atomic_int_t* atomic);
void thread_atomic_int_store(thread_atomic_int_t* atomic, int desired);
int thread_atomic_int_inc(thread_atomic_int_t* atomic);
int thread_atomic_int_dec(thread_atomic_int_t* atomic);
int thread_atomic_int_add(thread_atomic_int_t* atomic, int value);
int thread_atomic_int_sub(thread_atomic_int_t* atomic, int value);
int thread_atomic_int_swap(thread_atomic_int_t* atomic, int desired);
int thread_atomic_int_compare_and_swap(thread_atomic_int_t* atomic, int expected, int desired);

typedef void* thread_tls_t;
thread_tls_t thread_tls_create(void);
void thread_tls_destroy(thread_tls_t tls);
void thread_tls_set(thread_tls_t tls, void* value);
void* thread_tls_get(thread_tls_t tls);

union thread_mutex_t
{
	void* align;
	char data[64];
};

union thread_atomic_int_t
{
	void* align;
	long i;
};

union thread_atomic_ptr_t
{
	void* ptr;
};
