#pragma once

#ifndef THREAD_U64
#define THREAD_U64 unsigned long long
#endif

#define THREAD_STACK_SIZE_DEFAULT (0)
#define THREAD_SIGNAL_WAIT_INFINITE (-1)
#define THREAD_QUEUE_WAIT_INFINITE (-1)

typedef void* thread_id_t;
thread_id_t thread_current_thread_id(void);
void thread_yield(void);
void thread_set_high_priority(void);
void thread_exit(int return_code);

typedef void* thread_ptr_t;
thread_ptr_t thread_create(int (*thread_proc)(void*), void* user_data, int stack_size);
void thread_destroy(thread_ptr_t thread);
int thread_join(thread_ptr_t thread);
int thread_detach(thread_ptr_t thread);

typedef union thread_mutex_t thread_mutex_t;
void thread_mutex_init(thread_mutex_t* mutex);
void thread_mutex_term(thread_mutex_t* mutex);
void thread_mutex_lock(thread_mutex_t* mutex);
void thread_mutex_unlock(thread_mutex_t* mutex);

typedef union thread_signal_t thread_signal_t;
void thread_signal_init(thread_signal_t* signal);
void thread_signal_term(thread_signal_t* signal);
void thread_signal_raise(thread_signal_t* signal);
int thread_signal_wait(thread_signal_t* signal, int timeout_ms);

typedef union thread_atomic_int_t thread_atomic_int_t;
int thread_atomic_int_load(thread_atomic_int_t* atomic);
void thread_atomic_int_store(thread_atomic_int_t* atomic, int desired);
int thread_atomic_int_inc(thread_atomic_int_t* atomic);
int thread_atomic_int_dec(thread_atomic_int_t* atomic);
int thread_atomic_int_add(thread_atomic_int_t* atomic, int value);
int thread_atomic_int_sub(thread_atomic_int_t* atomic, int value);
int thread_atomic_int_swap(thread_atomic_int_t* atomic, int desired);
int thread_atomic_int_compare_and_swap(thread_atomic_int_t* atomic, int expected, int desired);

typedef union thread_atomic_ptr_t thread_atomic_ptr_t;
void* thread_atomic_ptr_load(thread_atomic_ptr_t* atomic);
void thread_atomic_ptr_store(thread_atomic_ptr_t* atomic, void* desired);
void* thread_atomic_ptr_swap(thread_atomic_ptr_t* atomic, void* desired);
void* thread_atomic_ptr_compare_and_swap(thread_atomic_ptr_t* atomic, void* expected, void* desired);

typedef union thread_timer_t thread_timer_t;
void thread_timer_init(thread_timer_t* timer);
void thread_timer_term(thread_timer_t* timer);
void thread_timer_wait(thread_timer_t* timer, THREAD_U64 nanoseconds);

typedef void* thread_tls_t;
thread_tls_t thread_tls_create(void);
void thread_tls_destroy(thread_tls_t tls);
void thread_tls_set(thread_tls_t tls, void* value);
void* thread_tls_get(thread_tls_t tls);

typedef struct thread_queue_t thread_queue_t;
void thread_queue_init(thread_queue_t* queue, int size, void** values, int count);
void thread_queue_term(thread_queue_t* queue);
int thread_queue_produce(thread_queue_t* queue, void* value, int timeout_ms);
void* thread_queue_consume(thread_queue_t* queue, int timeout_ms);
int thread_queue_count(thread_queue_t* queue);

union thread_mutex_t
{
	void* align;
	char data[64];
};

union thread_signal_t
{
	void* align;
	char data[116];
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

union thread_timer_t
{
	void* data;
	char d[8];
};

struct thread_queue_t
{
	thread_signal_t data_ready;
	thread_signal_t space_open;
	thread_atomic_int_t count;
	thread_atomic_int_t head;
	thread_atomic_int_t tail;
	void** values;
	int size;
#ifndef NDEBUG
	thread_atomic_int_t id_produce_is_set;
	thread_id_t id_produce;
	thread_atomic_int_t id_consume_is_set;
	thread_id_t id_consume;
#endif
};
