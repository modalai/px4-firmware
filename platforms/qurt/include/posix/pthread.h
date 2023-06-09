
#pragma once

#include <string.h>
#include <sched.h>
#include <errno.h>
#include <time.h>

#define PTHREAD_NAME_LEN 16
#define PTHREAD_STACK_MIN 4096*2

typedef int pthread_once_t;

int pthread_once(pthread_once_t *once_control, void (*init_routine)(void));

typedef int pthread_key_t;

int pthread_key_create(pthread_key_t *key, void (*destructor)(void*));
int pthread_key_delete(pthread_key_t key);
int pthread_setspecific(pthread_key_t key, const void *value);
void *pthread_getspecific(pthread_key_t key);

typedef unsigned int pthread_t;
typedef unsigned int cpu_set_t;

typedef struct pthread_attr_t
{
    void         *stackaddr;
    int          internal_stack;
    size_t       stacksize;
    int          priority;
    unsigned int timetest_id;
    cpu_set_t    cpumask;
    char         name[PTHREAD_NAME_LEN];
    int          ext_context;
    int          detachstate;
} pthread_attr_t;

int pthread_attr_init(pthread_attr_t *attr);
int pthread_attr_destroy(pthread_attr_t *attr);
int pthread_attr_getschedparam(const pthread_attr_t *restrict attr, sched_param *restrict param);
int pthread_attr_setschedparam(pthread_attr_t *restrict attr, const sched_param *restrict param);
int pthread_attr_setstacksize(pthread_attr_t *attr, size_t stacksize);
int pthread_create(pthread_t * tid, const pthread_attr_t * attr, void *(*start)(void *), void *arg);
pthread_t pthread_self(void);
int pthread_join(pthread_t thread, void **value_ptr);
int pthread_cancel(pthread_t thread);
int pthread_kill(pthread_t thread, int sig);
void pthread_exit(void *value_ptr);
int pthread_detach(pthread_t id);

static inline int pthread_equal(pthread_t t1, pthread_t t2)
{
    return t1 == t2;
}


typedef unsigned int pthread_mutex_t;

typedef struct pthread_mutexattr_t pthread_mutexattr_t;
struct pthread_mutexattr_t
{
    int is_initialized;
    int type;
    int pshared;
    int protocol;
};

#define PTHREAD_MUTEX_INITIALIZER ((pthread_mutex_t) 0xFFFFFFFF)
#define PTHREAD_MUTEX_RECURSIVE 2

int pthread_mutexattr_init(pthread_mutexattr_t *attr);
int pthread_mutexattr_settype(pthread_mutexattr_t *attr, int type);
int pthread_mutexattr_destroy(pthread_mutexattr_t *attr);
int pthread_mutex_init(pthread_mutex_t *mutex, pthread_mutexattr_t *attr);
int pthread_mutex_trylock(pthread_mutex_t *mutex);
int pthread_mutex_lock(pthread_mutex_t *mutex);
int pthread_mutex_unlock(pthread_mutex_t *mutex);
int pthread_mutex_destroy(pthread_mutex_t *mutex);

typedef unsigned int pthread_cond_t;

#define PTHREAD_COND_INITIALIZER ((pthread_cond_t) 0xFFFFFFFF)

int pthread_cond_signal(pthread_cond_t *cond);
int pthread_cond_wait(pthread_cond_t *cond, pthread_mutex_t *mutex);
int pthread_cond_timedwait(pthread_cond_t * cond, pthread_mutex_t * mutex, const struct timespec *time);
int pthread_cond_broadcast(pthread_cond_t *cond);
int pthread_cond_destroy(pthread_cond_t *cond);
