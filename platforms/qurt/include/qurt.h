/* qurt.h */
#pragma once

#include <string.h>

// #define QURT_EOK                        0
// 
// #define QURT_THREAD_ATTR_NAME_MAXLEN            16  /**< Maximum name length. */
// 
// typedef unsigned int qurt_thread_t;
// 
// typedef struct _qurt_thread_attr {
// 
//     char name[QURT_THREAD_ATTR_NAME_MAXLEN]; /**< Thread name. */
//     unsigned char tcb_partition;  /**< Indicates whether the thread TCB resides in RAM or
//                                        on chip memory (in other words, TCM). */
//     unsigned char  stid;          /**< SW thread ID used to configure "stid" register
//                                        for profiling pusposes */
//     unsigned short priority;      /**< Thread priority. */
//     unsigned char  asid;          /**< Address space ID. */
//     unsigned char  bus_priority;  /**< Internal bus priority. */
//     unsigned short timetest_id;   /**< Timetest ID. */
//     unsigned int   stack_size;    /**< Thread stack size. */
//     void *stack_addr;             /**< Pointer to the stack address base, the range of the stack is
//                                        (stack_addr, stack_addr+stack_size-1). */
//     unsigned short detach_state;  /**< Detach state of the thread */
// 
// } qurt_thread_attr_t;
// 
// typedef unsigned long long                  qurt_timer_duration_t;

// void *qurt_malloc( unsigned int size);
// 
// void qurt_free( void *ptr);

// int qurt_timer_sleep(qurt_timer_duration_t duration);
// 
// void qurt_thread_exit(int status);
// 
// static inline void qurt_thread_attr_init (qurt_thread_attr_t *attr)
// {
// 
//     attr->name[0] = 0;
//     // attr->tcb_partition = QURT_THREAD_ATTR_TCB_PARTITION_DEFAULT;
//     // attr->priority = QURT_THREAD_ATTR_PRIORITY_DEFAULT;
//     // attr->asid = QURT_THREAD_ATTR_ASID_DEFAULT;
//     // attr->bus_priority = QURT_THREAD_ATTR_BUS_PRIO_DEFAULT;
//     // attr->timetest_id = QURT_THREAD_ATTR_TIMETEST_ID_DEFAULT;
//     attr->stack_size = 0;
//     attr->stack_addr = 0;
//     // attr->detach_state = QURT_THREAD_ATTR_CREATE_DETACHED;
//     // attr->stid = QURT_THREAD_ATTR_STID_DEFAULT;
// }
// 
// static inline void qurt_thread_attr_set_name (qurt_thread_attr_t *attr, char *name)
// {
//     strlcpy (attr->name, name, QURT_THREAD_ATTR_NAME_MAXLEN);
//     attr->name[QURT_THREAD_ATTR_NAME_MAXLEN - 1] = 0;
// }
// 
// static inline void qurt_thread_attr_set_stack_size (qurt_thread_attr_t *attr, unsigned int stack_size)
// {
//     attr->stack_size = stack_size;
// }
// 
// static inline void qurt_thread_attr_set_stack_addr (qurt_thread_attr_t *attr, void *stack_addr)
// {
//     attr->stack_addr = stack_addr;
// }
// 
// static inline void qurt_thread_attr_set_priority (qurt_thread_attr_t *attr, unsigned short priority)
// {
//     attr->priority = priority;
// }
// 
// int qurt_thread_create (qurt_thread_t *thread_id, qurt_thread_attr_t *attr, void (*entrypoint) (void *), void *arg);
// 
