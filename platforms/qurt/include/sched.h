
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#define SCHED_FIFO 0

typedef struct sched_param sched_param;
struct sched_param
{
    void *unimplemented;
    int  sched_priority;
};

int sched_get_priority_max(int policy);

static inline int sched_yield(void)
{
   return 0;
}

#ifdef __cplusplus
}
#endif
