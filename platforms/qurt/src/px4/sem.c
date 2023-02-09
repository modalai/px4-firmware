
#include <px4_platform_common/sem.h>

int px4_sem_init(px4_sem_t *s, int pshared, unsigned value) {
    (void) pshared;
    qurt_sem_init_val(s, value);
    return 0;

}

int px4_sem_setprotocol(px4_sem_t *s, int protocol) {
    (void) s;
    (void) protocol;
    return 0;

}

int px4_sem_wait(px4_sem_t *s) {
    return qurt_sem_down(s);

}

int px4_sem_trywait(px4_sem_t *sem) {
    return qurt_sem_try_down(sem);

}

int px4_sem_timedwait(px4_sem_t *sem, const struct timespec *abstime) {
    // unsigned long long int uSec = (abstime->tv_sec * 1000000) + (abstime->tv_nsec / 1000);
    // return qurt_sem_down_timed(sem, uSec);
    return 0;

}

int px4_sem_post(px4_sem_t *s) {
    return qurt_sem_up(s);

}

int px4_sem_getvalue(px4_sem_t *s, int *sval) {
    *sval = qurt_sem_get_val(s);
    return 0;
}

int px4_sem_destroy(px4_sem_t *s) {
    qurt_sem_destroy(s);
    return 0;

}
