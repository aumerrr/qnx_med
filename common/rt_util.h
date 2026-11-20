/*
 * rt_util.h
 *
 *  Created on: 19-Nov-2026
 *      Author: APPLE
 */

/* rt_util.h - small helpers shared by servo.c / ctl_loop.c */
#ifndef RT_UTIL_H
#define RT_UTIL_H
#include <stdint.h>
#include <time.h>
#include <pthread.h>
#include <sched.h>
#include <sys/neutrino.h>

static inline int64_t rt_now_ns(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return (int64_t)t.tv_sec * 1000000000LL + t.tv_nsec;
}

static inline void rt_sleep_until_ns(int64_t abs_ns)
{
    struct timespec ts;
    ts.tv_sec  = (time_t)(abs_ns / 1000000000LL);
    ts.tv_nsec = (long)(abs_ns % 1000000000LL);
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, NULL);
}

/* SCHED_FIFO at the given priority for the calling thread. 0 = ok. */
static inline int rt_set_prio(int prio)
{
    struct sched_param sp;
    sp.sched_priority = prio;
    return pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp) == 0 ? 0 : -1;
}

/* Restrict the calling thread to the CPUs in 'mask' (bit0 = CPU0).
 * On the stock image only the single-CPU clusters (_cpu-N) exist, so valid
 * masks are 0x1,0x2,0x4,0x8 (and 0xF = _all). Anything else -> EINVAL.
 * The instructor handout says ThreadCtl takes the ADDRESS of the mask; older
 * docs pass the value. Try address first, then value. mask==0 -> do nothing.
 * Returns 0 ok, -1 failed (caller should print and carry on / abort). */
static inline int rt_pin_self(unsigned mask)
{
    unsigned m = mask;
    if (mask == 0)
        return 0;
    if (ThreadCtl(_NTO_TCTL_RUNMASK, &m) == 0)
        return 0;
    if (ThreadCtl(_NTO_TCTL_RUNMASK, (void *)(uintptr_t)mask) == 0)
        return 0;
    return -1;
}
#endif
