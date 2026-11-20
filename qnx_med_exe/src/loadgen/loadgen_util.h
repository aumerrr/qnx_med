/* loadgen_util.h - small helpers shared by cpu_hog, msg_flood (Person B) */
#ifndef LOADGEN_UTIL_H
#define LOADGEN_UTIL_H

#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <errno.h>
#include <time.h>
#include <pthread.h>
#include <sched.h>

#ifndef EOK
#define EOK 0
#endif

#ifdef __QNX__
#include <sys/neutrino.h>
#endif

static inline uint64_t lg_now_ns(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000000ULL + (uint64_t)ts.tv_nsec;
}

/* Scheduling policy + priority for the calling thread. Returns 0 on success. */
static inline int lg_set_policy(int policy, int prio)
{
    struct sched_param sp;
    memset(&sp, 0, sizeof(sp));
    sp.sched_priority = prio;
    return pthread_setschedparam(pthread_self(), policy, &sp);
}

/* SCHED_FIFO: the thread runs until it blocks, equal priorities never share. */
static inline int lg_set_prio(int prio)  { return lg_set_policy(SCHED_FIFO, prio); }

/* SCHED_RR: equal priorities time-slice. This is the QNX default policy, so a
 * load at the default priority 10 in RR does NOT freeze sshd, io-sock, etc. */
static inline int lg_set_prio_rr(int prio) { return lg_set_policy(SCHED_RR, prio); }

/* Pin the calling thread to one CPU. Returns 0 on success.
 * On QNX the runmask is passed by value; if the kernel wants a pointer
 * (some docs show that) we fall back to passing the address. */
static inline int lg_pin_cpu(int cpu)
{
#ifdef __QNX__
    uintptr_t mask = (uintptr_t)1 << cpu;
    if (ThreadCtl(_NTO_TCTL_RUNMASK, (void *)mask) != -1) return 0;
    unsigned int m = (unsigned int)mask;
    if (ThreadCtl(_NTO_TCTL_RUNMASK, &m) != -1) return 0;
    return -1;
#else
    cpu_set_t set;
    CPU_ZERO(&set);
    CPU_SET(cpu, &set);
    return sched_setaffinity(0, sizeof(set), &set);
#endif
}

#endif
