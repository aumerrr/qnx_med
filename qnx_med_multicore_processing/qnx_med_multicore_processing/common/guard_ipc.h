/* guard_ipc.h - what the guardian publishes for the dashboard (Person A).
 *
 *   /guard_stats  guard_stats_t  written by guardian, read by monitor (seqlock, like /ctl_stats)
 *
 * The guardian is the recovery manager. It runs on core 0 above every load and:
 *   1. watches ctl_loop's cycle counter; if it stops for GUARD_STARVE_NS the loop is being
 *      starved -> it puts the actuator in SAFE at once (servo to centre)
 *   2. if the system stays disturbed (starved or not NORMAL) longer than the grace timer,
 *      it lowers the priority of every non-critical thread that can run on the real-time
 *      cores and sits at or above GUARD_DEMOTE_MIN_PRIO (CPU hogs, message flooders)
 *   3. measures CPU use per core and lists where our threads run, for the dashboard
 * Every value can be overridden with -DNAME=value.
 */
#ifndef GUARD_IPC_H
#define GUARD_IPC_H
#include <stdint.h>

#define SHM_GUARD_NAME        "/guard_stats"

#ifndef PRIO_GUARD
#define PRIO_GUARD            100          /* above every load we start (65, 90)              */
#endif
#ifndef GUARD_TICK_NS
#define GUARD_TICK_NS         5000000LL    /* health check every 5 ms                          */
#endif
#ifndef GUARD_STARVE_NS
#define GUARD_STARVE_NS       20000000LL   /* no control cycle for 20 ms = loop starved       */
#endif
#ifndef GUARD_GRACE_MS
#define GUARD_GRACE_MS        1000         /* disturbed this long -> demote the offenders      */
#endif
#ifndef GUARD_DEMOTE_MIN_PRIO
#define GUARD_DEMOTE_MIN_PRIO 55           /* = PRIO_ADC: at or above this a thread can delay  */
#endif                                     /*   our chain (adc 55, ctl 60, PWM 70)             */
#ifndef GUARD_DEMOTE_TO
#define GUARD_DEMOTE_TO       5            /* below everything we care about                   */
#endif
#ifndef RT_CLUSTER_MASK
#define RT_CLUSTER_MASK       0x6u         /* cores 1-2                                        */
#endif

#define GUARD_MAX_CPU         8
#define GUARD_MAX_THREADS     40
#define GUARD_MAX_DEMOTED     12
#define GUARD_NAME_LEN        20

/* health as seen by the guardian */
enum { GH_OK = 0, GH_DISTURBED = 1, GH_STARVED = 2, GH_NO_LOOP = 3 };

/* role of a listed thread */
enum { GR_CRIT = 0, GR_AUX = 1, GR_LOAD = 2, GR_FOREIGN = 3, GR_DEMOTED = 4 };

typedef struct {
    int32_t  pid;
    int32_t  tid;
    char     name[GUARD_NAME_LEN];     /* program name                                    */
    char     what[12];                 /* "servo PWM", "ctl loop", "hog", ...             */
    uint8_t  prio;                     /* current priority                                */
    uint8_t  policy;                   /* SCHED_FIFO / SCHED_RR / ...                     */
    uint8_t  state;                    /* kernel thread state (STATE_*)                   */
    uint8_t  role;                     /* GR_*                                            */
    int16_t  last_cpu;                 /* core the thread last ran on                     */
    int16_t  cpu_pct_x10;              /* CPU use over the last scan, in 0.1 %            */
    uint32_t runmask;                  /* cores it may run on (0 = could not read)        */
    uint8_t  demoted_from;             /* old priority if the guardian lowered it, else 0 */
    uint8_t  pad[3];
} guard_thread_t;

typedef struct {
    int64_t  t_ns;                     /* CLOCK_MONOTONIC                                 */
    int32_t  pid;
    int32_t  tid;
    char     name[GUARD_NAME_LEN];
    uint8_t  old_prio;
    uint8_t  new_prio;
    uint16_t pad;
} guard_demote_t;

typedef struct {
    volatile uint64_t seq;             /* seqlock                                         */
    int64_t  now_ns;                   /* time of the last update                         */
    uint64_t scans;
    int32_t  ncpu;
    int32_t  health;                   /* GH_*                                            */
    int32_t  cpu_pct_x10[GUARD_MAX_CPU];/* busy % per core, 0.1 % units                   */
    int64_t  episode_ns;               /* how long the current disturbance has lasted     */
    int32_t  grace_ms;                 /* demotion timer                                  */
    int32_t  demote_min_prio;
    uint32_t forced_safe;              /* times the guardian put the actuator in SAFE     */
    uint32_t demotions;                /* threads lowered so far                          */
    int32_t  busy_foreign;             /* non-critical threads >= min prio on the RT cores */
    int32_t  nthreads;
    guard_thread_t th[GUARD_MAX_THREADS];
    int32_t  ndemoted;                 /* entries in dem[], newest last                   */
    int32_t  pad;
    guard_demote_t dem[GUARD_MAX_DEMOTED];
} guard_stats_t;

#endif
