/* config.h - project-wide tunables. Every value can be overridden with -DNAME=value.
 * Owner: C (integrator). Announce changes in the team chat before editing. */
#ifndef CONFIG_H
#define CONFIG_H

/* ---- control loop ---- */
#ifndef LOOP_PERIOD_NS
#define LOOP_PERIOD_NS      5000000LL   /* 5 ms control period                       */
#endif
#ifndef WAKE_LATE_LIMIT_US
#define WAKE_LATE_LIMIT_US  500         /* wake lateness above this = overrun        */
#endif
#ifndef DEADLINE_NS
#define DEADLINE_NS         LOOP_PERIOD_NS /* work time above this = overrun         */
#endif
#ifndef N_RECOVERY
#define N_RECOVERY          50          /* consecutive compliant cycles to leave RECOVERY */
#endif
#ifndef POT_STALE_US
#define POT_STALE_US        20000       /* pot sample older than this = stale        */
#endif

/* ---- servo mapping (proportional): pulse = P_MIN + permille*(P_MAX-P_MIN)/1000 ---- */
#ifndef P_MIN_US
#define P_MIN_US            600
#endif
#ifndef P_MAX_US
#define P_MAX_US            2400
#endif
#ifndef SAFE_PULSE_US
#define SAFE_PULSE_US       1500        /* safe state = servo centred                */
#endif

/* ---- priorities (SCHED_FIFO): PWM > control loop > ADC reader ---- */
#define PRIO_PWM            70
#define PRIO_CTL            60
#define PRIO_ADC            55

/* ---- CPU masks (stock image: only single-CPU masks valid; A will change for clusters) ---- */
#define MASK_SYS            0x1u        /* CPU0: drivers, monitor, shell             */
#define MASK_CTL            0x2u        /* CPU1: ctl_loop                            */
#define MASK_PWM            0x4u        /* CPU2: servo PWM thread (see servo.c)      */
#define MASK_LOAD           0x8u        /* CPU3: our own load generators             */

#endif
