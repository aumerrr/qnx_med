/* ipc_defs.h - the contract between adc_reader, ctl_loop, actuator_rm, monitor.
 * Owner: C. FROZEN unless announced in the team chat.
 *
 * Shared memory objects (shm_open + mmap, read-only for consumers):
 *   /pot        pot_sample_t   written by adc_reader (B), read by ctl_loop (A)
 *   /ctl_stats  ctl_stats_t    written by ctl_loop (A), read by monitor (C)
 * Both use a seqlock: writer does seq++ (odd), writes data, seq++ (even).
 * Reader copies, retrying while seq is odd or changed during the copy.
 */
#ifndef IPC_DEFS_H
#define IPC_DEFS_H
#include <stdint.h>

#define SHM_POT_NAME        "/pot"
#define SHM_STATS_NAME      "/ctl_stats"
#define ACTUATOR_PATH       "/dev/actuator"

/* ---- pot sample (adc_reader -> ctl_loop) ---- */
#define POT_OK     0
#define POT_ERROR  1                    /* last I2C read failed; permille is the last good value */

typedef struct {
    volatile uint64_t seq;              /* seqlock counter                             */
    int64_t  ts_ns;                     /* CLOCK_MONOTONIC time the ADC was read       */
    int32_t  permille;                  /* pot position 0..1000                        */
    int32_t  raw;                       /* raw ADS1115 count (for the dashboard)       */
    int32_t  status;                    /* POT_OK / POT_ERROR                          */
    int32_t  pad;
} pot_sample_t;

/* ---- controller state ---- */
typedef enum {
    ST_NORMAL   = 0,
    ST_SAFE     = 1,
    ST_RECOVERY = 2
} ctl_state_t;

/* ---- command bits the monitor sets in ctl_stats_t.cmd_flags (ctl_loop reads and clears) ---- */
#define CMD_INJECT_OVERRUN  (1u << 0)   /* ctl_loop burns > deadline once              */
#define CMD_RESET           (1u << 1)   /* clear stats, go back to NORMAL              */

/* ---- control-loop statistics (ctl_loop -> monitor). All times in ns unless noted. ---- */
typedef struct {
    volatile uint64_t seq;              /* seqlock counter                             */
    int32_t  state;                     /* ctl_state_t                                 */
    int32_t  pad0;
    uint64_t cycles;                    /* control cycles executed                     */

    int64_t  wake_late_last_ns;         /* lateness of the timer pulse vs its slot     */
    int64_t  wake_late_min_ns;
    int64_t  wake_late_max_ns;
    int64_t  wake_late_avg_ns;
    int64_t  wake_late_std_ns;          /* jitter = standard deviation of wake lateness */

    int64_t  work_last_ns;              /* time spent in the cycle body                */
    int64_t  work_max_ns;

    int64_t  latency_last_ns;           /* pot sample timestamp -> actuator command sent */
    int64_t  latency_avg_ns;
    int64_t  latency_max_ns;

    uint64_t misses;                    /* deadline / wake-late violations             */
    uint64_t safe_activations;          /* times the SAFE state was entered            */
    uint32_t recovery_count;            /* consecutive compliant cycles so far         */
    uint32_t recovery_target;           /* N_RECOVERY                                  */

    int32_t  pot_permille;              /* last pot value used                         */
    int32_t  servo_us;                  /* last pulse sent to the actuator             */

    volatile uint32_t cmd_flags;        /* written by monitor, cleared by ctl_loop     */
    uint32_t pad1;
} ctl_stats_t;

/* ---- actuator_rm interface (/dev/actuator) ----
 *   write(fd, "1500", n)  text integer pulse in us, clamped, junk rejected
 *   read(fd, buf, n)      "<last_cmd_us> <state>\n"
 *   devctl(ACT_*):        see below (use a QNX DCMD_MISC-based code in actuator_rm)
 */
#define ACT_SAFE        1               /* apply SAFE_PULSE_US immediately             */
#define ACT_RUN         2               /* leave safe state                            */
#define ACT_GET_STATS   3               /* returns servo_stats_t (see servo.h)         */

#endif
