/* fake_stats.c - bring-up tool: pretends to be ctl_loop so the monitor can be tested and
 * demoed before A's loop exists.  Owner: C.
 *
 * Runs a real periodic loop (LOOP_PERIOD_NS), measures its real wake lateness, runs the
 * NORMAL -> SAFE -> RECOVERY -> NORMAL state machine from the workplan, and publishes
 * ctl_stats_t in /ctl_stats.  Reads /pot if fake_pot or adc_reader runs, else makes a sine.
 * Reacts to the monitor's cmd_flags: CMD_RESET.
 * Does NOT drive the servo.   usage: fake_stats      (Ctrl-C to stop)
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stddef.h>
#include <math.h>
#include <time.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include "config.h"
#include "ipc_defs.h"

static int64_t now_ns(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return (int64_t)t.tv_sec * 1000000000LL + t.tv_nsec;
}

static void sleep_until(int64_t t)
{
    struct timespec ts = { (time_t)(t / 1000000000LL), (long)(t % 1000000000LL) };
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, NULL);
}

/* running mean / standard deviation (Welford) */
typedef struct { uint64_t n; double mean, m2; } run_t;
static void run_add(run_t *r, double x)
{
    r->n++;
    double d = x - r->mean;
    r->mean += d / r->n;
    r->m2 += d * (x - r->mean);
}
static double run_std(const run_t *r) { return r->n > 1 ? sqrt(r->m2 / (r->n - 1)) : 0; }

static ctl_stats_t L;          /* local working copy */
static run_t wake_run, lat_run;

static void reset_stats(void)
{
    memset(&L, 0, sizeof L);
    memset(&wake_run, 0, sizeof wake_run);
    memset(&lat_run, 0, sizeof lat_run);
    L.state = ST_NORMAL;
    L.recovery_target = N_RECOVERY;
    L.wake_late_min_ns = INT64_MAX;
}

/* copy everything between seq and cmd_flags, leaving cmd_flags to the monitor */
static void publish(volatile ctl_stats_t *sh)
{
    size_t a = offsetof(ctl_stats_t, state), b = offsetof(ctl_stats_t, cmd_flags);
    sh->seq++;                                  /* odd: writing */
    __sync_synchronize();
    memcpy((char *)sh + a, (const char *)&L + a, b - a);
    __sync_synchronize();
    sh->seq++;                                  /* even: consistent */
}

static void *open_shm(const char *name, size_t size, int create)
{
    int fd = shm_open(name, create ? (O_CREAT | O_RDWR) : O_RDONLY, 0666);
    if (fd < 0) return NULL;
    struct stat st;
    if (create) { if (ftruncate(fd, size) != 0) { close(fd); return NULL; } }
    else if (fstat(fd, &st) != 0 || (size_t)st.st_size < size) { close(fd); return NULL; }
    void *p = mmap(NULL, size, create ? PROT_READ | PROT_WRITE : PROT_READ, MAP_SHARED, fd, 0);
    close(fd);
    return p == MAP_FAILED ? NULL : p;
}

int main(void)
{
    volatile ctl_stats_t *sh = open_shm(SHM_STATS_NAME, sizeof(ctl_stats_t), 1);
    if (!sh) { perror("fake_stats: /ctl_stats"); return 1; }
    sh->cmd_flags = 0;
    reset_stats();
    publish(sh);
    printf("fake_stats: publishing %s every %.1f ms (Ctrl-C to stop)\n",
           SHM_STATS_NAME, LOOP_PERIOD_NS / 1e6);

    volatile pot_sample_t *pot = NULL;
    int64_t t0 = now_ns(), next = t0 + LOOP_PERIOD_NS;

    for (;;) {
        sleep_until(next);
        int64_t wake = now_ns();
        int64_t late = wake - next;

        /* commands from the monitor */
        uint32_t cmd = __atomic_exchange_n(&sh->cmd_flags, 0, __ATOMIC_SEQ_CST);
        if (cmd & CMD_RESET) reset_stats();

        /* pot: /pot if someone publishes it, else a 4 s sine */
        if (!pot) pot = open_shm(SHM_POT_NAME, sizeof(pot_sample_t), 0);
        int pm;
        int64_t pot_ts;
        if (pot) { pm = pot->permille; pot_ts = pot->ts_ns; }
        else { pm = (int)(500 + 500 * sin(2 * M_PI * (wake - t0) / 4e9)); pot_ts = wake - 300000; }

        int servo = L.state == ST_NORMAL ? P_MIN_US + pm * (P_MAX_US - P_MIN_US) / 1000 : SAFE_PULSE_US;
        int64_t latency = now_ns() - pot_ts;

        int64_t work = now_ns() - wake;

        /* statistics */
        L.cycles++;
        L.wake_late_last_ns = late;
        if (late < L.wake_late_min_ns) L.wake_late_min_ns = late;
        if (late > L.wake_late_max_ns) L.wake_late_max_ns = late;
        run_add(&wake_run, (double)late);
        L.wake_late_avg_ns = (int64_t)wake_run.mean;
        L.wake_late_std_ns = (int64_t)run_std(&wake_run);
        L.work_last_ns = work;
        if (work > L.work_max_ns) L.work_max_ns = work;
        L.latency_last_ns = latency;
        if (latency > L.latency_max_ns) L.latency_max_ns = latency;
        run_add(&lat_run, (double)latency);
        L.latency_avg_ns = (int64_t)lat_run.mean;
        L.pot_permille = pm;
        L.servo_us = servo;

        /* state machine (workplan section 1) */
        int overrun = late > WAKE_LATE_LIMIT_US * 1000LL || work > DEADLINE_NS;
        if (overrun) {
            L.misses++;
            if (L.state == ST_NORMAL) { L.state = ST_SAFE; L.safe_activations++; }
            L.recovery_count = 0;                   /* one bad cycle resets the count */
        } else if (L.state == ST_SAFE) {
            L.state = ST_RECOVERY;
            L.recovery_count = 1;
        } else if (L.state == ST_RECOVERY) {
            if (++L.recovery_count >= (uint32_t)N_RECOVERY) { L.state = ST_NORMAL; L.recovery_count = 0; }
        }
        publish(sh);

        /* next slot: after an overrun skip the missed slots and stay on the grid */
        next += LOOP_PERIOD_NS;
        int64_t now = now_ns();
        while (next <= now) next += LOOP_PERIOD_NS;
    }
}
