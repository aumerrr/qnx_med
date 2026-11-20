/* ctl_loop.c - hard real-time control loop (Person A's job, written by B)
 *
 * /pot (adc_reader) --> ctl_loop --write--> /dev/actuator (actuator_rm) --> servo
 *                          |   ^
 *                  /ctl_stats  cmd_flags (monitor)
 *
 * - POSIX timer (CLOCK_MONOTONIC) delivering a QNX PULSE; the loop blocks in
 *   MsgReceivePulse. The timer is re-armed every cycle for the next absolute
 *   slot, so after an overrun missed slots are SKIPPED (no burst of catch-up).
 * - Per cycle: wake lateness, work time, pot-to-command latency, jitter
 *   (standard deviation of wake lateness), deadline misses.
 * - State machine: NORMAL -> SAFE -> RECOVERY -> NORMAL (after N_RECOVERY
 *   consecutive compliant cycles). SAFE/RUN are sent to the actuator by devctl.
 * - Nothing is printed inside the loop. A low-priority logger thread prints
 *   state changes (and, with -v, one status line per second).
 *
 * usage: ctl_loop [-v]
 * Start with:  on -C 1 /tmp/ctl_loop -v     (the loop sets SCHED_FIFO PRIO_CTL itself)
 */
#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <errno.h>
#include <time.h>
#include <signal.h>
#include <fcntl.h>
#include <unistd.h>
#include <pthread.h>
#include <sched.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/neutrino.h>
#include <devctl.h>

#include "config.h"
#include "ipc_defs.h"
#include "rt_util.h"

#ifndef CLOCK_TICK_US
#define CLOCK_TICK_US     100               /* system clock tick requested at start-up */
#endif
#define PULSE_CODE_TICK   (_PULSE_CODE_MINAVAIL + 1)

/* ---------------- time ---------------- */

static uint64_t g_t0;

static int64_t now_ns(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (int64_t)ts.tv_sec * 1000000000LL + (int64_t)ts.tv_nsec;
}

static void arm_abs(timer_t tid, int64_t ns)
{
    struct itimerspec its;
    memset(&its, 0, sizeof(its));
    its.it_value.tv_sec  = (time_t)(ns / 1000000000LL);
    its.it_value.tv_nsec = (long)(ns % 1000000000LL);
    int flags = TIMER_ABSTIME;
#ifdef TIMER_PRECISE
    flags |= TIMER_PRECISE;                              /* ask for no tick rounding / tolerance */
#endif
    timer_settime(tid, flags, &its, NULL);               /* one-shot */
}

/* Run n cycles of the plain timer loop (no control work) and return the worst wake lateness;
 * *min_out gets the best one. Used only for start-up phase alignment. */
static int64_t cal_run(int chid, timer_t tid, int64_t *slot, int n, int64_t *min_out)
{
    int64_t mx = 0, mn = INT64_MAX;
    for (int i = 0; i < n; i++) {
        struct _pulse p;
        if (MsgReceivePulse(chid, &p, sizeof(p), NULL) == -1) {
            if (errno == EINTR) { i--; continue; }
            break;
        }
        int64_t l = now_ns() - *slot;
        if (l > mx) mx = l;
        if (l < mn) mn = l;
        *slot += LOOP_PERIOD_NS;
        int64_t t = now_ns();
        while (*slot <= t) *slot += LOOP_PERIOD_NS;
        arm_abs(tid, *slot);
    }
    if (min_out) *min_out = mn;
    return mx;
}

/* sqrt without libm: Newton, warm-started from the previous value */
static double my_sqrt(double v, double guess)
{
    if (v <= 0.0) return 0.0;
    double x = guess > 0.0 ? guess : v;
    for (int i = 0; i < 60; i++) {
        double nx = 0.5 * (x + v / x);
        double d = nx - x;
        x = nx;
        if (d < 1e-3 && d > -1e-3) break;
    }
    return x;
}

/* ---------------- event ring (loop -> logger, lock free) ---------------- */

enum { EV_SAFE = 1, EV_RECOVERING, EV_NORMAL, EV_RESET };
#define R_LATE   1
#define R_WORK   2
#define R_INPUT  4

typedef struct { int64_t t_ns; int kind; int reason; int64_t late_ns; int64_t work_ns; } ev_t;
#define EV_N 64
static ev_t g_ev[EV_N];
static volatile uint32_t g_ev_head;

static void ev_push(int kind, int reason, int64_t late, int64_t work)
{
    uint32_t h = g_ev_head;
    ev_t *e = &g_ev[h % EV_N];
    e->t_ns = now_ns(); e->kind = kind; e->reason = reason;
    e->late_ns = late;  e->work_ns = work;
    __sync_synchronize();
    g_ev_head = h + 1;
}

/* ---------------- shared memory ---------------- */

static ctl_stats_t *g_st;

static int stats_snapshot(ctl_stats_t *out)
{
    for (int i = 0; i < 100; i++) {
        uint64_t s = g_st->seq;
        if (s & 1) continue;
        __sync_synchronize();
        memcpy(out, (const void *)g_st, sizeof(*out));
        __sync_synchronize();
        if (g_st->seq == s) return 0;
    }
    return -1;
}

static const char *state_name(int s)
{
    return s == ST_NORMAL ? "NORMAL" : s == ST_SAFE ? "SAFE" : "RECOVERY";
}

static volatile pot_sample_t *open_pot(void)
{
    for (int i = 0; i < 100; i++) {
        int fd = shm_open(SHM_POT_NAME, O_RDONLY, 0);
        if (fd >= 0) {
            void *m = mmap(NULL, sizeof(pot_sample_t), PROT_READ, MAP_SHARED, fd, 0);
            if (m != MAP_FAILED) return (volatile pot_sample_t *)m;
        }
        usleep(100000);                 /* adc_reader not up yet, wait up to 10 s */
    }
    return NULL;
}

static int pot_read(volatile pot_sample_t *p, pot_sample_t *out)
{
    for (int i = 0; i < 100; i++) {
        uint64_t s1 = p->seq;
        if (s1 & 1) continue;
        __sync_synchronize();
        out->ts_ns    = p->ts_ns;
        out->permille = p->permille;
        out->raw      = p->raw;
        out->status   = p->status;
        __sync_synchronize();
        if (p->seq == s1) { out->seq = s1; return 0; }
    }
    return -1;
}

/* ---------------- statistics accumulators ---------------- */

typedef struct {
    uint64_t cycles, misses, safe_act;
    int64_t  late_last, late_min, late_max;
    double   late_mean, late_m2, std_prev;
    int64_t  work_last, work_max;
    uint64_t lat_n;
    int64_t  lat_last, lat_max;
    double   lat_mean;
} acc_t;

static void acc_reset(acc_t *a)
{
    memset(a, 0, sizeof(*a));
    a->late_min = INT64_MAX;
}

static void acc_late(acc_t *a, int64_t late)
{
    a->late_last = late;
    if (late < a->late_min) a->late_min = late;
    if (late > a->late_max) a->late_max = late;
    double d = (double)late - a->late_mean;               /* Welford */
    a->late_mean += d / (double)a->cycles;
    a->late_m2   += d * ((double)late - a->late_mean);
}

static void publish(const acc_t *a, int state, uint32_t rec, int pm, int servo_us)
{
    double var = a->cycles > 1 ? a->late_m2 / (double)a->cycles : 0.0;
    int64_t std = (int64_t)my_sqrt(var, ((acc_t *)a)->std_prev);
    ((acc_t *)a)->std_prev = (double)std;

    uint64_t s = g_st->seq;
    g_st->seq = s + 1;                                    /* odd: writing */
    __sync_synchronize();
    g_st->state             = state;
    g_st->cycles            = a->cycles;
    g_st->wake_late_last_ns = a->late_last;
    g_st->wake_late_min_ns  = a->late_min == INT64_MAX ? 0 : a->late_min;
    g_st->wake_late_max_ns  = a->late_max;
    g_st->wake_late_avg_ns  = (int64_t)a->late_mean;
    g_st->wake_late_std_ns  = std;
    g_st->work_last_ns      = a->work_last;
    g_st->work_max_ns       = a->work_max;
    g_st->latency_last_ns   = a->lat_last;
    g_st->latency_avg_ns    = (int64_t)a->lat_mean;
    g_st->latency_max_ns    = a->lat_max;
    g_st->misses            = a->misses;
    g_st->safe_activations  = a->safe_act;
    g_st->recovery_count    = rec;
    g_st->recovery_target   = N_RECOVERY;
    g_st->pot_permille      = pm;
    g_st->servo_us          = servo_us;
    __sync_synchronize();
    g_st->seq = s + 2;                                    /* even: done */
}

/* ---------------- logger thread (low priority, never in the loop) ---------------- */

static void *logger(void *arg)
{
    int verbose = *(int *)arg;
    uint32_t tail = 0;
    int64_t last = now_ns();

    for (;;) {
        usleep(50000);

        uint32_t head = g_ev_head;
        if (head - tail > EV_N) tail = head - EV_N;
        while (tail != head) {
            ev_t *e = &g_ev[tail % EV_N];
            double t = (double)(e->t_ns - (int64_t)g_t0) / 1e9;
            switch (e->kind) {
            case EV_SAFE:
                printf("[%8.3f s] -> SAFE      (%s%s%s) late=%lld us work=%lld us\n", t,
                       (e->reason & R_LATE)  ? "wake late " : "",
                       (e->reason & R_WORK)  ? "work>deadline " : "",
                       (e->reason & R_INPUT) ? "pot input lost/stale" : "",
                       (long long)(e->late_ns / 1000), (long long)(e->work_ns / 1000));
                break;
            case EV_RECOVERING: printf("[%8.3f s] -> RECOVERY  (counting %d good cycles)\n", t, N_RECOVERY); break;
            case EV_NORMAL:     printf("[%8.3f s] -> NORMAL    (recovered)\n", t); break;
            case EV_RESET:      printf("[%8.3f s] reset requested\n", t); break;
            }
            tail++;
        }
        fflush(stdout);

        int64_t now = now_ns();
        if (verbose && now - last >= 1000000000LL) {
            last = now;
            ctl_stats_t s;
            if (stats_snapshot(&s) == 0)
                printf("%-8s cycles=%llu late last/max/std=%lld/%lld/%lld us work max=%lld us "
                       "lat avg/max=%lld/%lld us misses=%llu safe=%llu rec=%u/%u pot=%d servo=%d\n",
                       state_name(s.state), (unsigned long long)s.cycles,
                       (long long)(s.wake_late_last_ns / 1000), (long long)(s.wake_late_max_ns / 1000),
                       (long long)(s.wake_late_std_ns / 1000), (long long)(s.work_max_ns / 1000),
                       (long long)(s.latency_avg_ns / 1000), (long long)(s.latency_max_ns / 1000),
                       (unsigned long long)s.misses, (unsigned long long)s.safe_activations,
                       s.recovery_count, s.recovery_target, s.pot_permille, s.servo_us);
            fflush(stdout);
        }
    }
    return NULL;
}

/* ---------------- main ---------------- */

int main(int argc, char **argv)
{
    static int verbose;
    verbose = (argc > 1 && strcmp(argv[1], "-v") == 0);
    g_t0 = (uint64_t)now_ns();

    /* The default system tick (~1 ms) rounds every timer expiry up to a tick boundary, which
     * made EVERY cycle ~600 us late (over WAKE_LATE_LIMIT_US) so the loop stayed in SAFE.
     * Use a fine tick (needs root). Lateness then drops to well under 100 us. */
    {
        struct _clockperiod cp;
        memset(&cp, 0, sizeof cp);
        cp.nsec = (uint32_t)CLOCK_TICK_US * 1000u;
        if (ClockPeriod(CLOCK_REALTIME, &cp, NULL, 0) == -1)
            printf("ctl_loop: note: system tick cannot be changed on this kernel - using phase alignment\n");
        struct _clockperiod now_cp;
        if (ClockPeriod(CLOCK_REALTIME, NULL, &now_cp, 0) == 0)
            printf("ctl_loop: system clock tick = %u us\n", (unsigned)(now_cp.nsec / 1000));
    }

    /* shared memory out: /ctl_stats */
    int sfd = shm_open(SHM_STATS_NAME, O_RDWR | O_CREAT, 0666);
    if (sfd < 0) { perror("ctl_loop: shm_open /ctl_stats"); return 1; }
    if (ftruncate(sfd, sizeof(ctl_stats_t)) != 0) { perror("ctl_loop: ftruncate"); return 1; }
    void *sm = mmap(NULL, sizeof(ctl_stats_t), PROT_READ | PROT_WRITE, MAP_SHARED, sfd, 0);
    if (sm == MAP_FAILED) { perror("ctl_loop: mmap /ctl_stats"); return 1; }
    g_st = (ctl_stats_t *)sm;
    memset((void *)g_st, 0, sizeof(*g_st));
    g_st->state = ST_NORMAL;
    g_st->recovery_target = N_RECOVERY;

    /* inputs: /pot and /dev/actuator (wait for them to appear) */
    volatile pot_sample_t *pot = open_pot();
    if (!pot) { fprintf(stderr, "ctl_loop: /pot not found (is adc_reader running?)\n"); return 1; }

    int act = -1;
    for (int i = 0; i < 100 && act < 0; i++) {
        act = open(ACT_DEV_PATH, O_RDWR);
        if (act < 0) usleep(100000);
    }
    if (act < 0) { fprintf(stderr, "ctl_loop: cannot open %s (is actuator_rm running?)\n", ACT_DEV_PATH); return 1; }
    devctl(act, DCMD_ACT_RUN, NULL, 0, NULL);           /* start with the actuator free */

    /* timer -> pulse on a private channel */
    int chid = ChannelCreate(0);
    int coid = ConnectAttach(0, 0, chid, _NTO_SIDE_CHANNEL, 0);
    if (chid == -1 || coid == -1) { perror("ctl_loop: channel"); return 1; }

    /* logger first: it must NOT inherit the real-time priority */
    pthread_t lt;
    pthread_create(&lt, NULL, logger, &verbose);

    if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0)
        fprintf(stderr, "ctl_loop: warning: mlockall failed\n");

    struct sched_param sp;
    memset(&sp, 0, sizeof(sp));
    sp.sched_priority = PRIO_CTL;
    if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp) != 0)
        fprintf(stderr, "ctl_loop: warning: could not set FIFO priority %d (run as root)\n", PRIO_CTL);

    /* Keep the loop on its own core (CPU1), away from the servo PWM thread that spins on CPU2.
     * The logger thread was created before this, so it keeps the default (all CPUs). */
    if (rt_pin_self(MASK_CTL) != 0)
        fprintf(stderr, "ctl_loop: warning: could not pin to CPU mask 0x%x\n", MASK_CTL);

    struct sigevent ev;
    /* The pulse priority IS the priority the loop runs at after MsgReceivePulse.
     * SIGEV_PULSE_PRIO_INHERIT would drop it to the process's start priority (10, the shell
     * default) every cycle, so any priority-10 load (cpu_hog) delayed it by a whole time
     * slice. Deliver the tick at PRIO_CTL instead. */
    SIGEV_PULSE_INIT(&ev, coid, PRIO_CTL, PULSE_CODE_TICK, 0);
    timer_t tid;
    if (timer_create(CLOCK_MONOTONIC, &ev, &tid) == -1) { perror("ctl_loop: timer_create"); return 1; }

    printf("ctl_loop: period %lld us, wake-late limit %d us, deadline %lld us, N=%d, prio %d\n",
           (long long)(LOOP_PERIOD_NS / 1000), WAKE_LATE_LIMIT_US,
           (long long)(DEADLINE_NS / 1000), N_RECOVERY, PRIO_CTL);
    fflush(stdout);

    static acc_t a;
    acc_reset(&a);
    int      state = ST_NORMAL;
    uint32_t rec   = 0;
    int      last_pm = 0, last_servo = 0;
    const int64_t period = LOOP_PERIOD_NS;

    int64_t slot = now_ns() + 100000000LL;              /* first slot in 100 ms */
    arm_abs(tid, slot);

    /* ---- start-up phase alignment ----
     * With a fixed 1 ms system tick, a timer fires at the first tick after its expiry time, so
     * every cycle wakes late by the SAME amount (the distance from our slot to the next tick,
     * 0..1000 us, random per start). That is a constant offset, not jitter. Measure it, then
     * move the whole schedule so each slot sits just before a tick (margin), and verify.
     * The period (5 ms) is a whole number of ticks, so the alignment holds for every cycle. */
    {
        static const int margins_us[] = { 50, 100, 200, 300 };
        int64_t base_min = 0, applied = 0, verified = -1;
        cal_run(chid, tid, &slot, 20, &base_min);
        for (unsigned k = 0; k < sizeof margins_us / sizeof margins_us[0]; k++) {
            int64_t shift = base_min - (int64_t)margins_us[k] * 1000LL;
            if (shift <= 0) break;                          /* already better than this margin */
            slot += shift - applied;
            applied = shift;
            arm_abs(tid, slot);
            int64_t mx = cal_run(chid, tid, &slot, 10, NULL);
            if (mx < (int64_t)WAKE_LATE_LIMIT_US * 1000LL / 2) { verified = mx; break; }
        }
        if (applied && verified < 0) {                      /* nothing helped: undo */
            slot -= applied;
            applied = 0;
            arm_abs(tid, slot);
        }
        printf("ctl_loop: phase alignment: raw offset %lld us, shift %lld us, verified max %lld us\n",
               (long long)(base_min / 1000), (long long)(applied / 1000),
               (long long)(verified < 0 ? base_min / 1000 : verified / 1000));
        fflush(stdout);
    }

    for (;;) {
        struct _pulse pulse;
        if (MsgReceivePulse(chid, &pulse, sizeof(pulse), NULL) == -1) {
            if (errno == EINTR) continue;
            break;
        }
        int64_t t_wake = now_ns();
        int64_t late = t_wake - slot;
        if (late < 0) late = 0;

        /* re-arm for the next slot FIRST; skip slots that are already past */
        int64_t next = slot + period;
        while (next <= t_wake) next += period;
        int skipped = (int)((next - slot) / period) - 1;
        slot = next;
        arm_abs(tid, slot);

        a.cycles++;
        acc_late(&a, late);

        /* commands from the monitor */
        uint32_t flags = __atomic_exchange_n(&g_st->cmd_flags, 0u, __ATOMIC_ACQ_REL);
        if (flags & CMD_RESET) {
            acc_reset(&a);
            a.cycles = 1;
            acc_late(&a, late);
            if (state != ST_NORMAL) devctl(act, DCMD_ACT_RUN, NULL, 0, NULL);
            state = ST_NORMAL; rec = 0;
            ev_push(EV_RESET, 0, 0, 0);
        }

        /* input */
        pot_sample_t s;
        int rd = pot_read(pot, &s);
        int64_t age = rd == 0 ? t_wake - s.ts_ns : INT64_MAX;
        int input_bad = (rd != 0) || (s.status != POT_OK) || (age > (int64_t)POT_STALE_US * 1000LL);

        int late_bad = (late > (int64_t)WAKE_LATE_LIMIT_US * 1000LL) || skipped > 0;

        /* command: only while NORMAL and the cycle so far is healthy */
        if (state == ST_NORMAL && !input_bad && !late_bad) {
            long pm = s.permille;
            if (pm < 0) pm = 0;
            if (pm > 1000) pm = 1000;
            int pulse_us = P_MIN_US + (int)(pm * (P_MAX_US - P_MIN_US) / 1000);
            char buf[16];
            int n = snprintf(buf, sizeof(buf), "%d", pulse_us);
            if (write(act, buf, (size_t)n) == n) {
                int64_t t_cmd = now_ns();
                int64_t lat = t_cmd - s.ts_ns;
                a.lat_n++;
                a.lat_last = lat;
                if (lat > a.lat_max) a.lat_max = lat;
                a.lat_mean += ((double)lat - a.lat_mean) / (double)a.lat_n;
                last_pm = (int)pm;
                last_servo = pulse_us;
            }
        } else if (rd == 0) {
            last_pm = s.permille;
        }

        int64_t t_done = now_ns();
        int64_t work = t_done - t_wake;
        a.work_last = work;
        if (work > a.work_max) a.work_max = work;
        int work_bad = work > (int64_t)DEADLINE_NS;

        if (late_bad || work_bad) a.misses++;
        int compliant = !(late_bad || work_bad || input_bad);
        int reason = (late_bad ? R_LATE : 0) | (work_bad ? R_WORK : 0) | (input_bad ? R_INPUT : 0);

        /* state machine */
        switch (state) {
        case ST_NORMAL:
            if (!compliant) {
                devctl(act, DCMD_ACT_SAFE, NULL, 0, NULL);
                state = ST_SAFE; rec = 0; a.safe_act++;
                ev_push(EV_SAFE, reason, late, work);
            }
            break;
        case ST_SAFE:
            if (compliant) {
                state = ST_RECOVERY; rec = 1;
                ev_push(EV_RECOVERING, 0, late, work);
            }
            break;
        case ST_RECOVERY:
            if (compliant) rec++;
            else rec = 0;                               /* one bad cycle restarts the count */
            break;
        }
        if (state == ST_RECOVERY && rec >= (uint32_t)N_RECOVERY) {
            devctl(act, DCMD_ACT_RUN, NULL, 0, NULL);
            state = ST_NORMAL; rec = 0;
            ev_push(EV_NORMAL, 0, late, work);
        }

        publish(&a, state, rec, last_pm, last_servo);
    }
    return 0;
}
