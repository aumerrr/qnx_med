/* guardian - recovery manager and system observer (Person A).
 *
 * Runs on core 0 (on -C0) at PRIO_GUARD, above every load, and never touches the RT path
 * except through two actuator commands (SAFE / RUN).
 *
 * Every 5 ms:
 *   - reads ctl_loop's cycle counter from /ctl_stats.
 *   - STARVED: the counter has not moved for GUARD_STARVE_NS -> DCMD_ACT_SAFE at once
 *     (servo to centre). ctl_loop then sees its own overrun when it runs again, goes
 *     SAFE -> RECOVERY -> NORMAL and sends DCMD_ACT_RUN itself.
 *     If ctl_loop comes back healthy without ever going SAFE (false alarm), the guardian
 *     sends DCMD_ACT_RUN after 200 ms so the servo is not left frozen.
 *   - DISTURBED: ctl_loop is in SAFE or RECOVERY.
 * Every 250 ms (100 ms while disturbed) it scans all threads in /proc:
 *   - per-core CPU use (from the kernel's idle threads, one per core)
 *   - our threads and load threads: priority, cores allowed, core last used, CPU %
 *   - "offenders": threads that are not ours or a system driver, have a base priority
 *     >= GUARD_DEMOTE_MIN_PRIO and may run on the real-time cores (CPU hogs, message floods)
 * When a disturbance has lasted longer than the grace timer (-g, default 1000 ms), every
 * offender is lowered to GUARD_DEMOTE_TO with SchedSet(). A message flood is handled the
 * same way: the resource manager's threads run at the priority of the client they serve,
 * so lowering the flooding clients also lowers the server work they cause.
 *
 * usage: guardian [-g grace_ms] [-p min_prio] [-v]
 * Must run as root (priority 100, SchedSet on other processes, /proc access).
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <dirent.h>
#include <time.h>
#include <sched.h>
#include <pthread.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/neutrino.h>
#include <sys/procfs.h>
#include <sys/syspage.h>
#include <sys/states.h>
#include <devctl.h>

#include "config.h"
#include "ipc_defs.h"
#include "guard_ipc.h"

#define MS 1000000LL

static int64_t now_ns(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return (int64_t)t.tv_sec * 1000000000LL + t.tv_nsec;
}

static int64_t g_t0;
static int     g_verbose;

#define LOG(...) do { printf("[%8.3f s] ", (now_ns() - g_t0) / 1e9); printf(__VA_ARGS__); \
                      printf("\n"); fflush(stdout); } while (0)

/* ---------------- who is who ---------------- */

static const char *CRIT_NAMES[] = { "ctl_loop", "actuator_rm", "adc_reader", "fake_pot", NULL };
static const char *AUX_NAMES[]  = { "guardian", "monitor", NULL };
static const char *LOAD_NAMES[] = { "cpu_hog", "msg_flood", NULL };
/* system programs the guardian must never lower, even if they sit at a high priority */
static const char *SYS_NAMES[]  = { "procnto-smp-instr", "procnto-smp", "procnto", "i2c-bcm2711",
    "wdtkick", "devc-serminiuart", "devc-serpl011", "devc-pty", "devb-sdmmc-bcm2711", "io-sock",
    "io-usb-otg", "spi-bcm2711", "pci-server", "slogger2", "sshd", "sshd-session", "qconn",
    "dhcpcd", "pipe", "random", "mqueue", "dumper", "devf-ram", "ksh", "sh", "pidin", "on",
    "slay", "su", "login", "pdebug", NULL };

static int in_list(const char *name, const char **list)
{
    for (int i = 0; list[i]; i++)
        if (strcmp(name, list[i]) == 0) return 1;
    return 0;
}

/* ---------------- process names (cached, from /proc/<pid>/exefile) ---------------- */

#define NAME_SLOTS 512
static struct { int pid; char name[GUARD_NAME_LEN]; } g_names[NAME_SLOTS];

static const char *proc_name(int pid)
{
    unsigned h = (unsigned)pid % NAME_SLOTS;
    for (int i = 0; i < NAME_SLOTS; i++) {
        unsigned k = (h + (unsigned)i) % NAME_SLOTS;
        if (g_names[k].pid == pid) return g_names[k].name;
        if (g_names[k].pid == 0) {
            char path[64], buf[256];
            snprintf(path, sizeof path, "/proc/%d/exefile", pid);
            int fd = open(path, O_RDONLY);
            ssize_t n = fd >= 0 ? read(fd, buf, sizeof buf - 1) : -1;
            if (fd >= 0) close(fd);
            if (n > 0) {
                buf[n] = '\0';
                char *nl = strchr(buf, '\n');
                if (nl) *nl = '\0';
                const char *base = strrchr(buf, '/');
                base = base ? base + 1 : buf;
                snprintf(g_names[k].name, GUARD_NAME_LEN, "%s", base);
            } else {
                snprintf(g_names[k].name, GUARD_NAME_LEN, "pid%d", pid);
            }
            g_names[k].pid = pid;
            return g_names[k].name;
        }
    }
    return "?";
}

/* ---------------- per-thread memory between scans ---------------- */

#define TH_SLOTS 1024
typedef struct {
    int      pid, tid;
    uint64_t sutime;           /* CPU time used so far (ns)                 */
    uint32_t runmask;          /* cached run mask, 0 = unknown              */
    int      mask_age;         /* scans since the mask was read             */
    uint8_t  demoted_from;     /* old priority if we lowered it             */
} th_ent_t;

static th_ent_t g_tab[2][TH_SLOTS];
static int      g_cur;

static th_ent_t *tab_find(th_ent_t *t, int pid, int tid, int create)
{
    unsigned h = ((unsigned)pid * 2654435761u + (unsigned)tid * 40503u) % TH_SLOTS;
    for (int i = 0; i < TH_SLOTS; i++) {
        th_ent_t *e = &t[(h + (unsigned)i) % TH_SLOTS];
        if (e->pid == pid && e->tid == tid) return e;
        if (e->pid == 0) {
            if (!create) return NULL;
            memset(e, 0, sizeof *e);
            e->pid = pid; e->tid = tid;
            return e;
        }
    }
    return NULL;
}

/* run mask of another thread: ThreadCtl executed in that thread's context through /proc.
 * Passing zero masks means "do not change, just return the current ones". */
static uint32_t read_runmask(int fd, int tid, int ncpu)
{
#if defined(DCMD_PROC_THREADCTL) && defined(_NTO_TCTL_RUNMASK_GET_AND_SET_INHERIT)
    procfs_threadctl tc;
    memset(&tc, 0, sizeof tc);
    tc.tid = tid;
    tc.cmd = _NTO_TCTL_RUNMASK_GET_AND_SET_INHERIT;
    int words = RMSK_SIZE(ncpu);
    int *size = (int *)tc.data;
    *size = words;
    if (devctl(fd, DCMD_PROC_THREADCTL, &tc, sizeof tc, NULL) == EOK) {
        unsigned *rm = (unsigned *)(size + 1);
        return rm[0];
    }
#else
    (void)fd; (void)tid; (void)ncpu;
#endif
    return 0;
}

/* ---------------- shared memory ---------------- */

static guard_stats_t *G;

static void pub_begin(void) { G->seq++; __sync_synchronize(); }
static void pub_end(void)   { __sync_synchronize(); G->seq++; }

static int seq_copy(const volatile void *src, void *dst, size_t n)
{
    const volatile uint64_t *seq = (const volatile uint64_t *)src;
    for (int i = 0; i < 1000; i++) {
        uint64_t a = *seq;
        if (a & 1) continue;
        __sync_synchronize();
        memcpy(dst, (const void *)src, n);
        __sync_synchronize();
        if (*seq == a) return 0;
    }
    return -1;
}

static volatile ctl_stats_t *map_ctl_stats(void)
{
    int fd = shm_open(SHM_STATS_NAME, O_RDONLY, 0);
    if (fd < 0) return NULL;
    struct stat st;
    if (fstat(fd, &st) != 0 || (size_t)st.st_size < sizeof(ctl_stats_t)) { close(fd); return NULL; }
    void *p = mmap(NULL, sizeof(ctl_stats_t), PROT_READ, MAP_SHARED, fd, 0);
    close(fd);
    return p == MAP_FAILED ? NULL : (volatile ctl_stats_t *)p;
}

/* ---------------- the scan ---------------- */

typedef struct { int pid, tid; uint8_t prio; char name[GUARD_NAME_LEN]; } cand_t;
#define MAX_CAND 64

static int64_t g_last_scan;

static const char *label(const char *name, const procfs_status *st, int busy)
{
    if (!strcmp(name, "actuator_rm")) return st->real_priority == PRIO_PWM ? "servo PWM" : "msg handler";
    if (!strcmp(name, "ctl_loop"))    return st->real_priority == PRIO_CTL ? "ctl loop" : "logger";
    if (!strcmp(name, "adc_reader"))  return "pot reader";
    if (!strcmp(name, "fake_pot"))    return "fake pot";
    if (!strcmp(name, "guardian"))    return "guardian";
    if (!strcmp(name, "monitor"))     return "dashboard";
    if (!strcmp(name, "cpu_hog"))     return busy ? "hog" : "hog main";
    if (!strcmp(name, "msg_flood"))   return busy ? "flooder" : "flood main";
    return "foreign";
}

static int role_rank(int r)
{
    switch (r) { case GR_CRIT: return 0; case GR_AUX: return 1; case GR_LOAD: return 2;
                 case GR_DEMOTED: return 3; default: return 4; }
}

/* Scans every thread. Fills the dashboard table and returns the offenders in cand[]. */
static int scan(int ncpu, int min_prio, cand_t *cand)
{
    int64_t t = now_ns();
    int64_t dt = g_last_scan ? t - g_last_scan : 0;
    g_last_scan = t;

    th_ent_t *prev = g_tab[g_cur];
    th_ent_t *cur  = g_tab[g_cur ^ 1];
    memset(cur, 0, sizeof(th_ent_t) * TH_SLOTS);

    static guard_thread_t rows[GUARD_MAX_THREADS];
    int nrows = 0, ncand = 0, busy_foreign = 0;
    int32_t cpu_busy[GUARD_MAX_CPU];
    for (int i = 0; i < GUARD_MAX_CPU; i++) cpu_busy[i] = -1;

    DIR *d = opendir("/proc");
    if (!d) return 0;
    struct dirent *de;
    while ((de = readdir(d)) != NULL) {
        char *end;
        long pid = strtol(de->d_name, &end, 10);
        if (*end != '\0' || pid <= 0) continue;

        char path[64];
        snprintf(path, sizeof path, "/proc/%ld/ctl", pid);
        int fd = open(path, O_RDWR);
        if (fd < 0) fd = open(path, O_RDONLY);
        if (fd < 0) { snprintf(path, sizeof path, "/proc/%ld/as", pid); fd = open(path, O_RDWR); }
        if (fd < 0) fd = open(path, O_RDONLY);
        if (fd < 0) continue;

        const char *name = pid == 1 ? "procnto" : proc_name((int)pid);
        int is_crit = in_list(name, CRIT_NAMES), is_aux = in_list(name, AUX_NAMES);
        int is_load = in_list(name, LOAD_NAMES), is_sys = pid == 1 || in_list(name, SYS_NAMES);

        procfs_status st;
        memset(&st, 0, sizeof st);
        st.tid = 1;
        while (devctl(fd, DCMD_PROC_TIDSTATUS, &st, sizeof st, NULL) == EOK) {
            int tid = (int)st.tid;
            th_ent_t *p = tab_find(prev, (int)pid, tid, 0);
            th_ent_t *e = tab_find(cur, (int)pid, tid, 1);
            int pct = 0;
            if (e) {
                if (p) { e->runmask = p->runmask; e->mask_age = p->mask_age + 1; e->demoted_from = p->demoted_from; }
                e->sutime = st.sutime;
                if (p && dt > 0 && st.sutime >= p->sutime)
                    pct = (int)((st.sutime - p->sutime) * 1000ULL / (uint64_t)dt);
                if (pct > 1000) pct = 1000;
            }

            /* the kernel's idle threads (priority 0), one per core */
            if (pid == 1 && st.real_priority == 0 && st.last_cpu < GUARD_MAX_CPU && st.last_cpu < ncpu && p)
                cpu_busy[st.last_cpu] = 1000 - pct;

            int is_intr = st.state == STATE_INTR || st.priority >= 250;
            int offender = !is_crit && !is_aux && !is_sys && !is_intr && st.real_priority >= min_prio;

            int listed = is_crit || is_aux || is_load || offender || (e && e->demoted_from);
            if (e && (listed) && (e->runmask == 0 || e->mask_age > 20)) {
                uint32_t m = read_runmask(fd, tid, ncpu);
                if (m) e->runmask = m;
                e->mask_age = 0;
            }
            uint32_t mask = e ? e->runmask : 0;
            if (offender && mask && !(mask & RT_CLUSTER_MASK)) offender = 0;   /* contained elsewhere */

            if (offender) {
                busy_foreign++;
                if (ncand < MAX_CAND) {
                    cand[ncand].pid = (int)pid; cand[ncand].tid = tid; cand[ncand].prio = st.real_priority;
                    snprintf(cand[ncand].name, GUARD_NAME_LEN, "%s", name);
                    ncand++;
                }
            }
            if (listed && nrows < GUARD_MAX_THREADS) {
                guard_thread_t *r = &rows[nrows++];
                memset(r, 0, sizeof *r);
                r->pid = (int32_t)pid; r->tid = tid;
                snprintf(r->name, sizeof r->name, "%s", name);
                snprintf(r->what, sizeof r->what, "%s", label(name, &st, pct > 300));
                r->prio = st.priority; r->policy = st.policy; r->state = st.state;
                r->last_cpu = (int16_t)st.last_cpu;
                r->cpu_pct_x10 = (int16_t)pct;
                r->runmask = mask;
                r->demoted_from = e ? e->demoted_from : 0;
                r->role = is_crit ? (strcmp(r->what, "logger") ? GR_CRIT : GR_AUX)
                        : is_aux  ? GR_AUX
                        : r->demoted_from ? GR_DEMOTED
                        : is_load ? GR_LOAD : GR_FOREIGN;
            }
            st.tid++;
        }
        close(fd);
    }
    closedir(d);
    g_cur ^= 1;

    /* sort rows: critical, aux, loads, demoted, foreign (insertion sort, stable) */
    for (int i = 1; i < nrows; i++) {
        guard_thread_t x = rows[i];
        int j = i - 1;
        while (j >= 0 && role_rank(rows[j].role) > role_rank(x.role)) { rows[j + 1] = rows[j]; j--; }
        rows[j + 1] = x;
    }

    pub_begin();
    G->scans++;
    for (int i = 0; i < GUARD_MAX_CPU; i++) G->cpu_pct_x10[i] = cpu_busy[i];
    G->busy_foreign = busy_foreign;
    G->nthreads = nrows;
    memcpy(G->th, rows, sizeof(rows[0]) * (size_t)nrows);
    pub_end();
    return ncand;
}

static void demote(const cand_t *c, int n)
{
    for (int i = 0; i < n; i++) {
        struct sched_param sp;
        memset(&sp, 0, sizeof sp);
        sp.sched_priority = GUARD_DEMOTE_TO;
        if (SchedSet(c[i].pid, c[i].tid, SCHED_RR, &sp) == -1) {
            LOG("could not lower %s pid %d tid %d: %s", c[i].name, c[i].pid, c[i].tid, strerror(errno));
            continue;
        }
        th_ent_t *e = tab_find(g_tab[g_cur], c[i].pid, c[i].tid, 0);
        if (e) e->demoted_from = c[i].prio;
        LOG("DEMOTED %s pid %d tid %d: priority %d -> %d", c[i].name, c[i].pid, c[i].tid,
            c[i].prio, GUARD_DEMOTE_TO);

        pub_begin();
        if (G->ndemoted == GUARD_MAX_DEMOTED) {
            memmove(&G->dem[0], &G->dem[1], sizeof(G->dem[0]) * (GUARD_MAX_DEMOTED - 1));
            G->ndemoted--;
        }
        guard_demote_t *dm = &G->dem[G->ndemoted++];
        dm->t_ns = now_ns(); dm->pid = c[i].pid; dm->tid = c[i].tid;
        snprintf(dm->name, sizeof dm->name, "%s", c[i].name);
        dm->old_prio = c[i].prio; dm->new_prio = GUARD_DEMOTE_TO;
        G->demotions++;
        pub_end();
    }
}

/* ---------------- main ---------------- */

int main(int argc, char **argv)
{
    int grace_ms = GUARD_GRACE_MS, min_prio = GUARD_DEMOTE_MIN_PRIO, opt;
    while ((opt = getopt(argc, argv, "g:p:v")) != -1) {
        switch (opt) {
        case 'g': grace_ms = atoi(optarg); break;
        case 'p': min_prio = atoi(optarg); break;
        case 'v': g_verbose = 1; break;
        default:
            fprintf(stderr, "usage: %s [-g grace_ms] [-p min_prio] [-v]\n", argv[0]);
            return 1;
        }
    }
    if (grace_ms < 0) grace_ms = 0;
    g_t0 = now_ns();

    int sfd = shm_open(SHM_GUARD_NAME, O_RDWR | O_CREAT, 0666);
    if (sfd < 0) { perror("guardian: shm_open"); return 1; }
    if (ftruncate(sfd, sizeof(guard_stats_t)) != 0) { perror("guardian: ftruncate"); return 1; }
    void *m = mmap(NULL, sizeof(guard_stats_t), PROT_READ | PROT_WRITE, MAP_SHARED, sfd, 0);
    if (m == MAP_FAILED) { perror("guardian: mmap"); return 1; }
    G = (guard_stats_t *)m;
    memset((void *)G, 0, sizeof *G);

    int ncpu = _syspage_ptr->num_cpu;
    if (ncpu > GUARD_MAX_CPU) ncpu = GUARD_MAX_CPU;
    G->ncpu = ncpu;
    G->grace_ms = grace_ms;
    G->demote_min_prio = min_prio;
    for (int i = 0; i < GUARD_MAX_CPU; i++) G->cpu_pct_x10[i] = -1;

    if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0)
        fprintf(stderr, "guardian: warning: mlockall failed\n");
    struct sched_param sp;
    memset(&sp, 0, sizeof sp);
    sp.sched_priority = PRIO_GUARD;
    if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp) != 0)
        fprintf(stderr, "guardian: warning: could not set priority %d (run as root)\n", PRIO_GUARD);

    LOG("guardian: %d cores, grace %d ms, lowers threads with priority >= %d to %d, starve limit %lld ms",
        ncpu, grace_ms, min_prio, GUARD_DEMOTE_TO, (long long)(GUARD_STARVE_NS / MS));

    volatile ctl_stats_t *cs = NULL;
    int act = -1;
    uint64_t last_cycles = 0;
    int armed = 0;                       /* ctl_loop has produced cycles at least once */
    int64_t last_adv = now_ns(), ep_start = 0, forced_t = 0, resumed_t = 0, next_try = 0;
    int forced = 0, was_starved = 0;
    uint64_t forced_safe_act = 0;
    static cand_t cand[MAX_CAND];

    scan(ncpu, min_prio, cand);          /* first scan only sets the CPU-time baseline */
    int64_t next = now_ns(), next_scan = next + 250 * MS;

    for (;;) {
        next += GUARD_TICK_NS;
        struct timespec ts = { (time_t)(next / 1000000000LL), (long)(next % 1000000000LL) };
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, NULL);
        int64_t now = now_ns();
        if (now - next > 100 * MS) next = now;            /* we were held up: do not burst */

        if ((!cs || act < 0) && now >= next_try) {
            if (!cs) cs = map_ctl_stats();
            if (act < 0) act = open(ACT_DEV_PATH, O_RDWR);
            next_try = now + 1000 * MS;
        }

        ctl_stats_t s;
        int have = cs && seq_copy(cs, &s, sizeof s) == 0;
        int health = GH_NO_LOOP, starved = 0;
        if (have) {
            if (s.cycles != last_cycles) {
                if (last_cycles != 0 || s.cycles > 1) armed = 1;
                last_cycles = s.cycles;
                last_adv = now;
            }
            starved = armed && now - last_adv > GUARD_STARVE_NS;
            health = starved ? GH_STARVED : (s.state != ST_NORMAL ? GH_DISTURBED : GH_OK);

            /* 1. starved: put the actuator in SAFE now - ctl_loop cannot do it while it cannot run */
            if (starved && !forced && act >= 0) {
                if (devctl(act, DCMD_ACT_SAFE, NULL, 0, NULL) == EOK) {
                    forced = 1; forced_t = now; forced_safe_act = s.safe_activations;
                    pub_begin(); G->forced_safe++; pub_end();
                    LOG("ctl_loop starved (no cycle for %lld ms) -> actuator SAFE",
                        (long long)((now - last_adv) / MS));
                } else {
                    close(act); act = -1;                    /* server restarted: reopen */
                }
            }
            if (was_starved && !starved) resumed_t = now;
            was_starved = starved;
            /* ctl_loop is back: if it went SAFE itself it owns the RUN; if not, release it */
            if (forced && !starved) {
                if (s.safe_activations != forced_safe_act || s.state != ST_NORMAL) {
                    forced = 0;
                    LOG("ctl_loop running again (%s) - it handles the recovery",
                        s.state == ST_SAFE ? "SAFE" : s.state == ST_RECOVERY ? "RECOVERY" : "NORMAL");
                } else if (now - resumed_t > 200 * MS) {
                    if (act >= 0) devctl(act, DCMD_ACT_RUN, NULL, 0, NULL);
                    forced = 0;
                    LOG("false alarm: ctl_loop healthy without an overrun -> actuator RUN");
                }
            }
            (void)forced_t;
        }

        /* 2. the grace timer */
        int disturbed = health == GH_STARVED || health == GH_DISTURBED;
        if (disturbed && ep_start == 0) {
            ep_start = now;
            LOG("disturbance started (%s)", health == GH_STARVED ? "loop starved" : "loop not NORMAL");
        }
        if (!disturbed && ep_start) {
            LOG("disturbance over after %.0f ms", (now - ep_start) / 1e6);
            ep_start = 0;
        }
        int64_t episode = ep_start ? now - ep_start : 0;

        /* 3. scan: 4x a second, 10x a second while disturbed */
        if (now >= next_scan) {
            int n = scan(ncpu, min_prio, cand);
            if (disturbed && episode >= (int64_t)grace_ms * MS && n > 0)
                demote(cand, n);
            next_scan = now + (disturbed ? 100 : 250) * MS;
        }

        pub_begin();
        G->now_ns = now;
        G->health = health;
        G->episode_ns = episode;
        pub_end();
    }
}
