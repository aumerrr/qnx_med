/* monitor - live dashboard for the surgical RT controller.  Owner: C.
 *
 * Reads  /ctl_stats (ctl_stats_t, from ctl_loop)  and  /pot (pot_sample_t, from adc_reader),
 * asks   /dev/actuator for DCMD_ACT_STATS once a second,
 * reads  /guard_stats (guard_stats_t, from guardian): CPU use per core, where our threads run,
 *        which load threads the guardian lowered,
 * writes only ctl_stats_t.cmd_flags (reset), starts and stops the test loads,
 * and logs one CSV line per second.  Never touches the RT path otherwise.
 *
 * usage: monitor [-p] [-f] [-l file.csv] [-b bindir] [-m rtmask] [-T seconds]
 *   -p  plain output (one line per second)  - automatic when stdout is not a terminal (Momentics)
 *   -f  force the full-screen view
 *   -l  start CSV logging to this file at once (key c toggles; default monitor_log.csv)
 *   -b  folder holding cpu_hog / msg_flood (default: monitor's own folder)
 *   -m  run mask of the real-time cores, used to put loads l and m there with
 *       "on -R <mask>" (default 0x6 = cores 1-2; 0 = start loads without "on")
 *   -T  load timer: every load stops by itself after this many seconds (default 20)
 * Keys: l CPU load on the real-time cores, m message flood on /dev/actuator,
 *       b contained load (priority 90, core 3 only), k stop load, r reset, c CSV, q quit.
 * In the Momentics console type the key and press Enter.
 * Run it on CPU0, normal priority:   on -C 0 ./monitor   (see scripts/run_all.sh)
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <signal.h>
#include <termios.h>
#include <time.h>
#include <spawn.h>
#include <pthread.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/select.h>
#include <sys/wait.h>
#include "config.h"
#include "ipc_defs.h"
#include "monitor.h"

#define TICK_NS    200000000LL       /* refresh 5 times a second */
#define SEC_NS     1000000000LL
#define MAX_LOAD   4

extern char **environ;

/* The test loads. "SECS" is replaced by the load timer (-T), so every load also stops by
 * itself even if the dashboard dies.
 *   l  2 hog threads at priority 65 on the real-time cores: above ctl_loop (60) and
 *      adc_reader (55), below the servo PWM (70) -> forces an overrun; the guardian lowers them
 *   m  2 threads flooding /dev/actuator with requests at priority 65 on the real-time cores:
 *      the resource manager serves them at 65 -> same effect through message passing
 *   b  1 hog at priority 90 (above everything of ours) pinned to core 3: contained, no effect */
enum { LK_NONE = 0, LK_CPU, LK_MSG, LK_BG };
typedef struct { const char *label; int on_rt; const char *argv[8]; } load_def_t;
static const load_def_t LOADS[] = {
    [LK_CPU] = { "CPU hog x2 @65 on RT cores",        1, { "cpu_hog", "2", "65", "SECS", "-1", "rr", NULL } },
    [LK_MSG] = { "msg flood x2 @65 -> /dev/actuator", 1, { "msg_flood", "actuator", "0", "65", "SECS", "2", NULL } },
    [LK_BG]  = { "CPU hog @90 on core 3 (contained)", 0, { "cpu_hog", "1", "90", "SECS", "3", "fifo", NULL } },
};

static volatile sig_atomic_t g_quit;
static mon_view_t  v;
static volatile ctl_stats_t  *g_st;       /* mapped /ctl_stats */
static volatile pot_sample_t *g_pot;      /* mapped /pot       */
static int    g_nload;                    /* 1 while a load is running   */
static int    g_load_kind;
static int64_t g_load_until;
static int    g_load_secs = 20;
static char   g_rtmask[24] = "0x6";
static char   g_bindir[256] = ".";
static volatile guard_stats_t *g_gs;      /* mapped /guard_stats */
static FILE  *g_csv;
static int    g_tty_raw;
static struct termios g_tio_old;
static int64_t g_t0;

static void on_sig(int s) { (void)s; g_quit = 1; }

static int64_t now_ns(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return (int64_t)t.tv_sec * SEC_NS + t.tv_nsec;
}

static double up_s(void) { return (now_ns() - g_t0) / 1e9; }

/* ---------- events and messages ---------- */

static void event(const char *fmt, const char *a, const char *b)
{
    if (v.n_ev == MAX_EVENTS) {
        memmove(&v.ev[0], &v.ev[1], sizeof(v.ev[0]) * (MAX_EVENTS - 1));
        v.n_ev--;
    }
    mon_event_t *e = &v.ev[v.n_ev++];
    e->t_s = up_s();
    snprintf(e->text, sizeof e->text, fmt, a, b);
    char line[128];
    snprintf(line, sizeof line, "EVENT %.1f s: %s", e->t_s, e->text);
    ui_note(line);
}

static void say(const char *text)
{
    snprintf(v.msg, sizeof v.msg, "%s", text);
    ui_note(text);
}

/* ---------- shared memory ---------- */

/* consistent copy of a seqlocked struct whose first field is the uint64 seq */
static int seq_copy(const volatile void *src, void *dst, size_t n)
{
    const volatile uint64_t *seq = (const volatile uint64_t *)src;
    for (int i = 0; i < 1000; i++) {
        uint64_t a = *seq;
        if (a & 1) continue;                    /* writer busy */
        __sync_synchronize();
        memcpy(dst, (const void *)src, n);
        __sync_synchronize();
        if (*seq == a) return 0;
    }
    return -1;
}

static void *map_shm(const char *name, size_t size, int want_write, int *writable)
{
    int fd = -1, prot = PROT_READ;
    *writable = 0;
    if (want_write) {
        fd = shm_open(name, O_RDWR, 0);
        if (fd >= 0) { prot |= PROT_WRITE; *writable = 1; }
    }
    if (fd < 0) fd = shm_open(name, O_RDONLY, 0);
    if (fd < 0) return NULL;
    struct stat st;
    if (fstat(fd, &st) != 0 || (size_t)st.st_size < size) { close(fd); return NULL; }
    void *p = mmap(NULL, size, prot, MAP_SHARED, fd, 0);
    close(fd);
    return p == MAP_FAILED ? NULL : p;
}

static void send_cmd(uint32_t flag, const char *what)
{
    if (!g_st || !v.cmd_ok) { say("no writable /ctl_stats - is ctl_loop running?"); return; }
    __atomic_fetch_or(&g_st->cmd_flags, flag, __ATOMIC_SEQ_CST);
    say(what);
}

/* ---------- actuator ---------- */

/* Asked from a separate thread: if actuator_rm is busy or starved by a load, only this thread
 * waits - the dashboard keeps drawing and shows "not answering". */
static pthread_mutex_t g_act_lock = PTHREAD_MUTEX_INITIALIZER;
static act_stats_t     g_act_last;
static int64_t         g_act_t;                  /* time of the last good answer, 0 = never */

static void *act_thread(void *arg)
{
    (void)arg;
    int fd = -1;
    for (;;) {
        if (fd < 0) fd = open(ACT_DEV_PATH, O_RDWR);
        act_stats_t a;
        if (fd >= 0 && devctl(fd, DCMD_ACT_STATS, &a, sizeof a, NULL) == EOK) {
            pthread_mutex_lock(&g_act_lock);
            g_act_last = a;
            g_act_t = now_ns();
            pthread_mutex_unlock(&g_act_lock);
        } else if (fd >= 0) {
            close(fd);                           /* server restarted or gone: reopen later */
            fd = -1;
        }
        sleep(1);
    }
    return NULL;
}

static void poll_actuator(void)
{
    pthread_mutex_lock(&g_act_lock);
    int64_t t = g_act_t;
    v.act = g_act_last;
    pthread_mutex_unlock(&g_act_lock);
    v.act_age_s = t ? (now_ns() - t) / 1e9 : -1;
    v.have_act = t && v.act_age_s < 2.5;
}

/* custom cluster names per core, read once from "pidin syspage=cluster" */
static void read_clusters(void)
{
    FILE *f = popen("pidin syspage=cluster 2>/dev/null", "r");
    if (!f) return;
    char line[256];
    while (fgets(line, sizeof line, f)) {
        char *cm = strstr(line, "cpumask:"), *nm = strstr(line, "name:");
        if (!cm || !nm) continue;
        unsigned long long mask = strtoull(cm + 8, NULL, 16);
        char *a = strchr(nm, '('), *b = a ? strchr(a, ')') : NULL;
        if (!a || !b || a[1] == '_') continue;   /* names starting with _ are the built-in ones */
        *b = '\0';
        for (int c = 0; c < 8; c++) {
            if (!(mask & (1ULL << c))) continue;
            size_t l = strlen(v.core_cl[c]);
            snprintf(v.core_cl[c] + l, sizeof v.core_cl[c] - l, "%s%s", l ? "," : "", a + 1);
        }
    }
    pclose(f);
}

/* ---------- load programs ---------- */

/* start a program quietly (no terminal output); "on" and "slay" are found through PATH */
static int spawn_quiet(char *const argv[], pid_t *pid)
{
    posix_spawn_file_actions_t fa;
    posix_spawn_file_actions_init(&fa);
    posix_spawn_file_actions_addopen(&fa, 0, "/dev/null", O_RDONLY, 0);
    posix_spawn_file_actions_addopen(&fa, 1, "/dev/null", O_WRONLY, 0);
    posix_spawn_file_actions_addopen(&fa, 2, "/dev/null", O_WRONLY, 0);
    int r = posix_spawnp(pid, argv[0], &fa, NULL, argv, environ);
    posix_spawn_file_actions_destroy(&fa);
    return r;
}

static void load_on(int kind)
{
    if (g_nload) { say("a load is already running - press k first"); return; }
    const load_def_t *L = &LOADS[kind];
    char path[512], secs[16];
    snprintf(path, sizeof path, "%s/%s", g_bindir, L->argv[0]);
    snprintf(secs, sizeof secs, "%d", g_load_secs);

    char *argv[16];
    int n = 0;
    int use_on = L->on_rt && strcmp(g_rtmask, "0") != 0;
    int guard_ok = v.have_guard && !v.guard_stale;
    if (use_on) { argv[n++] = "on"; argv[n++] = "-R"; argv[n++] = g_rtmask; }
    argv[n++] = path;
    for (int i = 1; L->argv[i] && n < 15; i++)
        argv[n++] = (char *)(strcmp(L->argv[i], "SECS") == 0 ? secs : L->argv[i]);
    argv[n] = NULL;

    pid_t pid;
    int r = spawn_quiet(argv, &pid);
    if (r != 0) {
        char e[600];
        snprintf(e, sizeof e, "cannot start %s: %s", use_on ? "on" : path, strerror(r));
        say(e);
        return;
    }
    g_nload = 1;
    g_load_kind = kind;
    g_load_until = now_ns() + (int64_t)g_load_secs * SEC_NS;
    char m[160];
    snprintf(m, sizeof m, "load started: %s for %d s%s%s", L->label, g_load_secs,
             use_on ? "" : (L->on_rt ? " (no RT mask: all cores)" : ""),
             L->on_rt && !guard_ok ? "  WARNING: guardian not running - no recovery until the timer ends" : "");
    say(m);
    event("LOAD ON: %s%s", L->label, "");
}

/* stop every load program by name (the "on" wrapper may already have exited, so pids are not used) */
static void load_kill_all(void)
{
    char *argv[] = { "slay", "-f", "cpu_hog", "msg_flood", NULL };
    pid_t pid;
    if (spawn_quiet(argv, &pid) == 0) waitpid(pid, NULL, 0);
}

static void load_off(void)
{
    if (!g_nload) { say("no load running"); load_kill_all(); return; }
    load_kill_all();
    g_nload = 0;
    g_load_kind = LK_NONE;
    say("load stopped");
    event("LOAD OFF (key k)%s%s", "", "");
}

static void load_timer(int64_t now)
{
    if (g_nload && now >= g_load_until) {
        load_kill_all();                         /* normally already gone by itself */
        g_nload = 0;
        g_load_kind = LK_NONE;
        { char t[16]; snprintf(t, sizeof t, "%d", g_load_secs); event("LOAD OFF (timer %s s)%s", t, ""); }
    }
}

static void reap_children(void)
{
    while (waitpid(-1, NULL, WNOHANG) > 0) ;
}

/* ---------- CSV ---------- */

static void csv_toggle(void)
{
    if (g_csv) { fclose(g_csv); g_csv = NULL; v.logging = 0; say("CSV logging off"); return; }
    g_csv = fopen(v.log_path, "a");
    if (!g_csv) { char e[320]; snprintf(e, sizeof e, "cannot open %s: %s", v.log_path, strerror(errno)); say(e); return; }
    fseek(g_csv, 0, SEEK_END);
    if (ftell(g_csv) == 0)
        fprintf(g_csv, "time_s,state,cycles,wake_last_us,wake_min_us,wake_avg_us,wake_max_us,"
                       "jitter_std_us,work_max_us,lat_avg_us,lat_max_us,misses,safe_activations,"
                       "recovery,pot_permille,servo_us,load,guard_health,demotions,"
                       "cpu0_pct,cpu1_pct,cpu2_pct,cpu3_pct\n");
    v.logging = 1;
    char m[320]; snprintf(m, sizeof m, "CSV logging to %s", v.log_path); say(m);
}

static void csv_line(void)
{
    if (!g_csv || !v.have_stats) return;
    const ctl_stats_t *s = &v.s;
    static const char *LK[] = { "none", "cpu", "msg", "contained" };
    double c[4];
    for (int i = 0; i < 4; i++)
        c[i] = v.have_guard && i < v.g.ncpu && v.g.cpu_pct_x10[i] >= 0 ? v.g.cpu_pct_x10[i] / 10.0 : -1;
    fprintf(g_csv, "%.1f,%s,%llu,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%llu,%llu,%u,%d,%d,%s,%d,%u,"
                   "%.1f,%.1f,%.1f,%.1f\n",
            v.uptime_s, ui_state_name(s->state), (unsigned long long)s->cycles,
            s->wake_late_last_ns / 1e3, s->wake_late_min_ns / 1e3, s->wake_late_avg_ns / 1e3,
            s->wake_late_max_ns / 1e3, s->wake_late_std_ns / 1e3, s->work_max_ns / 1e3,
            s->latency_avg_ns / 1e3, s->latency_max_ns / 1e3,
            (unsigned long long)s->misses, (unsigned long long)s->safe_activations,
            s->recovery_count, s->pot_permille, s->servo_us, LK[g_nload ? g_load_kind : 0],
            v.have_guard ? v.g.health : -1, v.have_guard ? v.g.demotions : 0u, c[0], c[1], c[2], c[3]);
    fflush(g_csv);
}

/* ---------- keys ---------- */

static void handle_key(int c)
{
    switch (c) {
    case 'l': case 'L': load_on(LK_CPU); break;
    case 'm': case 'M': load_on(LK_MSG); break;
    case 'b': case 'B': load_on(LK_BG); break;
    case 'k': case 'K': load_off(); break;
    case 'r': case 'R': v.n_ev = 0; send_cmd(CMD_RESET, "reset sent (stats cleared, back to NORMAL)");
                        event("RESET%s%s", "", ""); break;
    case 'c': case 'C': csv_toggle(); break;
    case 'q': case 'Q': g_quit = 1; break;
    case '\n': case '\r': case ' ': break;
    default: say("keys: l m b k r c q"); break;
    }
}

static void tty_restore(void)
{
    if (g_tty_raw) tcsetattr(STDIN_FILENO, TCSANOW, &g_tio_old);
    g_tty_raw = 0;
}

/* ---------- main ---------- */

int main(int argc, char **argv)
{
    int plain = !isatty(STDOUT_FILENO);
    int log_now = 0;
    snprintf(v.log_path, sizeof v.log_path, "monitor_log.csv");

    const char *slash = strrchr(argv[0], '/');
    if (slash) snprintf(g_bindir, sizeof g_bindir, "%.*s", (int)(slash - argv[0]), argv[0]);

    int opt;
    while ((opt = getopt(argc, argv, "pfl:b:m:T:h")) != -1) {
        switch (opt) {
        case 'p': plain = 1; break;
        case 'f': plain = 0; break;
        case 'l': snprintf(v.log_path, sizeof v.log_path, "%s", optarg); log_now = 1; break;
        case 'b': snprintf(g_bindir, sizeof g_bindir, "%s", optarg); break;
        case 'm': snprintf(g_rtmask, sizeof g_rtmask, "%s", optarg); break;
        case 'T': g_load_secs = atoi(optarg); if (g_load_secs < 1) g_load_secs = 1; break;
        default:
            fprintf(stderr, "usage: %s [-p plain] [-f full] [-l file.csv] [-b bindir] [-m rtmask] [-T secs]\n", argv[0]);
            return 1;
        }
    }

    signal(SIGINT, on_sig);
    signal(SIGTERM, on_sig);
    signal(SIGPIPE, SIG_IGN);

    if (isatty(STDIN_FILENO) && tcgetattr(STDIN_FILENO, &g_tio_old) == 0) {
        struct termios t = g_tio_old;
        t.c_lflag &= ~(ICANON | ECHO);           /* single keys, no echo */
        t.c_cc[VMIN] = 0;
        t.c_cc[VTIME] = 0;
        if (tcsetattr(STDIN_FILENO, TCSANOW, &t) == 0) g_tty_raw = 1;
    }

    g_t0 = now_ns();
    read_clusters();
    {
        pthread_t at;
        if (pthread_create(&at, NULL, act_thread, NULL) == 0) pthread_detach(at);
    }
    ui_init(plain);
    if (log_now) csv_toggle();
    say("ready");

    int stdin_ok = 1, prev_state = -1;
    uint64_t last_cycles = 0, sec_cycles = 0, prev_safe = 0, prev_miss = 0;
    uint32_t prev_dem = 0, prev_forced = 0;
    int prev_gh = -1;
    int64_t g_last_upd = 0, g_upd_seen = g_t0;
    v.load_secs = g_load_secs;
    int64_t next_tick = g_t0, next_sec = g_t0 + SEC_NS, last_advance = g_t0;

    while (!g_quit) {
        /* wait for a key or the next tick */
        int64_t wait = next_tick - now_ns();
        if (wait < 0) wait = 0;
        struct timeval tv = { (time_t)(wait / SEC_NS), (long)((wait % SEC_NS) / 1000) };
        fd_set rf;
        FD_ZERO(&rf);
        if (stdin_ok) FD_SET(STDIN_FILENO, &rf);
        int n = select(stdin_ok ? STDIN_FILENO + 1 : 0, &rf, NULL, NULL, &tv);
        if (n > 0 && FD_ISSET(STDIN_FILENO, &rf)) {
            char buf[16];
            ssize_t r = read(STDIN_FILENO, buf, sizeof buf);
            if (r == 0) stdin_ok = 0;            /* no input attached (e.g. started with &) */
            for (ssize_t i = 0; i < r; i++) handle_key((unsigned char)buf[i]);
        }

        int64_t now = now_ns();
        if (now < next_tick) continue;
        while (next_tick <= now) next_tick += TICK_NS;

        reap_children();
        load_timer(now);
        v.n_load = g_nload;
        v.load_name = g_nload ? LOADS[g_load_kind].label : "none";
        v.load_left_s = g_nload ? (g_load_until - now) / 1e9 : 0;

        /* /guard_stats */
        int gdummy;
        if (!g_gs) g_gs = map_shm(SHM_GUARD_NAME, sizeof(guard_stats_t), 0, &gdummy);
        v.have_guard = g_gs && seq_copy(g_gs, &v.g, sizeof v.g) == 0;
        if (v.have_guard) {
            if (v.g.now_ns != g_last_upd) { g_last_upd = v.g.now_ns; g_upd_seen = now; }
            v.guard_stale = now - g_upd_seen > SEC_NS;
            if (prev_gh >= 0 && v.g.forced_safe > prev_forced)
                event("GUARD: loop starved -> actuator SAFE%s%s", "", "");
            if (prev_gh >= 0 && v.g.demotions > prev_dem) {
                const guard_demote_t *d = &v.g.dem[v.g.ndemoted - 1];
                char t[48];
                snprintf(t, sizeof t, "%u thread(s), e.g. %s %d->%d", v.g.demotions - prev_dem,
                         d->name, d->old_prio, d->new_prio);
                event("GUARD lowered %s%s", t, "");
            }
            prev_forced = v.g.forced_safe;
            prev_dem = v.g.demotions;
            prev_gh = v.g.health;
            if (now - g_upd_seen > 3 * SEC_NS) {           /* guardian restarted: remap */
                munmap((void *)g_gs, sizeof(guard_stats_t));
                g_gs = NULL;
            }
        }
        v.uptime_s = (now - g_t0) / 1e9;

        /* /ctl_stats */
        if (!g_st) {
            g_st = map_shm(SHM_STATS_NAME, sizeof(ctl_stats_t), 1, &v.cmd_ok);
            last_advance = now;
        }
        v.have_stats = g_st && seq_copy(g_st, &v.s, sizeof v.s) == 0;
        if (v.have_stats) {
            if (v.s.cycles != last_cycles) { last_cycles = v.s.cycles; last_advance = now; }
            v.stale = now - last_advance > SEC_NS;
            if (now - last_advance > 3 * SEC_NS) {          /* writer may have recreated it: remap */
                munmap((void *)g_st, sizeof(ctl_stats_t));
                g_st = NULL;
            }
            /* counters catch short states the 5 Hz sampling could miss */
            if (prev_state >= 0 && v.s.safe_activations > prev_safe) {
                char n[24];
                snprintf(n, sizeof n, "%llu", (unsigned long long)v.s.safe_activations);
                event("OVERRUN -> SAFE (activation #%s)%s", n, "");
            } else if (prev_state >= 0 && v.s.misses > prev_miss) {
                char n[24];
                snprintf(n, sizeof n, "%llu", (unsigned long long)(v.s.misses - prev_miss));
                event("deadline miss x%s%s", n, "");
            }
            prev_safe = v.s.safe_activations;
            prev_miss = v.s.misses;
            if (v.s.state != prev_state) {
                if (prev_state >= 0)
                    event("%s -> %s", ui_state_name(prev_state), ui_state_name(v.s.state));
                prev_state = v.s.state;
            }
        }

        /* /pot */
        int dummy;
        if (!g_pot) g_pot = map_shm(SHM_POT_NAME, sizeof(pot_sample_t), 0, &dummy);
        v.have_pot = g_pot && seq_copy(g_pot, &v.pot, sizeof v.pot) == 0;
        if (v.have_pot) v.pot_age_ms = (now_ns() - v.pot.ts_ns) / 1e6;   /* fresh time: we may have been preempted */

        /* once a second: rate, actuator, CSV, plain line */
        int new_sec = now >= next_sec;
        if (new_sec) {
            while (next_sec <= now) next_sec += SEC_NS;
            v.cycles_per_s = v.have_stats && v.s.cycles >= sec_cycles ? (double)(v.s.cycles - sec_cycles) : 0;
            sec_cycles = v.have_stats ? v.s.cycles : 0;
            poll_actuator();
            csv_line();
        }
        if (!plain || new_sec) ui_draw(&v);
    }

    if (g_nload) load_kill_all();
    if (g_csv) fclose(g_csv);
    tty_restore();
    ui_done();
    return 0;
}
