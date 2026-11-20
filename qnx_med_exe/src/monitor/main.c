/* monitor - live dashboard for the surgical RT controller.  Owner: C.
 *
 * Reads  /ctl_stats (ctl_stats_t, from ctl_loop)  and  /pot (pot_sample_t, from adc_reader),
 * asks   /dev/actuator for DCMD_ACT_STATS once a second,
 * writes only ctl_stats_t.cmd_flags (reset), starts and stops the cpu_hog load,
 * and logs one CSV line per second.  Never touches the RT path otherwise.
 *
 * usage: monitor [-p] [-f] [-l file.csv] [-b bindir]
 *   -p  plain output (one line per second)  - automatic when stdout is not a terminal (Momentics)
 *   -f  force the full-screen view
 *   -l  start CSV logging to this file at once (key c toggles; default monitor_log.csv)
 *   -b  folder holding cpu_hog (default: monitor's own folder)
 * Keys: l load on (cpu_hog), k load off, r reset, c CSV, q quit.
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

/* started by key l: the one load program we use - 4 busy threads, prio 10, round-robin, forever */
static const char *LOAD_CMDS[][8] = {
    { "cpu_hog", "4", "10", "0", "-1", "rr", NULL },
};
#define N_LOAD_CMDS ((int)(sizeof LOAD_CMDS / sizeof LOAD_CMDS[0]))

static volatile sig_atomic_t g_quit;
static mon_view_t  v;
static volatile ctl_stats_t  *g_st;       /* mapped /ctl_stats */
static volatile pot_sample_t *g_pot;      /* mapped /pot       */
static int    g_actfd = -1;
static pid_t  g_load[MAX_LOAD];
static int    g_nload;
static char   g_bindir[256] = ".";
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

static void poll_actuator(void)
{
    v.have_act = 0;
    if (g_actfd < 0) g_actfd = open(ACT_DEV_PATH, O_RDWR);
    if (g_actfd < 0) return;
    if (devctl(g_actfd, DCMD_ACT_STATS, &v.act, sizeof v.act, NULL) == EOK) {
        v.have_act = 1;
    } else {
        close(g_actfd);                          /* server restarted or gone: reopen later */
        g_actfd = -1;
    }
}

/* ---------- load programs ---------- */

static int spawn_quiet(const char *const *cmd, pid_t *pid)
{
    char path[512];
    snprintf(path, sizeof path, "%s/%s", g_bindir, cmd[0]);
    char *argv[10];
    int n = 0;
    argv[n++] = path;
    for (int i = 1; cmd[i] && n < 9; i++) argv[n++] = (char *)cmd[i];
    argv[n] = NULL;
    posix_spawn_file_actions_t fa;
    posix_spawn_file_actions_init(&fa);
    posix_spawn_file_actions_addopen(&fa, 0, "/dev/null", O_RDONLY, 0);
    posix_spawn_file_actions_addopen(&fa, 1, "/dev/null", O_WRONLY, 0);
    posix_spawn_file_actions_addopen(&fa, 2, "/dev/null", O_WRONLY, 0);
    int r = posix_spawn(pid, path, &fa, NULL, argv, environ);
    posix_spawn_file_actions_destroy(&fa);
    return r;
}

static void load_on(void)
{
    if (g_nload) { say("load already running"); return; }
    for (int i = 0; i < N_LOAD_CMDS && g_nload < MAX_LOAD; i++) {
        pid_t pid;
        int r = spawn_quiet(LOAD_CMDS[i], &pid);
        if (r == 0) {
            g_load[g_nload++] = pid;
        } else {
            char e[320];
            snprintf(e, sizeof e, "cannot start %s/%s: %s", g_bindir, LOAD_CMDS[i][0], strerror(r));
            say(e);
        }
    }
    if (g_nload) {
        char m[64];
        snprintf(m, sizeof m, "load started: %d processes", g_nload);
        say(m);
        event("LOAD ON (cpu_hog)%s%s", "", "");
    }
}

static void load_off(void)
{
    if (!g_nload) { say("no load running"); return; }
    for (int i = 0; i < g_nload; i++) kill(g_load[i], SIGTERM);
    for (int i = 0; i < g_nload; i++) waitpid(g_load[i], NULL, 0);
    g_nload = 0;
    say("load stopped");
    event("LOAD OFF%s%s", "", "");
}

static void reap_children(void)
{
    pid_t p;
    while ((p = waitpid(-1, NULL, WNOHANG)) > 0)
        for (int i = 0; i < g_nload; i++)
            if (g_load[i] == p) { g_load[i] = g_load[--g_nload]; break; }
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
                       "recovery,pot_permille,servo_us,load_on\n");
    v.logging = 1;
    char m[320]; snprintf(m, sizeof m, "CSV logging to %s", v.log_path); say(m);
}

static void csv_line(void)
{
    if (!g_csv || !v.have_stats) return;
    const ctl_stats_t *s = &v.s;
    fprintf(g_csv, "%.1f,%s,%llu,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%llu,%llu,%u,%d,%d,%d\n",
            v.uptime_s, ui_state_name(s->state), (unsigned long long)s->cycles,
            s->wake_late_last_ns / 1e3, s->wake_late_min_ns / 1e3, s->wake_late_avg_ns / 1e3,
            s->wake_late_max_ns / 1e3, s->wake_late_std_ns / 1e3, s->work_max_ns / 1e3,
            s->latency_avg_ns / 1e3, s->latency_max_ns / 1e3,
            (unsigned long long)s->misses, (unsigned long long)s->safe_activations,
            s->recovery_count, s->pot_permille, s->servo_us, g_nload > 0);
    fflush(g_csv);
}

/* ---------- keys ---------- */

static void handle_key(int c)
{
    switch (c) {
    case 'l': case 'L': load_on(); break;
    case 'k': case 'K': load_off(); break;
    case 'r': case 'R': v.n_ev = 0; send_cmd(CMD_RESET, "reset sent (stats cleared, back to NORMAL)");
                        event("RESET%s%s", "", ""); break;
    case 'c': case 'C': csv_toggle(); break;
    case 'q': case 'Q': g_quit = 1; break;
    case '\n': case '\r': case ' ': break;
    default: say("keys: l k r c q"); break;
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
    while ((opt = getopt(argc, argv, "pfl:b:h")) != -1) {
        switch (opt) {
        case 'p': plain = 1; break;
        case 'f': plain = 0; break;
        case 'l': snprintf(v.log_path, sizeof v.log_path, "%s", optarg); log_now = 1; break;
        case 'b': snprintf(g_bindir, sizeof g_bindir, "%s", optarg); break;
        default:
            fprintf(stderr, "usage: %s [-p plain] [-f full] [-l file.csv] [-b bindir]\n", argv[0]);
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
    ui_init(plain);
    if (log_now) csv_toggle();
    say("ready");

    int stdin_ok = 1, prev_state = -1;
    uint64_t last_cycles = 0, sec_cycles = 0, prev_safe = 0, prev_miss = 0;
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
        v.n_load = g_nload;
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

    if (g_nload) load_off();
    if (g_csv) fclose(g_csv);
    if (g_actfd >= 0) close(g_actfd);
    tty_restore();
    ui_done();
    return 0;
}
