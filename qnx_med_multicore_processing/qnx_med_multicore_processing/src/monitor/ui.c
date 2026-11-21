/* ui.c - draws the dashboard. Full mode: ANSI screen redrawn in place (ssh / terminal).
 * Plain mode: one status line per second (Momentics console, logs). Owner: C. */
#include <stdio.h>
#include <string.h>
#include "config.h"
#include "monitor.h"

static int g_plain;

#define RULE "-------------------------------------------------------------------------------"
#define L(...) do { printf(__VA_ARGS__); fputs("\033[K\n", stdout); } while (0)

const char *ui_state_name(int s)
{
    switch (s) {
    case ST_NORMAL:   return "NORMAL";
    case ST_SAFE:     return "SAFE";
    case ST_RECOVERY: return "RECOVERY";
    }
    return "?";
}

static const char *state_color(int s)
{
    switch (s) {
    case ST_NORMAL:   return "\033[1;42;30m";   /* black on green  */
    case ST_SAFE:     return "\033[1;41;37m";   /* white on red    */
    case ST_RECOVERY: return "\033[1;43;30m";   /* black on yellow */
    }
    return "\033[1;45;37m";
}

static double us(int64_t ns) { return ns / 1000.0; }

static const char *health_name(int h)
{
    switch (h) {
    case GH_OK:        return "OK";
    case GH_DISTURBED: return "DISTURBED";
    case GH_STARVED:   return "LOOP STARVED";
    case GH_NO_LOOP:   return "NO LOOP";
    }
    return "?";
}

static const char *role_name(int r)
{
    switch (r) {
    case GR_CRIT:    return "\033[1;32mCRITICAL\033[0m";
    case GR_AUX:     return "support ";
    case GR_LOAD:    return "\033[1;33mLOAD    \033[0m";
    case GR_FOREIGN: return "\033[1;33mFOREIGN \033[0m";
    case GR_DEMOTED: return "\033[1;36mDEMOTED \033[0m";
    }
    return "?       ";
}

/* "0x6 (cores 1-2)"-style text for a run mask */
static const char *mask_text(char *out, size_t n, uint32_t m)
{
    if (m == 0) { snprintf(out, n, "?"); return out; }
    char cores[32] = "";
    int first = 1;
    for (int c = 0; c < 8; c++)
        if (m & (1u << c)) {
            size_t l = strlen(cores);
            snprintf(cores + l, sizeof cores - l, "%s%d", first ? "" : ",", c);
            first = 0;
        }
    snprintf(out, n, "0x%x[%s]", m, cores);
    return out;
}

static char policy_ch(int p)
{
    return p == 1 ? 'f' : p == 2 ? 'r' : p == 4 ? 's' : 'o';   /* SCHED_FIFO 1, RR 2, SPORADIC 4 */
}

static const char *bar(char *out, int width, double frac)
{
    if (frac < 0) frac = 0;
    if (frac > 1) frac = 1;
    int n = (int)(frac * width + 0.5);
    out[0] = '[';
    for (int i = 0; i < width; i++) out[1 + i] = i < n ? '#' : '.';
    out[width + 1] = ']';
    out[width + 2] = '\0';
    return out;
}

void ui_init(int plain)
{
    g_plain = plain;
    if (!plain) fputs("\033[?25l\033[H\033[2J", stdout);   /* hide cursor, clear */
    fflush(stdout);
}

void ui_done(void)
{
    if (!g_plain) fputs("\033[0m\033[?25h\n", stdout);
    fflush(stdout);
}

void ui_note(const char *text)
{
    if (g_plain) { printf("  >> %s\n", text); fflush(stdout); }
}

static void draw_plain(const mon_view_t *v)
{
    const ctl_stats_t *s = &v->s;
    if (!v->have_stats) {
        printf("[%7.1fs] WAITING for /ctl_stats (start ctl_loop or fake_stats)\n", v->uptime_s);
        fflush(stdout);
        return;
    }
    char cpu[64] = "";
    if (v->have_guard)
        for (int i = 0, l = 0; i < v->g.ncpu && i < 4; i++)
            l += snprintf(cpu + l, sizeof cpu - (size_t)l, "%s%d", i ? "/" : " cpu%=",
                          v->g.cpu_pct_x10[i] < 0 ? -1 : v->g.cpu_pct_x10[i] / 10);
    printf("[%7.1fs] %-8s%s cyc=%llu wake(us) last/avg/max/jit=%.1f/%.1f/%.1f/%.1f "
           "lat(us) avg/max=%.1f/%.1f miss=%llu safe=%llu rec=%u/%u pot=%d servo=%d load=%s%s%s\n",
           v->uptime_s, ui_state_name(s->state), v->stale ? "(STALE)" : "",
           (unsigned long long)s->cycles,
           us(s->wake_late_last_ns), us(s->wake_late_avg_ns), us(s->wake_late_max_ns),
           us(s->wake_late_std_ns), us(s->latency_avg_ns), us(s->latency_max_ns),
           (unsigned long long)s->misses, (unsigned long long)s->safe_activations,
           s->recovery_count, s->recovery_target, s->pot_permille, s->servo_us,
           v->n_load ? v->load_name : "off", cpu,
           v->have_guard ? (v->g.health == GH_OK ? " guard=OK" : " guard=DISTURBED") : " guard=none");
    fflush(stdout);
}

void ui_draw(const mon_view_t *v)
{
    if (g_plain) { draw_plain(v); return; }

    const ctl_stats_t *s = &v->s;
    char b[64];

    printf("\033[H");
    L(" \033[1mQNX SURGICAL RT CONTROLLER  -  live monitor\033[0m               uptime %8.1f s", v->uptime_s);
    L(" %s", RULE);

    if (v->n_load)
        L(" SYSTEM     \033[1;43;30m UNDER LOAD \033[0m %s - stops in %4.1f s", v->load_name, v->load_left_s);
    else if (v->have_guard && v->g.busy_foreign > 0)
        L(" SYSTEM     \033[1;43;30m UNDER LOAD \033[0m %d outside high-priority thread(s) on the RT cores",
          v->g.busy_foreign);
    else
        L(" SYSTEM     \033[1;42;30m NORMAL     \033[0m no load running");

    if (!v->have_stats) {
        L(" STATE      \033[1;45;37m WAITING  \033[0m  /ctl_stats not found: start ctl_loop (or fake_stats)");
        L(" RECOVERY   -");
    } else {
        L(" STATE      %s %-9s\033[0m  cycles %-10llu (%5.0f /s)%s",
          state_color(s->state), ui_state_name(s->state),
          (unsigned long long)s->cycles, v->cycles_per_s,
          v->stale ? "   \033[1;31mSTALE: loop not updating\033[0m" : "");
        double f = s->recovery_target ? (double)s->recovery_count / s->recovery_target : 0;
        L(" RECOVERY   %s %u/%u compliant cycles", bar(b, 30, f),
          s->recovery_count, s->recovery_target);
    }

    L(" %s", RULE);
    L(" \033[1mTIMING (us)            last       min       avg       max   jitter(std)\033[0m");
    L(" wake lateness     %9.1f %9.1f %9.1f %9.1f %9.1f",
      us(s->wake_late_last_ns), us(s->wake_late_min_ns), us(s->wake_late_avg_ns),
      us(s->wake_late_max_ns), us(s->wake_late_std_ns));
    L(" work time         %9.1f %9s %9s %9.1f", us(s->work_last_ns), "", "", us(s->work_max_ns));
    L(" pot->cmd latency  %9.1f %9s %9.1f %9.1f", us(s->latency_last_ns), "",
      us(s->latency_avg_ns), us(s->latency_max_ns));
    L(" limits: wake-late %d us, deadline %.1f ms, period %.1f ms",
      WAKE_LATE_LIMIT_US, DEADLINE_NS / 1e6, LOOP_PERIOD_NS / 1e6);
    L(" deadline misses %s%-8llu\033[0m  safe-state activations %-6llu",
      s->misses ? "\033[1;31m" : "\033[1;32m",
      (unsigned long long)s->misses, (unsigned long long)s->safe_activations);

    L(" %s", RULE);
    L(" POT      %4d permille  %s", s->pot_permille, bar(b, 30, s->pot_permille / 1000.0));
    if (v->have_pot)
        L("          raw %6d   sample age %6.2f ms   %s", v->pot.raw, v->pot_age_ms,
          v->pot.status == POT_OK ? "ok" : "\033[1;31mI2C ERROR\033[0m");
    else
        L("          /pot not found (adc_reader / fake_pot not running)");
    L(" SERVO    %4d us        %s", s->servo_us,
      bar(b, 30, (double)(s->servo_us - P_MIN_US) / (P_MAX_US - P_MIN_US)));
    if (v->have_act)
        L(" ACTUATOR %s%-4s\033[0m  last %4u us  writes %-8u rejects %-4u ignored %-6u safe %u",
          v->act.state == ACT_STATE_SAFE ? "\033[1;31m" : "\033[1;32m",
          v->act.state == ACT_STATE_SAFE ? "SAFE" : "RUN",
          v->act.last_us, v->act.writes, v->act.rejects, v->act.ignored, v->act.safe_count);
    else if (v->act_age_s >= 0)
        L(" ACTUATOR \033[1;31mnot answering for %.1f s\033[0m (resource manager busy or starved)", v->act_age_s);
    else
        L(" ACTUATOR /dev/actuator not running");

    L(" %s", RULE);
    if (!v->have_guard) {
        L(" GUARD    \033[1;31mnot running\033[0m - no recovery manager, no core/thread view (start guardian)");
    } else {
        const guard_stats_t *g = &v->g;
        L(" GUARD    %s%-12s\033[0m %5.2f s of %.1f s timer  forced-safe %u  lowered %u%s",
          g->health == GH_OK ? "\033[1;32m" : "\033[1;31m", health_name(g->health),
          g->episode_ns / 1e9, g->grace_ms / 1000.0, g->forced_safe, g->demotions,
          v->guard_stale ? "  \033[1;31m(STALE)\033[0m" : "");
        /* one line per core: cluster, busy %, and the threads of ours / loads using it most */
        L(" \033[1mCORE CLUSTER      BUSY                TOP THREADS ON THIS CORE (%%CPU)\033[0m");
        for (int c = 0; c < g->ncpu && c < 8; c++) {
            char bb[16], line[200], tops[120] = "";
            int used[GUARD_MAX_THREADS] = { 0 }, tl = 0;
            for (int k = 0; k < 3; k++) {                 /* pick the 3 busiest threads on core c */
                int best = -1;
                for (int i = 0; i < g->nthreads; i++)
                    if (!used[i] && g->th[i].last_cpu == c && g->th[i].cpu_pct_x10 >= 1 &&
                        (best < 0 || g->th[i].cpu_pct_x10 > g->th[best].cpu_pct_x10)) best = i;
                if (best < 0) break;
                used[best] = 1;
                tl += snprintf(tops + tl, sizeof tops - (size_t)tl, "%s/%d %.1f  ",
                               g->th[best].name, g->th[best].tid, g->th[best].cpu_pct_x10 / 10.0);
            }
            int pct = g->cpu_pct_x10[c];
            char ptxt[16];
            if (pct < 0) snprintf(ptxt, sizeof ptxt, " --%%");
            else         snprintf(ptxt, sizeof ptxt, "%3d%%", (pct + 5) / 10);
            snprintf(line, sizeof line, " cpu%d %-11.11s %s %s  %s", c,
                     v->core_cl[c][0] ? v->core_cl[c] : "-",
                     bar(bb, 10, pct < 0 ? 0 : pct / 1000.0), ptxt,
                     tops[0] ? tops : "(idle / system only)");
            L("%s", line);
        }
        L(" \033[1m%-12s %5s %-11s %6s %-14s %4s %5s %s\033[0m",
          "PROCESS", "TID", "THREAD", "PRIO", "CORES ALLOWED", "LAST", "%CPU", "ROLE");
        int shown = 0;
        for (int i = 0; i < g->nthreads && shown < 16; i++, shown++) {
            const guard_thread_t *t = &g->th[i];
            char mk[24], pr[16];
            if (t->demoted_from) snprintf(pr, sizeof pr, "%d<-%d", t->prio, t->demoted_from);
            else                 snprintf(pr, sizeof pr, "%d%c", t->prio, policy_ch(t->policy));
            L(" %-12.12s %5d %-11.11s %6s %-14s %4d %5.1f %s", t->name, t->tid, t->what, pr,
              mask_text(mk, sizeof mk, t->runmask), t->last_cpu, t->cpu_pct_x10 / 10.0, role_name(t->role));
        }
        if (g->nthreads > shown) L("   ... %d more thread(s)", g->nthreads - shown);
    }
    L(" CSV %s %s", v->logging ? "ON ->" : "off", v->logging ? v->log_path : "");
    L(" \033[1mEVENTS\033[0m");
    for (int i = 0; i < MAX_EVENTS; i++) {
        if (i < v->n_ev) L("   %8.1f s  %s", v->ev[i].t_s, v->ev[i].text);
        else             L(" ");
    }
    L(" %s", RULE);
    L(" [l] CPU load  [m] msg flood  [b] contained load  [k] stop   (loads stop after %d s)",
      v->load_secs);
    L(" [r] reset    [c] CSV on/off  [q] quit%s", v->cmd_ok ? "" : "   (i/r disabled: /ctl_stats read-only)");
    L(" > %s", v->msg);
    fputs("\033[J", stdout);
    fflush(stdout);
}
