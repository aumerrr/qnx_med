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
    printf("[%7.1fs] %-8s%s cyc=%llu wake(us) last/avg/max/jit=%.1f/%.1f/%.1f/%.1f "
           "lat(us) avg/max=%.1f/%.1f miss=%llu safe=%llu rec=%u/%u pot=%d servo=%d load=%s\n",
           v->uptime_s, ui_state_name(s->state), v->stale ? "(STALE)" : "",
           (unsigned long long)s->cycles,
           us(s->wake_late_last_ns), us(s->wake_late_avg_ns), us(s->wake_late_max_ns),
           us(s->wake_late_std_ns), us(s->latency_avg_ns), us(s->latency_max_ns),
           (unsigned long long)s->misses, (unsigned long long)s->safe_activations,
           s->recovery_count, s->recovery_target, s->pot_permille, s->servo_us,
           v->n_load ? "ON" : "off");
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
    else
        L(" ACTUATOR /dev/actuator not running");

    L(" %s", RULE);
    L(" LOAD %s%-3s\033[0m (%d procs)    CSV %s %s", v->n_load ? "\033[1;33m" : "",
      v->n_load ? "ON" : "off", v->n_load, v->logging ? "ON ->" : "off", v->logging ? v->log_path : "");
    L(" \033[1mEVENTS\033[0m");
    for (int i = 0; i < MAX_EVENTS; i++) {
        if (i < v->n_ev) L("   %8.1f s  %s", v->ev[i].t_s, v->ev[i].text);
        else             L(" ");
    }
    L(" %s", RULE);
    L(" [l] cpu_hog load on  [k] load off");
    L(" [r] reset    [c] CSV on/off  [q] quit%s", v->cmd_ok ? "" : "   (i/r disabled: /ctl_stats read-only)");
    L(" > %s", v->msg);
    fputs("\033[J", stdout);
    fflush(stdout);
}
