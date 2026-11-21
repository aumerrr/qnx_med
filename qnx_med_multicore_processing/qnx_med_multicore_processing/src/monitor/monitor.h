/* monitor.h - shared between monitor main.c (data, keys, load, CSV) and ui.c (drawing).
 * Owner: C. */
#ifndef MONITOR_H
#define MONITOR_H
#include <stdint.h>
#include "ipc_defs.h"
#include "guard_ipc.h"

#define MAX_EVENTS 5

typedef struct {
    double t_s;                 /* seconds since monitor start */
    char   text[72];
} mon_event_t;

typedef struct {
    double      uptime_s;

    int         have_stats;     /* /ctl_stats is mapped and readable          */
    int         stale;          /* cycles have not advanced for > 1 s         */
    int         cmd_ok;         /* /ctl_stats is writable (keys i / r work)   */
    ctl_stats_t s;              /* consistent copy (seqlock)                  */
    double      cycles_per_s;

    int          have_pot;      /* /pot is mapped                             */
    pot_sample_t pot;
    double       pot_age_ms;

    int         have_act;       /* /dev/actuator answered DCMD_ACT_STATS      */
    act_stats_t act;
    double      act_age_s;      /* seconds since /dev/actuator last answered, -1 never */
    char        core_cl[8][32]; /* custom cluster name(s) of each core          */

    int         n_load;         /* 1 while a load we started is running        */
    const char *load_name;      /* what the running load is                    */
    double      load_left_s;    /* seconds until the load timer stops it      */
    int         load_secs;      /* load timer length                          */

    int           have_guard;   /* /guard_stats is mapped (guardian running)  */
    int           guard_stale;  /* guardian has not updated for > 1 s         */
    guard_stats_t g;            /* cores, threads, demotions                  */
    int         logging;        /* CSV logging on                             */
    char        log_path[160];

    mon_event_t ev[MAX_EVENTS]; /* newest last                                */
    int         n_ev;
    char        msg[200];        /* feedback for the last key                  */
} mon_view_t;

void ui_init(int plain);
void ui_draw(const mon_view_t *v);
void ui_note(const char *text);            /* plain mode: print a one-off line */
void ui_done(void);
const char *ui_state_name(int s);

#endif
