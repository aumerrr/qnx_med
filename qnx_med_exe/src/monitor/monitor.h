/* monitor.h - shared between monitor main.c (data, keys, load, CSV) and ui.c (drawing).
 * Owner: C. */
#ifndef MONITOR_H
#define MONITOR_H
#include <stdint.h>
#include "ipc_defs.h"

#define MAX_EVENTS 6

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

    int         n_load;         /* load processes we started and still alive  */
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
