/* pot_mirror.c - copy the pot (/pot, from adc_reader) to /dev/actuator (Person B)
 * Bring-up / measurement tool: stands in for the control loop.
 *
 * usage: pot_mirror [period_ms=20]
 *
 * pulse = P_MIN_US + permille * (P_MAX_US - P_MIN_US) / 1000   (config.h)
 *
 * Prints once per second:
 *   pulse            last pulse sent
 *   writes           writes per second (period_ms = 20 -> 50)
 *   write avg / max  time of one write() to /dev/actuator, in us
 *   age avg / max    pot sample age when the command is sent, in us
 *   stale            cycles where age > POT_STALE_US
 *   pot_err          cycles where adc_reader reported POT_ERROR
 *   write_errors     writes refused by actuator_rm
 */
#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <errno.h>
#include <time.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/mman.h>
#include <sys/stat.h>

#include "config.h"
#include "ipc_defs.h"

static uint64_t now_ns(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000000ULL + (uint64_t)ts.tv_nsec;
}

/* seqlock read: retry while the writer is mid-update */
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

int main(int argc, char **argv)
{
    long period_ms = argc > 1 ? atol(argv[1]) : 20;
    if (period_ms < 1) period_ms = 1;
    const long period_ns = period_ms * 1000000L;

    int shm = shm_open(SHM_POT_NAME, O_RDONLY, 0);
    if (shm < 0) { perror("pot_mirror: shm_open /pot (is adc_reader running?)"); return 1; }
    void *map = mmap(NULL, sizeof(pot_sample_t), PROT_READ, MAP_SHARED, shm, 0);
    if (map == MAP_FAILED) { perror("pot_mirror: mmap"); return 1; }
    volatile pot_sample_t *pot = (volatile pot_sample_t *)map;

    int fd = open(ACT_DEV_PATH, O_WRONLY);
    if (fd < 0) { perror("pot_mirror: open " ACT_DEV_PATH " (is actuator_rm running?)"); return 1; }

    printf("pot_mirror: /pot -> %s every %ld ms, pulse %d..%d us\n",
           ACT_DEV_PATH, period_ms, P_MIN_US, P_MAX_US);
    fflush(stdout);

    struct timespec next;
    clock_gettime(CLOCK_MONOTONIC, &next);
    uint64_t last_print = now_ns();

    unsigned long writes = 0, stale = 0, pot_err = 0, werr = 0, rfail = 0;
    double w_sum = 0, w_max = 0, a_sum = 0, a_max = 0;
    int pulse = 0;

    for (;;) {
        next.tv_nsec += period_ns;
        while (next.tv_nsec >= 1000000000L) { next.tv_nsec -= 1000000000L; next.tv_sec++; }
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, NULL);

        pot_sample_t s;
        if (pot_read(pot, &s) != 0) { rfail++; continue; }

        if (s.status != POT_OK) pot_err++;

        long pm = s.permille;
        if (pm < 0) pm = 0;
        if (pm > 1000) pm = 1000;
        pulse = P_MIN_US + (int)(pm * (P_MAX_US - P_MIN_US) / 1000);

        char buf[16];
        int n = snprintf(buf, sizeof(buf), "%d", pulse);

        uint64_t t0 = now_ns();
        ssize_t r = write(fd, buf, (size_t)n);
        uint64_t t1 = now_ns();

        if (r != n) werr++;
        writes++;

        double w_us = (double)(t1 - t0) / 1000.0;
        w_sum += w_us; if (w_us > w_max) w_max = w_us;

        double age_us = (double)((int64_t)t1 - s.ts_ns) / 1000.0;
        a_sum += age_us; if (age_us > a_max) a_max = age_us;
        if (age_us > (double)POT_STALE_US) stale++;

        if (t1 - last_print >= 1000000000ULL) {
            last_print = t1;
            printf("pulse=%d us  writes=%lu  write avg=%.1f us max=%.1f us  "
                   "age avg=%.0f us max=%.0f us  stale=%lu pot_err=%lu write_errors=%lu\n",
                   pulse, writes, writes ? w_sum / writes : 0.0, w_max,
                   writes ? a_sum / writes : 0.0, a_max, stale, pot_err, werr);
            fflush(stdout);
            writes = stale = pot_err = werr = rfail = 0;
            w_sum = w_max = a_sum = a_max = 0;
        }
    }
    return 0;
}
