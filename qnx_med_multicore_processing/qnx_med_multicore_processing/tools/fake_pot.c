/* fake_pot.c - bring-up tool: writes a sine wave into /pot so ctl_loop can run without
 * the ADC. usage: fake_pot [period_s]   (default 4 s per sweep). Ctrl-C to stop. */
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include <time.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/mman.h>
#include "ipc_defs.h"

static int64_t now_ns(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return (int64_t)t.tv_sec * 1000000000LL + t.tv_nsec;
}

int main(int argc, char **argv)
{
    double period = argc > 1 ? atof(argv[1]) : 4.0;
    if (period <= 0) period = 4.0;

    int fd = shm_open(SHM_POT_NAME, O_CREAT | O_RDWR, 0666);
    if (fd < 0) { perror("shm_open"); return 1; }
    if (ftruncate(fd, sizeof(pot_sample_t)) != 0) { perror("ftruncate"); return 1; }
    pot_sample_t *p = mmap(NULL, sizeof *p, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    if (p == MAP_FAILED) { perror("mmap"); return 1; }

    printf("fake_pot: sine, %.1f s per sweep, writing %s\n", period, SHM_POT_NAME);
    int64_t t0 = now_ns();
    for (;;) {
        double t = (now_ns() - t0) / 1e9;
        int pm = (int)(500.0 + 500.0 * sin(2.0 * M_PI * t / period));
        p->seq++;                              /* odd: writing */
        p->ts_ns    = now_ns();
        p->permille = pm;
        p->raw      = pm * 26;
        p->status   = POT_OK;
        p->seq++;                              /* even: done */
        usleep(1000);                          /* about 1 kHz like the ADC */
    }
}
