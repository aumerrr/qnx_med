/* cpu_hog.c - CPU load generator (Person B)
 *
 * usage: cpu_hog [threads=1] [prio=10] [seconds=0] [cpu=-1] [policy=rr|fifo]
 *   threads  number of busy threads
 *   prio     SCHED_FIFO priority of the threads (control loop is 60, PWM 70)
 *   seconds  run time, 0 = until Ctrl+C / slay
 *   cpu      pin every thread to this CPU, -1 = let the scheduler choose
 *   policy   rr (default, like normal QNX processes: shares time with other
 *            threads at the same priority) or fifo (never yields to equal
 *            priority; with prio >= 10 on all cores it FREEZES sshd/network)
 *
 * examples:
 *   cpu_hog 4 10 30        4 hogs at prio 10 for 30 s (load on every core)
 *   cpu_hog 1 10 0 3       one hog pinned to CPU3 until stopped
 *   cpu_hog 2 65 5 1       2 hogs at prio 65 on CPU1 for 5 s (above the control loop)
 */
#include "loadgen_util.h"
#include <signal.h>
#include <unistd.h>

#define MAX_THREADS 32

static volatile sig_atomic_t g_stop;
static void on_sig(int s) { (void)s; g_stop = 1; }

typedef struct { int prio; int cpu; int fifo; } hog_arg_t;

static void *hog(void *p)
{
    hog_arg_t *a = (hog_arg_t *)p;
    if ((a->fifo ? lg_set_prio(a->prio) : lg_set_prio_rr(a->prio)) != 0)
        fprintf(stderr, "cpu_hog: warning: could not set priority %d (run as root)\n", a->prio);
    if (a->cpu >= 0 && lg_pin_cpu(a->cpu) != 0)
        fprintf(stderr, "cpu_hog: warning: could not pin to CPU%d\n", a->cpu);

    volatile uint64_t x = 0;
    while (!g_stop) {
        for (uint32_t i = 0; i < 1000000u; i++) x += i;
    }
    return NULL;
}

int main(int argc, char **argv)
{
    int      threads = argc > 1 ? atoi(argv[1]) : 1;
    int      prio    = argc > 2 ? atoi(argv[2]) : 10;
    unsigned seconds = argc > 3 ? (unsigned)atoi(argv[3]) : 0;
    int      cpu     = argc > 4 ? atoi(argv[4]) : -1;
    int      fifo    = (argc > 5 && strcmp(argv[5], "fifo") == 0);

    if (threads < 1) threads = 1;
    if (threads > MAX_THREADS) threads = MAX_THREADS;

    signal(SIGINT, on_sig);
    signal(SIGTERM, on_sig);
    /* Hard stop that does not depend on our own threads: the hog threads run above the main
     * thread, so on fully loaded cores the main thread never wakes to check the time. SIGALRM's
     * default action terminates the process from the kernel side, 1 s after the normal stop. */
    if (seconds) alarm(seconds + 1);

    pthread_t tid[MAX_THREADS];
    hog_arg_t arg = { prio, cpu, fifo };

    for (int i = 0; i < threads; i++) {
        if (pthread_create(&tid[i], NULL, hog, &arg) != 0) {
            perror("cpu_hog: pthread_create");
            threads = i;
            break;
        }
    }

    printf("cpu_hog: %d thread(s), prio %d (%s), cpu %d, %s\n", threads, prio,
           fifo ? "FIFO" : "RR", cpu, seconds ? "timed" : "until stopped");
    fflush(stdout);

    uint64_t t0 = lg_now_ns();
    while (!g_stop) {
        if (seconds && lg_now_ns() - t0 >= (uint64_t)seconds * 1000000000ULL) break;
        struct timespec d = { 0, 100 * 1000 * 1000 };
        nanosleep(&d, NULL);
    }
    g_stop = 1;
    for (int i = 0; i < threads; i++) pthread_join(tid[i], NULL);

    printf("cpu_hog: stopped\n");
    return 0;
}
