/* overrun_inject.c - force ONE control-loop overrun (Person B)
 *
 * usage: overrun_inject <burst_ms> [prio=65] [cpus=1,2] [delay_s=0]
 *   burst_ms  how long every burst thread spins
 *   prio      SCHED_FIFO priority: above ctl_loop (60), below the PWM thread (70)
 *   cpus      comma list of CPUs to saturate, one busy thread per CPU.
 *             It must cover EVERY core the control loop may run on; otherwise
 *             the loop just moves to a free core and never overruns.
 *   delay_s   wait before the burst
 *
 * examples:
 *   overrun_inject 20            20 ms burst on CPU1 and CPU2
 *   overrun_inject 50 65 1 3     50 ms burst on CPU1 only, after 3 s
 */
#include "loadgen_util.h"
#include <unistd.h>

#define MAX_CPUS 8

typedef struct {
    int                cpu;
    int                prio;
    uint64_t           burst_ns;
    pthread_barrier_t *bar;
    int                id;
} inj_arg_t;

static uint64_t g_start_ns;             /* set by thread 0 when the burst begins */

static void *burst(void *p)
{
    inj_arg_t *a = (inj_arg_t *)p;

    if (lg_set_prio(a->prio) != 0)
        fprintf(stderr, "overrun_inject: warning: could not set priority %d (run as root)\n", a->prio);
    if (lg_pin_cpu(a->cpu) != 0)
        fprintf(stderr, "overrun_inject: warning: could not pin to CPU%d\n", a->cpu);

    pthread_barrier_wait(a->bar);       /* all threads start together */

    uint64_t t0 = lg_now_ns();
    if (a->id == 0) g_start_ns = t0;
    volatile uint64_t x = 0;
    while (lg_now_ns() - t0 < a->burst_ns) x++;
    return NULL;
}

int main(int argc, char **argv)
{
    if (argc < 2) {
        fprintf(stderr, "usage: %s <burst_ms> [prio=65] [cpus=1,2] [delay_s=0]\n", argv[0]);
        return 2;
    }
    unsigned burst_ms = (unsigned)atoi(argv[1]);
    int      prio     = argc > 2 ? atoi(argv[2]) : 65;
    const char *cpus  = argc > 3 ? argv[3] : "1,2";
    unsigned delay_s  = argc > 4 ? (unsigned)atoi(argv[4]) : 0;

    int cpu_list[MAX_CPUS];
    int n = 0;
    char buf[64];
    strncpy(buf, cpus, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = '\0';
    for (char *tok = strtok(buf, ","); tok && n < MAX_CPUS; tok = strtok(NULL, ","))
        cpu_list[n++] = atoi(tok);
    if (n == 0 || burst_ms == 0) {
        fprintf(stderr, "overrun_inject: need burst_ms > 0 and at least one cpu\n");
        return 2;
    }

    printf("overrun_inject: %u ms burst at prio %d on cpus %s, in %u s\n",
           burst_ms, prio, cpus, delay_s);
    fflush(stdout);
    if (delay_s) sleep(delay_s);

    pthread_barrier_t bar;
    pthread_barrier_init(&bar, NULL, (unsigned)n);

    pthread_t tid[MAX_CPUS];
    inj_arg_t arg[MAX_CPUS];
    for (int i = 0; i < n; i++) {
        arg[i].cpu = cpu_list[i];
        arg[i].prio = prio;
        arg[i].burst_ns = (uint64_t)burst_ms * 1000000ULL;
        arg[i].bar = &bar;
        arg[i].id = i;
        if (pthread_create(&tid[i], NULL, burst, &arg[i]) != 0) {
            perror("overrun_inject: pthread_create");
            return 1;
        }
    }
    for (int i = 0; i < n; i++) pthread_join(tid[i], NULL);

    printf("overrun_inject: done, burst started at monotonic t=%llu ns\n",
           (unsigned long long)g_start_ns);
    return 0;
}
