/* msg_flood.c - communication load generator (Person B)
 *
 * usage:
 *   msg_flood server   [prio=10]
 *   msg_flood client   [rate=0] [prio=10] [seconds=0] [threads=1]
 *   msg_flood actuator [rate=0] [prio=10] [seconds=0] [threads=1]
 *
 *   server    a dummy message server that just replies; start it first
 *   client    flood the dummy server with QNX messages (MsgSend)
 *   actuator  flood /dev/actuator with DCMD_ACT_STATS requests (read-only,
 *             does not move the servo) to load the resource manager
 *   rate      messages per second PER THREAD, 0 = as fast as possible
 *   seconds   run time, 0 = until Ctrl+C / slay
 *
 * examples:
 *   msg_flood server 10 &
 *   msg_flood client 0 10 30 2       2 flooding threads for 30 s
 *   msg_flood actuator 5000 10 30    5000 requests/s at the resource manager
 */
#include "loadgen_util.h"
#include <signal.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/neutrino.h>
#include <sys/dispatch.h>
#include <devctl.h>

#include "actuator_ipc.h"

#define SRV_NAME     "flood_srv"
#define MAX_THREADS  32

typedef struct { uint32_t seq; uint8_t pad[60]; } flood_msg_t;

enum { MODE_CLIENT, MODE_ACTUATOR };

typedef struct {
    int               mode;
    int               prio;
    long              rate;             /* per thread, 0 = max */
    volatile uint64_t count;
    volatile uint64_t errors;
} worker_t;

static volatile sig_atomic_t g_stop;
static void on_sig(int s) { (void)s; g_stop = 1; }

/* ---------------- server ---------------- */

static int run_server(int prio)
{
    if (lg_set_prio_rr(prio) != 0)
        fprintf(stderr, "msg_flood: warning: could not set priority %d\n", prio);

    name_attach_t *att = name_attach(NULL, SRV_NAME, 0);
    if (att == NULL) { perror("msg_flood: name_attach"); return 1; }
    printf("msg_flood: server '%s' ready, prio %d\n", SRV_NAME, prio);
    fflush(stdout);

    for (;;) {
        flood_msg_t m;
        int rcvid = MsgReceive(att->chid, &m, sizeof(m), NULL);
        if (rcvid == -1) {
            if (errno == EINTR) continue;
            perror("msg_flood: MsgReceive");
            return 1;
        }
        if (rcvid == 0) continue;       /* pulse (e.g. client disconnected) */
        MsgReply(rcvid, EOK, &m, sizeof(m));
    }
}

/* ---------------- client / actuator workers ---------------- */

static void *worker(void *p)
{
    worker_t *w = (worker_t *)p;

    if (lg_set_prio_rr(w->prio) != 0)
        fprintf(stderr, "msg_flood: warning: could not set priority %d\n", w->prio);

    int h = -1;
    for (int tries = 0; tries < 50 && !g_stop; tries++) {
        h = (w->mode == MODE_CLIENT) ? name_open(SRV_NAME, 0)
                                     : open(ACT_DEV_PATH, O_RDWR);
        if (h >= 0) break;
        usleep(100000);                 /* server not up yet, retry for 5 s */
    }
    if (h < 0) {
        fprintf(stderr, "msg_flood: cannot connect (%s)\n",
                w->mode == MODE_CLIENT ? "is 'msg_flood server' running?" : ACT_DEV_PATH);
        w->errors++;
        return NULL;
    }

    uint64_t period_ns = w->rate > 0 ? 1000000000ULL / (uint64_t)w->rate : 0;
    struct timespec next;
    clock_gettime(CLOCK_MONOTONIC, &next);

    flood_msg_t m, r;
    memset(&m, 0, sizeof(m));

    while (!g_stop) {
        int ok;
        if (w->mode == MODE_CLIENT) {
            m.seq++;
            ok = (MsgSend(h, &m, sizeof(m), &r, sizeof(r)) != -1);
        } else {
            act_stats_t st;
            ok = (devctl(h, DCMD_ACT_STATS, &st, sizeof(st), NULL) == 0);
        }
        if (ok) w->count++; else w->errors++;

        if (period_ns) {
            next.tv_nsec += (long)(period_ns % 1000000000ULL);
            next.tv_sec  += (time_t)(period_ns / 1000000000ULL);
            while (next.tv_nsec >= 1000000000L) { next.tv_nsec -= 1000000000L; next.tv_sec++; }
            clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, NULL);
        }
    }
    return NULL;
}

static int run_flood(int mode, long rate, int prio, unsigned seconds, int threads)
{
    if (threads < 1) threads = 1;
    if (threads > MAX_THREADS) threads = MAX_THREADS;

    static worker_t w[MAX_THREADS];
    pthread_t tid[MAX_THREADS];

    for (int i = 0; i < threads; i++) {
        memset((void *)&w[i], 0, sizeof(w[i]));
        w[i].mode = mode;
        w[i].prio = prio;
        w[i].rate = rate;
        if (pthread_create(&tid[i], NULL, worker, &w[i]) != 0) {
            perror("msg_flood: pthread_create");
            threads = i;
            break;
        }
    }

    printf("msg_flood: %s, %d thread(s), rate %ld/s per thread, prio %d, %s\n",
           mode == MODE_CLIENT ? "client" : "actuator", threads, rate, prio,
           seconds ? "timed" : "until stopped");
    fflush(stdout);

    uint64_t t0 = lg_now_ns(), last_t = t0, last_n = 0;
    while (!g_stop) {
        struct timespec d = { 1, 0 };
        nanosleep(&d, NULL);
        uint64_t now = lg_now_ns(), n = 0, e = 0;
        for (int i = 0; i < threads; i++) { n += w[i].count; e += w[i].errors; }
        double secs = (double)(now - last_t) / 1e9;
        printf("msg_flood: %.0f msg/s  total=%llu errors=%llu\n",
               secs > 0 ? (double)(n - last_n) / secs : 0.0,
               (unsigned long long)n, (unsigned long long)e);
        fflush(stdout);
        last_t = now; last_n = n;
        if (seconds && now - t0 >= (uint64_t)seconds * 1000000000ULL) break;
    }
    g_stop = 1;
    for (int i = 0; i < threads; i++) pthread_join(tid[i], NULL);
    printf("msg_flood: stopped\n");
    return 0;
}

/* ---------------- main ---------------- */

int main(int argc, char **argv)
{
    if (argc < 2) {
        fprintf(stderr, "usage: %s server [prio] | client|actuator [rate] [prio] [seconds] [threads]\n",
                argv[0]);
        return 2;
    }
    signal(SIGINT, on_sig);
    signal(SIGTERM, on_sig);

    if (strcmp(argv[1], "server") == 0)
        return run_server(argc > 2 ? atoi(argv[2]) : 10);

    int mode;
    if (strcmp(argv[1], "client") == 0)        mode = MODE_CLIENT;
    else if (strcmp(argv[1], "actuator") == 0) mode = MODE_ACTUATOR;
    else { fprintf(stderr, "msg_flood: unknown mode '%s'\n", argv[1]); return 2; }

    long     rate    = argc > 2 ? atol(argv[2]) : 0;
    int      prio    = argc > 3 ? atoi(argv[3]) : 10;
    unsigned seconds = argc > 4 ? (unsigned)atoi(argv[4]) : 0;
    int      threads = argc > 5 ? atoi(argv[5]) : 1;
    return run_flood(mode, rate, prio, seconds, threads);
}
