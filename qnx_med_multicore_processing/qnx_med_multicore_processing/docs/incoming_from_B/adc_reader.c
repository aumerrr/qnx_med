/* adc_reader.c - ADS1115 pot reader, publishes /pot (Person B)
 *
 * usage: adc_reader [i2c_dev] [raw_min] [raw_max] [-v]
 *        adc_reader /dev/i2c1 100 26300
 *
 * Reads the ADS1115 (A0, continuous, 860 SPS) every ADC_PERIOD_NS and
 * publishes pot_sample_t in shared memory SHM_POT_NAME using the seqlock
 * agreed in the contracts (writer: seq odd while writing, even when done).
 *
 * On an I2C error the sample keeps its old ts_ns and permille, status becomes
 * POT_ERROR, so the control loop sees both an error flag and a stale
 * timestamp. After several errors in a row the ADC is re-configured.
 *
 * Uses common/ipc_defs.h (pot_sample_t, SHM_POT_NAME, POT_OK / POT_ERROR)
 * and common/config.h (PRIO_ADC).
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <errno.h>
#include <time.h>
#include <fcntl.h>
#include <unistd.h>
#include <pthread.h>
#include <sched.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <devctl.h>
#include <hw/i2c.h>

#include "config.h"
#include "ipc_defs.h"                   /* also includes actuator_ipc.h */

#define ADC_PERIOD_NS     2000000L      /* poll every 2 ms (ADC makes a sample every ~1.2 ms) */
#define ADC_PRIO          PRIO_ADC      /* config.h: PWM 70 > ctl 60 > adc 55 */
#define ADS_ADDR          0x48
#define ERR_REINIT_AFTER  5             /* consecutive errors before re-config */

/* ---------------- I2C (ADS1115) ---------------- */

static int adc_config(int fd)
{
    struct { i2c_send_t hdr; uint8_t d[3]; } m;
    memset(&m, 0, sizeof(m));
    m.hdr.slave.addr = ADS_ADDR;
    m.hdr.slave.fmt  = I2C_ADDRFMT_7BIT;
    m.hdr.len  = 3;
    m.hdr.stop = 1;
    m.d[0] = 0x01;      /* config register */
    m.d[1] = 0x42;      /* A0 vs GND, +-4.096 V, continuous */
    m.d[2] = 0xE3;      /* 860 SPS, comparator off */
    return devctl(fd, DCMD_I2C_SEND, &m, sizeof(m), NULL);
}

static int adc_read_raw(int fd, int *raw)
{
    struct { i2c_sendrecv_t hdr; uint8_t d[2]; } m;
    memset(&m, 0, sizeof(m));
    m.hdr.slave.addr = ADS_ADDR;
    m.hdr.slave.fmt  = I2C_ADDRFMT_7BIT;
    m.hdr.send_len = 1;
    m.hdr.recv_len = 2;
    m.hdr.stop = 1;
    m.d[0] = 0x00;      /* conversion register */
    int r = devctl(fd, DCMD_I2C_SENDRECV, &m, sizeof(m), NULL);
    if (r != 0) return r;
    *raw = (int16_t)(((unsigned)m.d[0] << 8) | m.d[1]);
    return 0;
}

/* ---------------- publish (seqlock) ---------------- */

static volatile pot_sample_t *g_pot;

static uint64_t now_ns(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000000ULL + (uint64_t)ts.tv_nsec;
}

static void publish_ok(int raw, int permille)
{
    int64_t  ts = (int64_t)now_ns();
    uint64_t s  = g_pot->seq;
    g_pot->seq = s + 1;                 /* odd: writing */
    __sync_synchronize();
    g_pot->ts_ns    = ts;
    g_pot->permille = permille;
    g_pot->raw      = raw;
    g_pot->status   = POT_OK;
    __sync_synchronize();
    g_pot->seq = s + 2;                 /* even: done */
}

static void publish_error(void)
{
    uint64_t s = g_pot->seq;
    g_pot->seq = s + 1;
    __sync_synchronize();
    g_pot->status = POT_ERROR;          /* ts_ns and permille stay old */
    __sync_synchronize();
    g_pot->seq = s + 2;
}

/* ---------------- main ---------------- */

int main(int argc, char **argv)
{
    const char *dev = argc > 1 ? argv[1] : "/dev/i2c1";
    long raw_min    = argc > 2 ? atol(argv[2]) : 100;
    long raw_max    = argc > 3 ? atol(argv[3]) : 26300;
    int  verbose    = (argc > 4 && strcmp(argv[4], "-v") == 0);

    if (raw_max <= raw_min) {
        fprintf(stderr, "adc_reader: raw_max must be larger than raw_min\n");
        return 2;
    }

    int fd = open(dev, O_RDWR);
    if (fd < 0) { perror("adc_reader: open i2c"); return 1; }

    if (adc_config(fd) != 0) {
        fprintf(stderr, "adc_reader: ADC config failed (wiring? address 0x%02x)\n", ADS_ADDR);
        return 1;
    }

    int shm = shm_open(SHM_POT_NAME, O_RDWR | O_CREAT, 0666);
    if (shm < 0) { perror("adc_reader: shm_open"); return 1; }
    if (ftruncate(shm, sizeof(pot_sample_t)) != 0) { perror("ftruncate"); return 1; }
    void *map = mmap(NULL, sizeof(pot_sample_t), PROT_READ | PROT_WRITE,
                     MAP_SHARED, shm, 0);
    if (map == MAP_FAILED) { perror("adc_reader: mmap"); return 1; }
    g_pot = (volatile pot_sample_t *)map;

    memset((void *)g_pot, 0, sizeof(pot_sample_t));
    publish_error();                    /* not valid until first good read */

    struct sched_param sp;
    memset(&sp, 0, sizeof(sp));
    sp.sched_priority = ADC_PRIO;
    if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp) != 0)
        fprintf(stderr, "adc_reader: warning: could not set FIFO priority %d\n", ADC_PRIO);

    printf("adc_reader: publishing %s (dev %s, raw %ld..%ld, period %ld us)\n",
           SHM_POT_NAME, dev, raw_min, raw_max, ADC_PERIOD_NS / 1000);
    fflush(stdout);

    struct timespec next;
    clock_gettime(CLOCK_MONOTONIC, &next);

    unsigned long samples = 0, errors = 0;
    int consec_err = 0;
    int raw = 0, permille = 0;
    uint64_t last_print = now_ns();

    for (;;) {
        next.tv_nsec += ADC_PERIOD_NS;
        while (next.tv_nsec >= 1000000000L) { next.tv_nsec -= 1000000000L; next.tv_sec++; }
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, NULL);

        if (adc_read_raw(fd, &raw) == 0) {
            consec_err = 0;
            long pm = ((long)raw - raw_min) * 1000L / (raw_max - raw_min);
            if (pm < 0)    pm = 0;
            if (pm > 1000) pm = 1000;
            permille = (int)pm;
            publish_ok(raw, permille);
            samples++;
        } else {
            errors++;
            publish_error();
            if (++consec_err >= ERR_REINIT_AFTER) {
                adc_config(fd);         /* try to bring the ADC back */
                consec_err = 0;
            }
        }

        if (verbose) {
            uint64_t t = now_ns();
            if (t - last_print >= 1000000000ULL) {
                last_print = t;
                printf("raw=%6d permille=%4d seq=%llu samples=%lu errors=%lu\n",
                       raw, permille, (unsigned long long)g_pot->seq, samples, errors);
                fflush(stdout);
            }
        }
    }
    return 0;
}
