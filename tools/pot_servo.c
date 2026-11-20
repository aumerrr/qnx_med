/* pot_servo.c - pot (ADS1115, I2C) drives the servo, full pot travel -> full servo travel.
 * ADC code from B (adc.c), servo from A (servo.c / servo.h).
 *
 * usage: pot_servo /dev/i2cN [raw_min raw_max]   normal mode (defaults 0 .. 26400)
 *        pot_servo 1500                           hold a fixed pulse (calibration)
 *
 * Proportional mapping (the whole pot sweep drives the whole servo sweep):
 *        pot_permille = (raw - raw_min) * 1000 / (raw_max - raw_min)     0..1000
 *        pulse_us     = P_MIN_US + pot_permille * (P_MAX_US - P_MIN_US) / 1000
 * With a 300 deg pot and a 180 deg servo the servo turns 0.6 deg per pot degree.
 *
 * NOTE: servo.h / servo.c clamp the pulse to SERVO_MIN_US..SERVO_MAX_US. Those must
 * cover P_MIN_US..P_MAX_US, otherwise the servo will stop early.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <devctl.h>
#include <sys/neutrino.h>
#include <hw/i2c.h>
#include "rt_util.h"
#include "servo.h"

#define ADDR      0x48
#define PERIOD_NS 10000000LL      /* read the pot + update the servo every 10 ms */

/* ---- calibration: measure these once, then edit ---- */
#define POT_RANGE_DEG    300      /* pot rotation from one end stop to the other        */
#define SERVO_SPAN_DEG   180      /* servo rotation between P_MIN_US and P_MAX_US       */
#define P_MIN_US         600      /* pulse at the servo's one end stop (back off 50 us if it hums) */
#define P_MAX_US         2400     /* pulse at the servo's other end stop                */
#define DEADBAND_PM      4        /* ignore pot changes smaller than this (permille)    */

static int wr(int fd, const uint8_t *d, int n) {
    struct { i2c_send_t h; uint8_t b[8]; } m;
    memset(&m, 0, sizeof m);
    m.h.slave.addr = ADDR; m.h.slave.fmt = I2C_ADDRFMT_7BIT;
    m.h.len = n; m.h.stop = 1;
    memcpy(m.b, d, n);
    return devctl(fd, DCMD_I2C_SEND, &m, sizeof(m.h) + n, NULL);
}
static int rd(int fd, uint8_t reg, uint8_t *out, int n) {
    struct { i2c_sendrecv_t h; uint8_t b[8]; } m;
    memset(&m, 0, sizeof m);
    m.h.slave.addr = ADDR; m.h.slave.fmt = I2C_ADDRFMT_7BIT;
    m.h.send_len = 1; m.h.recv_len = n; m.h.stop = 1;
    m.b[0] = reg;
    int r = devctl(fd, DCMD_I2C_SENDRECV, &m, sizeof(m.h) + n, NULL);
    if (r == EOK) memcpy(out, m.b, n);
    return r;
}

static void print_servo_stats(void)
{
    servo_stats_t s;
    servo_get_stats(&s);
    printf("   servo PWM: frames=%llu  falling-edge error max=%lld us  overruns=%llu\n",
           (unsigned long long)s.frames, (long long)(s.fall_late_max_ns / 1000),
           (unsigned long long)s.overruns);
}

/* warn if servo.h would clamp part of our pulse range */
static void check_range(void)
{
    if (P_MIN_US < SERVO_MIN_US || P_MAX_US > SERVO_MAX_US)
        fprintf(stderr, "WARNING: servo.h clamps to %d..%d us but this file uses %d..%d us -"
                        " widen SERVO_MIN_US/SERVO_MAX_US (and any clamp in servo.c)\n",
                SERVO_MIN_US, SERVO_MAX_US, P_MIN_US, P_MAX_US);
}

int main(int argc, char **argv)
{
    if (argc < 2) {
        fprintf(stderr, "usage: %s /dev/i2cN [min max]   or   %s <pulse_us>\n", argv[0], argv[0]);
        return 1;
    }
    setvbuf(stdout, NULL, _IONBF, 0);

    /* ---- calibration mode: hold one fixed pulse width ---- */
    if (argv[1][0] != '/') {
        int us = atoi(argv[1]);
        if (servo_init() != 0) { fprintf(stderr, "servo_init failed (run as root?)\n"); return 1; }
        check_range();
        servo_set_us(us);
        printf("holding %d us - watch the servo\n", servo_get_us());
        for (;;) {
            sleep(1);
            print_servo_stats();
        }
    }

    /* ---- normal mode: pot -> servo ---- */
    int mn = argc > 2 ? atoi(argv[2]) : 0;
    int mx = argc > 3 ? atoi(argv[3]) : 26400;
    if (mx <= mn) { fprintf(stderr, "raw_max must be larger than raw_min\n"); return 1; }

    if (rt_set_prio(60) != 0) fprintf(stderr, "could not set priority 60 (run as root?)\n");
    if (rt_pin_self(0x2) != 0) fprintf(stderr, "could not pin to CPU1\n");

    int fd = open(argv[1], O_RDWR);
    if (fd < 0) { perror("open i2c"); return 1; }

    /* ADS1115: A0 vs GND, +-4.096 V, continuous, 860 SPS */
    uint8_t cfg[3] = { 0x01, 0x42, 0xE3 };
    int r = wr(fd, cfg, 3);
    if (r != EOK) { fprintf(stderr, "config write failed: %s\n", strerror(r)); return 1; }

    if (servo_init() != 0) { fprintf(stderr, "servo_init failed (run as root?)\n"); return 1; }
    check_range();
    printf("pot_servo running: pot %d deg -> servo %d deg (%d..%d us), ratio %.2f\n",
           POT_RANGE_DEG, SERVO_SPAN_DEG, P_MIN_US, P_MAX_US,
           (double)SERVO_SPAN_DEG / POT_RANGE_DEG);

    uint8_t b[2];
    int raw = 0, read_errors = 0, have = 0;
    long filt = 0, held = -1000;
    unsigned n = 0;
    int64_t late_max = 0;
    int64_t next = rt_now_ns() + PERIOD_NS;

    /* timestamp pairing: t1 = pot input change seen, t2 = PWM rising edge that carries it */
    int64_t t0 = rt_now_ns();                        /* time zero for printed timestamps */
    int pend = 0;
    int64_t ts1 = 0, t_set = 0;

    for (;;) {
        rt_sleep_until_ns(next);
        int64_t late = rt_now_ns() - next;           /* how late this cycle woke up */
        if (late > late_max) late_max = late;

        if (rd(fd, 0x00, b, 2) == EOK)
            raw = (int16_t)((b[0] << 8) | b[1]);
        else
            read_errors++;                           /* keep the last value */
        int64_t t_in = rt_now_ns();                  /* moment this pot sample was taken */

        /* smoothing: move 1/4 of the way to the new reading each cycle */
        if (!have) { filt = raw; have = 1; }
        else filt += (raw - filt) / 4;

        long pm = (filt - mn) * 1000 / (mx - mn);    /* pot position 0..1000 */
        if (pm < 0) pm = 0;
        if (pm > 1000) pm = 1000;

        long prev_held = held;
        /* ignore tiny noise, but always follow the pot onto either end */
        if (labs(pm - held) >= DEADBAND_PM || pm == 0 || pm == 1000) held = pm;
        int input_changed = (held != prev_held && prev_held != -1000);

        /* proportional mapping: full pot travel -> full servo travel */
        long pot_deg_x10 = held * POT_RANGE_DEG / 100;              /* pot angle, 0.1 deg units (display) */
        long pulse = P_MIN_US + held * (P_MAX_US - P_MIN_US) / 1000;
        if (pulse > P_MAX_US) pulse = P_MAX_US;
        if (pulse < P_MIN_US) pulse = P_MIN_US;
        servo_set_us((int)pulse);
        if (input_changed && !pend) {                /* start a new timestamp pair */
            pend = 1;
            ts1 = t_in;
            t_set = rt_now_ns();
        }
        if (pend) {                                  /* first PWM rising edge after the new pulse was set */
            int64_t t2 = servo_last_rise_ns();
            if (t2 > t_set) {
                printf("t1(pot)=%9.6f s  t2(pwm)=%9.6f s  diff=%lld us\n",
                       (ts1 - t0) / 1e9, (t2 - t0) / 1e9, (long long)((t2 - ts1) / 1000));
                pend = 0;
            }
        }

        if (++n % 50 == 0) {                         /* every 0.5 s */
            printf("raw=%6d  pot=%5.1f deg  pulse=%4ld us  servo=%5.1f deg  wake-late max=%lld us  read_errors=%d\n",
                   raw, pot_deg_x10 / 10.0, pulse,
                   (pulse - P_MIN_US) * (double)SERVO_SPAN_DEG / (P_MAX_US - P_MIN_US),
                   (long long)(late_max / 1000), read_errors);
            late_max = 0;
        }
        if (n % 500 == 0)                            /* every 5 s */
            print_servo_stats();

        next += PERIOD_NS;
        int64_t now = rt_now_ns();
        while (next <= now) next += PERIOD_NS;      /* missed a slot: stay on the grid */
    }
}
