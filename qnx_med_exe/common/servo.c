/* servo.c - software PWM for a hobby servo on one GPIO (BCM2711, QNX 8.0).
 *
 * 20 ms frame, 1000..2000 us pulse. A high-priority FIFO thread sleeps to an
 * absolute time, raises the pin through memory-mapped GPIO registers (same
 * approach as int_test), sleeps to just before the falling edge, then spins
 * the last SPIN_NS for a precise edge, and lowers the pin.
 *
 * Build: qcc -Vgcc_ntoaarch64le -O2 -o servo_test servo_test.c servo.c -lm
 * Run as root (mmap_device_io + priority > 63 need privileges).
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <stdint.h>
#include <pthread.h>
#include <sys/mman.h>
#include <sys/neutrino.h>
#include <hw/inout.h>
#include "rt_util.h"
#include "servo.h"

#ifndef SERVO_GPIO
#define SERVO_GPIO 22              /* BCM number (physical pin 15). B uses 17 and 27 - keep clear. */
#endif
#ifndef SERVO_PWM_PRIO
#define SERVO_PWM_PRIO 70          /* highest of our threads */
#endif
#ifndef SERVO_PWM_CPUMASK
#define SERVO_PWM_CPUMASK 0x4u     /* CPU2 on the stock image; 0 = don't pin */
#endif
#ifndef SERVO_SPIN_NS
#define SERVO_SPIN_NS 2600000LL      /* spin through the whole pulse (system tick is too coarse to sleep inside it) */
#endif
#define FRAME_NS 20000000LL

#define GPIO_BASE   0xFE200000ULL
#define GPIO_LEN    0x100
#define GPFSEL0     0x00
#define GPSET0      0x1C
#define GPCLR0      0x28

static volatile int       g_pulse_us = 1500;
static volatile int       g_run;
static uintptr_t          g_gpio;
static pthread_t          g_tid;
static volatile servo_stats_t g_stats;
static volatile int64_t   g_last_rise_ns;   /* time of the latest rising edge (for timestamp output) */

static inline void pin_high(void) { out32(g_gpio + GPSET0, 1u << SERVO_GPIO); }
static inline void pin_low(void)  { out32(g_gpio + GPCLR0, 1u << SERVO_GPIO); }

static void *pwm_thread(void *arg)
{
    (void)arg;
    if (rt_set_prio(SERVO_PWM_PRIO) != 0)
        fprintf(stderr, "servo: could not set priority %d (run as root?)\n", SERVO_PWM_PRIO);
    if (rt_pin_self(SERVO_PWM_CPUMASK) != 0)
        fprintf(stderr, "servo: could not pin to mask 0x%x: %s\n", SERVO_PWM_CPUMASK, strerror(errno));

    int64_t frame = rt_now_ns() + 2000000LL;
    while (g_run) {
        rt_sleep_until_ns(frame);
        if (!g_run)
            break;

        pin_high();
        int64_t t_rise = rt_now_ns();
        g_last_rise_ns = t_rise;               /* published for pot_servo's timestamp print */
        int64_t late = t_rise - frame;
        if (late > g_stats.rise_late_max_ns)
            g_stats.rise_late_max_ns = late;

        int us = g_pulse_us;
        int64_t t_fall = t_rise + (int64_t)us * 1000LL;
        if (t_fall - SERVO_SPIN_NS > rt_now_ns())
            rt_sleep_until_ns(t_fall - SERVO_SPIN_NS);
        while (rt_now_ns() < t_fall)
            ;                                  /* short spin for a precise falling edge */
        pin_low();
        int64_t err = rt_now_ns() - t_fall;
        if (err > g_stats.fall_late_max_ns)
            g_stats.fall_late_max_ns = err;

        g_stats.frames++;
        frame += FRAME_NS;
        int64_t now = rt_now_ns();
        while (frame <= now) {                 /* overran: skip missed slots, stay on the grid */
            frame += FRAME_NS;
            g_stats.overruns++;
        }
    }
    pin_low();
    return NULL;
}

int servo_init(void)
{
    if (ThreadCtl(_NTO_TCTL_IO, 0) == -1) {
        perror("servo: ThreadCtl(_NTO_TCTL_IO)");
        return -1;
    }
    g_gpio = mmap_device_io(GPIO_LEN, GPIO_BASE);
    if (g_gpio == MAP_DEVICE_FAILED) {
        perror("servo: mmap_device_io(GPIO)");
        return -1;
    }
    /* set the pin to output: function 001 in the pin's 3-bit FSEL field */
    uintptr_t fsel = g_gpio + GPFSEL0 + 4 * (SERVO_GPIO / 10);
    unsigned  sh   = 3u * (SERVO_GPIO % 10);
    uint32_t  v    = in32(fsel);
    v &= ~(7u << sh);
    v |= (1u << sh);
    out32(fsel, v);
    pin_low();

    memset((void *)&g_stats, 0, sizeof g_stats);
    g_run = 1;
    if (pthread_create(&g_tid, NULL, pwm_thread, NULL) != 0) {
        perror("servo: pthread_create");
        g_run = 0;
        return -1;
    }
    return 0;
}

void servo_set_us(int us)
{
    if (us < SERVO_MIN_US) us = SERVO_MIN_US;
    if (us > SERVO_MAX_US) us = SERVO_MAX_US;
    g_pulse_us = us;                           /* aligned 32-bit store; the PWM thread reads it once per frame */
}

int servo_get_us(void) { return g_pulse_us; }

int64_t servo_last_rise_ns(void) { return g_last_rise_ns; }

void servo_get_stats(servo_stats_t *s)
{
    s->frames           = g_stats.frames;
    s->rise_late_max_ns = g_stats.rise_late_max_ns;
    s->fall_late_max_ns = g_stats.fall_late_max_ns;
    s->overruns         = g_stats.overruns;
}

void servo_shutdown(void)
{
    if (g_run) {
        g_run = 0;
        pthread_join(g_tid, NULL);
    }
}
