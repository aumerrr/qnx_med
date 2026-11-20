/*
 * servo.h
 *
 *  Created on: 19-Nov-2026
 *      Author: APPLE
 */
/* servo.h - software-PWM servo output (Person 1) */
#ifndef SERVO_H
#define SERVO_H
#include <stdint.h>

#define SERVO_MIN_US 500   /* stay inside this range until end stops are checked */
#define SERVO_MAX_US 2500

typedef struct {
    uint64_t frames;           /* PWM frames generated */
    int64_t  rise_late_max_ns; /* worst lateness of the rising edge vs its frame slot */
    int64_t  fall_late_max_ns; /* worst error of the falling edge vs its target */
    uint64_t overruns;         /* frames that started late enough to skip a slot */
} servo_stats_t;

int  servo_init(void);          /* map GPIO, start PWM thread. 0 ok, -1 error */
void servo_set_us(int us);      /* pulse width, clamped to SERVO_MIN_US..SERVO_MAX_US */
int  servo_get_us(void);
int64_t servo_last_rise_ns(void); /* rt_now_ns() time of the latest rising edge */
void servo_get_stats(servo_stats_t *s);
void servo_shutdown(void);      /* stop thread, drive pin low */
#endif
