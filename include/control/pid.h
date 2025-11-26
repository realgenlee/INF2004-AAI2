#ifndef CONTROL_PID_H
#define CONTROL_PID_H
#pragma once
#include <stdint.h>

typedef struct {
    float kp, ki, kd;
    float integ;
    float prev_err;
    float out_min, out_max;
    float integ_min, integ_max;
    float dt_s;
    uint8_t first;
} pid_t;

void  pid_init(pid_t* p, float kp, float ki, float kd, float dt_s,
               float out_min, float out_max, float integ_min, float integ_max);
void  pid_reset(pid_t* p);
void  pid_set_gains(pid_t* p, float kp, float ki, float kd);
float pid_update(pid_t* p, float setpoint, float measurement);

#endif