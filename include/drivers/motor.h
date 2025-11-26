#ifndef DRIVERS_MOTOR_H
#define DRIVERS_MOTOR_H
#pragma once
#include <stdint.h>
#include <stdbool.h>

void motor_init_all(void);
void motor_set_speed_percent(uint speed_percent, bool anticlockwise);
void motor_stop(void);

void motor_set_signed(float right_percent, float left_percent);

#endif