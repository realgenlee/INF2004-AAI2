#pragma once
#include <stdbool.h>
#include <stdint.h>

// Call once at boot.
void motor_init_all(void);

// Set left/right motor command in [-1.0, +1.0] (sign = direction).
void motor_set(float left, float right);

// Emergency stop both motors.
void motor_stop(void);
