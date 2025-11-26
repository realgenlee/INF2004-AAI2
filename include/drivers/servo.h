#ifndef SERVO_H
#define SERVO_H
#pragma once
#include "pico/stdlib.h"
#include <stdbool.h>

void servo_init(void);

void servo_set_angle(uint8_t angle);

uint8_t servo_get_angle(void);

void servo_center(void);

#endif