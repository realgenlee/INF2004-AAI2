#ifndef ULTRASONIC_H
#define ULTRASONIC_H
#pragma once
#include "pico/stdlib.h"
#include <stdbool.h>

void ultrasonic_init(void);

float ultrasonic_measure_cm(void);

bool ultrasonic_measure_valid(float* distance_cm);

bool ultrasonic_is_object_within(float threshold_cm);

float ultrasonic_measure_averaged_cm(int samples);

void ultrasonic_print_data(void);

typedef struct {
    float left_distance_cm;
    float center_distance_cm;
    float right_distance_cm;
    bool left_clear;
    bool right_clear;
    float estimated_width_cm;
} obstacle_scan_t;

#endif