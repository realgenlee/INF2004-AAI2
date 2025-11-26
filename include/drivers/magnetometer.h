#ifndef DRIVERS_MAGNETOMETER_H
#define DRIVERS_MAGNETOMETER_H
#pragma once
#include "pico/stdlib.h"
#include <stdbool.h>
#include <stdint.h>

typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
    float heading;
} magnetometer_data_t;

bool magnetometer_init(void);

bool magnetometer_read_raw(int16_t *mx, int16_t *my, int16_t *mz);

bool magnetometer_read_data(magnetometer_data_t *data);

bool magnetometer_read_data_raw(magnetometer_data_t *data);

float magnetometer_calculate_heading(int16_t mx, int16_t my);

float magnetometer_get_heading(void);

float magnetometer_get_heading_raw(void);

void magnetometer_print_data(void);

void magnetometer_start_calibration(void);
void magnetometer_update_calibration(int16_t mx, int16_t my, int16_t mz);
void magnetometer_finish_calibration(void);

void magnetometer_reset_filter(void);

#endif