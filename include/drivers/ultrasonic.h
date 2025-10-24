#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include "pico/stdlib.h"
#include <stdbool.h>

// Initialize ultrasonic sensor
void ultrasonic_init(void);

// Measure distance in centimeters
// Returns NAN if out of range or timeout
float ultrasonic_measure_cm(void);

// Measure distance and check if within valid range
// Returns true if valid, false if out of range/timeout
bool ultrasonic_measure_valid(float* distance_cm);

// Check if object is within specified distance
bool ultrasonic_is_object_within(float threshold_cm);

// Get averaged distance over multiple samples
float ultrasonic_measure_averaged_cm(int samples);

// Print ultrasonic sensor data
void ultrasonic_print_data(void);

#endif