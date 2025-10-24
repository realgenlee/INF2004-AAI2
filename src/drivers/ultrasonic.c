#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "config.h"
#include "drivers/ultrasonic.h"

void ultrasonic_init(void) {
    // Configure trigger pin (output)
    gpio_init(ULTRASONIC_TRIG_GPIO);
    gpio_set_dir(ULTRASONIC_TRIG_GPIO, GPIO_OUT);
    gpio_put(ULTRASONIC_TRIG_GPIO, 0);

    // Configure echo pin (input)
    gpio_init(ULTRASONIC_ECHO_GPIO);
    gpio_set_dir(ULTRASONIC_ECHO_GPIO, GPIO_IN);
    gpio_pull_down(ULTRASONIC_ECHO_GPIO);  // Disable floating

    printf("Ultrasonic sensor initialized: TRIG=GP%d, ECHO=GP%d\n",
           ULTRASONIC_TRIG_GPIO, ULTRASONIC_ECHO_GPIO);
    printf("  Range: %.0f-%.0f cm\n", 
           ULTRASONIC_MIN_DISTANCE_CM, ULTRASONIC_MAX_DISTANCE_CM);
}

float ultrasonic_measure_cm(void) {
    // Send 10µs trigger pulse
    gpio_put(ULTRASONIC_TRIG_GPIO, 0);
    sleep_us(2);
    gpio_put(ULTRASONIC_TRIG_GPIO, 1);
    sleep_us(10);
    gpio_put(ULTRASONIC_TRIG_GPIO, 0);

    // Wait for echo to go high (with timeout)
    uint64_t start_wait = time_us_64();
    while (!gpio_get(ULTRASONIC_ECHO_GPIO)) {
        if ((time_us_64() - start_wait) > ULTRASONIC_ECHO_TIMEOUT_US) {
            return NAN;  // No response
        }
    }

    // Calculate max echo timeout based on max range
    uint32_t echo_high_timeout_us = (uint32_t)(
        (2.0f * ULTRASONIC_MAX_DISTANCE_CM) / ULTRASONIC_SPEED_CM_PER_US + 0.5f
    );

    // Measure echo high duration
    uint64_t t_start = time_us_64();
    while (gpio_get(ULTRASONIC_ECHO_GPIO)) {
        if ((time_us_64() - t_start) > echo_high_timeout_us) {
            return NAN;  // Timed out while high
        }
    }
    uint32_t t_us = (uint32_t)(time_us_64() - t_start);

    // Convert to distance: (time * speed of sound) / 2
    float d_cm = (t_us * ULTRASONIC_SPEED_CM_PER_US) / 2.0f;

    // Validate range
    if (d_cm < ULTRASONIC_MIN_DISTANCE_CM || d_cm > ULTRASONIC_MAX_DISTANCE_CM) {
        return NAN;
    }

    return d_cm;
}

bool ultrasonic_measure_valid(float* distance_cm) {
    if (distance_cm == NULL) return false;
    
    *distance_cm = ultrasonic_measure_cm();
    return !isnan(*distance_cm);
}

bool ultrasonic_is_object_within(float threshold_cm) {
    float distance = ultrasonic_measure_cm();
    
    if (isnan(distance)) {
        return false;  // Out of range or timeout
    }
    
    return (distance <= threshold_cm);
}

float ultrasonic_measure_averaged_cm(int samples) {
    if (samples <= 1) {
        return ultrasonic_measure_cm();
    }

    float sum = 0.0f;
    int valid_samples = 0;

    for (int i = 0; i < samples; i++) {
        float measurement = ultrasonic_measure_cm();
        
        if (!isnan(measurement)) {
            sum += measurement;
            valid_samples++;
        }
        
        // Small delay between measurements
        sleep_ms(10);
    }

    if (valid_samples == 0) {
        return NAN;  // No valid measurements
    }

    return sum / (float)valid_samples;
}

void ultrasonic_print_data(void) {
    float distance = ultrasonic_measure_cm();

    if (isnan(distance)) {
        printf("[ULTRASONIC] Out of range / timeout\n");
    } else {
        printf("[ULTRASONIC] Distance: %.2f cm\n", distance);
    }
}