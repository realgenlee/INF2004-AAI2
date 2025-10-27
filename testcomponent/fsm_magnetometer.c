
#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "config.h"
#include "drivers/magnetometer.h"
#include "fsm.h"

// Minimal FSM to test ONLY the Magnetometer (LSM303) extracted from combfsm.c
// - Initializes I2C + magnetometer
// - Periodically reads raw XYZ and heading, and prints them
// - No motors, no encoders, no IR, no ultrasonic, no WiFi/MQTT

static uint32_t last_print_ms = 0;

static inline uint32_t now_ms(void) {
    return to_ms_since_boot(get_absolute_time());
}

void fsm_init(void) {
    stdio_init_all();

    // Initialize magnetometer (LSM303)
    if (magnetometer_init()) {
        printf("\n=== COMBFSM: MAGNETOMETER-ONLY TEST MODE ===\n");
        printf("I2C: SDA=GP%d, SCL=GP%d, LSM303_MAG_ADDR=0x%02X\n",
               I2C_SDA_PIN, I2C_SCL_PIN, LSM303_MAG_ADDR);
        printf("Print interval: %d ms\n", MAG_PRINT_INTERVAL_MS);
        printf("Rotate the board slowly to see heading change (0°=N, 90°=E, 180°=S, 270°=W)\n");
    } else {
        printf("[MAG] Initialization FAILED. Check wiring and I2C pins in config.h\n");
    }
}

void fsm_step(void) {
    uint32_t t = now_ms();
    if (t - last_print_ms >= (uint32_t)MAG_PRINT_INTERVAL_MS) {
        magnetometer_data_t d;
        if (magnetometer_read_data(&d)) {
            // If driver didn't set heading, compute a quick heading from X,Y
            float heading = isnan(d.heading) || isinf(d.heading) ? magnetometer_get_heading() : d.heading;
            printf("[MAG] raw: x=%d y=%d z=%d | heading=%.1f°\n", d.x, d.y, d.z, heading);
        } else {
            printf("[MAG] read failed\n");
        }
        last_print_ms = t;
    }
}
