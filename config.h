#pragma once
#include "pico/stdlib.h"

// ==== Motor pins ====
#define PIN_M1_PWM   10
#define PIN_M1_DIR   11
#define PIN_M2_PWM    8
#define PIN_M2_DIR    9

// ==== Encoder pins ====
#define PIN_ENC_LEFT   16
#define PIN_ENC_RIGHT  26

// ==== Button pins ====
#define PIN_BTN_DIR    21
#define PIN_BTN_SPD    20

// ==== IR Sensor 1: Line Follower ====
#define IR_LINE_ADC_GPIO              28     // A0 -> GP28 (ADC2)
#define IR_LINE_ADC_CHANNEL           2      // ADC channel 2
#define IR_LINE_DIGITAL_GPIO          7      // D0 -> GP7

// ==== IR Sensor 2: Barcode Scanner ====
#define IR_BARCODE_ADC_GPIO           1     // A0 -> GP27 (ADC1)
#define IR_BARCODE_ADC_CHANNEL        1      // ADC channel 1
#define IR_BARCODE_DIGITAL_GPIO       0      // D0 -> GP6

// ==== Ultrasonic Sensor ====
#define ULTRASONIC_TRIG_GPIO          4      // Trigger pin
#define ULTRASONIC_ECHO_GPIO          5      // Echo pin

// ==== I2C Magnetometer/Accelerometer (LSM303) ====
#define I2C_PORT                i2c1         // Using I2C1
#define I2C_SDA_PIN             2            // SDA -> GP2
#define I2C_SCL_PIN             3            // SCL -> GP3
#define I2C_BAUDRATE            100000       // 100 kHz
#define LSM303_ACC_ADDR         0x19         // Accelerometer I2C address
#define LSM303_MAG_ADDR         0x1E         // Magnetometer I2C address

// ===== Control & hardware config =====
#define CTRL_HZ                 100
#define CONTROL_DT_MS           (1000/CTRL_HZ)
#define ENCODER_CPR             20.0f
#define WHEEL_DIAMETER_MM       65.0f
#define ENCODER_PULLUP          false
#define PWM_WRAP                4095
#define STALL_PWM_THRESHOLD     0.50f
#define STALL_WINDOW_MS         200

// ===== Line Follower IR config =====
#define IR_LINE_WHITE_HIGH           0       
#define IR_LINE_THRESHOLD            2000    // Calibrate this
#define IR_LINE_PRINT_INTERVAL_MS    500     

// ===== Barcode Scanner IR config =====
#define IR_BARCODE_WHITE_HIGH        0
#define IR_BARCODE_THRESHOLD         2000    // Calibrate this
#define IR_BARCODE_SAMPLE_RATE_US    1000    // Sample every 1ms for barcode
#define BARCODE_MAX_BARS             50      // Max bars to detect

// ===== Ultrasonic Sensor config =====
#define ULTRASONIC_SPEED_CM_PER_US      0.0343f   // Speed of sound in cm/µs
#define ULTRASONIC_MIN_DISTANCE_CM      2.0f      // Minimum measurable distance
#define ULTRASONIC_MAX_DISTANCE_CM      400.0f    // Maximum measurable distance
#define ULTRASONIC_ECHO_TIMEOUT_US      25000     // Timeout waiting for echo
#define ULTRASONIC_PRINT_INTERVAL_MS    500       // Print interval

// ===== Magnetometer config =====
#define MAG_PRINT_INTERVAL_MS           500