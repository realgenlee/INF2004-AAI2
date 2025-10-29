#pragma once
#include "pico/stdlib.h"

// ==== Motor pins ====
#define M1_A 8
#define M1_B 9
#define M2_A 10
#define M2_B 11

// ==== Encoder pins ====
#define LEFT_ENCODER_PIN   16
#define RIGHT_ENCODER_PIN  26

// ==== Direction calibration ====
// If one side runs opposite when given the same command, flip its flag to 1.
#define MOTOR_R_INVERT 0   // 0=normal, 1=invert RIGHT motor (M1)
#define MOTOR_L_INVERT 1   // 0=normal, 1=invert LEFT  motor (M2)

// ==== Button pins ====
#define BUTTON_DIR    21
#define BUTTON_SPD    20

// ==== PWM config ====
#define PWM_WRAP_8BIT 255
#define PWM_CLKDIV_DEFAULT 4.0f

// ==== App config ====
#define DEBOUNCE_MS 200
#define DISPLAY_INTERVAL_MS 3000  // print every 3 s

// ==== IR Sensor 1: Line Follower ====
#define IR_LINE_ADC_GPIO              28     // A0 -> GP28 (ADC2)
#define IR_LINE_ADC_CHANNEL           2      // ADC channel 2
#define IR_LINE_DIGITAL_GPIO          7      // D0 -> GP7

// ==== IR Sensor 2: Barcode Scanner ====
#define IR_BARCODE_ADC_GPIO           27     // A0 -> GP27 (ADC1)
#define IR_BARCODE_ADC_CHANNEL        1      // ADC channel 1
#define IR_BARCODE_DIGITAL_GPIO       6      // D0 -> GP6

// ==== Ultrasonic Sensor ====
#define ULTRASONIC_TRIG_GPIO          5      // Trigger pin
#define ULTRASONIC_ECHO_GPIO          4      // Echo pin

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
#define ENCODER_PULLUP          1
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
#define MAG_FILTER_SIZE                 8         // Moving average filter size (default 8 samples)
                                                  // Smaller value (4-6): Faster response, less smoothing
                                                  // Larger value (10-16): More smoothing, slower response

// ===== PID & velocity control =====
#define SPEED_MAX_MM_S            75.0f    //90.0f // tune to your robot's realistic max
#define PID_KP_VEL                0.7f     //0.5f  // start conservative, then tune
#define PID_KI_VEL                2.10f    //1.1f
#define PID_KD_VEL                0.001f    //0.001f    
#define PID_OUT_MIN              (-100.0f) // mapped to motor % duty (signed)
#define PID_OUT_MAX               (100.0f)
#define PID_INTEG_MIN            (-200.0f) // anti-windup clamp
#define PID_INTEG_MAX             (200.0f)

// ===== IMU Heading Correction =====
#define HEADING_KP                3.5f    //2.0f // Proportional gain for heading correction
#define HEADING_KI                0.1f    //0.1f  // Integral gain for heading correction  
#define HEADING_KD                2.0f    //0.0f   // Derivative gain for heading correction
#define HEADING_DEADZONE          1.0f     // Ignore errors < 2° to avoid jitter
#define MAX_HEADING_CORRECTION    15.0f    // Max ±20% duty adjustment for heading
