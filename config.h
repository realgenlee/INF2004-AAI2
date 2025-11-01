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
#define MOTOR_R_INVERT 1
#define MOTOR_L_INVERT 0

// ==== Button pins ====
#define BUTTON_DIR    21
#define BUTTON_SPD    20

// ==== PWM config ====
#define PWM_WRAP_8BIT 255
#define PWM_CLKDIV_DEFAULT 4.0f

// ==== App config ====
#define DEBOUNCE_MS 200
#define DISPLAY_INTERVAL_MS 3000

// ==== IR Sensor 1: Line Follower ====
#define IR_LINE_ADC_GPIO              28
#define IR_LINE_ADC_CHANNEL           2
#define IR_LINE_DIGITAL_GPIO          7

// ==== IR Sensor 2: Barcode Scanner ====
#define IR_BARCODE_ADC_GPIO           27
#define IR_BARCODE_ADC_CHANNEL        1
#define IR_BARCODE_DIGITAL_GPIO       6

// ==== Ultrasonic Sensor ====
#define ULTRASONIC_TRIG_GPIO          5
#define ULTRASONIC_ECHO_GPIO          4

// ==== I2C Magnetometer/Accelerometer (LSM303) ====
#define I2C_PORT                i2c1
#define I2C_SDA_PIN             2
#define I2C_SCL_PIN             3
#define I2C_BAUDRATE            100000
#define LSM303_ACC_ADDR         0x19
#define LSM303_MAG_ADDR         0x1E

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
#define IR_LINE_THRESHOLD            2000
#define IR_LINE_PRINT_INTERVAL_MS    500

// ===== Barcode Scanner IR config =====
#define IR_BARCODE_WHITE_HIGH        0
#define IR_BARCODE_THRESHOLD         2000
#define IR_BARCODE_SAMPLE_RATE_US    1000
#define BARCODE_MAX_BARS             50

// ===== Ultrasonic Sensor config =====
#define ULTRASONIC_SPEED_CM_PER_US      0.0343f
#define ULTRASONIC_MIN_DISTANCE_CM      2.0f
#define ULTRASONIC_MAX_DISTANCE_CM      400.0f
#define ULTRASONIC_ECHO_TIMEOUT_US      25000
#define ULTRASONIC_PRINT_INTERVAL_MS    500

// ===== Magnetometer config =====
#define MAG_PRINT_INTERVAL_MS           500
#define MAG_FILTER_SIZE                 10    // Increased for smoother readings

// ===== Motor Deadband Compensation =====
#define MOTOR_DEADBAND_PERCENT     8.0f   // Minimum PWM to overcome friction

// ===== Motor Characterization (PER WHEEL) =====
// Left motor (adjust these based on testing)
#define MOTOR_L_MIN_PWM      18.0f    // Minimum PWM to start moving
#define MOTOR_L_MAX_PWM      35.0f    // PWM at max speed - tune this!

// Right motor (adjust these based on testing)
#define MOTOR_R_MIN_PWM      18.0f    // Minimum PWM to start moving
#define MOTOR_R_MAX_PWM      35.0f    // PWM at max speed - tune this!

// ===== PID Gains (PER WHEEL) =====
#define SPEED_MAX_MM_S             350.0f

// Left wheel PID gains
#define PID_L_KP             0.3f     // Start with P-only
#define PID_L_KI             0.01f     // Add after Kp is tuned
#define PID_L_KD             0.0f     // Usually not needed

// Right wheel PID gains
#define PID_R_KP             0.35f     // Start with P-only
#define PID_R_KI             0.01f     // Add after Kp is tuned
#define PID_R_KD             0.0f     // Usually not needed

// PID output limits (same for both)
#define PID_OUT_MIN         (-40.0f)
#define PID_OUT_MAX          (40.0f)
#define PID_INTEG_MIN       (-5.0f)
#define PID_INTEG_MAX        (5.0f)

// ===== IMU Heading Correction =====
// These control how aggressively the robot corrects its heading to drive straight
#define HEADING_KP                2.5f     // Increased for stronger correction
#define HEADING_KI                0.4f    // Increased slightly
#define HEADING_KD                0.5f     // Increased damping
#define HEADING_DEADZONE          1.0f     // Keep sensitive
#define MAX_HEADING_CORRECTION   30.0f    // Increased max correction