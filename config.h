#pragma once
#include "pico/stdlib.h"

// ==== pins configuration ====
#define PIN_M1_PWM   10
#define PIN_M1_DIR   11
#define PIN_M2_PWM    8
#define PIN_M2_DIR    9

#define PIN_ENC_LEFT   16
#define PIN_ENC_RIGHT  26

#define PIN_BTN_DIR    21
#define PIN_BTN_SPD    20

// ===== Control & hardware config =====
#define CTRL_HZ                 100
#define CONTROL_DT_MS           (1000/CTRL_HZ)
#define ENCODER_CPR             20.0f
#define WHEEL_DIAMETER_MM       65.0f
#define ENCODER_PULLUP          false   // H206 board usually provides pull-up
#define PWM_WRAP                4095    // 12-bit PWM
#define STALL_PWM_THRESHOLD     0.50f
#define STALL_WINDOW_MS         200
