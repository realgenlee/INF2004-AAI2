#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"
#include "config.h"
#include "drivers/servo.h"

#define SERVO_PWM_FREQ_HZ      50
#define SERVO_MIN_PULSE_US     500    // 0° position
#define SERVO_MAX_PULSE_US     2500   // 180° position
#define SERVO_PERIOD_US        20000  // 20ms period (50Hz)

static uint8_t current_angle = 90;
static uint servo_slice;
static uint servo_channel;

void servo_init(void) {
    gpio_set_function(SERVO_GPIO, GPIO_FUNC_PWM);
    
    servo_slice = pwm_gpio_to_slice_num(SERVO_GPIO);
    servo_channel = pwm_gpio_to_channel(SERVO_GPIO);
    
    uint32_t clock_freq = clock_get_hz(clk_sys);
    uint32_t divider = 64;
    uint32_t wrap = (clock_freq / (divider * SERVO_PWM_FREQ_HZ)) - 1;
    
    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, divider);
    pwm_config_set_wrap(&config, wrap);
    
    pwm_init(servo_slice, &config, true);
    
    servo_set_angle(100);
}

void servo_set_angle(uint8_t angle) {
    if (angle > 180) {
        angle = 180;
    }
    
    uint32_t pulse_us = SERVO_MIN_PULSE_US + 
                        ((SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US) * angle) / 180;
    
    uint32_t wrap = pwm_hw->slice[servo_slice].top;
    
    uint32_t level = (pulse_us * (wrap + 1)) / SERVO_PERIOD_US;
    
    pwm_set_chan_level(servo_slice, servo_channel, level);
    
    current_angle = angle;
}

uint8_t servo_get_angle(void) {
    return current_angle;
}

void servo_center(void) {
    servo_set_angle(100);
}