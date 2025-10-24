#include <math.h>
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "drivers/motor.h"
#include "config.h"

static inline void pwm_init_pin(uint pin, uint16_t top) {
    gpio_set_function(pin, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(pin);
    pwm_config cfg = pwm_get_default_config();
    pwm_config_set_clkdiv(&cfg, 1.0f);
    pwm_config_set_wrap(&cfg, top);
    pwm_init(slice, &cfg, true);
}

static inline void motor_set_cmd(uint pwm_pin, uint dir_pin, float cmd) {
    bool fwd = (cmd >= 0.0f);
    float mag = fabsf(cmd);
    if (mag > 1.0f) mag = 1.0f;
    gpio_put(dir_pin, fwd ? 1 : 0);
    uint16_t level = (uint16_t)(mag * PWM_WRAP);
    pwm_set_gpio_level(pwm_pin, level);
}

void motor_init_all(void) {
    // Direction pins are simple GPIO outs
    gpio_init(PIN_M1_DIR); gpio_set_dir(PIN_M1_DIR, GPIO_OUT);
    gpio_init(PIN_M2_DIR); gpio_set_dir(PIN_M2_DIR, GPIO_OUT);

    // PWM pins use PWM slices
    pwm_init_pin(PIN_M1_PWM, PWM_WRAP);
    pwm_init_pin(PIN_M2_PWM, PWM_WRAP);

    motor_stop();
}

void motor_set(float left, float right) {
    motor_set_cmd(PIN_M1_PWM, PIN_M1_DIR, left);
    motor_set_cmd(PIN_M2_PWM, PIN_M2_DIR, right);
}

void motor_stop(void) {
    motor_set_cmd(PIN_M1_PWM, PIN_M1_DIR, 0.0f);
    motor_set_cmd(PIN_M2_PWM, PIN_M2_DIR, 0.0f);
}
