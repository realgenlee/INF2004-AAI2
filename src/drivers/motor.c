#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "config.h"
#include "drivers/motor.h"

static inline uint get_pwm_slice(uint gpio) { return pwm_gpio_to_slice_num(gpio); }

static inline void set_side_AB(uint pinA, uint pinB, uint slice, uint16_t level, bool use_A_pwm) {
    if (level == 0) {
        // Brake/coast low
        gpio_set_function(pinA, GPIO_FUNC_SIO); gpio_put(pinA, 0);
        gpio_set_function(pinB, GPIO_FUNC_SIO); gpio_put(pinB, 0);
        return;
    }
    if (use_A_pwm) {
        // A = PWM, B = LOW
        gpio_set_function(pinB, GPIO_FUNC_SIO); gpio_put(pinB, 0);
        gpio_set_function(pinA, GPIO_FUNC_PWM); pwm_set_chan_level(slice, PWM_CHAN_A, level);
    } else {
        // B = PWM, A = LOW
        gpio_set_function(pinA, GPIO_FUNC_SIO); gpio_put(pinA, 0);
        gpio_set_function(pinB, GPIO_FUNC_PWM); pwm_set_chan_level(slice, PWM_CHAN_B, level);
    }
}

void motor_init_all(void) {
    // Safe state low
    gpio_init(M1_A); gpio_set_dir(M1_A, GPIO_OUT); gpio_put(M1_A, 0);
    gpio_init(M1_B); gpio_set_dir(M1_B, GPIO_OUT); gpio_put(M1_B, 0);
    gpio_init(M2_A); gpio_set_dir(M2_A, GPIO_OUT); gpio_put(M2_A, 0);
    gpio_init(M2_B); gpio_set_dir(M2_B, GPIO_OUT); gpio_put(M2_B, 0);

    // Configure both PWM slices (GP8/9 share slice 4, GP10/11 share slice 5)
    uint slice1 = get_pwm_slice(M1_A);
    uint slice2 = get_pwm_slice(M2_A);

    pwm_config cfg = pwm_get_default_config();
    pwm_config_set_clkdiv(&cfg, PWM_CLKDIV_DEFAULT);
    pwm_init(slice1, &cfg, true);
    pwm_init(slice2, &cfg, true);

    pwm_set_wrap(slice1, PWM_WRAP_8BIT);
    pwm_set_wrap(slice2, PWM_WRAP_8BIT);
}

void motor_set_speed_percent(uint speed_percent, bool anticlockwise) {
    if (speed_percent > 100) speed_percent = 100;
    uint16_t level = (PWM_WRAP_8BIT * speed_percent) / 100;

    uint slice1 = get_pwm_slice(M1_A); // right motor slice
    uint slice2 = get_pwm_slice(M2_A); // left motor slice

    pwm_set_wrap(slice1, PWM_WRAP_8BIT);
    pwm_set_wrap(slice2, PWM_WRAP_8BIT);

    // Decide for each side whether "A=PWM" (use_A_pwm=true) or "B=PWM" (use_A_pwm=false)
    // Start from global orientation: anticlockwise=false => A=PWM, true => B=PWM.
    bool use_A_pwm_right = !anticlockwise;
    bool use_A_pwm_left  = !anticlockwise;

    // Apply per-side inversion from config.h
    if (MOTOR_R_INVERT) use_A_pwm_right = !use_A_pwm_right;
    if (MOTOR_L_INVERT) use_A_pwm_left  = !use_A_pwm_left;

    set_side_AB(M1_A, M1_B, slice1, level, use_A_pwm_right);
    set_side_AB(M2_A, M2_B, slice2, level, use_A_pwm_left);
}

void motor_stop(void) {
    gpio_set_function(M1_A, GPIO_FUNC_SIO); gpio_put(M1_A, 0);
    gpio_set_function(M1_B, GPIO_FUNC_SIO); gpio_put(M1_B, 0);
    gpio_set_function(M2_A, GPIO_FUNC_SIO); gpio_put(M2_A, 0);
    gpio_set_function(M2_B, GPIO_FUNC_SIO); gpio_put(M2_B, 0);
}
