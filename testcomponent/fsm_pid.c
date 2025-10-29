#include "pico/stdlib.h"
#include "pico/time.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include "config.h"
#include "drivers/motor.h"
#include "drivers/encoder.h"
#include "control/pid.h"
#include "fsm.h"

// ---- Unified GPIO ISR for buttons + encoders ----
static volatile bool g_toggle_dir_req = false;
static volatile bool g_change_speed_req = false;
static volatile uint32_t g_last_dir_ms = 0;
static volatile uint32_t g_last_spd_ms = 0;

static inline uint32_t now_ms(void) { return to_ms_since_boot(get_absolute_time()); }

static void gpio_isr_unified(uint gpio, uint32_t events) {
    encoder_on_gpio_irq(gpio, events);
    uint32_t t = now_ms();
    if (gpio == BUTTON_DIR && (events & GPIO_IRQ_EDGE_FALL)) {
        if ((t - g_last_dir_ms) > DEBOUNCE_MS) { g_toggle_dir_req = true; g_last_dir_ms = t; }
    }
    if (gpio == BUTTON_SPD && (events & GPIO_IRQ_EDGE_FALL)) {
        if ((t - g_last_spd_ms) > DEBOUNCE_MS) { g_change_speed_req = true; g_last_spd_ms = t; }
    }
}

// ---- App state ----
static bool anticlockwise = false;     // your existing "direction" flag
static uint speed_pct = 70;            // UI setpoint 40..100%
static uint32_t last_print_ms = 0;

// PID controllers (one per wheel); outputs map to signed duty [-100..100]
static pid_t pid_r, pid_l;

// cached for velocity estimate
static int32_t prev_l_ticks = 0, prev_r_ticks = 0;

// convenience
static inline float dt_s(void) { return CONTROL_DT_MS / 1000.0f; }

void fsm_init(void) {
    stdio_init_all();
    motor_init_all();
    encoder_init();

    // Buttons
    gpio_init(BUTTON_DIR); gpio_set_dir(BUTTON_DIR, GPIO_IN); gpio_pull_up(BUTTON_DIR);
    gpio_init(BUTTON_SPD); gpio_set_dir(BUTTON_SPD, GPIO_IN); gpio_pull_up(BUTTON_SPD);

    // Register unified ISR
#if ENCODER_COUNT_BOTH_EDGES
    const uint32_t enc_edge = GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL;
#else
    const uint32_t enc_edge = GPIO_IRQ_EDGE_RISE;
#endif
    gpio_set_irq_enabled_with_callback(BUTTON_DIR, GPIO_IRQ_EDGE_FALL, true, &gpio_isr_unified);
    gpio_set_irq_enabled(BUTTON_SPD, GPIO_IRQ_EDGE_FALL, true);
    gpio_set_irq_enabled(LEFT_ENCODER_PIN,  enc_edge, true);
    gpio_set_irq_enabled(RIGHT_ENCODER_PIN, enc_edge, true);

    // PID setup
    pid_init(&pid_r, PID_KP_VEL, PID_KI_VEL, PID_KD_VEL, dt_s(),
             PID_OUT_MIN, PID_OUT_MAX, PID_INTEG_MIN, PID_INTEG_MAX);
    pid_init(&pid_l, PID_KP_VEL, PID_KI_VEL, PID_KD_VEL, dt_s(),
             PID_OUT_MIN, PID_OUT_MAX, PID_INTEG_MIN, PID_INTEG_MAX);

    // start in open-loop just to be safe (0 duty)
    motor_set_signed(0, 0);

    // seed tick baselines
    prev_l_ticks = (int32_t)encoder_left_count();
    prev_r_ticks = (int32_t)encoder_right_count();

    printf("FSM+PID ready. CPR=%.0f | wheel=%.1fmm | mm/tick=%.2f | dt=%.3fs\n",
           ENCODER_CPR, WHEEL_DIAMETER_MM, encoder_mm_per_tick(), dt_s());
    printf("GP21 toggle direction | GP20 randomize 40..100%%\n");
}

// helper: UI % -> target wheel speed (mm/s), signed by direction
static float target_mm_s_from_ui(void) {
    float sign = anticlockwise ? +1.0f : -1.0f;  // keep your current forward/back mapping
    return sign * (SPEED_MAX_MM_S * (speed_pct / 100.0f));
}

void fsm_step(void) {
    if (g_toggle_dir_req) {
        g_toggle_dir_req = false;
        anticlockwise = !anticlockwise;
        pid_reset(&pid_r); pid_reset(&pid_l);  // avoid kick
        printf("Direction: %s\n", anticlockwise ? "Anticlockwise" : "Clockwise");
    }
    if (g_change_speed_req) {
        g_change_speed_req = false;
        speed_pct = 40 + (rand() % 61);
        printf("Speed setpoint: %u%%\n", speed_pct);
    }

    // --- Measure wheel speeds from encoder deltas ---
    int32_t cur_l = (int32_t)encoder_left_count();
    int32_t cur_r = (int32_t)encoder_right_count();
    int32_t dlt   = cur_l - prev_l_ticks;
    int32_t drt   = cur_r - prev_r_ticks;
    prev_l_ticks = cur_l;
    prev_r_ticks = cur_r;

    float mm_per_tick = encoder_mm_per_tick();
    float meas_l_mm_s = (dlt * mm_per_tick) / dt_s();
    float meas_r_mm_s = (drt * mm_per_tick) / dt_s();

    // --- PID control: same target both wheels for now ---
    float tgt_mm_s = target_mm_s_from_ui();
    float u_r = pid_update(&pid_r, tgt_mm_s, meas_r_mm_s);  // [-100..100] duty %
    float u_l = pid_update(&pid_l, tgt_mm_s, meas_l_mm_s);

    // --- Drive motors with signed duty (handles inversion internally) ---
    motor_set_signed(u_r, u_l);

    // --- Periodic status print ---
    uint32_t t = now_ms();
    if (t - last_print_ms >= DISPLAY_INTERVAL_MS) {
        printf("--- Status ---\n");
        printf("Set: %.0f mm/s | L: %.0f | R: %.0f | Out L/R: %.0f / %.0f\n",
               tgt_mm_s, meas_l_mm_s, meas_r_mm_s, u_l, u_r);
        last_print_ms = t;
        // (optional) zero the long counters to keep numbers small:
        encoder_reset_counts();
        prev_l_ticks = prev_r_ticks = 0;
    }
}
