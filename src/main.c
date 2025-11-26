#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "pico/stdlib.h"
#include "pico/time.h"
#include "hardware/adc.h"

#include "config.h"
#include "networking/wifi_mqtt.h"

#include "drivers/motor.h"
#include "drivers/encoder.h"
#include "drivers/ir_line_follower.h"
#include "drivers/ir_barcode_scanner.h"
#include "drivers/magnetometer.h"
#include "drivers/ultrasonic.h"
#include "drivers/servo.h"

// ============================================================================
// CONFIGURATION - LINE FOLLOWING
// ============================================================================

#define LINE_FOLLOW_BASE_SPEED              21.0f

#define LINE_FOLLOW_MAX_CORRECTION_BASE     8.0f
#define LINE_FOLLOW_MAX_CORRECTION_BOOST    16.0f
#define LINE_FOLLOW_TIMEBOOST_THRESHOLD     3.0f

#define ZONE1_DEVIATION                     100.0f
#define ZONE1_GAIN                          0.7f
#define ZONE2_DEVIATION                     250.0f
#define ZONE2_GAIN                          1.5f
#define ZONE3_DEVIATION                     350.0f
#define ZONE3_GAIN                          3.3f
#define ZONE4_GAIN                          5.5f

#define WHITE_STUCK_TIME_MS                 70
#define BLACK_STUCK_TIME_MS                 70
#define MAX_WHITE_TIME_BOOST                15.0f
#define MAX_BLACK_TIME_BOOST                9.0f

// ============================================================================
// CONFIGURATION - ROBOT DIMENSIONS
// ============================================================================

#define ROBOT_WIDTH_CM                      12.8f
#define ROBOT_LENGTH_CM                     19.0f
#define ROBOT_HALF_WIDTH_CM                 (ROBOT_WIDTH_CM / 2.0f)
#define ROBOT_HALF_LENGTH_CM                (ROBOT_LENGTH_CM / 2.0f)

// ============================================================================
// CONFIGURATION - OBSTACLE AVOIDANCE
// ============================================================================

#define OBSTACLE_THRESHOLD_CM               20.0f
#define OBSTACLE_SAFETY_MARGIN_CM           2.0f

// Sweep parameters
#define SWEEP_START_ANGLE                   60
#define SWEEP_END_ANGLE                     140
#define SWEEP_STEP_ANGLE                    15
#define SWEEP_SAMPLES_PER_ANGLE             3

// Obstacle clear detection
#define OBSTACLE_CLEAR_THRESHOLD_CM         60.0f
#define MIN_ALONGSIDE_DISTANCE_MM           80.0f
#define MAX_ALONGSIDE_DISTANCE_MM           1000.0f

// Speeds
#define AVOID_TURN_SPEED                    30.0f
#define AVOID_DRIVE_SPEED                   32.0f

// Timing
#define ALONGSIDE_TIMEOUT_MS                10000
#define FLOOR_SWEEP_TIMEOUT_MS              20000

// Servo angles
#define SERVO_WATCH_RIGHT                   15
#define SERVO_WATCH_LEFT                    165

// Angle calculation
#define MIN_TURN_ANGLE_DEG                  30.0f
#define MAX_TURN_ANGLE_DEG                  80.0f
#define ANGLE_BOOST_FACTOR                  1.2f

// Side selection
#define SIDE_SELECTION_MARGIN_CM            8.0f

// Floor sweep parameters
// Turn 90 degree right first
#define FLOOR_SWEEP_START_OFFSET_DEG        90.0f
// Then sweep 180 degree total
#define FLOOR_SWEEP_TOTAL_ANGLE_DEG         180.0f
// Step size during sweep
#define FLOOR_SWEEP_STEP_DEG                15.0f
// How often to check for line
#define FLOOR_SWEEP_CHECK_INTERVAL_MS       200

// ============================================================================
// CONFIGURATION - MQTT
// ============================================================================

#ifndef CMD_TOPIC
#define CMD_TOPIC                           "cmd/robot1/#"
#endif

#define MQTT_PUBLISH_INTERVAL_MS            150

// ============================================================================
// TELEMETRY STRUCTURE
// ============================================================================

typedef struct {
    int      ultra_cm;
    uint16_t ir_line_raw;
    bool     ir_on_line;
    int32_t  left_ticks;
    int32_t  right_ticks;
    float    v_l_mm_s;
    float    v_r_mm_s;
    float    dist_l_mm;
    float    dist_r_mm;
    float    heading_deg;
    int16_t  mx;
    int16_t  my;
    int16_t  mz;
    char     barcode_char;
    int      bars_count;
} telemetry_t;

// ============================================================================
// FSM STATE DEFINITIONS
// ============================================================================

typedef enum {
    STATE_IDLE,
    STATE_LINE_FOLLOW,
    STATE_BARCODE_TURN,
    STATE_OBSTACLE_AVOID
} fsm_state_t;

static const char *STATE_NAMES[] = {
    "IDLE",
    "LINE_FOLLOW",
    "BARCODE_TURN",
    "OBSTACLE_AVOID"
};

// ============================================================================
// HELPER FUNCTIONS
// ============================================================================

// Get current time in milliseconds
static inline uint32_t now_ms(void) {
    return to_ms_since_boot(get_absolute_time());
}

// Clamp float between two limits
static inline float clampf(float x, float lo, float hi) {
    return (x < lo) ? lo : (x > hi ? hi : x);
}

// Convert degrees to radians
static inline float deg_to_rad(float deg) {
    return deg * (float) M_PI / 180.0f;
}

// Convert radians to degrees
static inline float rad_to_deg(float rad) {
    return rad * 180.0f / (float) M_PI;
}

// Normalize angle to [-180, 180]
static inline float normalize_angle(float angle) {
    while (angle > 180.0f) angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
    return angle;
}

// Normalize heading to [0, 360)
static inline float normalize_heading(float heading) {
    while (heading < 0.0f) heading += 360.0f;
    while (heading >= 360.0f) heading -= 360.0f;
    return heading;
}

// ============================================================================
// GLOBAL STATE
// ============================================================================

static fsm_state_t current_state = STATE_IDLE;
static uint32_t    state_entry_time = 0;

static volatile bool     g_start_button_pressed = false;
static volatile uint32_t g_last_button_ms = 0;

static float target_heading = 0.0f;
static bool  heading_locked = false;
static bool  imu_ready = false;

static char pending_barcode = '\0';
static int  pending_turn_dir = 0;
static bool pending_turn_armed = false;
static char detected_barcode = '\0';
static int  barcode_bars = 0;

// ============================================================================
// OBSTACLE AVOIDANCE STATE
// ============================================================================

// Initial distance to obstacle
static float obs_distance_a_cm = 0.0f;
// Side clearance
static float obs_clearance_b_cm = 0.0f;
// Diagonal distance
static float obs_diagonal_c_cm = 0.0f;
// Turn angle
static float obs_turn_angle_deg = 0.0f;

static float obs_left_clearance_cm = 0.0f;
static float obs_right_clearance_cm = 0.0f;

// -1 = left, +1 = right
static int   obs_chosen_side = 0;
// Heading before avoidance
static float obs_original_heading = 0.0f;
// Current target heading
static float obs_target_heading = 0.0f;

static uint32_t obs_drive_start_ticks = 0;
static int      obs_servo_watch_angle = 90;

// Floor sweep state
static float floor_sweep_start_heading = 0.0f;
static float floor_sweep_current_target = 0.0f;
static bool  floor_sweep_initial_turn_done = false;

// Internal obstacle phases
typedef enum {
    OA_PHASE_DETECT = 0,
    OA_PHASE_TURN_AWAY,
    OA_PHASE_DRIVE_DIAGONAL,
    OA_PHASE_TURN_PARALLEL,
    OA_PHASE_DRIVE_ALONGSIDE,
    OA_PHASE_RETURN_TURN_AWAY,
    OA_PHASE_RETURN_DRIVE_DIAGONAL,
    OA_PHASE_RETURN_TURN_STRAIGHT,
    OA_PHASE_FLOOR_SWEEP
} obstacle_phase_t;

static obstacle_phase_t obs_phase = OA_PHASE_DETECT;
static uint32_t         obs_phase_entry_time = 0;

// ============================================================================
// LINE FOLLOWING STATE
// ============================================================================

static float    ir_filtered = 0.0f;
static uint16_t ir_raw_latest = 0;
static bool     ir_on_line_latest = false;
static uint32_t white_time_start = 0;
static uint32_t black_time_start = 0;
static bool     was_on_white = false;
static bool     was_on_black = false;

#define VEL_WINDOW_MS                      150
static uint32_t vel_window_start = 0;
static uint32_t vel_l_start = 0;
static uint32_t vel_r_start = 0;
static float    vel_l = 0.0f;
static float    vel_r = 0.0f;
static float    total_dist_l_mm = 0.0f;
static float    total_dist_r_mm = 0.0f;

static float               cached_ultrasonic_cm = NAN;
static magnetometer_data_t cached_mag = {0};
static bool                cached_mag_valid = false;

static telemetry_t g_telemetry = {0};

static uint32_t last_control_ms = 0;

// ============================================================================
// ISR - GPIO CALLBACK
// ============================================================================

// Handle encoder and button interrupts
static void gpio_isr_callback(uint gpio, uint32_t events) {
    encoder_on_gpio_irq(gpio, events);

    uint32_t t = now_ms();
    if (gpio == BUTTON_DIR && (events & GPIO_IRQ_EDGE_FALL)) {
        if ((t - g_last_button_ms) > DEBOUNCE_MS) {
            g_start_button_pressed = true;
            g_last_button_ms = t;
        }
    }
}

// ============================================================================
// VELOCITY & DISTANCE TRACKING
// ============================================================================

// Update velocity estimates using encoder counts
static void update_velocity(uint32_t now) {
    uint32_t elapsed = now - vel_window_start;
    if (elapsed < VEL_WINDOW_MS) {
        return;
    }

    uint32_t current_left = encoder_left_count();
    uint32_t current_right = encoder_right_count();
    int32_t  ticks_left = (int32_t)(current_left - vel_l_start);
    int32_t  ticks_right = (int32_t)(current_right - vel_r_start);

    float mm_per_tick = encoder_mm_per_tick();
    float window_s = elapsed / 1000.0f;

    float inst_left = (ticks_left * mm_per_tick) / window_s;
    float inst_right = (ticks_right * mm_per_tick) / window_s;

    static bool  ema_init = false;
    static float ema_left = 0.0f;
    static float ema_right = 0.0f;
    const float  alpha = 0.40f;

    if (!ema_init) {
        ema_left = inst_left;
        ema_right = inst_right;
        ema_init = true;
    } else {
        ema_left = alpha * inst_left + (1.0f - alpha) * ema_left;
        ema_right = alpha * inst_right + (1.0f - alpha) * ema_right;
    }

    vel_l = ema_left;
    vel_r = ema_right;

    total_dist_l_mm += ticks_left * mm_per_tick;
    total_dist_r_mm += ticks_right * mm_per_tick;

    vel_l_start = current_left;
    vel_r_start = current_right;
    vel_window_start = now;
}

// Get travelled distance in mm since reference encoder ticks
static float get_distance_since_mm(uint32_t ref_ticks) {
    uint32_t current = (encoder_left_count() + encoder_right_count()) / 2;
    int32_t  delta = (int32_t)(current - ref_ticks);
    return delta * encoder_mm_per_tick();
}

// Get average encoder ticks between left and right
static uint32_t get_avg_encoder_ticks(void) {
    return (encoder_left_count() + encoder_right_count()) / 2;
}

// ============================================================================
// SENSOR CACHE UPDATE
// ============================================================================

// Update cached sensor readings once per control step
static void update_sensor_cache(void) {
    if (current_state == STATE_LINE_FOLLOW ||
        current_state == STATE_IDLE ||
        current_state == STATE_BARCODE_TURN ||
        current_state == STATE_OBSTACLE_AVOID) {
        cached_ultrasonic_cm = ultrasonic_measure_cm();
    }

    if (imu_ready) {
        cached_mag_valid = magnetometer_read_data(&cached_mag);
    } else {
        cached_mag_valid = false;
    }
}

// ============================================================================
// LINE FOLLOWING
// ============================================================================

// Compute gain based on deviation from threshold
static float calculate_adaptive_gain(float deviation) {
    float abs_dev = fabsf(deviation);

    if (abs_dev < ZONE1_DEVIATION) return ZONE1_GAIN;
    if (abs_dev < ZONE2_DEVIATION) return ZONE2_GAIN;
    if (abs_dev < ZONE3_DEVIATION) return ZONE3_GAIN;
    return ZONE4_GAIN;
}

// Compute time-based boost when stuck on white or black
static float calculate_time_boost(uint32_t now, bool on_white, bool on_black) {
    float boost = 1.0f;

    if (on_white) {
        if (!was_on_white) {
            white_time_start = now;
            was_on_white = true;
        }

        uint32_t duration = now - white_time_start;
        if (duration > WHITE_STUCK_TIME_MS) {
            boost = 1.0f + (float)(duration - WHITE_STUCK_TIME_MS) / 100.0f;
            if (boost > MAX_WHITE_TIME_BOOST) {
                boost = MAX_WHITE_TIME_BOOST;
            }
        }

        was_on_black = false;
        black_time_start = 0;
    } else if (on_black) {
        if (!was_on_black) {
            black_time_start = now;
            was_on_black = true;
        }

        uint32_t duration = now - black_time_start;
        if (duration > BLACK_STUCK_TIME_MS) {
            boost = 1.0f + (float)(duration - BLACK_STUCK_TIME_MS) / 100.0f;
            if (boost > MAX_BLACK_TIME_BOOST) {
                boost = MAX_BLACK_TIME_BOOST;
            }
        }

        was_on_white = false;
        white_time_start = 0;
    } else {
        was_on_white = false;
        was_on_black = false;
        white_time_start = 0;
        black_time_start = 0;
    }

    return boost;
}

// Compute motor PWM for line following
static void line_follow_control(uint32_t now, float *pwm_left, float *pwm_right) {
    uint16_t ir_raw = ir_line_read_adc_averaged(4);
    ir_raw_latest = ir_raw;

    ir_filtered = IR_SENSOR_FILTER_ALPHA * (float)ir_raw +
                  (1.0f - IR_SENSOR_FILTER_ALPHA) * ir_filtered;

    float deviation = ir_filtered - IR_LINE_THRESHOLD;

    bool on_white = (ir_filtered < IR_LINE_EDGE_LOW_LIMIT);
    bool on_black = (ir_filtered > IR_LINE_EDGE_HIGH_LIMIT);
    ir_on_line_latest = !on_white;

    float adaptive_gain = calculate_adaptive_gain(deviation);
    float time_boost = calculate_time_boost(now, on_white, on_black);
    float total_gain = adaptive_gain * time_boost;

    float max_correction = (time_boost >= LINE_FOLLOW_TIMEBOOST_THRESHOLD)
                           ? LINE_FOLLOW_MAX_CORRECTION_BOOST
                           : LINE_FOLLOW_MAX_CORRECTION_BASE;

    float correction = (deviation / 1000.0f) * total_gain * max_correction;
    correction = clampf(correction, -max_correction, max_correction);

    *pwm_left = LINE_FOLLOW_BASE_SPEED - correction;
    *pwm_right = LINE_FOLLOW_BASE_SPEED + correction;

    *pwm_left = clampf(*pwm_left, -50.0f, 50.0f);
    *pwm_right = clampf(*pwm_right, -50.0f, 50.0f);
}

// Reset internal line following state
static void reset_line_follow_state(void) {
    ir_filtered = (float)ir_line_read_adc_averaged(8);
    white_time_start = 0;
    black_time_start = 0;
    was_on_white = false;
    was_on_black = false;
}

// ============================================================================
// BARCODE TURN HELPER
// ============================================================================

// Map barcode character to left/right turn direction
static int get_turn_direction(char ch) {
    switch (ch) {
        case 'A': case 'C': case 'E': case 'G': case 'I':
        case 'K': case 'M': case 'O': case 'Q': case 'S':
        case 'U': case 'W': case 'Y':
            return +1;
        case 'B': case 'D': case 'F': case 'H': case 'J':
        case 'L': case 'N': case 'P': case 'R': case 'T':
        case 'V': case 'X': case 'Z':
            return -1;
        default:
            return 0;
    }
}

// ============================================================================
// IMU TURN HELPER
// ============================================================================

// Turn the robot to a target heading within a tolerance
static bool turn_to_heading(float target_heading_local, float tolerance) {
    if (!imu_ready) {
        return true;
    }

    magnetometer_data_t md;
    if (!magnetometer_read_data(&md)) {
        return false;
    }

    float error = normalize_angle(target_heading_local - md.heading);

    if (fabsf(error) <= tolerance) {
        motor_set_signed(0, 0);
        return true;
    }

    float turn_pwm = AVOID_TURN_SPEED;
    if (fabsf(error) < 20.0f) {
        turn_pwm = AVOID_TURN_SPEED * 0.7f;
    }

    if (error > 0) {
        motor_set_signed(-turn_pwm, +turn_pwm);
    } else {
        motor_set_signed(+turn_pwm, -turn_pwm);
    }

    return false;
}

// ============================================================================
// OBSTACLE AVOIDANCE - SWEEP & CALCULATION
// ============================================================================

// Sweep servo and ultrasonic to measure space on both sides of obstacle
static void perform_obstacle_sweep(void) {
    float left_distances[10];
    int   left_count = 0;
    float right_distances[10];
    int   right_count = 0;

    for (int angle = SWEEP_START_ANGLE; angle <= SWEEP_END_ANGLE; angle += SWEEP_STEP_ANGLE) {
        servo_set_angle(angle);
        sleep_ms(150);

        float distance_cm = ultrasonic_measure_averaged_cm(SWEEP_SAMPLES_PER_ANGLE);

        if (isnan(distance_cm)) {
            distance_cm = 999.0f;
        }

        if (angle > 90) {
            if (left_count < 10) {
                left_distances[left_count++] = distance_cm;
            }
        } else if (angle < 90) {
            if (right_count < 10) {
                right_distances[right_count++] = distance_cm;
            }
        }
    }

    servo_center();
    sleep_ms(100);

    float left_avg = 0.0f;
    float left_min = 999.0f;

    for (int i = 0; i < left_count; i++) {
        left_avg += left_distances[i];
        if (left_distances[i] < left_min) {
            left_min = left_distances[i];
        }
    }

    if (left_count > 0) {
        left_avg /= left_count;
    }

    float right_avg = 0.0f;
    float right_min = 999.0f;

    for (int i = 0; i < right_count; i++) {
        right_avg += right_distances[i];
        if (right_distances[i] < right_min) {
            right_min = right_distances[i];
        }
    }

    if (right_count > 0) {
        right_avg /= right_count;
    }

    obs_left_clearance_cm = left_min;
    obs_right_clearance_cm = right_min;

    float clearance_diff = fabsf(left_avg - right_avg);

    if (clearance_diff < SIDE_SELECTION_MARGIN_CM) {
        if (left_min > right_min + 3.0f) {
            obs_chosen_side = -1;
            obs_servo_watch_angle = SERVO_WATCH_RIGHT;
        } else if (right_min > left_min + 3.0f) {
            obs_chosen_side = +1;
            obs_servo_watch_angle = SERVO_WATCH_LEFT;
        } else {
            obs_chosen_side = -1;
            obs_servo_watch_angle = SERVO_WATCH_RIGHT;
        }
    } else {
        if (left_avg > right_avg) {
            obs_chosen_side = -1;
            obs_servo_watch_angle = SERVO_WATCH_RIGHT;
        } else {
            obs_chosen_side = +1;
            obs_servo_watch_angle = SERVO_WATCH_LEFT;
        }
    }
}

// Compute diagonal distance and turn angle for avoidance path
static void calculate_avoidance_geometry(void) {
    float a = obs_distance_a_cm;

    float obstacle_clearance = (obs_chosen_side > 0) ? obs_left_clearance_cm : obs_right_clearance_cm;
    if (obstacle_clearance > 80.0f) obstacle_clearance = 80.0f;
    if (obstacle_clearance < 8.0f) obstacle_clearance = 8.0f;

    float b = obstacle_clearance + ROBOT_HALF_WIDTH_CM + OBSTACLE_SAFETY_MARGIN_CM;
    obs_clearance_b_cm = b;

    float c = sqrtf(a * a + b * b);
    obs_diagonal_c_cm = c * 2.0f / 3.0f;

    float theta_rad = atanf(b / a);
    float theta_deg = rad_to_deg(theta_rad) * ANGLE_BOOST_FACTOR;
    theta_deg = clampf(theta_deg, MIN_TURN_ANGLE_DEG, MAX_TURN_ANGLE_DEG);
    obs_turn_angle_deg = theta_deg;
}

// ============================================================================
// STATE HANDLERS - IDLE, LINE FOLLOW, BARCODE TURN
// ============================================================================

// Enter idle state
static void state_idle_enter(void) {
    motor_set_signed(0, 0);
    g_start_button_pressed = false;
}

// Run idle state
static void state_idle_run(uint32_t now) {
    (void) now;
    motor_set_signed(0, 0);

    if (g_start_button_pressed) {
        g_start_button_pressed = false;
        current_state = STATE_LINE_FOLLOW;
    }
}

// Enter line follow state
static void state_line_follow_enter(void) {
    ir_barcode_reset();
    detected_barcode = '\0';
    pending_barcode = '\0';
    pending_turn_dir = 0;
    pending_turn_armed = false;
    reset_line_follow_state();

    if (imu_ready && cached_mag_valid) {
        target_heading = cached_mag.heading;
        heading_locked = true;
    }
}

// Run line follow state
static void state_line_follow_run(uint32_t now) {
    if (!isnan(cached_ultrasonic_cm) && cached_ultrasonic_cm < OBSTACLE_THRESHOLD_CM) {
        obs_distance_a_cm = cached_ultrasonic_cm;
        current_state = STATE_OBSTACLE_AVOID;
        return;
    }

    if (!pending_turn_armed && ir_barcode_has_char()) {
        char ch = ir_barcode_get_char();
        if (ch != '\0') {
            pending_barcode = ch;
            pending_turn_dir = get_turn_direction(ch);
            pending_turn_armed = true;
        }
        ir_barcode_clear_char();
    }

    if (pending_turn_armed) {
        bool line_on_black = (ir_filtered > IR_LINE_EDGE_HIGH_LIMIT);
        bool barcode_on_black = line_on_black;

        if (line_on_black && barcode_on_black) {
            detected_barcode = pending_barcode;
            barcode_bars = 29;
            pending_turn_armed = false;
            current_state = STATE_BARCODE_TURN;
            return;
        }
    }

    float pwm_left;
    float pwm_right;
    line_follow_control(now, &pwm_left, &pwm_right);
    motor_set_signed(pwm_right, pwm_left);
}

// Enter barcode turn state
static void state_barcode_turn_enter(void) {
    motor_set_signed(0, 0);
    sleep_ms(200);
}

// Run barcode turn state
static void state_barcode_turn_run(uint32_t now) {
    (void) now;

    int dir = get_turn_direction(detected_barcode);

    if (dir == 0 || !imu_ready || !heading_locked) {
        current_state = STATE_LINE_FOLLOW;
        return;
    }

    magnetometer_data_t md;
    if (!magnetometer_read_data(&md)) {
        current_state = STATE_LINE_FOLLOW;
        return;
    }

    float target_heading_local = normalize_heading(md.heading + (dir > 0 ? 90.0f : -90.0f));

    uint32_t turn_start = now_ms();
    bool     done = false;

    while (!done && (now_ms() - turn_start) < TURN_TIMEOUT_MS) {
        if (magnetometer_read_data(&md)) {
            float error = normalize_angle(target_heading_local - md.heading);

            if (fabsf(error) <= HEADING_TOLERANCE) {
                done = true;
                break;
            }

            float turn_pwm = (fabsf(error) < APPROACH_THRESHOLD) ? MIN_TURN_SPEED : MAX_TURN_SPEED;
            if (error > 0) {
                motor_set_signed(-turn_pwm, +turn_pwm);
            } else {
                motor_set_signed(+turn_pwm, -turn_pwm);
            }
        }
        sleep_ms(10);
    }

    motor_set_signed(0, 0);
    target_heading = target_heading_local;
    sleep_ms(200);

    ir_barcode_reset();
    detected_barcode = '\0';
    reset_line_follow_state();

    current_state = STATE_LINE_FOLLOW;
}

// ============================================================================
// OBSTACLE AVOIDANCE
// ============================================================================

// Forward declarations of per-phase handlers
static void oa_enter_detect(uint32_t now);
static void oa_enter_turn_away(uint32_t now);
static void oa_enter_drive_diagonal(uint32_t now);
static void oa_enter_turn_parallel(uint32_t now);
static void oa_enter_drive_alongside(uint32_t now);
static void oa_enter_return_turn_away(uint32_t now);
static void oa_enter_return_drive_diagonal(uint32_t now);
static void oa_enter_return_turn_straight(uint32_t now);
static void oa_enter_floor_sweep(uint32_t now);

static void oa_run_detect(uint32_t now);
static void oa_run_turn_away(uint32_t now);
static void oa_run_drive_diagonal(uint32_t now);
static void oa_run_turn_parallel(uint32_t now);
static void oa_run_drive_alongside(uint32_t now);
static void oa_run_return_turn_away(uint32_t now);
static void oa_run_return_drive_diagonal(uint32_t now);
static void oa_run_return_turn_straight(uint32_t now);
static void oa_run_floor_sweep(uint32_t now);

// Change internal obstacle phase and call its enter function
static void obstacle_avoid_set_phase(obstacle_phase_t phase, uint32_t now) {
    obs_phase = phase;
    obs_phase_entry_time = now;

    switch (phase) {
        case OA_PHASE_DETECT:                 oa_enter_detect(now); break;
        case OA_PHASE_TURN_AWAY:              oa_enter_turn_away(now); break;
        case OA_PHASE_DRIVE_DIAGONAL:         oa_enter_drive_diagonal(now); break;
        case OA_PHASE_TURN_PARALLEL:          oa_enter_turn_parallel(now); break;
        case OA_PHASE_DRIVE_ALONGSIDE:        oa_enter_drive_alongside(now); break;
        case OA_PHASE_RETURN_TURN_AWAY:       oa_enter_return_turn_away(now); break;
        case OA_PHASE_RETURN_DRIVE_DIAGONAL:  oa_enter_return_drive_diagonal(now); break;
        case OA_PHASE_RETURN_TURN_STRAIGHT:   oa_enter_return_turn_straight(now); break;
        case OA_PHASE_FLOOR_SWEEP:            oa_enter_floor_sweep(now); break;
    }
}

// Enter detect phase
static void oa_enter_detect(uint32_t now) {
    (void) now;
    motor_set_signed(0, 0);
    sleep_ms(200);
}

// Enter turn away phase
static void oa_enter_turn_away(uint32_t now) {
    (void) now;

    float turn_deg = obs_turn_angle_deg * obs_chosen_side;
    obs_target_heading = normalize_heading(obs_original_heading + turn_deg);
}

// Enter drive diagonal phase
static void oa_enter_drive_diagonal(uint32_t now) {
    (void) now;
    obs_drive_start_ticks = get_avg_encoder_ticks();
}

// Enter turn parallel phase
static void oa_enter_turn_parallel(uint32_t now) {
    (void) now;
    obs_target_heading = obs_original_heading;
}

// Enter drive alongside phase
static void oa_enter_drive_alongside(uint32_t now) {
    (void) now;
    obs_drive_start_ticks = get_avg_encoder_ticks();
}

// Enter return turn away phase
static void oa_enter_return_turn_away(uint32_t now) {
    (void) now;

    float turn_deg = -(obs_turn_angle_deg * 2.0f / 3.0f) * obs_chosen_side;
    obs_target_heading = normalize_heading(obs_original_heading + turn_deg);

    servo_center();
}

// Enter return drive diagonal phase
static void oa_enter_return_drive_diagonal(uint32_t now) {
    (void) now;
    obs_drive_start_ticks = get_avg_encoder_ticks();
}

// Enter return straight phase
static void oa_enter_return_turn_straight(uint32_t now) {
    (void) now;
    obs_target_heading = obs_original_heading;
}

// Enter floor sweep phase
static void oa_enter_floor_sweep(uint32_t now) {
    (void) now;

    if (imu_ready && cached_mag_valid) {
        floor_sweep_start_heading = obs_original_heading;
        floor_sweep_current_target = normalize_heading(
            floor_sweep_start_heading + FLOOR_SWEEP_START_OFFSET_DEG
        );
    }

    floor_sweep_initial_turn_done = false;
}

// Run detect phase
static void oa_run_detect(uint32_t now) {
    float distance = ultrasonic_measure_averaged_cm(5);

    if (isnan(distance) || distance >= OBSTACLE_THRESHOLD_CM) {
        motor_set_signed(0, 0);
        current_state = STATE_LINE_FOLLOW;
        return;
    }

    obs_distance_a_cm = distance;

    if (imu_ready && cached_mag_valid) {
        obs_original_heading = cached_mag.heading;
    }

    perform_obstacle_sweep();
    calculate_avoidance_geometry();

    obstacle_avoid_set_phase(OA_PHASE_TURN_AWAY, now);
}

// Run turn away phase
static void oa_run_turn_away(uint32_t now) {
    if ((now - obs_phase_entry_time) > TURN_TIMEOUT_MS) {
        motor_set_signed(0, 0);
        obstacle_avoid_set_phase(OA_PHASE_DRIVE_DIAGONAL, now);
        return;
    }

    if (turn_to_heading(obs_target_heading, 5.0f)) {
        motor_set_signed(0, 0);
        sleep_ms(150);
        obs_drive_start_ticks = get_avg_encoder_ticks();
        obstacle_avoid_set_phase(OA_PHASE_DRIVE_DIAGONAL, now);
    }
}

// Run drive diagonal phase
static void oa_run_drive_diagonal(uint32_t now) {
    float target_mm = (obs_diagonal_c_cm + ROBOT_HALF_LENGTH_CM) * 10.0f;
    float driven_mm = get_distance_since_mm(obs_drive_start_ticks);

    if (driven_mm >= target_mm) {
        motor_set_signed(0, 0);
        sleep_ms(100);
        obstacle_avoid_set_phase(OA_PHASE_TURN_PARALLEL, now);
        return;
    }

    if ((now - obs_phase_entry_time) > 6000) {
        motor_set_signed(0, 0);
        obstacle_avoid_set_phase(OA_PHASE_TURN_PARALLEL, now);
        return;
    }

    motor_set_signed(AVOID_DRIVE_SPEED, AVOID_DRIVE_SPEED);
}

// Run turn parallel phase
static void oa_run_turn_parallel(uint32_t now) {
    if ((now - obs_phase_entry_time) > TURN_TIMEOUT_MS) {
        motor_set_signed(0, 0);
        obstacle_avoid_set_phase(OA_PHASE_DRIVE_ALONGSIDE, now);
        return;
    }

    if (turn_to_heading(obs_target_heading, 5.0f)) {
        motor_set_signed(0, 0);
        sleep_ms(150);

        servo_set_angle(obs_servo_watch_angle);
        sleep_ms(250);

        obs_drive_start_ticks = get_avg_encoder_ticks();
        obstacle_avoid_set_phase(OA_PHASE_DRIVE_ALONGSIDE, now);
    }
}

// Run drive alongside phase
static void oa_run_drive_alongside(uint32_t now) {
    if ((now - obs_phase_entry_time) > ALONGSIDE_TIMEOUT_MS) {
        motor_set_signed(0, 0);
        obstacle_avoid_set_phase(OA_PHASE_RETURN_TURN_AWAY, now);
        return;
    }

    float driven_mm = get_distance_since_mm(obs_drive_start_ticks);

    if (driven_mm >= MAX_ALONGSIDE_DISTANCE_MM) {
        motor_set_signed(0, 0);
        obstacle_avoid_set_phase(OA_PHASE_RETURN_TURN_AWAY, now);
        return;
    }

    float side_distance_cm = ultrasonic_measure_cm();

    if (driven_mm >= MIN_ALONGSIDE_DISTANCE_MM) {
        if (isnan(side_distance_cm) || side_distance_cm > OBSTACLE_CLEAR_THRESHOLD_CM) {
            motor_set_signed(0, 0);
            sleep_ms(100);

            obstacle_avoid_set_phase(OA_PHASE_RETURN_TURN_AWAY, now);
            return;
        }
    }

    motor_set_signed(AVOID_DRIVE_SPEED, AVOID_DRIVE_SPEED);
}

// Run return turn away phase
static void oa_run_return_turn_away(uint32_t now) {
    if ((now - obs_phase_entry_time) > TURN_TIMEOUT_MS) {
        motor_set_signed(0, 0);
        obstacle_avoid_set_phase(OA_PHASE_RETURN_DRIVE_DIAGONAL, now);
        return;
    }

    if (turn_to_heading(obs_target_heading, 5.0f)) {
        motor_set_signed(0, 0);
        sleep_ms(150);
        obs_drive_start_ticks = get_avg_encoder_ticks();
        obstacle_avoid_set_phase(OA_PHASE_RETURN_DRIVE_DIAGONAL, now);
    }
}

// Run return drive diagonal phase
static void oa_run_return_drive_diagonal(uint32_t now) {
    float target_mm = obs_diagonal_c_cm * 10.0f;
    float driven_mm = get_distance_since_mm(obs_drive_start_ticks);

    if (driven_mm >= target_mm) {
        motor_set_signed(0, 0);
        sleep_ms(100);
        obstacle_avoid_set_phase(OA_PHASE_RETURN_TURN_STRAIGHT, now);
        return;
    }

    if ((now - obs_phase_entry_time) > 6000) {
        motor_set_signed(0, 0);
        obstacle_avoid_set_phase(OA_PHASE_RETURN_TURN_STRAIGHT, now);
        return;
    }

    motor_set_signed(AVOID_DRIVE_SPEED, AVOID_DRIVE_SPEED);
}

// Run return straight phase
static void oa_run_return_turn_straight(uint32_t now) {
    if ((now - obs_phase_entry_time) > TURN_TIMEOUT_MS) {
        motor_set_signed(0, 0);
        obstacle_avoid_set_phase(OA_PHASE_FLOOR_SWEEP, now);
        return;
    }

    if (turn_to_heading(obs_target_heading, 5.0f)) {
        motor_set_signed(0, 0);
        sleep_ms(150);
        obstacle_avoid_set_phase(OA_PHASE_FLOOR_SWEEP, now);
    }
}

// Run floor sweep phase to refind line
static void oa_run_floor_sweep(uint32_t now) {
    if ((now - obs_phase_entry_time) > FLOOR_SWEEP_TIMEOUT_MS) {
        motor_set_signed(0, 0);
        current_state = STATE_LINE_FOLLOW;
        return;
    }

    uint16_t ir_raw = ir_line_read_adc_averaged(4);

    if (ir_raw > (IR_LINE_EDGE_LOW_LIMIT + 150)) {
        motor_set_signed(0, 0);
        sleep_ms(200);
        current_state = STATE_LINE_FOLLOW;
        return;
    }

    if (!floor_sweep_initial_turn_done) {
        if (turn_to_heading(floor_sweep_current_target, 7.0f)) {
            floor_sweep_initial_turn_done = true;

            floor_sweep_current_target = normalize_heading(
                floor_sweep_start_heading +
                FLOOR_SWEEP_START_OFFSET_DEG -
                FLOOR_SWEEP_TOTAL_ANGLE_DEG
            );

            motor_set_signed(0, 0);
            sleep_ms(100);
        }
        return;
    }

    if (turn_to_heading(floor_sweep_current_target, 3.0f)) {
        motor_set_signed(0, 0);
        current_state = STATE_LINE_FOLLOW;
        return;
    }

    (void) ir_raw;
}

// Enter combined obstacle avoidance state
static void state_obstacle_avoid_enter(uint32_t now) {
    obstacle_avoid_set_phase(OA_PHASE_DETECT, now);
}

// Run combined obstacle avoidance state
static void state_obstacle_avoid_run(uint32_t now) {
    switch (obs_phase) {
        case OA_PHASE_DETECT:                 oa_run_detect(now); break;
        case OA_PHASE_TURN_AWAY:              oa_run_turn_away(now); break;
        case OA_PHASE_DRIVE_DIAGONAL:         oa_run_drive_diagonal(now); break;
        case OA_PHASE_TURN_PARALLEL:          oa_run_turn_parallel(now); break;
        case OA_PHASE_DRIVE_ALONGSIDE:        oa_run_drive_alongside(now); break;
        case OA_PHASE_RETURN_TURN_AWAY:       oa_run_return_turn_away(now); break;
        case OA_PHASE_RETURN_DRIVE_DIAGONAL:  oa_run_return_drive_diagonal(now); break;
        case OA_PHASE_RETURN_TURN_STRAIGHT:   oa_run_return_turn_straight(now); break;
        case OA_PHASE_FLOOR_SWEEP:            oa_run_floor_sweep(now); break;
    }
}

// ============================================================================
// FSM MAIN STEP
// ============================================================================

// Run one FSM step and update telemetry
static void fsm_step(uint32_t now) {
    if ((now - last_control_ms) < 5) {
        return;
    }
    last_control_ms = now;

    update_velocity(now);
    update_sensor_cache();

    if (current_state == STATE_LINE_FOLLOW) {
        ir_barcode_update();
    }

    static fsm_state_t previous_state = STATE_IDLE;
    if (current_state != previous_state) {
        state_entry_time = now;

        switch (current_state) {
            case STATE_IDLE:
                state_idle_enter();
                break;
            case STATE_LINE_FOLLOW:
                state_line_follow_enter();
                break;
            case STATE_BARCODE_TURN:
                state_barcode_turn_enter();
                break;
            case STATE_OBSTACLE_AVOID:
                state_obstacle_avoid_enter(now);
                break;
        }

        previous_state = current_state;
    }

    switch (current_state) {
        case STATE_IDLE:
            state_idle_run(now);
            break;
        case STATE_LINE_FOLLOW:
            state_line_follow_run(now);
            break;
        case STATE_BARCODE_TURN:
            state_barcode_turn_run(now);
            break;
        case STATE_OBSTACLE_AVOID:
            state_obstacle_avoid_run(now);
            break;
    }

    g_telemetry.ultra_cm = isnan(cached_ultrasonic_cm) ? -1 : (int)cached_ultrasonic_cm;
    g_telemetry.ir_line_raw = ir_raw_latest;
    g_telemetry.ir_on_line = ir_on_line_latest;
    g_telemetry.left_ticks = (int32_t)encoder_left_count();
    g_telemetry.right_ticks = (int32_t)encoder_right_count();
    g_telemetry.v_l_mm_s = vel_l;
    g_telemetry.v_r_mm_s = vel_r;
    g_telemetry.dist_l_mm = total_dist_l_mm;
    g_telemetry.dist_r_mm = total_dist_r_mm;

    if (cached_mag_valid) {
        g_telemetry.heading_deg = cached_mag.heading;
        g_telemetry.mx = cached_mag.x;
        g_telemetry.my = cached_mag.y;
        g_telemetry.mz = cached_mag.z;
    } else {
        g_telemetry.heading_deg = NAN;
        g_telemetry.mx = 0;
        g_telemetry.my = 0;
        g_telemetry.mz = 0;
    }

    g_telemetry.barcode_char = detected_barcode;
    g_telemetry.bars_count = barcode_bars;
}

// ============================================================================
// INITIALIZATION
// ============================================================================

// Initialise motors, sensors, interrupts, and IMU
static void system_init(void) {
    motor_init_all();
    encoder_init();
    ir_line_follower_init();
    ir_barcode_scanner_init();
    ultrasonic_init();
    servo_init();

    if (magnetometer_init()) {
        imu_ready = true;

        for (int i = 0; i < 10; i++) {
            magnetometer_data_t md;
            magnetometer_read_data(&md);
            sleep_ms(50);
        }

        magnetometer_data_t md;
        if (magnetometer_read_data(&md)) {
            target_heading = md.heading;
            heading_locked = true;
        }
    } else {
        imu_ready = false;
    }

    gpio_init(BUTTON_DIR);
    gpio_set_dir(BUTTON_DIR, GPIO_IN);
    gpio_pull_up(BUTTON_DIR);

#if ENCODER_COUNT_BOTH_EDGES
    const uint32_t enc_edge = GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL;
#else
    const uint32_t enc_edge = GPIO_IRQ_EDGE_RISE;
#endif

    gpio_set_irq_enabled_with_callback(BUTTON_DIR, GPIO_IRQ_EDGE_FALL, true, &gpio_isr_callback);
    gpio_set_irq_enabled(LEFT_ENCODER_PIN, enc_edge, true);
    gpio_set_irq_enabled(RIGHT_ENCODER_PIN, enc_edge, true);

    motor_set_signed(0, 0);
    vel_l_start = encoder_left_count();
    vel_r_start = encoder_right_count();
    vel_window_start = now_ms();
    last_control_ms = now_ms();
    ir_filtered = (float)ir_line_read_adc_averaged(8);
}

// ============================================================================
// MQTT
// ============================================================================

static void mqtt_message_callback(const char *topic, const uint8_t *payload, size_t len) {
    (void) topic;
    (void) payload;
    (void) len;
}

// Publish key telemetry over MQTT
static void publish_telemetry(void) {
    telemetry_t tm = g_telemetry;
    char        msg[96];

    snprintf(msg, sizeof(msg),
             "{\"ultrasonic\":%d,\"ir_line\":%d,\"on_line\":%d}",
             tm.ultra_cm, tm.ir_line_raw, tm.ir_on_line ? 1 : 0);
    mqtt_publish_text("robot/sensors", msg, 0, false);

    snprintf(msg, sizeof(msg),
             "{\"ls\":%d,\"rs\":%d}",
             (int)(tm.v_l_mm_s * 0.1f), (int)(tm.v_r_mm_s * 0.1f));
    mqtt_publish_text("robot/telemetry", msg, 0, false);

    snprintf(msg, sizeof(msg),
             "{\"heading\":%d}",
             isnan(tm.heading_deg) ? -1000 : (int)tm.heading_deg);
    mqtt_publish_text("robot/compass", msg, 0, false);

    snprintf(msg, sizeof(msg),
             "{\"state\":\"%s\"}", STATE_NAMES[current_state]);
    mqtt_publish_text("robot/state", msg, 0, false);
}

// ============================================================================
// MAIN
// ============================================================================

int main(void) {
    stdio_init_all();
    sleep_ms(300);

    adc_init();
    system_init();

    if (!wifi_mqtt_init()) {
        printf("[WiFi] Init failed\n");
    }
    if (!wifi_mqtt_connect()) {
        printf("[WiFi] Connect failed\n");
    }

    mqtt_set_message_cb(mqtt_message_callback);
    if (!mqtt_connect_broker()) {
        printf("[MQTT] Broker connect failed\n");
    }

    bool     mqtt_subscribed = false;
    uint32_t last_mqtt_pub = 0;

    printf("\n=== Robot Ready ===\n");
    printf("Press the button to start.\n\n");

    while (true) {
        wifi_mqtt_poll();

        if (mqtt_is_connected() && !mqtt_subscribed) {
            if (mqtt_subscribe_topic(CMD_TOPIC, 0)) {
                mqtt_subscribed = true;
            }
        }

        if (!mqtt_is_connected()) {
            mqtt_subscribed = false;
        }

        uint32_t now = now_ms();
        fsm_step(now);

        if (mqtt_is_connected() &&
            (now - last_mqtt_pub) >= MQTT_PUBLISH_INTERVAL_MS) {
            last_mqtt_pub = now;
            publish_telemetry();
        }

        tight_loop_contents();
    }
}