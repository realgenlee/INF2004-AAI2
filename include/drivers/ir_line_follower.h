#ifndef IR_LINE_FOLLOWER_H
#define IR_LINE_FOLLOWER_H

#include "pico/stdlib.h"

// Initialize line follower IR sensor
void ir_line_follower_init(void);

// Read raw ADC value (0-4095)
uint16_t ir_line_read_adc_raw(void);

// Read averaged ADC value
uint16_t ir_line_read_adc_averaged(int samples);

// Classify surface as "WHITE" or "BLACK"
const char* ir_line_classify_surface(uint16_t raw, uint16_t threshold);

// Check if currently on line (returns true if on black line)
bool ir_line_is_on_line(void);

// Get line position error (-1.0 to +1.0, 0 = centered)
// This is useful for PID control
float ir_line_get_position_error(void);

// Print line follower data
void ir_line_print_data(void);

#endif