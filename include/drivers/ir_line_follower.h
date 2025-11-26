#ifndef IR_LINE_FOLLOWER_H
#define IR_LINE_FOLLOWER_H
#pragma once
#include "pico/stdlib.h"

void ir_line_follower_init(void);

uint16_t ir_line_read_adc_raw(void);

uint16_t ir_line_read_adc_averaged(int samples);

const char* ir_line_classify_surface(uint16_t raw, uint16_t threshold);

bool ir_line_is_on_line(void);

float ir_line_get_normalized_position(void);

void ir_line_print_data(void);

#endif