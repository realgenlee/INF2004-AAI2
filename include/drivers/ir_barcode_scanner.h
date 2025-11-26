#ifndef IR_BARCODE_SCANNER_H
#define IR_BARCODE_SCANNER_H
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "pico/stdlib.h"

void ir_barcode_scanner_init(void);

void ir_barcode_update(void);

void ir_barcode_print_data(void);

uint16_t ir_barcode_read_adc_raw(void);

void ir_barcode_reset(void);

bool ir_barcode_has_char(void);

char ir_barcode_get_char(void);

void ir_barcode_clear_char(void);

#endif
