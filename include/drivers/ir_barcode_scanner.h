#ifndef IR_BARCODE_SCANNER_H
#define IR_BARCODE_SCANNER_H

#include "pico/stdlib.h"
#include <stdbool.h>

// Bar structure for barcode detection
typedef struct {
    bool is_black;      // true = black bar, false = white space
    uint32_t width_us;  // Duration in microseconds
} barcode_bar_t;

// Barcode data structure
typedef struct {
    barcode_bar_t bars[50];  // Array of detected bars
    uint8_t bar_count;        // Number of bars detected
    bool scan_complete;       // true when full barcode read
    char decoded_value[32];   // Decoded barcode string
} barcode_data_t;

// Initialize barcode scanner IR sensor
void ir_barcode_scanner_init(void);

// Start a new barcode scan
void ir_barcode_start_scan(void);

// Stop current barcode scan
void ir_barcode_stop_scan(void);

// Check if barcode scan is in progress
bool ir_barcode_is_scanning(void);

// Update barcode scan (call this in your control loop)
void ir_barcode_update(void);

// Get the latest barcode data
barcode_data_t* ir_barcode_get_data(void);

// Decode barcode (implement based on your barcode format)
bool ir_barcode_decode(barcode_data_t* data);

// Print barcode data for debugging
void ir_barcode_print_data(void);

// Read raw ADC value
uint16_t ir_barcode_read_adc_raw(void);

#endif