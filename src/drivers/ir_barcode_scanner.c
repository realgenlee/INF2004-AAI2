#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/gpio.h"
#include "config.h"
#include "drivers/ir_barcode_scanner.h"

static barcode_data_t barcode_data;
static bool scanning_active = false;
static uint64_t last_transition_us = 0;
static bool last_state = false;

void ir_barcode_scanner_init(void) {
    adc_gpio_init(IR_BARCODE_ADC_GPIO);
    
    gpio_init(IR_BARCODE_DIGITAL_GPIO);
    gpio_set_dir(IR_BARCODE_DIGITAL_GPIO, GPIO_IN);
    gpio_disable_pulls(IR_BARCODE_DIGITAL_GPIO);

    memset(&barcode_data, 0, sizeof(barcode_data_t));
    
    printf("Barcode Scanner IR initialized: ADC on GP%d (ch%d), Digital on GP%d\n",
           IR_BARCODE_ADC_GPIO, IR_BARCODE_ADC_CHANNEL, IR_BARCODE_DIGITAL_GPIO);
}

void ir_barcode_start_scan(void) {
    memset(&barcode_data, 0, sizeof(barcode_data_t));
    scanning_active = true;
    last_transition_us = time_us_64();
    last_state = gpio_get(IR_BARCODE_DIGITAL_GPIO);
    printf("[BARCODE] Scan started\n");
}

void ir_barcode_stop_scan(void) {
    scanning_active = false;
    printf("[BARCODE] Scan stopped, detected %d bars\n", barcode_data.bar_count);
}

bool ir_barcode_is_scanning(void) {
    return scanning_active;
}

void ir_barcode_update(void) {
    if (!scanning_active) return;
    
    bool current_state = gpio_get(IR_BARCODE_DIGITAL_GPIO);
    uint64_t now_us = time_us_64();
    
    // Detect state change (black to white or white to black)
    if (current_state != last_state) {
        uint32_t width_us = (uint32_t)(now_us - last_transition_us);
        
        // Store the bar if we have space
        if (barcode_data.bar_count < BARCODE_MAX_BARS) {
            barcode_data.bars[barcode_data.bar_count].is_black = last_state;
            barcode_data.bars[barcode_data.bar_count].width_us = width_us;
            barcode_data.bar_count++;
        } else {
            // Buffer full, stop scanning
            barcode_data.scan_complete = true;
            scanning_active = false;
        }
        
        last_state = current_state;
        last_transition_us = now_us;
    }
}

barcode_data_t* ir_barcode_get_data(void) {
    return &barcode_data;
}

bool ir_barcode_decode(barcode_data_t* data) {
    // TODO: Implement your specific barcode decoding algorithm
    // This depends on the barcode format (Code 39, Code 128, UPC, etc.)
    // For now, return false (not implemented)
    
    if (data->bar_count < 10) {
        return false; // Not enough data
    }
    
    // Example placeholder decoding logic
    snprintf(data->decoded_value, sizeof(data->decoded_value), 
             "BARCODE_%d", data->bar_count);
    
    return true;
}

void ir_barcode_print_data(void) {
    printf("[BARCODE] Bars detected: %d\n", barcode_data.bar_count);
    
    for (int i = 0; i < barcode_data.bar_count && i < 10; i++) {
        printf("  Bar %d: %s, width=%lu us\n", 
               i, 
               barcode_data.bars[i].is_black ? "BLACK" : "WHITE",
               barcode_data.bars[i].width_us);
    }
    
    if (barcode_data.scan_complete) {
        printf("  Decoded: %s\n", barcode_data.decoded_value);
    }
}

uint16_t ir_barcode_read_adc_raw(void) {
    adc_select_input(IR_BARCODE_ADC_CHANNEL);
    return adc_read();
}