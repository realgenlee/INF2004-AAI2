#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/gpio.h"
#include "config.h"
#include "drivers/ir_line_follower.h"

void ir_line_follower_init(void) {
    // Note: adc_init() should only be called once globally
    // If not already initialized, call it here
    adc_gpio_init(IR_LINE_ADC_GPIO);
    
    // Initialize digital input
    gpio_init(IR_LINE_DIGITAL_GPIO);
    gpio_set_dir(IR_LINE_DIGITAL_GPIO, GPIO_IN);
    gpio_disable_pulls(IR_LINE_DIGITAL_GPIO);

    printf("Line Follower IR initialized: ADC on GP%d (ch%d), Digital on GP%d\n",
           IR_LINE_ADC_GPIO, IR_LINE_ADC_CHANNEL, IR_LINE_DIGITAL_GPIO);
}

uint16_t ir_line_read_adc_raw(void) {
    adc_select_input(IR_LINE_ADC_CHANNEL);
    return adc_read();
}

uint16_t ir_line_read_adc_averaged(int samples) {
    if (samples <= 1) return ir_line_read_adc_raw();
    uint32_t acc = 0;
    for (int i = 0; i < samples; ++i) {
        acc += ir_line_read_adc_raw();
    }
    return (uint16_t)(acc / (uint32_t)samples);
}

const char* ir_line_classify_surface(uint16_t raw, uint16_t threshold) {
    if (IR_LINE_WHITE_HIGH) {
        return (raw >= threshold) ? "WHITE" : "BLACK";
    } else {
        return (raw <= threshold) ? "WHITE" : "BLACK";
    }
}

bool ir_line_is_on_line(void) {
    uint16_t raw = ir_line_read_adc_averaged(4);
    const char* surface = ir_line_classify_surface(raw, IR_LINE_THRESHOLD);
    return (surface[0] == 'B'); // 'B' for BLACK
}

float ir_line_get_position_error(void) {
    // This is a simplified version for a single sensor
    // For multiple sensors, you'd calculate centroid
    uint16_t raw = ir_line_read_adc_averaged(4);
    
    // Normalize to -1.0 (far left/white) to +1.0 (far right/black)
    float normalized = (float)raw / 4095.0f;
    
    if (IR_LINE_WHITE_HIGH) {
        return (normalized - 0.5f) * 2.0f;
    } else {
        return (0.5f - normalized) * 2.0f;
    }
}

void ir_line_print_data(void) {
    uint16_t raw = ir_line_read_adc_averaged(8);
    const char* cls = ir_line_classify_surface(raw, IR_LINE_THRESHOLD);
    bool on_line = ir_line_is_on_line();
    
    printf("[LINE] raw=%u surface=%s on_line=%s\n",
           raw, cls, on_line ? "YES" : "NO");
}