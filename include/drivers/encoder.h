#ifndef ENCODER_H
#define ENCODER_H

#include "pico/stdlib.h"

typedef struct {
    uint pin_a;                      // single digital OUT from H206 module
    volatile uint32_t ticks;         // incremented in ISR on selected edge
} encoder_t;

// Initialize a single-channel encoder input.
void encoder_init(encoder_t* enc, uint pin_a, bool pull_up);

// Read ticks since last call and clear the counter.
uint32_t encoder_read_and_clear(encoder_t* enc);

#endif
