#ifndef DRIVERS_ENCODER_H
#define DRIVERS_ENCODER_H
#pragma once
#include <stdint.h>
#include <stdbool.h>

void encoder_init(void);
void encoder_reset_counts(void);
uint32_t encoder_left_count(void);
uint32_t encoder_right_count(void);
float encoder_mm_per_tick(void);
float encoder_ticks_to_mm(uint32_t ticks);
void encoder_on_gpio_irq(uint gpio, uint32_t events);

#endif