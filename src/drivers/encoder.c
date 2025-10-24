#include "drivers/encoder.h"
#include "hardware/gpio.h"

// Simple lookup from gpio number -> encoder instance pointer.
static encoder_t* _enc_for_gpio[32] = {0};

static void _encoder_shared_isr(uint gpio, uint32_t events) {
    if ((events & GPIO_IRQ_EDGE_RISE) && _enc_for_gpio[gpio]) {
        _enc_for_gpio[gpio]->ticks++;
    }
}

void encoder_init(encoder_t* enc, uint pin_a, bool pull_up) {
    enc->pin_a = pin_a;
    enc->ticks = 0;

    gpio_init(pin_a);
    gpio_set_dir(pin_a, GPIO_IN);
    if (pull_up) gpio_pull_up(pin_a);
    else         gpio_disable_pulls(pin_a);

    _enc_for_gpio[pin_a] = enc;
    gpio_set_irq_enabled_with_callback(pin_a, GPIO_IRQ_EDGE_RISE, true, &_encoder_shared_isr);
}

uint32_t encoder_read_and_clear(encoder_t* enc) {
    uint32_t t = enc->ticks;
    enc->ticks = 0;
    return t;
}
