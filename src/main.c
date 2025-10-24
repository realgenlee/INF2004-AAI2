#include "pico/stdlib.h"
#include "config.h"
#include "fsm.h"

int main(void) {
    stdio_init_all();
    fsm_init();

    while (true) {
        fsm_step();
        sleep_ms(CONTROL_DT_MS);   // 100 Hz
    }
}
