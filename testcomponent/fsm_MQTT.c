#include <stdio.h>
#include "pico/stdlib.h"
#include "include/fsm.h"
#include "networking/wifi_mqtt.h"

//can connect, but not publish

static bool wifi_ok = false;   // separate Wi-Fi OK from MQTT OK

void fsm_init(void) {
    stdio_init_all();
    printf("[NET] bringup...\n");

    if (wifi_mqtt_init() && wifi_mqtt_connect()) {
        wifi_ok = true;
        // Try broker once during init (non-fatal if it fails)
        if (mqtt_connect_broker()) {
            printf("[MQTT] ✓ System ready\n");
        } else {
            printf("[MQTT] ! Broker not ready yet; will retry in loop\n");
        }
    } else {
        printf("[WiFi] × WiFi failed; check SSID/password/2.4GHz\n");
    }
}

void fsm_step(void) {
    if (wifi_ok) {
        // Drive MQTT keepalive + reconnects
        wifi_mqtt_poll();
    }
    // ...rest of your FSM work here...
}
