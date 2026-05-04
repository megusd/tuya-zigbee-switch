#include "base_components/battery.h"
#include "base_components/network_indicator.h"
#include "hal/gpio.h"

#include <stddef.h>
#include <stdint.h>

/*
 * FANLIGHT_BUILD excludes config_parser.c, but several shared modules still
 * reference these globals. Provide minimal defaults for this fixed fan/light
 * device profile.
 */
network_indicator_t network_indicator = {
    .leds                        = { NULL, NULL, NULL, NULL },
    .has_dedicated_led           = 0,
    .manual_state_when_connected = 1,
};

uint8_t allow_simultaneous_latching_pulses = 0;
uint8_t relay_clusters_cnt = 0;

battery_t battery = {
    .pin          = HAL_INVALID_PIN,
    .voltage_min  = 2000,
    .voltage_max  = 3000,
    .charge_range = 200,
};
