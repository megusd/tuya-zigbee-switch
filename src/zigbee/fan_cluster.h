/**
 * fan_cluster.h — ZCL Fan Control cluster (0x0202) for Tuya MCU fan
 *
 * Models an HVAC Fan Device (device type 0x0203) with six discrete speeds
 * mapped to DP1 (on/off) and DP3 (speed 1-6).
 *
 * FanMode encoding (ZCL spec Table 4.11):
 *   0x00 = Off          → DP1=false
 *   0x01 = Low          → DP1=true, DP3=1
 *   0x02 = Medium-Low   → DP1=true, DP3=2
 *   0x03 = Medium       → DP1=true, DP3=3
 *   0x04 = Medium-High  → DP1=true, DP3=4
 *   0x05 = High         → DP1=true, DP3=5
 *   0x06 = On (max)     → DP1=true, DP3=6
 *
 * FanModeSequence = 0x04 (Low/Med-Low/Med/Med-High/High)
 */

#ifndef _FAN_CLUSTER_H_
#define _FAN_CLUSTER_H_

#include "hal/zigbee.h"
#include <stdbool.h>
#include <stdint.h>

/* ZCL Fan Control cluster ID — also defined in consts.h */
#ifndef ZCL_CLUSTER_FAN_CONTROL
#define ZCL_CLUSTER_FAN_CONTROL    0x0202u
#endif

/* FanMode attribute (0x0000) values */
#define ZCL_FAN_MODE_OFF           0x00u
#define ZCL_FAN_MODE_LOW           0x01u
#define ZCL_FAN_MODE_MEDIUM_LOW    0x02u
#define ZCL_FAN_MODE_MEDIUM        0x03u
#define ZCL_FAN_MODE_MEDIUM_HIGH   0x04u
#define ZCL_FAN_MODE_HIGH          0x05u
#define ZCL_FAN_MODE_ON            0x06u  /* auto-selects "high" */

/* FanModeSequence attribute (0x0001) — 6-speed */
#define ZCL_FAN_MODE_SEQ_LOW_MED_HIGH_AUTO    0x04u

/* ZCL attribute IDs */
#define ZCL_ATTR_FAN_MODE          0x0000u
#define ZCL_ATTR_FAN_MODE_SEQUENCE 0x0001u

/* Number of attributes exposed */
#define FAN_CLUSTER_ATTR_COUNT     2u

typedef struct {
    uint8_t              endpoint;
    uint8_t              fan_mode;           /* current FanMode value */
    uint8_t              fan_mode_sequence;  /* always 0x04 */
    hal_zigbee_attribute attr_infos[FAN_CLUSTER_ATTR_COUNT];

    /* Callbacks fired when the host changes the fan mode */
    void (*on_mode_change)(uint8_t new_mode);
} zigbee_fan_cluster;

/**
 * Register the Fan Control cluster on the given endpoint.
 * Must be called before hal_zigbee_init().
 */
void fan_cluster_add_to_endpoint(zigbee_fan_cluster *cluster,
                                 hal_zigbee_endpoint *endpoint);

/**
 * Forward writable Fan Control attribute changes from the global dispatcher.
 */
void fan_cluster_callback_attr_write_trampoline(uint8_t endpoint,
                                                uint16_t attribute_id);

/**
 * Update the cached FanMode attribute and report it to the network.
 * Call this when the MCU sends a new fan state via UART.
 *
 * @param dp1_on     Current DP1 (fan on/off)
 * @param dp3_speed  Current DP3 (speed 1-6, ignored when dp1_on==false)
 */
void fan_cluster_update_from_dp(zigbee_fan_cluster *cluster,
                                bool dp1_on, uint8_t dp3_speed);

#endif /* _FAN_CLUSTER_H_ */
