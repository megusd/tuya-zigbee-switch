/**
 * color_temp_cluster.h — ZCL Color Temperature Light (device type 0x010C)
 *
 * Manages three clusters on endpoint 2 of the fan+light device:
 *   - On/Off      (0x0006) — mapped to DP9
 *   - Level       (0x0008) — CurrentLevel 0-254 mapped from DP10 (0-100, step=2)
 *   - Color Ctrl  (0x0300) — ColorTemperatureMireds 154-370 mapped from DP11
 *
 * Scaling:
 *   Brightness : DP10 (0-100, step=2) → ZCL level = dp10 * 254 / 100
 *   Color temp : DP11 (0-100, step=2) is mapped to 154-370 mireds.
 *
 * Compile-time behavior switch for DP11 direction:
 *   TUYA_DP11_WARM_AT_100 = 0 (default): DP11=0 warm,   DP11=100 cool
 *   TUYA_DP11_WARM_AT_100 = 1          : DP11=0 cool,   DP11=100 warm
 */

#ifndef _COLOR_TEMP_CLUSTER_H_
#define _COLOR_TEMP_CLUSTER_H_

#include "consts.h"
#include "hal/zigbee.h"
#include <stdbool.h>
#include <stdint.h>

/* ZCL cluster IDs — also defined in consts.h */
#ifndef ZCL_CLUSTER_COLOR_CONTROL
#define ZCL_CLUSTER_COLOR_CONTROL     0x0300u
#endif

/* ZCL Color Control attribute IDs */
#define ZCL_ATTR_COLOR_TEMPERATURE    0x0007u
#define ZCL_ATTR_COLOR_MODE           0x0008u
#define ZCL_ATTR_COLOR_TEMP_MIN       0x400Bu
#define ZCL_ATTR_COLOR_TEMP_MAX       0x400Cu

/* ColorMode = 0x02 (color temperature) */
#define ZCL_COLOR_MODE_COLOR_TEMP     0x02u

/* ZCL Level Control attribute IDs */
#define ZCL_ATTR_CURRENT_LEVEL        0x0000u

/* ZCL Level Control commands — use names from consts.h if available */
#ifndef ZCL_CMD_LEVEL_MOVE_TO_LEVEL
#define ZCL_CMD_LEVEL_MOVE_TO_LEVEL          0x00u
#endif
#ifndef ZCL_CMD_LEVEL_MOVE_TO_LEVEL_WITH_ON_OFF
#define ZCL_CMD_LEVEL_MOVE_TO_LEVEL_WITH_ON_OFF  0x04u
#endif

/* ZCL Color Control commands */
#define ZCL_CMD_COLOR_MOVE_TO_COLOR_TEMP     0x0Au

/* Color temperature limits (mireds) */
#define LIGHT_COLOR_TEMP_WARM_MIREDS  370u    /* 2700 K */
#define LIGHT_COLOR_TEMP_COOL_MIREDS  154u    /* ~6493 K */

#ifndef TUYA_DP11_WARM_AT_100
#define TUYA_DP11_WARM_AT_100 0
#endif

/* Number of attributes per cluster */
#define ONOFF_CLUSTER_ATTR_COUNT      1u
#define LEVEL_CLUSTER_ATTR_COUNT      1u
#define COLOR_CLUSTER_ATTR_COUNT      4u
#define COLOR_TEMP_TOTAL_ATTRS        (ONOFF_CLUSTER_ATTR_COUNT + \
                                       LEVEL_CLUSTER_ATTR_COUNT + \
                                       COLOR_CLUSTER_ATTR_COUNT)

/* Payload for MoveToLevel command (ZCL spec 3.10.2.3.1) */
typedef struct {
    uint8_t  level;
    uint16_t transition_time;
} __attribute__((packed)) zcl_move_to_level_t;

/* Payload for MoveToColorTemperature command (ZCL spec 5.2.2.3.15) */
typedef struct {
    uint16_t color_temperature_mireds;
    uint16_t transition_time;
} __attribute__((packed)) zcl_move_to_color_temp_t;

typedef struct {
    uint8_t  endpoint;

    /* On/Off cluster state */
    uint8_t  on_off;           /* 0 or 1 */

    /* Level cluster state */
    uint8_t  current_level;    /* 0-254 */

    /* Color Control cluster state */
    uint16_t color_temperature; /* mireds, 154-370 */
    uint8_t  color_mode;        /* always ZCL_COLOR_MODE_COLOR_TEMP */
    uint16_t color_temp_min;
    uint16_t color_temp_max;

    /* Attribute storage tables — one per cluster */
    hal_zigbee_attribute onoff_attrs[ONOFF_CLUSTER_ATTR_COUNT];
    hal_zigbee_attribute level_attrs[LEVEL_CLUSTER_ATTR_COUNT];
    hal_zigbee_attribute color_attrs[COLOR_CLUSTER_ATTR_COUNT];

    /* Callbacks to forward commands to the MCU */
    void (*on_onoff_change)(bool on);
    void (*on_level_change)(uint8_t level_0_254);
    void (*on_color_temp_change)(uint16_t mireds);
} zigbee_color_temp_cluster;

/**
 * Register all three clusters (OnOff, Level, Color) on the endpoint.
 * Call before hal_zigbee_init().
 */
void color_temp_cluster_add_to_endpoint(zigbee_color_temp_cluster *cluster,
                                        hal_zigbee_endpoint *endpoint);

/**
 * Update On/Off state from DP9 and report to network.
 */
void color_temp_cluster_update_onoff(zigbee_color_temp_cluster *cluster,
                                     bool on);

/**
 * Update brightness from DP10 (0-100, step=2) and report to network.
 */
void color_temp_cluster_update_level_dp(zigbee_color_temp_cluster *cluster,
                                        uint8_t dp10_value);

/**
 * Update color temperature from DP11 (0-100, step=2) and report to network.
 */
void color_temp_cluster_update_color_temp_dp(zigbee_color_temp_cluster *cluster,
                                             uint8_t dp11_value);

#endif /* _COLOR_TEMP_CLUSTER_H_ */
