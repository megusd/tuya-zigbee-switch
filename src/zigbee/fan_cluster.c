/**
 * fan_cluster.c — ZCL Fan Control cluster (0x0202) implementation
 *
 * FanMode is a writable ENUM8 attribute.  Coordinators (Z2M, ZHA) write to it
 * directly; there are no dedicated cluster commands in the ZCL spec for fan
 * mode changes.  The attribute-write callback translates the new value to the
 * Tuya MCU DP1/DP3 pair via the registered on_mode_change hook.
 */

#include "fan_cluster.h"
#include "cluster_common.h"
#include "consts.h"
#include "hal/zigbee.h"
#include "hal/printf_selector.h"

#include <stdbool.h>
#include <stdint.h>

#define FAN_MODE_MAX    0x06u

/* Trampoline table — indexed by endpoint number */
static zigbee_fan_cluster *fan_cluster_by_endpoint[10];

/* ---- Attribute-write callback -------------------------------------------- */

void fan_cluster_callback_attr_write(uint8_t endpoint, uint16_t attribute_id) {
    zigbee_fan_cluster *cluster = fan_cluster_by_endpoint[endpoint];
    if (cluster == NULL) return;

    if (attribute_id == ZCL_ATTR_FAN_MODE) {
        uint8_t new_mode = cluster->fan_mode;
        if (new_mode > FAN_MODE_MAX) {
            new_mode = ZCL_FAN_MODE_OFF;
            cluster->fan_mode = new_mode;
        }
        printf("fan_cluster: mode write → %u\r\n", new_mode);
        if (cluster->on_mode_change != NULL) {
            cluster->on_mode_change(new_mode);
        }
        hal_zigbee_notify_attribute_changed(endpoint, ZCL_CLUSTER_FAN_CONTROL,
                                            ZCL_ATTR_FAN_MODE);
    } else if (attribute_id == ZCL_ATTR_FAN_DIRECTION_CUSTOM) {
        uint8_t new_direction = cluster->fan_direction;
        if (new_direction != TUYA_FAN_DIRECTION_REVERSE) {
            new_direction = TUYA_FAN_DIRECTION_FORWARD;
            cluster->fan_direction = new_direction;
        }

        printf("fan_cluster: direction write -> %u\r\n", new_direction);
        if (cluster->on_direction_change != NULL) {
            cluster->on_direction_change(new_direction);
        }
        hal_zigbee_notify_attribute_changed(endpoint, ZCL_CLUSTER_FAN_CONTROL,
                                            ZCL_ATTR_FAN_DIRECTION_CUSTOM);
    }
}

void fan_cluster_callback_attr_write_trampoline(uint8_t endpoint,
                                                uint16_t attribute_id) {
    fan_cluster_callback_attr_write(endpoint, attribute_id);
}

/* Trampoline registered with the HAL */
static hal_zigbee_cmd_result_t fan_cluster_cmd_trampoline(uint8_t endpoint,
                                                          uint16_t cluster_id,
                                                          uint8_t command_id,
                                                          void *cmd_payload,
                                                          uint16_t cmd_payload_len) {
    /* Fan Control has no cluster-specific commands — all control via attr writes */
    (void)endpoint; (void)cluster_id; (void)command_id;
    (void)cmd_payload; (void)cmd_payload_len;
    return HAL_ZIGBEE_CMD_SKIPPED;
}

/* ---- Public API ----------------------------------------------------------- */

void fan_cluster_add_to_endpoint(zigbee_fan_cluster *cluster,
                                 hal_zigbee_endpoint *endpoint) {
    fan_cluster_by_endpoint[endpoint->endpoint] = cluster;
    cluster->endpoint = endpoint->endpoint;

    /* Default: off, 6-speed sequence */
    cluster->fan_mode          = ZCL_FAN_MODE_OFF;
    cluster->fan_mode_sequence = ZCL_FAN_MODE_SEQ_LOW_MED_HIGH_AUTO;
    cluster->fan_direction     = TUYA_FAN_DIRECTION_FORWARD;

    /* Attribute table */
    SETUP_ATTR(0, ZCL_ATTR_FAN_MODE, ZCL_DATA_TYPE_ENUM8,
               ATTR_WRITABLE, cluster->fan_mode);
    SETUP_ATTR(1, ZCL_ATTR_FAN_MODE_SEQUENCE, ZCL_DATA_TYPE_ENUM8,
               ATTR_READONLY, cluster->fan_mode_sequence);
    SETUP_ATTR(2, ZCL_ATTR_FAN_DIRECTION_CUSTOM, ZCL_DATA_TYPE_ENUM8,
               ATTR_WRITABLE, cluster->fan_direction);

    /* Register cluster on endpoint */
    endpoint->clusters[endpoint->cluster_count].cluster_id      = ZCL_CLUSTER_FAN_CONTROL;
    endpoint->clusters[endpoint->cluster_count].attribute_count = FAN_CLUSTER_ATTR_COUNT;
    endpoint->clusters[endpoint->cluster_count].attributes      = cluster->attr_infos;
    endpoint->clusters[endpoint->cluster_count].is_server       = 1;
    endpoint->clusters[endpoint->cluster_count].cmd_callback    = fan_cluster_cmd_trampoline;
    endpoint->cluster_count++;
}

void fan_cluster_update_from_dp(zigbee_fan_cluster *cluster,
                                bool dp1_on, uint8_t dp3_speed) {
    uint8_t new_mode;
    if (!dp1_on) {
        new_mode = ZCL_FAN_MODE_OFF;
    } else {
        /* Clamp speed to valid range 1-6, map to ZCL FanMode 1-6 */
        if (dp3_speed < 1u) dp3_speed = 1u;
        if (dp3_speed > 6u) dp3_speed = 6u;
        new_mode = dp3_speed;   /* ZCL FanMode 1=Low … 6=On (max) */
    }

    if (new_mode != cluster->fan_mode) {
        cluster->fan_mode = new_mode;
        hal_zigbee_notify_attribute_changed(cluster->endpoint,
                                            ZCL_CLUSTER_FAN_CONTROL,
                                            ZCL_ATTR_FAN_MODE);
    }
}

void fan_cluster_update_direction_from_dp(zigbee_fan_cluster *cluster,
                                          uint8_t dp4_direction) {
    uint8_t normalized = (dp4_direction == TUYA_FAN_DIRECTION_REVERSE)
                             ? TUYA_FAN_DIRECTION_REVERSE
                             : TUYA_FAN_DIRECTION_FORWARD;

    if (normalized != cluster->fan_direction) {
        cluster->fan_direction = normalized;
        hal_zigbee_notify_attribute_changed(cluster->endpoint,
                                            ZCL_CLUSTER_FAN_CONTROL,
                                            ZCL_ATTR_FAN_DIRECTION_CUSTOM);
    }
}
