/**
 * color_temp_cluster.c — ZCL Color Temperature Light clusters implementation
 *
 * Handles the three ZCL clusters that model the light half of the fan+light
 * device and translates between ZCL commands/attributes and Tuya DP values.
 */

#include "color_temp_cluster.h"
#include "cluster_common.h"
#include "consts.h"
#include "hal/zigbee.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

/* Trampoline table — indexed by endpoint number */
static zigbee_color_temp_cluster *ct_cluster_by_endpoint[10];

/* ---- Scaling helpers ------------------------------------------------------- */

/**
 * Map DP10 (10-100) → ZCL CurrentLevel (0-254).
 * Values below 10 are clamped to 0.
 */
static uint8_t dp10_to_level(uint8_t dp10) {
    if (dp10 <= 10u) return 0u;
    if (dp10 >= 100u) return 254u;
    return (uint8_t)(((uint16_t)(dp10 - 10u) * 254u) / 90u);
}

/**
 * Map DP11 (0-100) → mireds.
 * DP11=0 → warm (370 mired), DP11=100 → cool (154 mired).
 *   mireds = 370 - dp11 * (370-154) / 100
 */
static uint16_t dp11_to_mireds(uint8_t dp11) {
    if (dp11 >= 100u) return LIGHT_COLOR_TEMP_COOL_MIREDS;
    return (uint16_t)(LIGHT_COLOR_TEMP_WARM_MIREDS -
                      (uint16_t)dp11 * (LIGHT_COLOR_TEMP_WARM_MIREDS -
                                        LIGHT_COLOR_TEMP_COOL_MIREDS) / 100u);
}

/* ---- ZCL command callbacks ----------------------------------------------- */

static hal_zigbee_cmd_result_t onoff_cmd_callback(uint8_t endpoint,
                                                   uint16_t cluster_id,
                                                   uint8_t command_id,
                                                   void *cmd_payload,
                                                   uint16_t cmd_payload_len) {
    zigbee_color_temp_cluster *cluster = ct_cluster_by_endpoint[endpoint];
    if (cluster == NULL) return HAL_ZIGBEE_CMD_SKIPPED;

    switch (command_id) {
    case ZCL_CMD_ONOFF_ON:
    case ZCL_CMD_ON_WITH_RECALL_GLOBAL_SCENE:
        cluster->on_off = 1u;
        break;
    case ZCL_CMD_ONOFF_OFF:
    case ZCL_CMD_OFF_WITH_EFFECT:
        cluster->on_off = 0u;
        break;
    case ZCL_CMD_ONOFF_TOGGLE:
        cluster->on_off = cluster->on_off ? 0u : 1u;
        break;
    default:
        return HAL_ZIGBEE_CMD_SKIPPED;
    }

    if (cluster->on_onoff_change != NULL) {
        cluster->on_onoff_change(cluster->on_off != 0u);
    }
    hal_zigbee_notify_attribute_changed(endpoint, ZCL_CLUSTER_ON_OFF,
                                        ZCL_ATTR_ONOFF);
    return HAL_ZIGBEE_CMD_PROCESSED;
}

static hal_zigbee_cmd_result_t level_cmd_callback(uint8_t endpoint,
                                                   uint16_t cluster_id,
                                                   uint8_t command_id,
                                                   void *cmd_payload,
                                                   uint16_t cmd_payload_len) {
    zigbee_color_temp_cluster *cluster = ct_cluster_by_endpoint[endpoint];
    if (cluster == NULL) return HAL_ZIGBEE_CMD_SKIPPED;

    if ((command_id == ZCL_CMD_LEVEL_MOVE_TO_LEVEL ||
         command_id == ZCL_CMD_LEVEL_MOVE_TO_LEVEL_WITH_ON_OFF) &&
        cmd_payload_len >= sizeof(zcl_move_to_level_t)) {
        zcl_move_to_level_t *req = (zcl_move_to_level_t *)cmd_payload;
        cluster->current_level = req->level;

        if (cluster->on_level_change != NULL) {
            cluster->on_level_change(req->level);
        }
        hal_zigbee_notify_attribute_changed(endpoint, ZCL_CLUSTER_LEVEL_CONTROL,
                                            ZCL_ATTR_CURRENT_LEVEL);
        return HAL_ZIGBEE_CMD_PROCESSED;
    }
    return HAL_ZIGBEE_CMD_SKIPPED;
}

static hal_zigbee_cmd_result_t color_cmd_callback(uint8_t endpoint,
                                                   uint16_t cluster_id,
                                                   uint8_t command_id,
                                                   void *cmd_payload,
                                                   uint16_t cmd_payload_len) {
    zigbee_color_temp_cluster *cluster = ct_cluster_by_endpoint[endpoint];
    if (cluster == NULL) return HAL_ZIGBEE_CMD_SKIPPED;

    if (command_id == ZCL_CMD_COLOR_MOVE_TO_COLOR_TEMP &&
        cmd_payload_len >= sizeof(zcl_move_to_color_temp_t)) {
        zcl_move_to_color_temp_t *req = (zcl_move_to_color_temp_t *)cmd_payload;
        uint16_t mireds = req->color_temperature_mireds;

        /* Clamp to device capability */
        if (mireds < LIGHT_COLOR_TEMP_COOL_MIREDS) mireds = LIGHT_COLOR_TEMP_COOL_MIREDS;
        if (mireds > LIGHT_COLOR_TEMP_WARM_MIREDS) mireds = LIGHT_COLOR_TEMP_WARM_MIREDS;

        cluster->color_temperature = mireds;

        if (cluster->on_color_temp_change != NULL) {
            cluster->on_color_temp_change(mireds);
        }
        hal_zigbee_notify_attribute_changed(endpoint, ZCL_CLUSTER_COLOR_CONTROL,
                                            ZCL_ATTR_COLOR_TEMPERATURE);
        return HAL_ZIGBEE_CMD_PROCESSED;
    }
    return HAL_ZIGBEE_CMD_SKIPPED;
}

/* Trampoline wrappers using cluster_id to dispatch */
static hal_zigbee_cmd_result_t onoff_trampoline(uint8_t ep, uint16_t cid,
                                                 uint8_t cmd, void *pl, uint16_t plen) {
    return onoff_cmd_callback(ep, cid, cmd, pl, plen);
}

static hal_zigbee_cmd_result_t level_trampoline(uint8_t ep, uint16_t cid,
                                                 uint8_t cmd, void *pl, uint16_t plen) {
    return level_cmd_callback(ep, cid, cmd, pl, plen);
}

static hal_zigbee_cmd_result_t color_trampoline(uint8_t ep, uint16_t cid,
                                                 uint8_t cmd, void *pl, uint16_t plen) {
    return color_cmd_callback(ep, cid, cmd, pl, plen);
}

/* ---- Public API ----------------------------------------------------------- */

void color_temp_cluster_add_to_endpoint(zigbee_color_temp_cluster *cluster,
                                        hal_zigbee_endpoint *endpoint) {
    ct_cluster_by_endpoint[endpoint->endpoint] = cluster;
    cluster->endpoint = endpoint->endpoint;

    /* Defaults */
    cluster->on_off            = 0u;
    cluster->current_level     = 254u;
    cluster->color_temperature = LIGHT_COLOR_TEMP_WARM_MIREDS;
    cluster->color_mode        = ZCL_COLOR_MODE_COLOR_TEMP;
    cluster->color_temp_min    = LIGHT_COLOR_TEMP_COOL_MIREDS;
    cluster->color_temp_max    = LIGHT_COLOR_TEMP_WARM_MIREDS;

    /* ---- On/Off cluster ---- */
    cluster->onoff_attrs[0].attribute_id = ZCL_ATTR_ONOFF;
    cluster->onoff_attrs[0].data_type_id = ZCL_DATA_TYPE_BOOLEAN;
    cluster->onoff_attrs[0].flag         = ATTR_READONLY;
    cluster->onoff_attrs[0].size         = sizeof(cluster->on_off);
    cluster->onoff_attrs[0].value        = &cluster->on_off;

    endpoint->clusters[endpoint->cluster_count].cluster_id      = ZCL_CLUSTER_ON_OFF;
    endpoint->clusters[endpoint->cluster_count].attribute_count = ONOFF_CLUSTER_ATTR_COUNT;
    endpoint->clusters[endpoint->cluster_count].attributes      = cluster->onoff_attrs;
    endpoint->clusters[endpoint->cluster_count].is_server       = 1;
    endpoint->clusters[endpoint->cluster_count].cmd_callback    = onoff_trampoline;
    endpoint->cluster_count++;

    /* ---- Level Control cluster ---- */
    cluster->level_attrs[0].attribute_id = ZCL_ATTR_CURRENT_LEVEL;
    cluster->level_attrs[0].data_type_id = ZCL_DATA_TYPE_UINT8;
    cluster->level_attrs[0].flag         = ATTR_READONLY;
    cluster->level_attrs[0].size         = sizeof(cluster->current_level);
    cluster->level_attrs[0].value        = &cluster->current_level;

    endpoint->clusters[endpoint->cluster_count].cluster_id      = ZCL_CLUSTER_LEVEL_CONTROL;
    endpoint->clusters[endpoint->cluster_count].attribute_count = LEVEL_CLUSTER_ATTR_COUNT;
    endpoint->clusters[endpoint->cluster_count].attributes      = cluster->level_attrs;
    endpoint->clusters[endpoint->cluster_count].is_server       = 1;
    endpoint->clusters[endpoint->cluster_count].cmd_callback    = level_trampoline;
    endpoint->cluster_count++;

    /* ---- Color Control cluster ---- */
    cluster->color_attrs[0].attribute_id = ZCL_ATTR_COLOR_TEMPERATURE;
    cluster->color_attrs[0].data_type_id = ZCL_DATA_TYPE_UINT16;
    cluster->color_attrs[0].flag         = ATTR_READONLY;
    cluster->color_attrs[0].size         = sizeof(cluster->color_temperature);
    cluster->color_attrs[0].value        = (uint8_t *)&cluster->color_temperature;

    cluster->color_attrs[1].attribute_id = ZCL_ATTR_COLOR_MODE;
    cluster->color_attrs[1].data_type_id = ZCL_DATA_TYPE_ENUM8;
    cluster->color_attrs[1].flag         = ATTR_READONLY;
    cluster->color_attrs[1].size         = sizeof(cluster->color_mode);
    cluster->color_attrs[1].value        = &cluster->color_mode;

    cluster->color_attrs[2].attribute_id = ZCL_ATTR_COLOR_TEMP_MIN;
    cluster->color_attrs[2].data_type_id = ZCL_DATA_TYPE_UINT16;
    cluster->color_attrs[2].flag         = ATTR_READONLY;
    cluster->color_attrs[2].size         = sizeof(cluster->color_temp_min);
    cluster->color_attrs[2].value        = (uint8_t *)&cluster->color_temp_min;

    cluster->color_attrs[3].attribute_id = ZCL_ATTR_COLOR_TEMP_MAX;
    cluster->color_attrs[3].data_type_id = ZCL_DATA_TYPE_UINT16;
    cluster->color_attrs[3].flag         = ATTR_READONLY;
    cluster->color_attrs[3].size         = sizeof(cluster->color_temp_max);
    cluster->color_attrs[3].value        = (uint8_t *)&cluster->color_temp_max;

    endpoint->clusters[endpoint->cluster_count].cluster_id      = ZCL_CLUSTER_COLOR_CONTROL;
    endpoint->clusters[endpoint->cluster_count].attribute_count = COLOR_CLUSTER_ATTR_COUNT;
    endpoint->clusters[endpoint->cluster_count].attributes      = cluster->color_attrs;
    endpoint->clusters[endpoint->cluster_count].is_server       = 1;
    endpoint->clusters[endpoint->cluster_count].cmd_callback    = color_trampoline;
    endpoint->cluster_count++;
}

void color_temp_cluster_update_onoff(zigbee_color_temp_cluster *cluster,
                                     bool on) {
    uint8_t new_val = on ? 1u : 0u;
    if (new_val != cluster->on_off) {
        cluster->on_off = new_val;
        hal_zigbee_notify_attribute_changed(cluster->endpoint, ZCL_CLUSTER_ON_OFF,
                                            ZCL_ATTR_ONOFF);
    }
}

void color_temp_cluster_update_level_dp(zigbee_color_temp_cluster *cluster,
                                        uint8_t dp10_value) {
    uint8_t new_level = dp10_to_level(dp10_value);
    if (new_level != cluster->current_level) {
        cluster->current_level = new_level;
        hal_zigbee_notify_attribute_changed(cluster->endpoint,
                                            ZCL_CLUSTER_LEVEL_CONTROL,
                                            ZCL_ATTR_CURRENT_LEVEL);
    }
}

void color_temp_cluster_update_color_temp_dp(zigbee_color_temp_cluster *cluster,
                                             uint8_t dp11_value) {
    uint16_t new_mireds = dp11_to_mireds(dp11_value);
    if (new_mireds != cluster->color_temperature) {
        cluster->color_temperature = new_mireds;
        hal_zigbee_notify_attribute_changed(cluster->endpoint,
                                            ZCL_CLUSTER_COLOR_CONTROL,
                                            ZCL_ATTR_COLOR_TEMPERATURE);
    }
}
