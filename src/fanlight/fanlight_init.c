/**
 * fanlight_init.c — Fan+light device initialisation (Tuya MCU bridge)
 *
 * Zigbee topology:
 *   Endpoint 1 — Fan  (HA profile 0x0104, device type 0x0203)
 *     Clusters: Basic (0x0000), OTA (0x0019), Fan Control (0x0202)
 *
 *   Endpoint 2 — Color Temperature Light (device type 0x010C)
 *     Clusters: Basic (0x0000), On/Off (0x0006),
 *               Level (0x0008), Color (0x0300)
 *
 * Tuya MCU DP mapping:
 *   DP1  (bool)  → fan on/off
 *   DP2  (enum)  → fan mode (nature/sleep), currently unimplemented
 *   DP3  (enum)  → fan speed 1-6
 *   DP4  (enum)  → fan direction 0=forward, 1=reverse (custom attr on EP1)
 *   DP9  (bool)  → light on/off
 *   DP10 (value) → light brightness 0-100, step=2 → ZCL level 0-254
 *   DP11 (value) → light color temp 0-100, step=2 → ZCL mireds 154-370
 *                  direction controlled by TUYA_DP11_WARM_AT_100
 *   DP102/103    → fan countdown/timer (explicitly ignored)
 */

#include "fanlight_init.h"

#include "base_components/tuya_mcu.h"
#include "device_config/config_nv.h"
#include "device_config/device_type.h"
#include "device_config/nvm_items.h"
#include "device_config/reset.h"
#include "hal/nvm.h"
#include "hal/printf_selector.h"
#include "hal/system.h"
#include "hal/zigbee.h"
#include "hal/zigbee_ota.h"
#include "zigbee/basic_cluster.h"
#include "zigbee/color_temp_cluster.h"
#include "zigbee/consts.h"
#include "zigbee/fan_cluster.h"
#include "zigbee/general_commands.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

/* ---- ZCL device type IDs ------------------------------------------------- */
#define ZIGBEE_DEVICE_TYPE_FAN              0x0203u
#define ZIGBEE_DEVICE_TYPE_COLOR_TEMP_LIGHT 0x010Cu

/* ---- Static cluster / endpoint storage ----------------------------------- */
#define MAX_EP1_CLUSTERS   3u   /* Basic + OTA + FanControl */
#define MAX_EP2_CLUSTERS   4u   /* Basic + OnOff + Level + Color */
#define TOTAL_ZCL_CLUSTERS (MAX_EP1_CLUSTERS + MAX_EP2_CLUSTERS)

static hal_zigbee_cluster  clusters[TOTAL_ZCL_CLUSTERS];
static hal_zigbee_endpoint endpoints[2];

static zigbee_basic_cluster basic_ep1 = { .deviceEnable = 1 };
static zigbee_basic_cluster basic_ep2 = { .deviceEnable = 1 };

static zigbee_fan_cluster       g_fan_cluster;
static zigbee_color_temp_cluster g_light_cluster;

/* ---- Runtime fan state (needed for DP1/DP3 split) ----------------------- */
static bool    fan_on    = false;
static uint8_t fan_speed = 1u;  /* 1-6 */

/* ========================================================================== */
/* Forward declarations                                                         */
/* ========================================================================== */

static void on_dp_received(uint8_t dp_id, uint8_t dp_type, int32_t value);
static void on_fan_mode_change(uint8_t new_mode);
static void on_fan_direction_change(uint8_t new_direction);
static void on_light_onoff_change(bool on);
static void on_light_level_change(uint8_t level_0_254);
static void on_light_color_temp_change(uint16_t mireds);

static uint8_t clamp_to_even_0_100(uint8_t v) {
    if (v > 100u) v = 100u;
    if (v & 0x01u) {
        /* Nearest even, ties rounded up. */
        v = (uint8_t)(v + 1u);
        if (v > 100u) v = 100u;
    }
    return v;
}

/* ========================================================================== */
/* MCU → Zigbee  (DP callbacks)                                                */
/* ========================================================================== */

static void on_dp_received(uint8_t dp_id, uint8_t dp_type, int32_t value) {
    switch (dp_id) {
    case TUYA_DP_FAN_ONOFF:
        fan_on = (value != 0);
        fan_cluster_update_from_dp(&g_fan_cluster, fan_on, fan_speed);
        break;

    case TUYA_DP_FAN_MODE:
        printf("fanlight: fan mode DP2=%d (nature/sleep, unimplemented)\r\n",
               (int)value);
        break;

    case TUYA_DP_FAN_SPEED:
        fan_speed = (uint8_t)value;
        fan_cluster_update_from_dp(&g_fan_cluster, fan_on, fan_speed);
        break;

    case TUYA_DP_FAN_DIRECTION:
        fan_cluster_update_direction_from_dp(&g_fan_cluster, (uint8_t)value);
        break;

    case TUYA_DP_LIGHT_ONOFF:
        color_temp_cluster_update_onoff(&g_light_cluster, value != 0);
        break;

    case TUYA_DP_LIGHT_LEVEL:
        color_temp_cluster_update_level_dp(&g_light_cluster, (uint8_t)value);
        break;

    case TUYA_DP_LIGHT_COLORTEMP:
        color_temp_cluster_update_color_temp_dp(&g_light_cluster, (uint8_t)value);
        break;

    case TUYA_DP_UNKNOWN_102:
    case TUYA_DP_UNKNOWN_103:
        /* Known on this MCU as sleep-timer related DPs. Ignore explicitly. */
        break;

    default:
        printf("fanlight: unknown DP %u type=%u val=%d\r\n",
               dp_id, dp_type, (int)value);
        break;
    }
}

/* ========================================================================== */
/* Zigbee → MCU  (command/attribute callbacks)                                 */
/* ========================================================================== */

static void on_fan_mode_change(uint8_t new_mode) {
    if (new_mode == ZCL_FAN_MODE_OFF) {
        fan_on = false;
        tuya_mcu_send_bool(TUYA_DP_FAN_ONOFF, false);
    } else {
        /* ZCL FanMode 1-6 maps directly to DP3 speed 1-6 */
        fan_speed = new_mode;
        fan_on    = true;
        tuya_mcu_send_bool(TUYA_DP_FAN_ONOFF, true);
        tuya_mcu_send_enum(TUYA_DP_FAN_SPEED, fan_speed);
    }
}

static void on_fan_direction_change(uint8_t new_direction) {
    uint8_t dp4 = (new_direction == TUYA_FAN_DIRECTION_REVERSE)
                      ? TUYA_FAN_DIRECTION_REVERSE
                      : TUYA_FAN_DIRECTION_FORWARD;
    tuya_mcu_send_enum(TUYA_DP_FAN_DIRECTION, dp4);
}

static void on_light_onoff_change(bool on) {
    tuya_mcu_send_bool(TUYA_DP_LIGHT_ONOFF, on);
}

static void on_light_level_change(uint8_t level_0_254) {
    /* Scale 0-254 -> 0-100 and quantize to official step=2. */
    uint8_t dp10 = (uint8_t)(((uint16_t)level_0_254 * 100u + 127u) / 254u);
    dp10 = clamp_to_even_0_100(dp10);

    /* Light ON should not use DP10=0 as requested by official behavior notes. */
    if (g_light_cluster.on_off && dp10 == 0u) {
        dp10 = 2u;
    }
    tuya_mcu_send_value(TUYA_DP_LIGHT_LEVEL, (int32_t)dp10);
}

static void on_light_color_temp_change(uint16_t mireds) {
    uint16_t clamped = mireds;
    if (clamped < LIGHT_COLOR_TEMP_COOL_MIREDS) clamped = LIGHT_COLOR_TEMP_COOL_MIREDS;
    if (clamped > LIGHT_COLOR_TEMP_WARM_MIREDS) clamped = LIGHT_COLOR_TEMP_WARM_MIREDS;

#if TUYA_DP11_WARM_AT_100
    uint8_t dp11 = (uint8_t)(((uint32_t)(clamped - LIGHT_COLOR_TEMP_COOL_MIREDS) * 100u +
                              ((LIGHT_COLOR_TEMP_WARM_MIREDS - LIGHT_COLOR_TEMP_COOL_MIREDS) / 2u)) /
                             (LIGHT_COLOR_TEMP_WARM_MIREDS - LIGHT_COLOR_TEMP_COOL_MIREDS));
#else
    uint8_t dp11 = (uint8_t)(((uint32_t)(LIGHT_COLOR_TEMP_WARM_MIREDS - clamped) * 100u +
                              ((LIGHT_COLOR_TEMP_WARM_MIREDS - LIGHT_COLOR_TEMP_COOL_MIREDS) / 2u)) /
                             (LIGHT_COLOR_TEMP_WARM_MIREDS - LIGHT_COLOR_TEMP_COOL_MIREDS));
#endif
    dp11 = clamp_to_even_0_100(dp11);
    tuya_mcu_send_value(TUYA_DP_LIGHT_COLORTEMP, (int32_t)dp11);
}

/* ========================================================================== */
/* Device-type-change / reset guard                                             */
/* ========================================================================== */

static void process_device_type_change(void) {
    enum device_type_t stored;
    hal_nvm_status_t   st = hal_nvm_read(NV_ITEM_DEVICE_TYPE, sizeof(stored),
                                         (uint8_t *)&stored);
    if (st != HAL_NVM_SUCCESS) {
        stored = CURRENT_DEVICE_TYPE;
        hal_nvm_write(NV_ITEM_DEVICE_TYPE, sizeof(stored), (uint8_t *)&stored);
        return;
    }
    if (stored != CURRENT_DEVICE_TYPE) {
        stored = CURRENT_DEVICE_TYPE;
        hal_nvm_write(NV_ITEM_DEVICE_TYPE, sizeof(stored), (uint8_t *)&stored);
        hal_factory_reset();
        schedule_reboot(2000);
    }
}

/* ========================================================================== */
/* Public entry points (called from main.c instead of app_init / app_task)     */
/* ========================================================================== */

void fanlight_app_init(void) {
    handle_version_changes();

    /* Initialise Tuya MCU UART driver */
    tuya_mcu_init(on_dp_received);

    /* ---- Build endpoint 1: Fan ---- */
    hal_zigbee_cluster *ep1_clusters = clusters;
    endpoints[0].endpoint      = 1u;
    endpoints[0].profile_id    = ZCL_HA_PROFILE;
    endpoints[0].device_id     = ZIGBEE_DEVICE_TYPE_FAN;
    endpoints[0].device_version= 0u;
    endpoints[0].cluster_count = 0u;
    endpoints[0].clusters      = ep1_clusters;

    /* Basic cluster on EP1 */
    memcpy(basic_ep1.manuName + 1, "TuyaFanLight", 12);
    basic_ep1.manuName[0] = 12;
    memcpy(basic_ep1.modelId + 1, "ZTU-FanLight", 12);
    basic_ep1.modelId[0] = 12;
    basic_cluster_add_to_endpoint(&basic_ep1, &endpoints[0]);

    /* OTA cluster on EP1 */
    hal_ota_cluster_setup(&endpoints[0].clusters[endpoints[0].cluster_count]);
    endpoints[0].cluster_count++;

    /* Fan Control cluster */
    g_fan_cluster.on_mode_change = on_fan_mode_change;
    g_fan_cluster.on_direction_change = on_fan_direction_change;
    fan_cluster_add_to_endpoint(&g_fan_cluster, &endpoints[0]);

    /* ---- Build endpoint 2: Color Temperature Light ---- */
    hal_zigbee_cluster *ep2_clusters = clusters + endpoints[0].cluster_count;
    endpoints[1].endpoint      = 2u;
    endpoints[1].profile_id    = ZCL_HA_PROFILE;
    endpoints[1].device_id     = ZIGBEE_DEVICE_TYPE_COLOR_TEMP_LIGHT;
    endpoints[1].device_version= 0u;
    endpoints[1].cluster_count = 0u;
    endpoints[1].clusters      = ep2_clusters;

    /* Basic cluster on EP2 — reuse same strings */
    memcpy(basic_ep2.manuName, basic_ep1.manuName, sizeof(basic_ep1.manuName));
    memcpy(basic_ep2.modelId,  basic_ep1.modelId,  sizeof(basic_ep1.modelId));
    basic_cluster_add_to_endpoint(&basic_ep2, &endpoints[1]);

    /* Color Temperature Light clusters */
    g_light_cluster.on_onoff_change      = on_light_onoff_change;
    g_light_cluster.on_level_change      = on_light_level_change;
    g_light_cluster.on_color_temp_change = on_light_color_temp_change;
    color_temp_cluster_add_to_endpoint(&g_light_cluster, &endpoints[1]);

    /* ---- Zigbee stack init ---- */
    hal_zigbee_init(endpoints, 2u);
    hal_zigbee_init_ota();
    init_global_attr_write_callback();

    process_device_type_change();

    /* Query current MCU state so ZCL attributes are populated at boot */
    tuya_mcu_query_all();

    printf("fanlight: init complete\r\n");
}

void fanlight_app_task(void) {
    /* Process Tuya MCU receive data and heartbeat */
    tuya_mcu_task();

    /* Auto-join Zigbee network */
    if (hal_zigbee_get_network_status() != HAL_ZIGBEE_NETWORK_JOINED &&
        hal_zigbee_get_network_status() != HAL_ZIGBEE_NETWORK_JOINING) {
        hal_zigbee_start_network_steering();
    }
}
