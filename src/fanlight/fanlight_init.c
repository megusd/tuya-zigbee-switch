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
#include "zigbee/cluster_common.h"
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
#define ZIGBEE_DEVICE_TYPE_ONOFF_OUTPUT     0x0002u

/* ---- Fan mode DP2 values ------------------------------------------------- */
#define TUYA_FAN_MODE_NORMAL                0u
#define TUYA_FAN_MODE_NATURE                1u
#define TUYA_FAN_MODE_SLEEP                 2u

/* ---- Extra endpoints ----------------------------------------------------- */
#define WINTER_MODE_ENDPOINT                3u
#define NATURE_MODE_ENDPOINT                4u
#define SLEEP_MODE_ENDPOINT                 5u

typedef enum {
    MODE_SWITCH_WINTER = 0,
    MODE_SWITCH_NATURE,
    MODE_SWITCH_SLEEP,
} mode_switch_kind_t;

typedef struct {
    uint8_t              endpoint;
    uint8_t              on_off;
    hal_zigbee_attribute onoff_attrs[1];
} fanlight_mode_switch_cluster;

typedef struct {
    uint8_t              device_enable;
    char                 manu_name[32];
    char                 model_id[32];
    char                 location_desc[32];
    hal_zigbee_attribute attrs[7];
} fanlight_mode_basic_cluster;

/* Keep these local to avoid pulling config_parser globals into fanlight mode. */
static const uint8_t  mode_basic_zcl_ver    = 0x03u;
static const uint8_t  mode_basic_app_ver    = 0x03u;
static const uint8_t  mode_basic_stack_ver  = 0x02u;
static const uint8_t  mode_basic_hw_ver     = 0x00u;
static const uint8_t  mode_basic_power      = POWER_SOURCE_MAINS_1_PHASE;
static const uint16_t mode_basic_cluster_rev = 0x01u;

/* ---- Static cluster / endpoint storage ----------------------------------- */
#define MAX_EP1_CLUSTERS   3u   /* Basic + OTA + FanControl */
#define MAX_EP2_CLUSTERS   4u   /* Basic + OnOff + Level + Color */
#define MAX_EP3_CLUSTERS   2u   /* Basic + OnOff (winter mode) */
#define MAX_EP4_CLUSTERS   2u   /* Basic + OnOff (nature mode) */
#define MAX_EP5_CLUSTERS   2u   /* Basic + OnOff (sleep mode) */
#define TOTAL_ZCL_CLUSTERS (MAX_EP1_CLUSTERS + MAX_EP2_CLUSTERS + \
                            MAX_EP3_CLUSTERS + MAX_EP4_CLUSTERS + MAX_EP5_CLUSTERS)

static hal_zigbee_cluster  clusters[TOTAL_ZCL_CLUSTERS];
static hal_zigbee_endpoint endpoints[5];

static zigbee_basic_cluster basic_ep1 = { .deviceEnable = 1 };
static zigbee_basic_cluster basic_ep2 = { .deviceEnable = 1 };

static fanlight_mode_basic_cluster basic_ep3 = { .device_enable = 1 };
static fanlight_mode_basic_cluster basic_ep4 = { .device_enable = 1 };
static fanlight_mode_basic_cluster basic_ep5 = { .device_enable = 1 };

static zigbee_fan_cluster       g_fan_cluster;
static zigbee_color_temp_cluster g_light_cluster;
static fanlight_mode_switch_cluster g_winter_mode_cluster;
static fanlight_mode_switch_cluster g_nature_mode_cluster;
static fanlight_mode_switch_cluster g_sleep_mode_cluster;

static fanlight_mode_switch_cluster *mode_switch_by_endpoint[16];
static mode_switch_kind_t             mode_kind_by_endpoint[16];

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
static void mode_switch_apply_from_dp2(uint8_t dp2_mode);

static void set_zcl_str(char dest[32], const char *src) {
    size_t len = strlen(src);
    if (len > 31u) {
        len = 31u;
    }
    dest[0] = (char)len;
    memcpy(dest + 1, src, len);
}

static void mode_basic_cluster_add_to_endpoint(fanlight_mode_basic_cluster *cluster,
                                               hal_zigbee_endpoint *endpoint,
                                               const char *location_desc) {
    set_zcl_str(cluster->manu_name, "TuyaFanLight");
    set_zcl_str(cluster->model_id, "ZTU-FanLight");
    set_zcl_str(cluster->location_desc, location_desc);

    SETUP_ATTR_FOR_TABLE(cluster->attrs, 0, ZCL_ATTR_BASIC_ZCL_VER,
                         ZCL_DATA_TYPE_UINT8, ATTR_READONLY, mode_basic_zcl_ver);
    SETUP_ATTR_FOR_TABLE(cluster->attrs, 1, ZCL_ATTR_BASIC_APP_VER,
                         ZCL_DATA_TYPE_UINT8, ATTR_READONLY, mode_basic_app_ver);
    SETUP_ATTR_FOR_TABLE(cluster->attrs, 2, ZCL_ATTR_BASIC_STACK_VER,
                         ZCL_DATA_TYPE_UINT8, ATTR_READONLY, mode_basic_stack_ver);
    SETUP_ATTR_FOR_TABLE(cluster->attrs, 3, ZCL_ATTR_BASIC_HW_VER,
                         ZCL_DATA_TYPE_UINT8, ATTR_READONLY, mode_basic_hw_ver);
    SETUP_ATTR_FOR_TABLE(cluster->attrs, 4, ZCL_ATTR_BASIC_MFR_NAME,
                         ZCL_DATA_TYPE_CHAR_STR, ATTR_READONLY, cluster->manu_name);
    SETUP_ATTR_FOR_TABLE(cluster->attrs, 5, ZCL_ATTR_BASIC_MODEL_ID,
                         ZCL_DATA_TYPE_CHAR_STR, ATTR_READONLY, cluster->model_id);
    SETUP_ATTR_FOR_TABLE(cluster->attrs, 6, ZCL_ATTR_BASIC_LOC_DESC,
                         ZCL_DATA_TYPE_CHAR_STR, ATTR_READONLY, cluster->location_desc);

    endpoint->clusters[endpoint->cluster_count].cluster_id      = ZCL_CLUSTER_BASIC;
    endpoint->clusters[endpoint->cluster_count].attribute_count = 7u;
    endpoint->clusters[endpoint->cluster_count].attributes      = cluster->attrs;
    endpoint->clusters[endpoint->cluster_count].is_server       = 1u;
    endpoint->clusters[endpoint->cluster_count].cmd_callback    = NULL;
    endpoint->cluster_count++;

    (void)mode_basic_power;
    (void)mode_basic_cluster_rev;
}

static void set_mode_switch_state(fanlight_mode_switch_cluster *cluster,
                                  bool on) {
    uint8_t new_val = on ? 1u : 0u;
    if (cluster->on_off != new_val) {
        cluster->on_off = new_val;
        hal_zigbee_notify_attribute_changed(cluster->endpoint, ZCL_CLUSTER_ON_OFF,
                                            ZCL_ATTR_ONOFF);
    }
}

static void set_nature_sleep_state(bool nature_on, bool sleep_on) {
    set_mode_switch_state(&g_nature_mode_cluster, nature_on);
    set_mode_switch_state(&g_sleep_mode_cluster, sleep_on);
}

static hal_zigbee_cmd_result_t mode_switch_onoff_cmd_callback(
    uint8_t endpoint, uint16_t cluster_id, uint8_t command_id,
    void *cmd_payload, uint16_t cmd_payload_len) {
    fanlight_mode_switch_cluster *cluster = mode_switch_by_endpoint[endpoint];
    if (cluster == NULL || cluster_id != ZCL_CLUSTER_ON_OFF) {
        return HAL_ZIGBEE_CMD_SKIPPED;
    }

    uint8_t old_state = cluster->on_off;
    uint8_t new_state = old_state;

    switch (command_id) {
    case ZCL_CMD_ONOFF_ON:
    case ZCL_CMD_ON_WITH_RECALL_GLOBAL_SCENE:
        new_state = 1u;
        break;
    case ZCL_CMD_ONOFF_OFF:
    case ZCL_CMD_OFF_WITH_EFFECT:
        new_state = 0u;
        break;
    case ZCL_CMD_ONOFF_TOGGLE:
        new_state = old_state ? 0u : 1u;
        break;
    default:
        (void)cmd_payload;
        (void)cmd_payload_len;
        return HAL_ZIGBEE_CMD_SKIPPED;
    }

    if (new_state == old_state) {
        return HAL_ZIGBEE_CMD_PROCESSED;
    }

    switch (mode_kind_by_endpoint[endpoint]) {
    case MODE_SWITCH_WINTER:
        set_mode_switch_state(&g_winter_mode_cluster, (new_state != 0u));
        tuya_mcu_send_enum(TUYA_DP_FAN_DIRECTION,
                           (new_state != 0u) ? TUYA_FAN_DIRECTION_REVERSE
                                             : TUYA_FAN_DIRECTION_FORWARD);
        break;
    case MODE_SWITCH_NATURE:
        if (new_state != 0u) {
            set_nature_sleep_state(true, false);
            tuya_mcu_send_enum(TUYA_DP_FAN_MODE, TUYA_FAN_MODE_NATURE);
        } else {
            set_nature_sleep_state(false, false);
            tuya_mcu_send_enum(TUYA_DP_FAN_MODE, TUYA_FAN_MODE_NORMAL);
        }
        break;
    case MODE_SWITCH_SLEEP:
        if (new_state != 0u) {
            set_nature_sleep_state(false, true);
            tuya_mcu_send_enum(TUYA_DP_FAN_MODE, TUYA_FAN_MODE_SLEEP);
        } else {
            set_nature_sleep_state(false, false);
            tuya_mcu_send_enum(TUYA_DP_FAN_MODE, TUYA_FAN_MODE_NORMAL);
        }
        break;
    default:
        return HAL_ZIGBEE_CMD_SKIPPED;
    }

    return HAL_ZIGBEE_CMD_PROCESSED;
}

static void mode_switch_cluster_add_to_endpoint(
    fanlight_mode_switch_cluster *cluster, hal_zigbee_endpoint *endpoint,
    mode_switch_kind_t kind) {
    cluster->endpoint = endpoint->endpoint;
    cluster->on_off   = 0u;

    cluster->onoff_attrs[0].attribute_id = ZCL_ATTR_ONOFF;
    cluster->onoff_attrs[0].data_type_id = ZCL_DATA_TYPE_BOOLEAN;
    cluster->onoff_attrs[0].flag         = ATTR_READONLY;
    cluster->onoff_attrs[0].size         = sizeof(cluster->on_off);
    cluster->onoff_attrs[0].value        = &cluster->on_off;

    endpoint->clusters[endpoint->cluster_count].cluster_id      = ZCL_CLUSTER_ON_OFF;
    endpoint->clusters[endpoint->cluster_count].attribute_count = 1u;
    endpoint->clusters[endpoint->cluster_count].attributes      = cluster->onoff_attrs;
    endpoint->clusters[endpoint->cluster_count].is_server       = 1u;
    endpoint->clusters[endpoint->cluster_count].cmd_callback    =
        mode_switch_onoff_cmd_callback;
    endpoint->cluster_count++;

    mode_switch_by_endpoint[cluster->endpoint] = cluster;
    mode_kind_by_endpoint[cluster->endpoint]   = kind;
}

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
        mode_switch_apply_from_dp2((uint8_t)value);
        break;

    case TUYA_DP_FAN_SPEED:
        fan_speed = (uint8_t)value;
        fan_cluster_update_from_dp(&g_fan_cluster, fan_on, fan_speed);
        break;

    case TUYA_DP_FAN_DIRECTION:
        fan_cluster_update_direction_from_dp(&g_fan_cluster, (uint8_t)value);
        set_mode_switch_state(&g_winter_mode_cluster,
                              (value == TUYA_FAN_DIRECTION_REVERSE));
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

static void mode_switch_apply_from_dp2(uint8_t dp2_mode) {
    if (dp2_mode == TUYA_FAN_MODE_NATURE) {
        set_nature_sleep_state(true, false);
    } else if (dp2_mode == TUYA_FAN_MODE_SLEEP) {
        set_nature_sleep_state(false, true);
    } else {
        set_nature_sleep_state(false, false);
    }
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

    /* ---- Build endpoint 3: Winter Mode (DP4) ---- */
    hal_zigbee_cluster *ep3_clusters =
        clusters + endpoints[0].cluster_count + endpoints[1].cluster_count;
    endpoints[2].endpoint       = WINTER_MODE_ENDPOINT;
    endpoints[2].profile_id     = ZCL_HA_PROFILE;
    endpoints[2].device_id      = ZIGBEE_DEVICE_TYPE_ONOFF_OUTPUT;
    endpoints[2].device_version = 0u;
    endpoints[2].cluster_count  = 0u;
    endpoints[2].clusters       = ep3_clusters;

    mode_basic_cluster_add_to_endpoint(&basic_ep3, &endpoints[2], "Winter Mode");
    mode_switch_cluster_add_to_endpoint(&g_winter_mode_cluster, &endpoints[2],
                                        MODE_SWITCH_WINTER);

    /* ---- Build endpoint 4: Nature Mode (DP2=1) ---- */
    hal_zigbee_cluster *ep4_clusters =
        clusters + endpoints[0].cluster_count + endpoints[1].cluster_count +
        endpoints[2].cluster_count;
    endpoints[3].endpoint       = NATURE_MODE_ENDPOINT;
    endpoints[3].profile_id     = ZCL_HA_PROFILE;
    endpoints[3].device_id      = ZIGBEE_DEVICE_TYPE_ONOFF_OUTPUT;
    endpoints[3].device_version = 0u;
    endpoints[3].cluster_count  = 0u;
    endpoints[3].clusters       = ep4_clusters;

    mode_basic_cluster_add_to_endpoint(&basic_ep4, &endpoints[3], "Nature Mode");
    mode_switch_cluster_add_to_endpoint(&g_nature_mode_cluster, &endpoints[3],
                                        MODE_SWITCH_NATURE);

    /* ---- Build endpoint 5: Sleep Mode (DP2=2) ---- */
    hal_zigbee_cluster *ep5_clusters =
        clusters + endpoints[0].cluster_count + endpoints[1].cluster_count +
        endpoints[2].cluster_count + endpoints[3].cluster_count;
    endpoints[4].endpoint       = SLEEP_MODE_ENDPOINT;
    endpoints[4].profile_id     = ZCL_HA_PROFILE;
    endpoints[4].device_id      = ZIGBEE_DEVICE_TYPE_ONOFF_OUTPUT;
    endpoints[4].device_version = 0u;
    endpoints[4].cluster_count  = 0u;
    endpoints[4].clusters       = ep5_clusters;

    mode_basic_cluster_add_to_endpoint(&basic_ep5, &endpoints[4], "Sleep Mode");
    mode_switch_cluster_add_to_endpoint(&g_sleep_mode_cluster, &endpoints[4],
                                        MODE_SWITCH_SLEEP);

    /* ---- Zigbee stack init ---- */
    hal_zigbee_init(endpoints, 5u);
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
