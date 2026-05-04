/**
 * app_fanlight.c — app_init() / app_task() shim for MODULE_ZTU_FANLIGHT
 *
 * This file satisfies the app.h interface (called from main.c) by delegating
 * to the fanlight-specific initialisation in fanlight_init.c.
 *
 * Link this instead of src/app.c when TUYA_MCU_FANLIGHT is defined.
 */

#include "app.h"
#include "fanlight_init.h"

void app_init(void) {
    fanlight_app_init();
}

void app_task(void) {
    fanlight_app_task();
}
