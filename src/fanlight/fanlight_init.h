/**
 * fanlight_init.h — Fan+light device initialisation
 *
 * Provides app_init() / app_task() for the MODULE_ZTU_FANLIGHT build.
 * This replaces the generic parse_config() path entirely; do NOT link
 * config_parser.c or app.c when TUYA_MCU_FANLIGHT is defined.
 */

#ifndef _FANLIGHT_INIT_H_
#define _FANLIGHT_INIT_H_

void fanlight_app_init(void);
void fanlight_app_task(void);

#endif /* _FANLIGHT_INIT_H_ */
