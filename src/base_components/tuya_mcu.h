/**
 * tuya_mcu.h — Tuya MCU serial protocol driver
 *
 * Implements the Tuya UART frame protocol used to communicate between a
 * secondary Tuya MCU (fan+light controller) and this Zigbee gateway module.
 *
 * Frame layout:  0x55 0xAA | ver(1) | cmd(1) | len_hi(1) | len_lo(1) | data(len) | checksum(1)
 * Checksum     = (sum of bytes from ver through end of data) & 0xFF
 * Baud rate    : 9600 8N1
 *
 * DP types:
 *   0x01 = bool   (1 byte)
 *   0x02 = value  (4 bytes, big-endian)
 *   0x04 = enum   (1 byte)
 *
 * Commands used:
 *   0x00 = heartbeat
 *   0x06 = DP send   (host → MCU)
 *   0x07 = DP receive (MCU → host, status report)
 */

#ifndef _TUYA_MCU_H_
#define _TUYA_MCU_H_

#include <stdbool.h>
#include <stdint.h>

/* ---- Tuya serial frame constants ---------------------------------------- */
#define TUYA_FRAME_HEADER_0    0x55u
#define TUYA_FRAME_HEADER_1    0xAAu
#define TUYA_PROTO_VERSION     0x03u   /* version field sent in every frame   */

/* Commands */
#define TUYA_CMD_HEARTBEAT     0x00u
#define TUYA_CMD_PRODUCT_QUERY 0x01u
#define TUYA_CMD_INIT_QUERY    0x02u
#define TUYA_CMD_DP_SEND       0x06u
#define TUYA_CMD_DP_REPORT     0x07u
#define TUYA_CMD_DP_QUERY      0x08u

/* DP types */
#define TUYA_DP_TYPE_BOOL      0x01u
#define TUYA_DP_TYPE_VALUE     0x02u   /* 4-byte big-endian signed int        */
#define TUYA_DP_TYPE_ENUM      0x04u

/* DP IDs for the fan+light MCU */
#define TUYA_DP_FAN_ONOFF      1u
#define TUYA_DP_FAN_SPEED      3u
#define TUYA_DP_FAN_DIRECTION  4u
#define TUYA_DP_LIGHT_ONOFF    9u
#define TUYA_DP_LIGHT_LEVEL    10u
#define TUYA_DP_LIGHT_COLORTEMP 11u
#define TUYA_DP_UNKNOWN_102    102u
#define TUYA_DP_UNKNOWN_103    103u

/* Heartbeat period (ms) */
#define TUYA_HEARTBEAT_INTERVAL_MS  3000u

/* Maximum payload bytes in one frame (DPs are small) */
#define TUYA_MAX_PAYLOAD_LEN        64u

/* ---- Callback type -------------------------------------------------------- */

/**
 * Called by the driver whenever the MCU sends a complete DP report frame.
 *
 * @param dp_id    Datapoint identifier
 * @param dp_type  One of TUYA_DP_TYPE_* values
 * @param value    Decoded value: bool→0/1, enum→raw byte, value→int32
 */
typedef void (*tuya_dp_callback_t)(uint8_t dp_id, uint8_t dp_type,
                                   int32_t value);

/* ---- Public API ----------------------------------------------------------- */

/**
 * Initialise the UART hardware and prepare internal state.
 * Call once from app_init() before hal_zigbee_init().
 *
 * @param cb  Callback invoked for every received DP report
 */
void tuya_mcu_init(tuya_dp_callback_t cb);

/**
 * Periodic maintenance — call from app_task() every loop iteration.
 * Drains the receive FIFO, reassembles frames, fires callbacks, and sends
 * heartbeats on schedule.
 */
void tuya_mcu_task(void);

/**
 * Send a boolean DP to the MCU.
 */
void tuya_mcu_send_bool(uint8_t dp_id, bool value);

/**
 * Send an enum DP to the MCU.
 */
void tuya_mcu_send_enum(uint8_t dp_id, uint8_t value);

/**
 * Send a 32-bit integer DP to the MCU.
 */
void tuya_mcu_send_value(uint8_t dp_id, int32_t value);

/**
 * Request the MCU to report all current DP values.
 */
void tuya_mcu_query_all(void);

#endif /* _TUYA_MCU_H_ */
