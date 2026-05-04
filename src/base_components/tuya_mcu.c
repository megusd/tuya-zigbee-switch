/**
 * tuya_mcu.c — Tuya MCU serial protocol driver (Telink TLSR8258 backend)
 *
 * Uses the Telink SDK hardware UART0 at 9600 8N1 on:
 *   TX  TUYA_MCU_UART_TX_PIN  (default GPIO_PA2)
 *   RX  TUYA_MCU_UART_RX_PIN  (default GPIO_PA0)
 *
 * Override these at compile time by passing -DTUYA_MCU_UART_TX_PIN=... etc.
 * to the compiler.
 *
 * Receive is driven by polling the UART RX FIFO from app_task() so no
 * interrupt plumbing is required.  At 9600 baud the max data rate is
 * ~960 B/s; the task loop runs at >1 kHz, so no bytes are ever lost.
 */

#include "tuya_mcu.h"
#include "hal/timer.h"

/* Pull in Telink SDK platform headers similarly to other HAL sources. */
#pragma pack(push, 1)
#include "tl_common.h"
#pragma pack(pop)

/* ---- UART pin defaults --------------------------------------------------- */
#ifndef TUYA_MCU_UART_TX_PIN
#define TUYA_MCU_UART_TX_PIN    UART_TX_PA2
#endif
#ifndef TUYA_MCU_UART_RX_PIN
#define TUYA_MCU_UART_RX_PIN    UART_RX_PA0
#endif

/* ---- UART configuration -------------------------------------------------- */
#define TUYA_UART_BAUDRATE  9600u

/* DMA RX buffer format used by Telink drv_uart:
 * [0..3] = payload length (little-endian), [4..] = payload bytes. */
#define TUYA_UART_RX_DMA_BUF_LEN  128u
static uint8_t uart_rx_dma_buf[TUYA_UART_RX_DMA_BUF_LEN];

/* ---- Receive frame state machine ----------------------------------------- */
typedef enum {
    RX_STATE_HEADER0 = 0,
    RX_STATE_HEADER1,
    RX_STATE_VERSION,
    RX_STATE_COMMAND,
    RX_STATE_LEN_HI,
    RX_STATE_LEN_LO,
    RX_STATE_DATA,
    RX_STATE_CHECKSUM,
} rx_state_t;

static rx_state_t rx_state      = RX_STATE_HEADER0;
static uint8_t    rx_version;
static uint8_t    rx_command;
static uint16_t   rx_len;
static uint16_t   rx_data_idx;
static uint8_t    rx_data[TUYA_MAX_PAYLOAD_LEN];
static uint8_t    rx_checksum_accum;

/* ---- Heartbeat state ----------------------------------------------------- */
static uint32_t last_heartbeat_ms = 0;

/* ---- Registered callback ------------------------------------------------- */
static tuya_dp_callback_t dp_callback = NULL;

static void rx_process_byte(uint8_t byte);

/* ========================================================================== */
/* Internal helpers                                                             */
/* ========================================================================== */

static uint8_t compute_checksum(uint8_t version, uint8_t command,
                                uint8_t len_hi, uint8_t len_lo,
                                const uint8_t *data, uint16_t data_len) {
    uint8_t sum = version + command + len_hi + len_lo;
    for (uint16_t i = 0; i < data_len; i++) {
        sum += data[i];
    }
    return sum;
}

static void uart_write_byte(uint8_t byte) {
    while (!drv_uart_tx_start(&byte, 1u)) {
    }
}

static void on_uart_rx_packet(void) {
    uint32_t len = ((uint32_t)uart_rx_dma_buf[0]) |
                   ((uint32_t)uart_rx_dma_buf[1] << 8u) |
                   ((uint32_t)uart_rx_dma_buf[2] << 16u) |
                   ((uint32_t)uart_rx_dma_buf[3] << 24u);
    if (len > (TUYA_UART_RX_DMA_BUF_LEN - 4u)) {
        len = TUYA_UART_RX_DMA_BUF_LEN - 4u;
    }

    for (uint32_t i = 0; i < len; i++) {
        rx_process_byte(uart_rx_dma_buf[4u + i]);
    }

    /* Clear captured length for the next packet. */
    uart_rx_dma_buf[0] = 0u;
    uart_rx_dma_buf[1] = 0u;
    uart_rx_dma_buf[2] = 0u;
    uart_rx_dma_buf[3] = 0u;
}

static void send_frame(uint8_t command, const uint8_t *payload, uint16_t len) {
    uint8_t len_hi = (uint8_t)(len >> 8);
    uint8_t len_lo = (uint8_t)(len & 0xFFu);
    uint8_t csum   = compute_checksum(TUYA_PROTO_VERSION, command,
                                      len_hi, len_lo, payload, len);

    uart_write_byte(TUYA_FRAME_HEADER_0);
    uart_write_byte(TUYA_FRAME_HEADER_1);
    uart_write_byte(TUYA_PROTO_VERSION);
    uart_write_byte(command);
    uart_write_byte(len_hi);
    uart_write_byte(len_lo);
    for (uint16_t i = 0; i < len; i++) {
        uart_write_byte(payload[i]);
    }
    uart_write_byte(csum);
}

/* ---- Process a fully-received frame ------------------------------------- */
static void process_frame(uint8_t command, const uint8_t *data, uint16_t len) {
    if (command == TUYA_CMD_HEARTBEAT) {
        /* MCU heartbeat response — nothing to do */
        return;
    }

    if (command == TUYA_CMD_DP_REPORT || command == TUYA_CMD_DP_SEND) {
        /* DP data: dp_id(1) | dp_type(1) | dp_len_hi(1) | dp_len_lo(1) | value */
        uint16_t offset = 0;
        while (offset + 4u <= len) {
            uint8_t  dp_id   = data[offset];
            uint8_t  dp_type = data[offset + 1u];
            uint16_t dp_len  = ((uint16_t)data[offset + 2u] << 8u) |
                                (uint16_t)data[offset + 3u];
            offset += 4u;

            if (offset + dp_len > len) break;

            int32_t value = 0;
            if (dp_type == TUYA_DP_TYPE_BOOL && dp_len == 1u) {
                value = (int32_t)data[offset];
            } else if (dp_type == TUYA_DP_TYPE_ENUM && dp_len == 1u) {
                value = (int32_t)data[offset];
            } else if (dp_type == TUYA_DP_TYPE_VALUE && dp_len == 4u) {
                value = (int32_t)(((uint32_t)data[offset]     << 24u) |
                                  ((uint32_t)data[offset + 1u] << 16u) |
                                  ((uint32_t)data[offset + 2u] <<  8u) |
                                   (uint32_t)data[offset + 3u]);
            }

            if (dp_callback != NULL) {
                dp_callback(dp_id, dp_type, value);
            }

            offset += dp_len;
        }
    }
}

/* ---- Feed one received byte through the frame state machine ------------- */
static void rx_process_byte(uint8_t byte) {
    switch (rx_state) {
    case RX_STATE_HEADER0:
        if (byte == TUYA_FRAME_HEADER_0) rx_state = RX_STATE_HEADER1;
        break;
    case RX_STATE_HEADER1:
        rx_state = (byte == TUYA_FRAME_HEADER_1) ? RX_STATE_VERSION
                                                  : RX_STATE_HEADER0;
        break;
    case RX_STATE_VERSION:
        rx_version         = byte;
        rx_checksum_accum  = byte;
        rx_state           = RX_STATE_COMMAND;
        break;
    case RX_STATE_COMMAND:
        rx_command         = byte;
        rx_checksum_accum += byte;
        rx_state           = RX_STATE_LEN_HI;
        break;
    case RX_STATE_LEN_HI:
        rx_len             = (uint16_t)byte << 8u;
        rx_checksum_accum += byte;
        rx_state           = RX_STATE_LEN_LO;
        break;
    case RX_STATE_LEN_LO:
        rx_len            |= (uint16_t)byte;
        rx_checksum_accum += byte;
        rx_data_idx        = 0;
        rx_state = (rx_len == 0u) ? RX_STATE_CHECKSUM : RX_STATE_DATA;
        break;
    case RX_STATE_DATA:
        if (rx_data_idx < TUYA_MAX_PAYLOAD_LEN) {
            rx_data[rx_data_idx] = byte;
        }
        rx_checksum_accum += byte;
        rx_data_idx++;
        if (rx_data_idx >= rx_len) {
            rx_state = RX_STATE_CHECKSUM;
        }
        break;
    case RX_STATE_CHECKSUM: {
        uint16_t actual_len = rx_len < TUYA_MAX_PAYLOAD_LEN ? rx_len
                                                             : TUYA_MAX_PAYLOAD_LEN;
        if (byte == rx_checksum_accum) {
            process_frame(rx_command, rx_data, actual_len);
        } else {
            printf("tuya_mcu: bad checksum got=0x%02X exp=0x%02X\r\n",
                   byte, rx_checksum_accum);
        }
        rx_state = RX_STATE_HEADER0;
        break;
    }
    default:
        rx_state = RX_STATE_HEADER0;
        break;
    }
}

/* ========================================================================== */
/* Public API                                                                   */
/* ========================================================================== */

void tuya_mcu_init(tuya_dp_callback_t cb) {
    dp_callback = cb;

    drv_uart_pin_set(TUYA_MCU_UART_TX_PIN, TUYA_MCU_UART_RX_PIN);
    (void)drv_uart_init(TUYA_UART_BAUDRATE, uart_rx_dma_buf,
                        TUYA_UART_RX_DMA_BUF_LEN, on_uart_rx_packet);

    last_heartbeat_ms = hal_millis();

    printf("tuya_mcu: init OK (9600 baud)\r\n");
}

void tuya_mcu_task(void) {
    /* Send heartbeat on schedule */
    uint32_t now = hal_millis();
    if ((now - last_heartbeat_ms) >= TUYA_HEARTBEAT_INTERVAL_MS) {
        last_heartbeat_ms = now;
        send_frame(TUYA_CMD_HEARTBEAT, NULL, 0);
    }
}

void tuya_mcu_send_bool(uint8_t dp_id, bool value) {
    uint8_t payload[5];
    payload[0] = dp_id;
    payload[1] = TUYA_DP_TYPE_BOOL;
    payload[2] = 0x00u;
    payload[3] = 0x01u;
    payload[4] = value ? 0x01u : 0x00u;
    send_frame(TUYA_CMD_DP_SEND, payload, sizeof(payload));
}

void tuya_mcu_send_enum(uint8_t dp_id, uint8_t value) {
    uint8_t payload[5];
    payload[0] = dp_id;
    payload[1] = TUYA_DP_TYPE_ENUM;
    payload[2] = 0x00u;
    payload[3] = 0x01u;
    payload[4] = value;
    send_frame(TUYA_CMD_DP_SEND, payload, sizeof(payload));
}

void tuya_mcu_send_value(uint8_t dp_id, int32_t value) {
    uint8_t payload[8];
    uint32_t uval = (uint32_t)value;
    payload[0] = dp_id;
    payload[1] = TUYA_DP_TYPE_VALUE;
    payload[2] = 0x00u;
    payload[3] = 0x04u;
    payload[4] = (uint8_t)(uval >> 24u);
    payload[5] = (uint8_t)(uval >> 16u);
    payload[6] = (uint8_t)(uval >>  8u);
    payload[7] = (uint8_t)(uval);
    send_frame(TUYA_CMD_DP_SEND, payload, sizeof(payload));
}

void tuya_mcu_query_all(void) {
    send_frame(TUYA_CMD_DP_QUERY, NULL, 0);
}
