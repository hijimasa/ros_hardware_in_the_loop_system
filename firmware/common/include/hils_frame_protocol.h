#ifndef HILS_FRAME_PROTOCOL_H
#define HILS_FRAME_PROTOCOL_H

#include <stdint.h>
#include <stdbool.h>

/*
 * HILS Frame Protocol
 *
 * Used for transferring MJPEG frames between:
 *   - ROS bridge node (PC) -> Pico#1 (USB CDC)
 *   - Pico#1 (SPI Master) -> Pico#2 (SPI Slave)
 *
 * Frame format:
 *   [SYNC_0][SYNC_1][payload_length (4 bytes, LE)][payload (N bytes)][checksum (1 byte)]
 *
 * Total overhead: 7 bytes per frame.
 */

#define HILS_FRAME_SYNC_0       0xAA
#define HILS_FRAME_SYNC_1       0x55
#define HILS_FRAME_HEADER_SIZE  6       /* 2 sync + 4 length */
#define HILS_FRAME_MAX_PAYLOAD  65536   /* 64KB max JPEG frame */
#define HILS_FRAME_OVERHEAD     7       /* header + checksum */

/* UART configuration for Pico#1 <-> Pico#2 link */
#define HILS_UART_INSTANCE      uart0
#define HILS_UART_BAUDRATE      2000000   /* 2 Mbps */
#define HILS_UART_PIN_TX        0         /* Pico#1 TX → Pico#2 RX */
#define HILS_UART_PIN_RX        1         /* Pico#2 RX ← Pico#1 TX */

/* Packed frame header */
typedef struct __attribute__((packed)) {
    uint8_t  sync_0;          /* HILS_FRAME_SYNC_0 */
    uint8_t  sync_1;          /* HILS_FRAME_SYNC_1 */
    uint32_t payload_length;  /* little-endian */
} hils_frame_header_t;

/* Receiver state machine states */
typedef enum {
    HILS_RX_WAIT_SYNC0,
    HILS_RX_WAIT_SYNC1,
    HILS_RX_READ_LENGTH,
    HILS_RX_READ_PAYLOAD,
    HILS_RX_READ_CHECKSUM,
} hils_rx_state_t;

/* Compute XOR checksum over data */
static inline uint8_t hils_compute_checksum(const uint8_t *data, uint32_t len) {
    uint8_t cs = 0;
    for (uint32_t i = 0; i < len; i++) {
        cs ^= data[i];
    }
    return cs;
}

/* Build frame header */
static inline void hils_build_header(hils_frame_header_t *hdr, uint32_t payload_len) {
    hdr->sync_0 = HILS_FRAME_SYNC_0;
    hdr->sync_1 = HILS_FRAME_SYNC_1;
    hdr->payload_length = payload_len;
}

#endif /* HILS_FRAME_PROTOCOL_H */
