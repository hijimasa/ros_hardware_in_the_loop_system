/*
 * HILS CDC-UART Bridge - Pico#1
 *
 * Receives HILS-framed MJPEG data from simulation PC via USB CDC,
 * forwards to Pico#2 via UART.
 *
 * Uses stdio_usb_in_chars() for bulk reads instead of getchar_timeout_us()
 * which only reads 1 byte at a time.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "pico/stdlib.h"
#include "pico/stdio_usb.h"
#include "hils_frame_protocol.h"
#include "spi_sender.h"

#define LED_PIN 25

/* Receive buffer for frame payload */
#define RX_BUF_SIZE  HILS_FRAME_MAX_PAYLOAD
static uint8_t rx_payload[RX_BUF_SIZE];

/* Bulk read buffer from CDC */
#define CDC_BUF_SIZE  4096
static uint8_t cdc_buf[CDC_BUF_SIZE];

/* Framing state machine */
static hils_rx_state_t rx_state = HILS_RX_WAIT_SYNC0;
static uint8_t  length_buf[4];
static uint32_t length_idx = 0;
static uint32_t payload_length = 0;
static uint32_t payload_idx = 0;
static uint8_t  running_checksum = 0;

/* Stats */
static uint32_t frame_count = 0;

static void reset_state_machine(void) {
    rx_state = HILS_RX_WAIT_SYNC0;
    length_idx = 0;
    payload_length = 0;
    payload_idx = 0;
    running_checksum = 0;
}

static bool process_byte(uint8_t byte) {
    switch (rx_state) {
    case HILS_RX_WAIT_SYNC0:
        if (byte == HILS_FRAME_SYNC_0) {
            rx_state = HILS_RX_WAIT_SYNC1;
        }
        break;

    case HILS_RX_WAIT_SYNC1:
        if (byte == HILS_FRAME_SYNC_1) {
            rx_state = HILS_RX_READ_LENGTH;
            length_idx = 0;
        } else if (byte == HILS_FRAME_SYNC_0) {
            /* stay */
        } else {
            reset_state_machine();
        }
        break;

    case HILS_RX_READ_LENGTH:
        length_buf[length_idx++] = byte;
        if (length_idx == 4) {
            payload_length = (uint32_t)length_buf[0]
                           | ((uint32_t)length_buf[1] << 8)
                           | ((uint32_t)length_buf[2] << 16)
                           | ((uint32_t)length_buf[3] << 24);
            if (payload_length == 0 || payload_length > RX_BUF_SIZE) {
                reset_state_machine();
            } else {
                payload_idx = 0;
                running_checksum = 0;
                rx_state = HILS_RX_READ_PAYLOAD;
            }
        }
        break;

    case HILS_RX_READ_PAYLOAD:
        rx_payload[payload_idx++] = byte;
        running_checksum ^= byte;
        if (payload_idx >= payload_length) {
            rx_state = HILS_RX_READ_CHECKSUM;
        }
        break;

    case HILS_RX_READ_CHECKSUM:
        if (byte == running_checksum) {
            rx_state = HILS_RX_WAIT_SYNC0;
            return true;
        }
        reset_state_machine();
        break;
    }
    return false;
}

int main(void) {
    stdio_init_all();

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_put(LED_PIN, 0);

    spi_sender_init();
    reset_state_machine();

    while (1) {
        /* Bulk read from USB CDC via stdio_usb_in_chars (up to 4KB at a time) */
        int count = stdio_usb_in_chars((char *)cdc_buf, CDC_BUF_SIZE);
        if (count <= 0) {
            continue;
        }

        for (int i = 0; i < count; i++) {
            if (process_byte(cdc_buf[i])) {
                spi_sender_send_frame(rx_payload, payload_length);
                frame_count++;
                gpio_xor_mask(1u << LED_PIN);
            }
        }
    }

    return 0;
}
