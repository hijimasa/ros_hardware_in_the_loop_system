/*
 * UART Sender/Receiver for HILS CDC Bridge (Pico#1)
 *
 * Forward path: Sends framed MJPEG data to Pico#2 via UART TX.
 * Reverse path: Receives commands from Pico#2 via UART RX.
 */

#include "spi_sender.h"
#include "hils_frame_protocol.h"

#include "hardware/uart.h"
#include "hardware/gpio.h"
#include "pico/stdlib.h"
#include <string.h>

#define LED_PIN 25

/* Reverse channel state machine */
static hils_rx_state_t rev_state = HILS_RX_WAIT_SYNC0;
static uint8_t  rev_length_buf[4];
static uint32_t rev_length_idx = 0;
static uint32_t rev_payload_length = 0;
static uint32_t rev_payload_idx = 0;
static uint8_t  rev_checksum = 0;

#define REV_BUF_SIZE  64  /* commands are small */
static uint8_t rev_payload[REV_BUF_SIZE];

static void rev_reset(void) {
    rev_state = HILS_RX_WAIT_SYNC0;
    rev_length_idx = 0;
    rev_payload_length = 0;
    rev_payload_idx = 0;
    rev_checksum = 0;
}

void spi_sender_init(void) {
    uart_init(HILS_UART_INSTANCE, HILS_UART_BAUDRATE);
    gpio_set_function(HILS_UART_PIN_TX, GPIO_FUNC_UART);
    gpio_set_function(HILS_UART_PIN_RX, GPIO_FUNC_UART);  /* RX for reverse-channel commands */

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_put(LED_PIN, 0);

    rev_reset();
}

bool spi_sender_send_frame(const uint8_t *jpeg_data, uint32_t jpeg_len) {
    if (jpeg_len == 0 || jpeg_len > HILS_FRAME_MAX_PAYLOAD) {
        return false;
    }

    hils_frame_header_t hdr;
    hils_build_header(&hdr, jpeg_len);
    uint8_t checksum = hils_compute_checksum(jpeg_data, jpeg_len);

    uart_write_blocking(HILS_UART_INSTANCE, (const uint8_t *)&hdr, sizeof(hdr));
    uart_write_blocking(HILS_UART_INSTANCE, jpeg_data, jpeg_len);
    uart_write_blocking(HILS_UART_INSTANCE, &checksum, 1);

    return true;
}

bool spi_sender_poll_reverse(uint8_t *cmd_buf, uint32_t *cmd_len) {
    while (uart_is_readable(HILS_UART_INSTANCE)) {
        uint8_t byte = uart_getc(HILS_UART_INSTANCE);

        switch (rev_state) {
        case HILS_RX_WAIT_SYNC0:
            if (byte == HILS_FRAME_SYNC_0) rev_state = HILS_RX_WAIT_SYNC1;
            break;

        case HILS_RX_WAIT_SYNC1:
            if (byte == HILS_FRAME_SYNC_1) {
                rev_state = HILS_RX_READ_LENGTH;
                rev_length_idx = 0;
            } else if (byte == HILS_FRAME_SYNC_0) {
                /* stay */
            } else {
                rev_reset();
            }
            break;

        case HILS_RX_READ_LENGTH:
            rev_length_buf[rev_length_idx++] = byte;
            if (rev_length_idx == 4) {
                rev_payload_length = (uint32_t)rev_length_buf[0]
                                   | ((uint32_t)rev_length_buf[1] << 8)
                                   | ((uint32_t)rev_length_buf[2] << 16)
                                   | ((uint32_t)rev_length_buf[3] << 24);
                if (rev_payload_length == 0 || rev_payload_length > REV_BUF_SIZE) {
                    rev_reset();
                } else {
                    rev_payload_idx = 0;
                    rev_checksum = 0;
                    rev_state = HILS_RX_READ_PAYLOAD;
                }
            }
            break;

        case HILS_RX_READ_PAYLOAD:
            rev_payload[rev_payload_idx++] = byte;
            rev_checksum ^= byte;
            if (rev_payload_idx >= rev_payload_length) {
                rev_state = HILS_RX_READ_CHECKSUM;
            }
            break;

        case HILS_RX_READ_CHECKSUM:
            if (byte == rev_checksum) {
                memcpy(cmd_buf, rev_payload, rev_payload_length);
                *cmd_len = rev_payload_length;
                rev_reset();
                return true;
            }
            rev_reset();
            break;
        }
    }
    return false;
}
