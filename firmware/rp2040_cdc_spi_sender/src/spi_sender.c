/*
 * UART Sender for HILS CDC Bridge (Pico#1)
 * Sends framed MJPEG data to Pico#2 via UART.
 */

#include "spi_sender.h"
#include "hils_frame_protocol.h"

#include "hardware/uart.h"
#include "hardware/gpio.h"
#include "pico/stdlib.h"

#define LED_PIN 25

void spi_sender_init(void) {
    /* Initialize UART */
    uart_init(HILS_UART_INSTANCE, HILS_UART_BAUDRATE);
    gpio_set_function(HILS_UART_PIN_TX, GPIO_FUNC_UART);
    /* RX pin not needed on sender side */

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_put(LED_PIN, 0);
}

bool spi_sender_send_frame(const uint8_t *jpeg_data, uint32_t jpeg_len) {
    if (jpeg_len == 0 || jpeg_len > HILS_FRAME_MAX_PAYLOAD) {
        return false;
    }

    hils_frame_header_t hdr;
    hils_build_header(&hdr, jpeg_len);
    uint8_t checksum = hils_compute_checksum(jpeg_data, jpeg_len);

    /* Send header + payload + checksum via UART */
    uart_write_blocking(HILS_UART_INSTANCE, (const uint8_t *)&hdr, sizeof(hdr));
    uart_write_blocking(HILS_UART_INSTANCE, jpeg_data, jpeg_len);
    uart_write_blocking(HILS_UART_INSTANCE, &checksum, 1);

    return true;
}
