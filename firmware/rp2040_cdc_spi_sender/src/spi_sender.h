#ifndef SPI_SENDER_H
#define SPI_SENDER_H

#include <stdint.h>
#include <stdbool.h>

void spi_sender_init(void);
bool spi_sender_send_frame(const uint8_t *jpeg_data, uint32_t jpeg_len);

/* Poll UART RX for reverse-channel commands from Pico#2.
 * Returns true if a complete command was received.
 * cmd_buf must be at least 64 bytes. */
bool spi_sender_poll_reverse(uint8_t *cmd_buf, uint32_t *cmd_len);

#endif
