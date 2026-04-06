#ifndef SPI_SENDER_H
#define SPI_SENDER_H

#include <stdint.h>
#include <stdbool.h>

void spi_sender_init(void);
bool spi_sender_send_frame(const uint8_t *jpeg_data, uint32_t jpeg_len);

#endif
