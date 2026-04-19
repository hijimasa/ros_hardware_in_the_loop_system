#ifndef SPI_RECEIVER_H
#define SPI_RECEIVER_H

#include <stdint.h>
#include <stdbool.h>

/* Initialize SPI0 in slave mode with DMA for receiving frames from Pico#1. */
void spi_receiver_init(void);

/*
 * Poll for received data and run the framing state machine.
 * Call this from the main loop.
 * Returns true when a complete valid frame has been placed into the write buffer.
 */
bool spi_receiver_poll(void);
void spi_receiver_debug_check(void);

#endif /* SPI_RECEIVER_H */
