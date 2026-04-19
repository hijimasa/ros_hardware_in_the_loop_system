#ifndef FRAME_BUFFER_H
#define FRAME_BUFFER_H

#include <stdint.h>
#include <stdbool.h>

#define FRAME_BUF_SIZE  (64 * 1024)  /* 64KB per buffer */
#define NUM_FRAME_BUFS  2            /* double buffering */

typedef struct {
    uint8_t  data[FRAME_BUF_SIZE];
    uint32_t length;       /* actual JPEG data length */
    volatile bool ready;   /* set by receiver, cleared by UVC sender */
} frame_buf_t;

void frame_buffer_init(void);

/* Get the buffer currently being written to (by SPI receiver) */
frame_buf_t *frame_buffer_get_write_buf(void);

/* Get the buffer currently being read from (by UVC sender) */
frame_buf_t *frame_buffer_get_read_buf(void);

/* Swap write and read buffers. Call after a complete frame is received. */
void frame_buffer_swap(void);

#endif /* FRAME_BUFFER_H */
