#include "frame_buffer.h"
#include <string.h>

static frame_buf_t bufs[NUM_FRAME_BUFS];
static volatile int write_idx = 0;
static volatile int read_idx  = 1;

void frame_buffer_init(void) {
    memset(bufs, 0, sizeof(bufs));
    write_idx = 0;
    read_idx  = 1;
}

frame_buf_t *frame_buffer_get_write_buf(void) {
    return &bufs[write_idx];
}

frame_buf_t *frame_buffer_get_read_buf(void) {
    return &bufs[read_idx];
}

void frame_buffer_swap(void) {
    write_idx ^= 1;
    read_idx  ^= 1;
}
