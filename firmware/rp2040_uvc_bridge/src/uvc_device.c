/*
 * UVC Device for HILS Camera Bridge (Pico#2)
 *
 * Uses a dedicated transmit buffer to avoid race conditions with
 * the UART receiver's frame buffers.
 */

#include "bsp/board_api.h"
#include "tusb.h"
#include "uvc_device.h"
#include "frame_buffer.h"
#include "usb_descriptors.h"
#include <string.h>
#include <stdio.h>
#include "hardware/timer.h"

#include "blank_jpeg.h"

/* Dedicated UVC transmit buffer - TinyUSB reads from this during transfer,
 * completely independent of the UART receiver's double buffers.
 * Once a real frame is received, uvc_tx_buf holds the latest frame and is
 * re-sent continuously until a new frame arrives. */
#define UVC_TX_BUF_SIZE  FRAME_BUF_SIZE
static uint8_t uvc_tx_buf[UVC_TX_BUF_SIZE];
static uint32_t uvc_tx_len = 0;
static bool has_real_frame = false;  /* true once first UART frame received */

static volatile unsigned tx_busy = 0;
static unsigned already_sent = 0;
static uint32_t stream_start_us = 0;
#define STREAM_SETTLE_US  20000  /* 20ms for endpoint to settle after Alt 1 */

bool video_is_busy(void) {
    return tx_busy != 0;
}

static uint32_t frame_count = 0;

void video_task(void) {
    if (!tud_video_n_streaming(0, 0)) {
        if (already_sent) {
            printf("[UVC] streaming stopped (sent %lu frames)\n", (unsigned long)frame_count);
        }
        already_sent = 0;
        tx_busy = 0;  /* Reset on streaming stop — prevents re-stream deadlock */
        return;
    }

    if (tx_busy) {
        return;
    }

    if (!already_sent) {
        already_sent = 1;
        frame_count = 0;
        stream_start_us = time_us_32();
        printf("[UVC] streaming started\n");
    }

    /* Wait briefly after stream start for endpoint to settle */
    if (time_us_32() - stream_start_us < STREAM_SETTLE_US) {
        return;
    }

    /* Check for new frame from UART receiver */
    frame_buf_t *rbuf = frame_buffer_get_read_buf();
    if (rbuf->ready && rbuf->length > 0) {
        uvc_tx_len = rbuf->length;
        memcpy(uvc_tx_buf, rbuf->data, uvc_tx_len);
        rbuf->ready = false;
        if (!has_real_frame) {
            has_real_frame = true;
            printf("[UVC] first real frame received (%lu bytes)\n", (unsigned long)uvc_tx_len);
        }
    }

    /* Send: latest real frame if available, otherwise blank JPEG */
    const void *send_buf;
    uint32_t send_len;
    if (has_real_frame) {
        send_buf = uvc_tx_buf;
        send_len = uvc_tx_len;
    } else {
        send_buf = blank_jpeg_640x480;
        send_len = sizeof(blank_jpeg_640x480);
    }

    tx_busy = 1;
    if (!tud_video_n_frame_xfer(0, 0, (void *)send_buf, send_len)) {
        tx_busy = 0;
    }
}

void tud_video_frame_xfer_complete_cb(uint_fast8_t ctl_idx, uint_fast8_t stm_idx) {
    (void)ctl_idx;
    (void)stm_idx;
    tx_busy = 0;
    frame_count++;
    if (frame_count <= 3 || (frame_count % 100) == 0) {
        printf("[UVC] frame #%lu complete\n", (unsigned long)frame_count);
    }
}

int tud_video_commit_cb(uint_fast8_t ctl_idx, uint_fast8_t stm_idx,
                        video_probe_and_commit_control_t const *parameters) {
    (void)ctl_idx;
    (void)stm_idx;
    printf("[UVC] commit: fmt=%u frm=%u interval=%lu maxPayload=%lu maxFrame=%lu\n",
           parameters->bFormatIndex, parameters->bFrameIndex,
           (unsigned long)parameters->dwFrameInterval,
           (unsigned long)parameters->dwMaxPayloadTransferSize,
           (unsigned long)parameters->dwMaxVideoFrameSize);
    return VIDEO_ERROR_NONE;
}
