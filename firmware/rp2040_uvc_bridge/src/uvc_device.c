/*
 * UVC Device for HILS Camera Bridge (Pico#2)
 *
 * Streams MJPEG frames received from UART as a UVC camera.
 * Sends at the host-negotiated resolution. When no new frame is
 * available, re-sends the last frame to keep the stream alive.
 * Sends resolution commands back to Pico#1 via UART TX when
 * the host negotiates a new resolution.
 */

#include "bsp/board_api.h"
#include "tusb.h"
#include "uvc_device.h"
#include "frame_buffer.h"
#include "usb_descriptors.h"
#include "hils_frame_protocol.h"
#include <string.h>
#include <stdio.h>
#include "hardware/timer.h"
#include "hardware/uart.h"
#include "hardware/structs/usb_dpram.h"

#include "blank_jpeg.h"

/* Dedicated UVC transmit buffer */
#define UVC_TX_BUF_SIZE  FRAME_BUF_SIZE
static uint8_t uvc_tx_buf[UVC_TX_BUF_SIZE];
static uint32_t uvc_tx_len = 0;
static bool has_real_frame = false;

static volatile unsigned tx_busy = 0;
static unsigned already_sent = 0;
static uint32_t frame_count = 0;
static uint32_t xfer_fail_count = 0;

/* Current resolution — updated by tud_video_commit_cb */
static uint16_t current_width  = 640;
static uint16_t current_height = 480;

/* Frame size table (indexed by bFrameIndex, 1-based) */
static const struct { uint16_t w; uint16_t h; } frame_sizes[] = {
    {0, 0},           /* index 0: unused */
    {320, 240},       /* FRAME_INDEX_320x240 */
    {640, 480},       /* FRAME_INDEX_640x480 */
    {1280, 720},      /* FRAME_INDEX_1280x720 */
};

/* EP 0x81 buffer control register */
#define EP1_IN_BUF_CTRL  (&usb_dpram->ep_buf_ctrl[1].in)

bool video_is_busy(void) {
    return tx_busy != 0;
}

/* Wait for EP 0x81 hardware buffer to finish any pending transfer */
static void wait_ep_available_clear(void) {
    uint32_t timeout = time_us_32() + 10000;
    while (*EP1_IN_BUF_CTRL & USB_BUF_CTRL_AVAIL) {
        tud_task();
        if ((int32_t)(time_us_32() - timeout) > 0) {
            printf("[UVC] WARN: EP AVAIL timeout, clearing\n");
            *EP1_IN_BUF_CTRL &= ~USB_BUF_CTRL_AVAIL;
            break;
        }
    }
}

/* Send resolution command to Pico#1 via UART TX */
static void send_resolution_cmd(uint16_t width, uint16_t height, uint8_t frame_index) {
    hils_resolution_cmd_t cmd = {
        .msg_type    = HILS_MSG_TYPE_RESOLUTION_CMD,
        .width       = width,
        .height      = height,
        .frame_index = frame_index,
    };
    hils_frame_header_t hdr;
    hils_build_header(&hdr, sizeof(cmd));
    uint8_t checksum = hils_compute_checksum((const uint8_t *)&cmd, sizeof(cmd));

    uart_write_blocking(HILS_UART_INSTANCE, (const uint8_t *)&hdr, sizeof(hdr));
    uart_write_blocking(HILS_UART_INSTANCE, (const uint8_t *)&cmd, sizeof(cmd));
    uart_write_blocking(HILS_UART_INSTANCE, &checksum, 1);
}

/* Get the appropriate blank JPEG for the current resolution */
static void get_blank_jpeg(const uint8_t **buf, uint32_t *len) {
    if (current_width <= 320) {
        *buf = blank_jpeg_320x240;
        *len = sizeof(blank_jpeg_320x240);
    } else if (current_width <= 640) {
        *buf = blank_jpeg_640x480;
        *len = sizeof(blank_jpeg_640x480);
    } else {
        *buf = blank_jpeg_1280x720;
        *len = sizeof(blank_jpeg_1280x720);
    }
}

void video_task(void) {
    if (!tud_video_n_streaming(0, 0)) {
        if (already_sent) {
            wait_ep_available_clear();
            printf("[UVC] streaming stopped (sent %lu frames, %lu xfer fails)\n",
                   (unsigned long)frame_count, (unsigned long)xfer_fail_count);
        }
        already_sent = 0;
        tx_busy = 0;
        return;
    }

    if (tx_busy) {
        return;
    }

    if (!already_sent) {
        already_sent = 1;
        frame_count = 0;
        xfer_fail_count = 0;
        has_real_frame = false;
        printf("[UVC] streaming started (%ux%u)\n", current_width, current_height);
    }

    /* Always consume new UART frames */
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
        const uint8_t *blank;
        uint32_t blank_len;
        get_blank_jpeg(&blank, &blank_len);
        send_buf = blank;
        send_len = blank_len;
    }

    tx_busy = 1;
    if (!tud_video_n_frame_xfer(0, 0, (void *)send_buf, send_len)) {
        tx_busy = 0;
        xfer_fail_count++;
        if (xfer_fail_count <= 5) {
            printf("[UVC] xfer fail #%lu (len=%lu)\n",
                   (unsigned long)xfer_fail_count, (unsigned long)send_len);
        }
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

    uint8_t idx = parameters->bFrameIndex;
    if (idx >= 1 && idx <= NUM_FRAME_SIZES) {
        current_width  = frame_sizes[idx].w;
        current_height = frame_sizes[idx].h;
        /* Discard stale frames from previous resolution */
        has_real_frame = false;
        frame_buffer_get_read_buf()->ready = false;
        /* Notify ROS2 node of resolution change via Pico#1 */
        send_resolution_cmd(current_width, current_height, idx);
    }

    printf("[UVC] commit: fmt=%u frm=%u (%ux%u) maxPayload=%lu maxFrame=%lu\n",
           parameters->bFormatIndex, idx,
           current_width, current_height,
           (unsigned long)parameters->dwMaxPayloadTransferSize,
           (unsigned long)parameters->dwMaxVideoFrameSize);
    return VIDEO_ERROR_NONE;
}
