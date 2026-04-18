/*
 * USB CDC Receiver - HILS frame protocol parser
 *
 * Reads data from USB CDC (stdio_usb), parses HILS frames, and
 * dispatches to appropriate callbacks based on message type.
 */

#include "cdc_receiver.h"
#include "hils_frame_protocol.h"

#include "pico/stdlib.h"
#include "tusb.h"   /* tud_cdc_read/write for bulk transfer */

#include <string.h>
#include <stdlib.h>
#include <stdio.h>

/* State machine states */
enum {
    RX_WAIT_SYNC0,
    RX_WAIT_SYNC1,
    RX_READ_LENGTH,
    RX_READ_PAYLOAD,
    RX_READ_CHECKSUM,
};

void cdc_receiver_init(cdc_receiver_t *rx)
{
    memset(rx, 0, sizeof(*rx));
    rx->state = RX_WAIT_SYNC0;
    rx->payload = (uint8_t *)malloc(CDC_RX_MAX_PAYLOAD);
}

void cdc_receiver_set_callbacks(cdc_receiver_t *rx,
                                 cdc_points_callback_t on_points,
                                 cdc_imu_callback_t on_imu,
                                 cdc_config_callback_t on_config)
{
    rx->on_points = on_points;
    rx->on_imu = on_imu;
    rx->on_config = on_config;
}

static void dispatch_message(cdc_receiver_t *rx, const uint8_t *payload,
                              uint32_t len)
{
    if (len < 1) return;

    uint8_t msg_type = payload[0];

    switch (msg_type) {
    case HILS_MSG_TYPE_LIDAR_POINTS: {
        /* Header: msg_type(1) + data_type(1) + dot_num(2) + ts_lo(4) + ts_hi(4) = 12 */
        if (len < 12) return;
        uint8_t data_type = payload[1];
        uint16_t dot_num = payload[2] | ((uint16_t)payload[3] << 8);
        uint32_t ts_lo = payload[4] | ((uint32_t)payload[5] << 8) |
                         ((uint32_t)payload[6] << 16) | ((uint32_t)payload[7] << 24);
        uint32_t ts_hi = payload[8] | ((uint32_t)payload[9] << 8) |
                         ((uint32_t)payload[10] << 16) | ((uint32_t)payload[11] << 24);
        uint64_t timestamp_ns = ((uint64_t)ts_hi << 32) | ts_lo;

        const uint8_t *point_data = payload + 12;
        uint16_t point_data_len = len - 12;

        if (rx->on_points) {
            rx->on_points(data_type, dot_num, timestamp_ns,
                          point_data, point_data_len);
        }
        break;
    }

    case HILS_MSG_TYPE_LIDAR_IMU: {
        /* msg_type(1) + 6 floats (24) = 25 bytes */
        if (len < 25) return;
        float gyro_x, gyro_y, gyro_z, acc_x, acc_y, acc_z;
        memcpy(&gyro_x, payload + 1, 4);
        memcpy(&gyro_y, payload + 5, 4);
        memcpy(&gyro_z, payload + 9, 4);
        memcpy(&acc_x, payload + 13, 4);
        memcpy(&acc_y, payload + 17, 4);
        memcpy(&acc_z, payload + 21, 4);

        if (rx->on_imu) {
            rx->on_imu(gyro_x, gyro_y, gyro_z, acc_x, acc_y, acc_z);
        }
        break;
    }

    case HILS_MSG_TYPE_LIDAR_CONFIG: {
        /* msg_type(1) + sub_cmd(1) + data(N) */
        if (len < 2) return;
        uint8_t sub_cmd = payload[1];
        const uint8_t *data = (len > 2) ? payload + 2 : NULL;
        uint16_t data_len = (len > 2) ? len - 2 : 0;

        if (rx->on_config) {
            rx->on_config(sub_cmd, data, data_len);
        }
        break;
    }

    default:
        printf("[CDC] Unknown msg_type=0x%02X\n", msg_type);
        break;
    }
}

static void process_byte(cdc_receiver_t *rx, uint8_t byte)
{
    switch (rx->state) {
    case RX_WAIT_SYNC0:
        if (byte == HILS_FRAME_SYNC_0)
            rx->state = RX_WAIT_SYNC1;
        break;

    case RX_WAIT_SYNC1:
        if (byte == HILS_FRAME_SYNC_1) {
            rx->state = RX_READ_LENGTH;
            rx->length_idx = 0;
        } else if (byte == HILS_FRAME_SYNC_0) {
            /* Stay in SYNC1 state */
        } else {
            rx->state = RX_WAIT_SYNC0;
        }
        break;

    case RX_READ_LENGTH:
        rx->length_buf[rx->length_idx++] = byte;
        if (rx->length_idx >= 4) {
            rx->payload_length = rx->length_buf[0] |
                                  ((uint32_t)rx->length_buf[1] << 8) |
                                  ((uint32_t)rx->length_buf[2] << 16) |
                                  ((uint32_t)rx->length_buf[3] << 24);
            if (rx->payload_length == 0 || rx->payload_length > CDC_RX_MAX_PAYLOAD) {
                rx->state = RX_WAIT_SYNC0;
            } else {
                rx->payload_idx = 0;
                rx->checksum = 0;
                rx->state = RX_READ_PAYLOAD;
            }
        }
        break;

    case RX_READ_PAYLOAD:
        rx->payload[rx->payload_idx++] = byte;
        rx->checksum ^= byte;
        if (rx->payload_idx >= rx->payload_length) {
            rx->state = RX_READ_CHECKSUM;
        }
        break;

    case RX_READ_CHECKSUM:
        if (byte == (rx->checksum & 0xFF)) {
            dispatch_message(rx, rx->payload, rx->payload_length);
        }
        rx->state = RX_WAIT_SYNC0;
        break;
    }
}

void cdc_receiver_poll(cdc_receiver_t *rx)
{
    /* Bulk read from TinyUSB CDC — much faster than getchar_timeout_us() */
    if (!tud_cdc_available()) return;

    uint8_t buf[CDC_BULK_BUF_SIZE];
    uint32_t n = tud_cdc_read(buf, sizeof(buf));
    for (uint32_t i = 0; i < n; i++) {
        process_byte(rx, buf[i]);
    }
}

void cdc_send_status(uint8_t state, uint32_t points_sent, uint32_t udp_errors)
{
    /* Build HILS frame with status payload */
    uint8_t payload[10];
    payload[0] = HILS_MSG_TYPE_LIDAR_STATUS;
    payload[1] = state;
    payload[2] = points_sent & 0xFF;
    payload[3] = (points_sent >> 8) & 0xFF;
    payload[4] = (points_sent >> 16) & 0xFF;
    payload[5] = (points_sent >> 24) & 0xFF;
    payload[6] = udp_errors & 0xFF;
    payload[7] = (udp_errors >> 8) & 0xFF;
    payload[8] = (udp_errors >> 16) & 0xFF;
    payload[9] = (udp_errors >> 24) & 0xFF;

    uint8_t frame[10 + HILS_FRAME_OVERHEAD];
    hils_frame_header_t *hdr = (hils_frame_header_t *)frame;
    hils_build_header(hdr, sizeof(payload));
    memcpy(frame + HILS_FRAME_HEADER_SIZE, payload, sizeof(payload));
    frame[HILS_FRAME_HEADER_SIZE + sizeof(payload)] =
        hils_compute_checksum(payload, sizeof(payload));

    /* Write via TinyUSB CDC bulk */
    uint16_t total = HILS_FRAME_HEADER_SIZE + sizeof(payload) + 1;
    tud_cdc_write(frame, total);
    tud_cdc_write_flush();
}

void cdc_receiver_deinit(cdc_receiver_t *rx)
{
    if (rx->payload) {
        free(rx->payload);
        rx->payload = NULL;
    }
}
