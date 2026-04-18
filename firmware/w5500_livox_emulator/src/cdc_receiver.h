#ifndef CDC_RECEIVER_H
#define CDC_RECEIVER_H

#include <stdint.h>
#include <stdbool.h>

/*
 * USB CDC receiver for HILS framed messages from ROS node.
 *
 * Receives point cloud, IMU, and config commands from the PC
 * using the HILS frame protocol (AA 55 framing).
 */

/* Maximum payload: point batch header (12) + 5000 pts * 14 bytes = 70012 */
#define CDC_RX_MAX_PAYLOAD   65536
#define CDC_BULK_BUF_SIZE    4096

/* Callback types */
typedef void (*cdc_points_callback_t)(uint8_t data_type, uint16_t dot_num,
                                       uint64_t timestamp_ns,
                                       const uint8_t *point_data,
                                       uint16_t point_data_len);

typedef void (*cdc_imu_callback_t)(float gyro_x, float gyro_y, float gyro_z,
                                    float acc_x, float acc_y, float acc_z);

typedef void (*cdc_config_callback_t)(uint8_t sub_cmd,
                                       const uint8_t *data, uint16_t data_len);

/* Receiver state */
typedef struct {
    /* State machine */
    int      state;
    uint8_t  length_buf[4];
    uint8_t  length_idx;
    uint32_t payload_length;
    uint8_t  *payload;
    uint32_t payload_idx;
    uint8_t  checksum;

    /* Callbacks */
    cdc_points_callback_t on_points;
    cdc_imu_callback_t    on_imu;
    cdc_config_callback_t on_config;
} cdc_receiver_t;

/* Initialize receiver */
void cdc_receiver_init(cdc_receiver_t *rx);

/* Set callbacks */
void cdc_receiver_set_callbacks(cdc_receiver_t *rx,
                                 cdc_points_callback_t on_points,
                                 cdc_imu_callback_t on_imu,
                                 cdc_config_callback_t on_config);

/* Poll USB CDC and process incoming data (call from main loop) */
void cdc_receiver_poll(cdc_receiver_t *rx);

/* Send status report back to ROS node via CDC */
void cdc_send_status(uint8_t state, uint32_t points_sent, uint32_t udp_errors);

/* Free resources */
void cdc_receiver_deinit(cdc_receiver_t *rx);

#endif /* CDC_RECEIVER_H */
