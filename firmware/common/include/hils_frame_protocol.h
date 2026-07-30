#ifndef HILS_FRAME_PROTOCOL_H
#define HILS_FRAME_PROTOCOL_H

#include <stdint.h>
#include <stdbool.h>

/*
 * HILS Frame Protocol
 *
 * Used for transferring MJPEG frames between:
 *   - ROS bridge node (PC) -> Pico#1 (USB CDC)
 *   - Pico#1 (SPI Master) -> Pico#2 (SPI Slave)
 *
 * Frame format:
 *   [SYNC_0][SYNC_1][payload_length (4 bytes, LE)][payload (N bytes)][checksum (1 byte)]
 *
 * Total overhead: 7 bytes per frame.
 */

#define HILS_FRAME_SYNC_0       0xAA
#define HILS_FRAME_SYNC_1       0x55
#define HILS_FRAME_HEADER_SIZE  6       /* 2 sync + 4 length */
#define HILS_FRAME_MAX_PAYLOAD  65536   /* 64KB max JPEG frame */
#define HILS_FRAME_OVERHEAD     7       /* header + checksum */

/* UART configuration for Pico#1 <-> Pico#2 link */
#define HILS_UART_INSTANCE      uart0
#define HILS_UART_BAUDRATE      4000000   /* 4 Mbps — stable between RP2040s (same PLL) */
#define HILS_UART_PIN_TX        0         /* Pico#1 TX → Pico#2 RX */
#define HILS_UART_PIN_RX        1         /* Pico#2 RX ← Pico#1 TX */

/* Packed frame header */
typedef struct __attribute__((packed)) {
    uint8_t  sync_0;          /* HILS_FRAME_SYNC_0 */
    uint8_t  sync_1;          /* HILS_FRAME_SYNC_1 */
    uint32_t payload_length;  /* little-endian */
} hils_frame_header_t;

/* Receiver state machine states */
typedef enum {
    HILS_RX_WAIT_SYNC0,
    HILS_RX_WAIT_SYNC1,
    HILS_RX_READ_LENGTH,
    HILS_RX_READ_PAYLOAD,
    HILS_RX_READ_CHECKSUM,
} hils_rx_state_t;

/* Compute XOR checksum over data */
static inline uint8_t hils_compute_checksum(const uint8_t *data, uint32_t len) {
    uint8_t cs = 0;
    for (uint32_t i = 0; i < len; i++) {
        cs ^= data[i];
    }
    return cs;
}

/* Build frame header */
static inline void hils_build_header(hils_frame_header_t *hdr, uint32_t payload_len) {
    hdr->sync_0 = HILS_FRAME_SYNC_0;
    hdr->sync_1 = HILS_FRAME_SYNC_1;
    hdr->payload_length = payload_len;
}

/*
 * Reverse-channel command: resolution change notification
 *
 * Sent from Pico#2 -> Pico#1 -> ROS2 node when the UVC host
 * selects a new resolution. Uses the same AA 55 framing.
 * JPEG payloads always start with 0xFF (SOI), so msg_type=0x01
 * is unambiguous.
 */
#define HILS_MSG_TYPE_RESOLUTION_CMD  0x01

typedef struct __attribute__((packed)) {
    uint8_t  msg_type;       /* HILS_MSG_TYPE_RESOLUTION_CMD */
    uint16_t width;          /* little-endian */
    uint16_t height;         /* little-endian */
    uint8_t  frame_index;    /* UVC bFrameIndex (1-based) */
} hils_resolution_cmd_t;

#define HILS_RESOLUTION_CMD_SIZE  sizeof(hils_resolution_cmd_t)

/*
 * LiDAR message types (for Livox HILS bridge)
 *
 * Forward channel: ROS node -> W5500-EVB-Pico2
 *   0x10: Point cloud data batch
 *   0x11: IMU data
 *   0x12: Configuration command
 *
 * Reverse channel: W5500-EVB-Pico2 -> ROS node
 *   0x18: Status report
 */
#define HILS_MSG_TYPE_LIDAR_POINTS    0x10
#define HILS_MSG_TYPE_LIDAR_IMU       0x11
#define HILS_MSG_TYPE_LIDAR_CONFIG    0x12
#define HILS_MSG_TYPE_LIDAR_STATUS    0x18

/* Sub-commands for HILS_MSG_TYPE_LIDAR_CONFIG */
#define HILS_LIDAR_CFG_SET_HOST_IP    0x01  /* Set host IP for UDP target */
#define HILS_LIDAR_CFG_SET_LIDAR_IP   0x02  /* Set emulated LiDAR IP */
#define HILS_LIDAR_CFG_SET_SN         0x03  /* Set serial number string */
#define HILS_LIDAR_CFG_START          0x04  /* Start emulation */
#define HILS_LIDAR_CFG_STOP           0x05  /* Stop emulation */

/* Point cloud batch header (follows msg_type byte) */
typedef struct __attribute__((packed)) {
    uint8_t  msg_type;        /* HILS_MSG_TYPE_LIDAR_POINTS */
    uint8_t  data_type;       /* 1=CartesianHigh, 2=CartesianLow */
    uint16_t dot_num;         /* Number of points in this batch */
    uint32_t timestamp_lo;    /* Timestamp low 32 bits (ns) */
    uint32_t timestamp_hi;    /* Timestamp high 32 bits (ns) */
    /* Followed by dot_num * point_size bytes of point data */
} hils_lidar_points_header_t;

/* Cartesian high-precision point (matches Livox SDK2 format, 14 bytes) */
typedef struct __attribute__((packed)) {
    int32_t  x;               /* X axis, unit: mm */
    int32_t  y;               /* Y axis, unit: mm */
    int32_t  z;               /* Z axis, unit: mm */
    uint8_t  reflectivity;    /* 0-255 */
    uint8_t  tag;             /* Point tag */
} hils_lidar_point_cartesian_high_t;

/* IMU data (follows msg_type byte, 25 bytes total) */
typedef struct __attribute__((packed)) {
    uint8_t  msg_type;        /* HILS_MSG_TYPE_LIDAR_IMU */
    float    gyro_x;          /* rad/s */
    float    gyro_y;
    float    gyro_z;
    float    acc_x;           /* g */
    float    acc_y;
    float    acc_z;
} hils_lidar_imu_data_t;

/* Configuration command (follows msg_type byte) */
typedef struct __attribute__((packed)) {
    uint8_t  msg_type;        /* HILS_MSG_TYPE_LIDAR_CONFIG */
    uint8_t  sub_cmd;         /* HILS_LIDAR_CFG_* */
    uint8_t  data[];          /* Variable-length data */
} hils_lidar_config_cmd_t;

/* Status report from firmware */
typedef struct __attribute__((packed)) {
    uint8_t  msg_type;        /* HILS_MSG_TYPE_LIDAR_STATUS */
    uint8_t  state;           /* 0=idle, 1=ready, 2=streaming */
    uint32_t points_sent;     /* Total points sent via Ethernet */
    uint32_t udp_errors;      /* UDP send error count */
} hils_lidar_status_t;

/*
 * Fault injection commands (docs section 9.3 / Phase 5)
 *
 * Forward channel: ROS node -> Pico#1 -> device firmware
 *   0x50: set a firmware-level fault
 *   0x51: clear a firmware-level fault (fault_code 0 = clear all)
 *   0x5F: reboot into BOOTSEL for remote reflashing (magic-guarded)
 * Reverse channel: device firmware -> ROS node
 *   0x52: acknowledge a set/clear command
 *
 * fault_code values are namespaced per device firmware. UVC bridge
 * (rp2040_camera_uvc):
 *   0x01 USB_DETACH    arg0 = auto-reconnect after N ms (0 = stay
 *                             detached until FAULT_CLEAR)
 *   0x02 FRAME_DROP    arg0 = drop percentage 1-100 (deterministic
 *                             accumulator thinning, no RNG)
 *   0x03 PARTIAL_FRAME arg0 = percentage of each frame to keep, 1-99
 */
#define HILS_MSG_TYPE_FAULT_SET      0x50
#define HILS_MSG_TYPE_FAULT_CLEAR    0x51
#define HILS_MSG_TYPE_FAULT_ACK      0x52
#define HILS_MSG_TYPE_RESET_BOOTSEL  0x5F

#define HILS_FAULT_UVC_USB_DETACH     0x01
#define HILS_FAULT_UVC_FRAME_DROP     0x02
#define HILS_FAULT_UVC_PARTIAL_FRAME  0x03

/* I2C sensor emulator (rp2040_imu_invensense_mpu6050); codes are a
 * separate namespace from the UVC bridge:
 *   0x01 NACK        device disappears from the bus (IC_ENABLE=0)
 *   0x02 RESP_DELAY  arg0 = clock-stretch on the first read byte of
 *                          each transaction, in microseconds (<=50000)
 *   0x03 REG_FREEZE  register map stops taking simulation updates
 *   0x04 WHO_AM_I    arg0 = value returned instead of 0x68
 */
#define HILS_FAULT_I2C_NACK           0x01
#define HILS_FAULT_I2C_RESP_DELAY     0x02
#define HILS_FAULT_I2C_REG_FREEZE     0x03
#define HILS_FAULT_I2C_WHO_AM_I       0x04

#define HILS_FAULT_ACK_OK             0x00
#define HILS_FAULT_ACK_UNKNOWN_CODE   0x01
#define HILS_FAULT_ACK_BAD_ARG        0x02

/* "BOOT" little-endian; RESET_BOOTSEL without this magic is ignored */
#define HILS_RESET_BOOTSEL_MAGIC      0x544F4F42u

typedef struct __attribute__((packed)) {
    uint8_t  msg_type;        /* HILS_MSG_TYPE_FAULT_SET */
    uint8_t  fault_code;      /* HILS_FAULT_* (device namespace) */
    uint32_t arg0;            /* little-endian, meaning per fault_code */
    uint32_t arg1;            /* reserved, 0 */
} hils_fault_set_t;

typedef struct __attribute__((packed)) {
    uint8_t  msg_type;        /* HILS_MSG_TYPE_FAULT_CLEAR */
    uint8_t  fault_code;      /* 0 = clear all */
} hils_fault_clear_t;

typedef struct __attribute__((packed)) {
    uint8_t  msg_type;        /* HILS_MSG_TYPE_FAULT_ACK */
    uint8_t  fault_code;      /* echoed */
    uint8_t  status;          /* HILS_FAULT_ACK_* */
} hils_fault_ack_t;

typedef struct __attribute__((packed)) {
    uint8_t  msg_type;        /* HILS_MSG_TYPE_RESET_BOOTSEL */
    uint32_t magic;           /* HILS_RESET_BOOTSEL_MAGIC */
} hils_reset_bootsel_t;

/*
 * Servo PWM capture message types (for PWM servo-input HILS bridge)
 *
 * Reverse channel: firmware -> ROS node
 *   0x21: Measured servo PWM pulse widths (per channel)
 *
 * The firmware captures the PWM signal that the robot controller
 * outputs (instead of generating one) and forwards the measurement
 * to ROS for evaluation.
 */
#define HILS_MSG_TYPE_SERVO_MEASURED  0x21

typedef struct __attribute__((packed)) {
    uint8_t  msg_type;        /* HILS_MSG_TYPE_SERVO_MEASURED */
    uint8_t  channel_count;   /* Number of channels reported */
} hils_servo_measured_header_t;

typedef struct __attribute__((packed)) {
    uint8_t  channel;         /* Channel index */
    uint8_t  valid;           /* 1 if pulse captured recently, 0 if timed out */
    uint16_t pulse_us;        /* Most recent pulse width in microseconds */
    uint16_t period_us;       /* Measured period between rising edges (0 if unknown) */
} hils_servo_measured_channel_t;

#endif /* HILS_FRAME_PROTOCOL_H */
