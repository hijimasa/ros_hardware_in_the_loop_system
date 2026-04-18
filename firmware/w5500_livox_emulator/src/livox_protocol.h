#ifndef LIVOX_PROTOCOL_H
#define LIVOX_PROTOCOL_H

#include <stdint.h>
#include <stdbool.h>

/*
 * Livox SDK2 Protocol Implementation (Mid360)
 *
 * Emulates the Livox Mid360 LiDAR's network protocol so that
 * livox_ros_driver2 on the robot PC sees a real device.
 *
 * Channels (Mid360):
 *   Discovery: UDP 56000 (broadcast, shared)
 *   Command:   LiDAR listens 56100, host listens 56101
 *   Push msg:  LiDAR sends to host:56201
 *   PointCloud: LiDAR sends from 56300 to host:56301
 *   IMU:       LiDAR sends from 56400 to host:56401
 */

/* ── Port definitions (Mid360) ── */
#define LIVOX_PORT_DISCOVERY        56000
#define LIVOX_PORT_CMD_LIDAR        56100
#define LIVOX_PORT_CMD_HOST         56101
#define LIVOX_PORT_PUSH_HOST        56201
#define LIVOX_PORT_POINTCLOUD_LIDAR 56300
#define LIVOX_PORT_POINTCLOUD_HOST  56301
#define LIVOX_PORT_IMU_LIDAR        56400
#define LIVOX_PORT_IMU_HOST         56401

/* ── Command protocol (SDK2) ── */
#define LIVOX_SOF  0xAA

/* Command IDs */
#define LIVOX_CMD_LIDAR_SEARCH      0x0000
#define LIVOX_CMD_WORK_MODE_CTRL    0x0100
#define LIVOX_CMD_GET_INTERNAL_INFO 0x0101
#define LIVOX_CMD_PUSH_MSG          0x0102
#define LIVOX_CMD_REBOOT            0x0200

/* Command types */
#define LIVOX_CMD_TYPE_CMD  0  /* Request */
#define LIVOX_CMD_TYPE_ACK  1  /* Response */

/* Sender types */
#define LIVOX_SENDER_HOST   0
#define LIVOX_SENDER_LIDAR  1

/* Device types */
#define LIVOX_DEV_TYPE_MID360  9

/* Work modes */
#define LIVOX_WORK_MODE_NORMAL    0x01
#define LIVOX_WORK_MODE_STANDBY   0x02

/* ── Command packet header (24 bytes) ── */
typedef struct __attribute__((packed)) {
    uint8_t  sof;           /* 0xAA */
    uint8_t  version;       /* Protocol version (0) */
    uint16_t length;        /* Total packet length (header + data) */
    uint32_t seq_num;       /* Sequence number */
    uint16_t cmd_id;        /* Command ID */
    uint8_t  cmd_type;      /* 0=CMD, 1=ACK */
    uint8_t  sender_type;   /* 0=host, 1=lidar */
    uint8_t  rsvd[6];       /* Reserved */
    uint16_t crc16_h;       /* CRC-16/CCITT over first 18 bytes */
    uint32_t crc32_d;       /* CRC-32 over data portion */
} livox_sdk2_header_t;

#define LIVOX_SDK2_HEADER_SIZE  sizeof(livox_sdk2_header_t)

/* ── Detection/Discovery response ── */
typedef struct __attribute__((packed)) {
    uint8_t  ret_code;      /* 0 = success */
    uint8_t  dev_type;      /* 9 = Mid360 */
    char     sn[16];        /* Serial number */
    uint8_t  lidar_ip[4];   /* LiDAR IP address */
    uint16_t cmd_port;      /* LiDAR command port (56100) */
} livox_detection_data_t;

/* ── Point cloud data packet header ── */
typedef struct __attribute__((packed)) {
    uint8_t  version;       /* Data packet version */
    uint16_t length;        /* Packet length */
    uint16_t time_interval; /* Time between points (0.1 us units) */
    uint16_t dot_num;       /* Number of points */
    uint16_t udp_cnt;       /* UDP packet counter */
    uint8_t  frame_cnt;     /* Frame counter */
    uint8_t  data_type;     /* 0=IMU, 1=CartHigh, 2=CartLow, 3=Spherical */
    uint8_t  time_type;     /* Timestamp type */
    uint8_t  rsvd[12];      /* Reserved */
    uint32_t crc32;         /* CRC-32 */
    uint8_t  timestamp[8];  /* 64-bit timestamp (ns) */
} livox_point_packet_header_t;

#define LIVOX_POINT_HEADER_SIZE  sizeof(livox_point_packet_header_t)

/* Cartesian high-precision point (14 bytes) -- matches Livox SDK2 exactly */
typedef struct __attribute__((packed)) {
    int32_t  x;             /* mm */
    int32_t  y;             /* mm */
    int32_t  z;             /* mm */
    uint8_t  reflectivity;
    uint8_t  tag;
} livox_cartesian_high_point_t;

/* IMU raw point (24 bytes) */
typedef struct __attribute__((packed)) {
    float gyro_x, gyro_y, gyro_z;  /* rad/s */
    float acc_x, acc_y, acc_z;     /* g */
} livox_imu_raw_point_t;

/* ── Emulator state ── */
typedef struct {
    /* Network config */
    uint8_t  lidar_ip[4];
    uint8_t  host_ip[4];
    char     serial_number[16];

    /* Sockets */
    int      sock_discovery;   /* UDP 56000 */
    int      sock_cmd;         /* UDP 56100 */
    int      sock_pointcloud;  /* UDP 56300 */
    int      sock_imu;         /* UDP 56400 */

    /* State */
    uint8_t  work_mode;        /* 0x01=normal, 0x02=standby */
    bool     streaming;        /* true if work_mode == normal */
    uint16_t udp_cnt;          /* Point cloud UDP packet counter */
    uint8_t  frame_cnt;        /* Frame counter */
    uint32_t points_sent;      /* Total points sent */
    uint32_t udp_errors;       /* UDP error counter */

    /* Host endpoint discovered from search command */
    uint8_t  host_discovered_ip[4];
    uint16_t host_discovered_port;
    bool     host_known;
} livox_emulator_t;

/* Initialize emulator state */
void livox_emulator_init(livox_emulator_t *emu);

/* Open all required UDP sockets */
bool livox_emulator_open_sockets(livox_emulator_t *emu);

/* Poll discovery and command channels (call from main loop) */
void livox_emulator_poll(livox_emulator_t *emu);

/* Send a point cloud packet */
int livox_emulator_send_points(livox_emulator_t *emu,
                                const uint8_t *point_data,
                                uint16_t dot_num,
                                uint8_t data_type,
                                uint64_t timestamp_ns);

/* Send an IMU data packet */
int livox_emulator_send_imu(livox_emulator_t *emu,
                             const livox_imu_raw_point_t *imu_data,
                             uint64_t timestamp_ns);

#endif /* LIVOX_PROTOCOL_H */
