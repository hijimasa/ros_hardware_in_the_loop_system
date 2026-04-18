/*
 * Livox SDK2 Protocol Emulator for Mid360
 *
 * Handles discovery, command processing, and data streaming
 * to make livox_ros_driver2 believe it's talking to a real Mid360.
 */

#include "livox_protocol.h"
#include "w5500_spi.h"

#include <string.h>
#include <stdio.h>

/* ── CRC tables ── */

/* CRC-16/CCITT (poly 0x1021, init 0xFFFF) */
static uint16_t crc16_ccitt(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int j = 0; j < 8; j++) {
            if (crc & 0x8000)
                crc = (crc << 1) ^ 0x1021;
            else
                crc = crc << 1;
        }
    }
    return crc;
}

/* CRC-32 (poly 0x04C11DB7, init 0xFFFFFFFF, reflected) */
static uint32_t crc32_compute(const uint8_t *data, uint32_t len)
{
    uint32_t crc = 0xFFFFFFFF;
    for (uint32_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 1)
                crc = (crc >> 1) ^ 0xEDB88320;
            else
                crc = crc >> 1;
        }
    }
    return crc ^ 0xFFFFFFFF;
}

/* ── Helper: build SDK2 response packet ── */

static uint16_t build_sdk2_packet(uint8_t *buf, uint16_t buf_size,
                                   uint16_t cmd_id, uint8_t cmd_type,
                                   uint32_t seq_num,
                                   const uint8_t *data, uint16_t data_len)
{
    uint16_t total_len = LIVOX_SDK2_HEADER_SIZE + data_len;
    if (total_len > buf_size) return 0;

    memset(buf, 0, LIVOX_SDK2_HEADER_SIZE);

    livox_sdk2_header_t *hdr = (livox_sdk2_header_t *)buf;
    hdr->sof = LIVOX_SOF;
    hdr->version = 0;
    hdr->length = total_len;
    hdr->seq_num = seq_num;
    hdr->cmd_id = cmd_id;
    hdr->cmd_type = cmd_type;
    hdr->sender_type = LIVOX_SENDER_LIDAR;

    /* Copy data after header */
    if (data && data_len > 0) {
        memcpy(buf + LIVOX_SDK2_HEADER_SIZE, data, data_len);
    }

    /* CRC-16 over first 18 bytes of header */
    hdr->crc16_h = crc16_ccitt(buf, 18);

    /* CRC-32 over data portion */
    if (data_len > 0) {
        hdr->crc32_d = crc32_compute(buf + LIVOX_SDK2_HEADER_SIZE, data_len);
    } else {
        hdr->crc32_d = 0;
    }

    return total_len;
}

/* ── Emulator implementation ── */

void livox_emulator_init(livox_emulator_t *emu)
{
    memset(emu, 0, sizeof(*emu));

    /* Default IPs */
    emu->lidar_ip[0] = 192; emu->lidar_ip[1] = 168;
    emu->lidar_ip[2] = 1;   emu->lidar_ip[3] = 12;
    emu->host_ip[0] = 192;  emu->host_ip[1] = 168;
    emu->host_ip[2] = 1;    emu->host_ip[3] = 5;

    memcpy(emu->serial_number, "0TFDFH600100511", 16);

    emu->sock_discovery = -1;
    emu->sock_cmd = -1;
    emu->sock_pointcloud = -1;
    emu->sock_imu = -1;

    emu->work_mode = LIVOX_WORK_MODE_STANDBY;
    emu->streaming = false;
}

bool livox_emulator_open_sockets(livox_emulator_t *emu)
{
    emu->sock_discovery = w5500_udp_open(LIVOX_PORT_DISCOVERY);
    if (emu->sock_discovery < 0) {
        printf("[Livox] Failed to open discovery socket\n");
        return false;
    }

    emu->sock_cmd = w5500_udp_open(LIVOX_PORT_CMD_LIDAR);
    if (emu->sock_cmd < 0) {
        printf("[Livox] Failed to open command socket\n");
        return false;
    }

    emu->sock_pointcloud = w5500_udp_open(LIVOX_PORT_POINTCLOUD_LIDAR);
    if (emu->sock_pointcloud < 0) {
        printf("[Livox] Failed to open pointcloud socket\n");
        return false;
    }

    emu->sock_imu = w5500_udp_open(LIVOX_PORT_IMU_LIDAR);
    if (emu->sock_imu < 0) {
        printf("[Livox] Failed to open IMU socket\n");
        return false;
    }

    printf("[Livox] All sockets opened\n");
    return true;
}

/* Handle discovery search command */
static void handle_discovery(livox_emulator_t *emu,
                              const uint8_t *pkt, uint16_t pkt_len,
                              const uint8_t src_ip[4], uint16_t src_port)
{
    if (pkt_len < LIVOX_SDK2_HEADER_SIZE) return;

    const livox_sdk2_header_t *hdr = (const livox_sdk2_header_t *)pkt;
    if (hdr->sof != LIVOX_SOF) return;
    if (hdr->cmd_id != LIVOX_CMD_LIDAR_SEARCH) return;
    if (hdr->cmd_type != LIVOX_CMD_TYPE_CMD) return;

    printf("[Livox] Discovery from %d.%d.%d.%d:%d\n",
           src_ip[0], src_ip[1], src_ip[2], src_ip[3], src_port);

    /* Remember host */
    memcpy(emu->host_discovered_ip, src_ip, 4);
    emu->host_discovered_port = src_port;
    emu->host_known = true;

    /* Build detection response */
    livox_detection_data_t det;
    det.ret_code = 0;
    det.dev_type = LIVOX_DEV_TYPE_MID360;
    memcpy(det.sn, emu->serial_number, 16);
    memcpy(det.lidar_ip, emu->lidar_ip, 4);
    det.cmd_port = LIVOX_PORT_CMD_LIDAR;

    uint8_t resp_buf[128];
    uint16_t resp_len = build_sdk2_packet(resp_buf, sizeof(resp_buf),
                                           LIVOX_CMD_LIDAR_SEARCH,
                                           LIVOX_CMD_TYPE_ACK,
                                           hdr->seq_num,
                                           (const uint8_t *)&det,
                                           sizeof(det));

    if (resp_len > 0) {
        w5500_udp_sendto(emu->sock_discovery, resp_buf, resp_len,
                         src_ip, src_port);
    }
}

/* Handle command channel */
static void handle_command(livox_emulator_t *emu,
                            const uint8_t *pkt, uint16_t pkt_len,
                            const uint8_t src_ip[4], uint16_t src_port)
{
    if (pkt_len < LIVOX_SDK2_HEADER_SIZE) return;

    const livox_sdk2_header_t *hdr = (const livox_sdk2_header_t *)pkt;
    if (hdr->sof != LIVOX_SOF) return;
    if (hdr->cmd_type != LIVOX_CMD_TYPE_CMD) return;

    uint8_t resp_buf[256];
    uint16_t resp_len = 0;

    switch (hdr->cmd_id) {
    case LIVOX_CMD_WORK_MODE_CTRL: {
        /* Data: 1 byte work mode */
        if (pkt_len >= LIVOX_SDK2_HEADER_SIZE + 1) {
            uint8_t mode = pkt[LIVOX_SDK2_HEADER_SIZE];
            emu->work_mode = mode;
            emu->streaming = (mode == LIVOX_WORK_MODE_NORMAL);
            printf("[Livox] WorkMode=%d, streaming=%d\n", mode, emu->streaming);
        }
        /* ACK with ret_code=0 */
        uint8_t ret = 0;
        resp_len = build_sdk2_packet(resp_buf, sizeof(resp_buf),
                                      LIVOX_CMD_WORK_MODE_CTRL,
                                      LIVOX_CMD_TYPE_ACK,
                                      hdr->seq_num, &ret, 1);
        break;
    }

    case LIVOX_CMD_GET_INTERNAL_INFO: {
        /* Return minimal key-value info */
        /* Format: ret_code(1) + param_num(2) + key-value pairs */
        uint8_t info[64];
        memset(info, 0, sizeof(info));
        info[0] = 0;  /* ret_code = success */
        /* param_num = 0 (minimal response) */
        info[1] = 0;
        info[2] = 0;
        resp_len = build_sdk2_packet(resp_buf, sizeof(resp_buf),
                                      LIVOX_CMD_GET_INTERNAL_INFO,
                                      LIVOX_CMD_TYPE_ACK,
                                      hdr->seq_num, info, 3);
        break;
    }

    default:
        printf("[Livox] Unknown cmd_id=0x%04X\n", hdr->cmd_id);
        /* Generic ACK with success */
        uint8_t ret = 0;
        resp_len = build_sdk2_packet(resp_buf, sizeof(resp_buf),
                                      hdr->cmd_id,
                                      LIVOX_CMD_TYPE_ACK,
                                      hdr->seq_num, &ret, 1);
        break;
    }

    if (resp_len > 0) {
        w5500_udp_sendto(emu->sock_cmd, resp_buf, resp_len,
                         src_ip, src_port);
    }
}

void livox_emulator_poll(livox_emulator_t *emu)
{
    uint8_t buf[256];
    uint8_t src_ip[4];
    uint16_t src_port;
    int n;

    /* Poll discovery socket */
    n = w5500_udp_recvfrom(emu->sock_discovery, buf, sizeof(buf),
                           src_ip, &src_port);
    if (n > 0) {
        handle_discovery(emu, buf, n, src_ip, src_port);
    }

    /* Poll command socket */
    n = w5500_udp_recvfrom(emu->sock_cmd, buf, sizeof(buf),
                           src_ip, &src_port);
    if (n > 0) {
        handle_command(emu, buf, n, src_ip, src_port);
    }
}

int livox_emulator_send_points(livox_emulator_t *emu,
                                const uint8_t *point_data,
                                uint16_t dot_num,
                                uint8_t data_type,
                                uint64_t timestamp_ns)
{
    if (!emu->streaming) return 0;

    /* Determine point size */
    uint16_t point_size;
    if (data_type == 1) {
        point_size = sizeof(livox_cartesian_high_point_t);
    } else if (data_type == 2) {
        point_size = 8;  /* CartesianLow */
    } else {
        return -1;
    }

    uint16_t data_len = dot_num * point_size;

    /* Build point cloud packet */
    /* Must fit in Ethernet MTU: max UDP payload = 1472 bytes (1500 - IP 20 - UDP 8) */
    /* W5500 does NOT support IP fragmentation, so oversized packets get corrupted */
    /* 1472 - 36 (header) = 1436 bytes for data → 102 pts @ 14B/pt */
    uint8_t pkt[1472];

    /* For larger batches, split into multiple UDP packets */
    uint16_t max_pts_per_pkt = (sizeof(pkt) - LIVOX_POINT_HEADER_SIZE) / point_size;
    uint16_t pts_remaining = dot_num;
    const uint8_t *ptr = point_data;

    while (pts_remaining > 0) {
        uint16_t pts_this = (pts_remaining > max_pts_per_pkt) ?
                             max_pts_per_pkt : pts_remaining;
        uint16_t payload_len = pts_this * point_size;

        livox_point_packet_header_t *hdr = (livox_point_packet_header_t *)pkt;
        memset(hdr, 0, LIVOX_POINT_HEADER_SIZE);

        hdr->version = 5;
        hdr->length = LIVOX_POINT_HEADER_SIZE + payload_len;
        hdr->time_interval = 47;  /* ~4.7us between points (Mid360 ~200k pts/s) */
        hdr->dot_num = pts_this;
        hdr->udp_cnt = emu->udp_cnt++;
        hdr->frame_cnt = emu->frame_cnt;
        hdr->data_type = data_type;
        hdr->time_type = 0;  /* No sync */

        /* Timestamp (little-endian 64-bit ns) */
        memcpy(hdr->timestamp, &timestamp_ns, 8);

        /* Copy point data */
        memcpy(pkt + LIVOX_POINT_HEADER_SIZE, ptr, payload_len);

        /* CRC-32 over point data */
        hdr->crc32 = crc32_compute(pkt + LIVOX_POINT_HEADER_SIZE, payload_len);

        /* Send to host */
        int ret = w5500_udp_sendto(emu->sock_pointcloud,
                                    pkt, LIVOX_POINT_HEADER_SIZE + payload_len,
                                    emu->host_ip, LIVOX_PORT_POINTCLOUD_HOST);
        if (ret < 0) {
            emu->udp_errors++;
            return -1;
        }

        emu->points_sent += pts_this;
        pts_remaining -= pts_this;
        ptr += payload_len;

        /* Advance timestamp for next sub-packet */
        timestamp_ns += (uint64_t)pts_this * 47 * 100;  /* 47 * 0.1us per point */
    }

    emu->frame_cnt++;
    return dot_num;
}

int livox_emulator_send_imu(livox_emulator_t *emu,
                             const livox_imu_raw_point_t *imu_data,
                             uint64_t timestamp_ns)
{
    if (!emu->streaming) return 0;

    uint8_t pkt[LIVOX_POINT_HEADER_SIZE + sizeof(livox_imu_raw_point_t)];

    livox_point_packet_header_t *hdr = (livox_point_packet_header_t *)pkt;
    memset(hdr, 0, LIVOX_POINT_HEADER_SIZE);

    hdr->version = 5;
    hdr->length = sizeof(pkt);
    hdr->time_interval = 0;
    hdr->dot_num = 1;
    hdr->udp_cnt = emu->udp_cnt++;
    hdr->frame_cnt = emu->frame_cnt;
    hdr->data_type = 0;  /* IMU */
    hdr->time_type = 0;

    memcpy(hdr->timestamp, &timestamp_ns, 8);
    memcpy(pkt + LIVOX_POINT_HEADER_SIZE, imu_data, sizeof(*imu_data));

    hdr->crc32 = crc32_compute(pkt + LIVOX_POINT_HEADER_SIZE,
                                sizeof(livox_imu_raw_point_t));

    int ret = w5500_udp_sendto(emu->sock_imu,
                                pkt, sizeof(pkt),
                                emu->host_ip, LIVOX_PORT_IMU_HOST);
    if (ret < 0) {
        emu->udp_errors++;
        return -1;
    }

    return 1;
}
