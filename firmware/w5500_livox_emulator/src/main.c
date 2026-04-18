/*
 * W5500 Livox Mid360 Emulator - Main
 *
 * Receives point cloud and IMU data from ROS via USB CDC,
 * emulates a Livox Mid360 on Ethernet (W5500) using the SDK2 protocol.
 *
 * Hardware: W5500-EVB-Pico2
 *   - USB: CDC device (data from PC)
 *   - SPI0: W5500 Ethernet controller
 *   - UART0: Debug output (GP0=TX, GP1=RX)
 *   - LED: GP25 (status indicator)
 *
 * Data flow:
 *   PC (ROS node) -> USB CDC -> this firmware -> W5500 SPI -> Ethernet
 *   -> Robot PC (livox_ros_driver2)
 */

#include "pico/stdlib.h"
#include "tusb.h"

#include "w5500_spi.h"
#include "livox_protocol.h"
#include "cdc_receiver.h"
#include "hils_frame_protocol.h"

#include <stdio.h>
#include <string.h>

/* LED pin */
#define LED_PIN 25

/* Status report interval (ms) */
#define STATUS_INTERVAL_MS 5000

/* Global emulator state */
static livox_emulator_t g_emu;
static cdc_receiver_t   g_cdc_rx;
static uint32_t         g_last_status_ms = 0;

/* Default MAC address for emulated LiDAR */
static const uint8_t default_mac[6] = {0x02, 0x4C, 0x56, 0x58, 0x01, 0x00};
static const uint8_t default_subnet[4] = {255, 255, 255, 0};
static const uint8_t default_gateway[4] = {192, 168, 1, 1};

/* ── CDC callbacks ── */

static void on_points(uint8_t data_type, uint16_t dot_num,
                       uint64_t timestamp_ns,
                       const uint8_t *point_data, uint16_t point_data_len)
{
    (void)point_data_len;
    livox_emulator_send_points(&g_emu, point_data, dot_num,
                                data_type, timestamp_ns);
}

static void on_imu(float gyro_x, float gyro_y, float gyro_z,
                    float acc_x, float acc_y, float acc_z)
{
    livox_imu_raw_point_t imu;
    imu.gyro_x = gyro_x;
    imu.gyro_y = gyro_y;
    imu.gyro_z = gyro_z;
    imu.acc_x = acc_x;
    imu.acc_y = acc_y;
    imu.acc_z = acc_z;

    /* Use current time as timestamp (approximate) */
    uint64_t ts = (uint64_t)to_us_since_boot(get_absolute_time()) * 1000;
    livox_emulator_send_imu(&g_emu, &imu, ts);
}

static void on_config(uint8_t sub_cmd, const uint8_t *data, uint16_t data_len)
{
    switch (sub_cmd) {
    case HILS_LIDAR_CFG_SET_HOST_IP:
        if (data_len >= 4) {
            memcpy(g_emu.host_ip, data, 4);
            printf("[Main] Host IP set to %d.%d.%d.%d\n",
                   data[0], data[1], data[2], data[3]);
        }
        break;

    case HILS_LIDAR_CFG_SET_LIDAR_IP:
        if (data_len >= 4) {
            memcpy(g_emu.lidar_ip, data, 4);
            printf("[Main] LiDAR IP set to %d.%d.%d.%d\n",
                   data[0], data[1], data[2], data[3]);
            /* Reconfigure W5500 network with new IP */
            w5500_network_init(g_emu.lidar_ip, default_subnet,
                               default_gateway, default_mac);
        }
        break;

    case HILS_LIDAR_CFG_SET_SN:
        if (data_len >= 16) {
            memcpy(g_emu.serial_number, data, 16);
            printf("[Main] Serial number set\n");
        }
        break;

    case HILS_LIDAR_CFG_START:
        printf("[Main] Emulation READY (waiting for driver WorkModeControl)\n");
        /* Don't set streaming=true here. Streaming starts only when
         * livox_ros_driver2 sends WorkModeControl(Normal) via UDP.
         * This prevents sending data before the driver registers the device. */
        break;

    case HILS_LIDAR_CFG_STOP:
        printf("[Main] Emulation STOP\n");
        g_emu.streaming = false;
        g_emu.work_mode = LIVOX_WORK_MODE_STANDBY;
        break;

    default:
        printf("[Main] Unknown config sub_cmd=0x%02X\n", sub_cmd);
        break;
    }
}

/* ── LED patterns ── */

static void led_update(void)
{
    static uint32_t last_toggle = 0;
    uint32_t now = to_ms_since_boot(get_absolute_time());

    if (g_emu.streaming) {
        /* Solid ON when streaming */
        gpio_put(LED_PIN, 1);
    } else if (tud_cdc_connected()) {
        /* Slow blink when USB connected but not streaming */
        if (now - last_toggle >= 500) {
            gpio_xor_mask(1 << LED_PIN);
            last_toggle = now;
        }
    } else {
        /* Fast blink when USB not connected */
        if (now - last_toggle >= 100) {
            gpio_xor_mask(1 << LED_PIN);
            last_toggle = now;
        }
    }
}

/* ── Main ── */

int main(void)
{
    /* Initialize stdio (UART debug) */
    stdio_init_all();

    /* LED */
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    printf("\n=== W5500 Livox Mid360 Emulator ===\n");

    /* Initialize Livox emulator state */
    livox_emulator_init(&g_emu);

    /* Initialize W5500 SPI and network */
    w5500_spi_init();
    w5500_network_init(g_emu.lidar_ip, default_subnet,
                       default_gateway, default_mac);

    /* Open UDP sockets */
    if (!livox_emulator_open_sockets(&g_emu)) {
        printf("[Main] Failed to open sockets!\n");
        while (1) {
            gpio_xor_mask(1 << LED_PIN);
            sleep_ms(100);
        }
    }

    /* Initialize CDC receiver */
    cdc_receiver_init(&g_cdc_rx);
    cdc_receiver_set_callbacks(&g_cdc_rx, on_points, on_imu, on_config);

    printf("[Main] Ready. Waiting for USB CDC connection...\n");

    /* Main loop */
    while (1) {
        /* Poll Livox protocol (discovery + commands from robot PC) */
        livox_emulator_poll(&g_emu);

        /* Poll USB CDC (point cloud + IMU + config from simulation PC) */
        if (tud_cdc_connected()) {
            cdc_receiver_poll(&g_cdc_rx);
        }

        /* Periodic status report */
        uint32_t now = to_ms_since_boot(get_absolute_time());
        if (now - g_last_status_ms >= STATUS_INTERVAL_MS) {
            uint8_t state = g_emu.streaming ? 2 : (tud_cdc_connected() ? 1 : 0);
            cdc_send_status(state, g_emu.points_sent, g_emu.udp_errors);
            g_last_status_ms = now;
        }

        /* LED status indicator */
        led_update();
    }

    return 0;
}
