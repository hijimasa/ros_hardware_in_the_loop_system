/*
 * HILS I2C IMU Sensor Emulator (MPU-6050) - RP2040 Firmware
 *
 * Receives IMU sensor data from ROS2 node via USB CDC,
 * emulates an MPU-6050 I2C slave device using RP2040 hardware I2C.
 *
 * I2C slave output: SDA = GPIO 4, SCL = GPIO 5 (I2C0)
 * I2C address: 0x68 (MPU-6050 default, AD0=0)
 *
 * Data flow:
 *   ROS2 (Imu) -> USB CDC -> this firmware -> I2C slave register map
 *   External MCU (I2C master) reads registers as if talking to MPU-6050
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "pico/stdlib.h"
#include "pico/stdio_usb.h"
#include "pico/bootrom.h"
#include "hardware/i2c.h"
#include "hardware/irq.h"
#include "hardware/gpio.h"

#include "hils_frame_protocol.h"

/* ---------- Pin assignments ---------- */
#define SDA_PIN             4       /* I2C0 SDA (physical pin 6) */
#define SCL_PIN             5       /* I2C0 SCL (physical pin 7) */
#define LED_PIN             25
/* Driven high as a 3.3V rail for the external I2C pull-ups, so the
 * bus can be wired entirely on physical pins 5-8 (rail, SDA, SCL,
 * GND) without reaching the 3V3(OUT) pin 36. Two 4.7k pull-ups draw
 * at most ~1.4 mA, well inside the 12 mA drive strength. */
#define PULLUP_RAIL_PIN     3       /* physical pin 5 */

/* ---------- MPU-6050 Configuration ---------- */
#define MPU6050_ADDR        0x68    /* Default I2C address (AD0=0) */
#define I2C_BAUDRATE        400000  /* 400kHz Fast Mode */

/* MPU-6050 register addresses */
#define REG_ACCEL_XOUT_H    0x3B
#define REG_ACCEL_XOUT_L    0x3C
#define REG_ACCEL_YOUT_H    0x3D
#define REG_ACCEL_YOUT_L    0x3E
#define REG_ACCEL_ZOUT_H    0x3F
#define REG_ACCEL_ZOUT_L    0x40
#define REG_TEMP_OUT_H      0x41
#define REG_TEMP_OUT_L      0x42
#define REG_GYRO_XOUT_H     0x43
#define REG_GYRO_XOUT_L     0x44
#define REG_GYRO_YOUT_H     0x45
#define REG_GYRO_YOUT_L     0x46
#define REG_GYRO_ZOUT_H     0x47
#define REG_GYRO_ZOUT_L     0x48
#define REG_PWR_MGMT_1      0x6B
#define REG_PWR_MGMT_2      0x6C
#define REG_WHO_AM_I        0x75

/* Default register values */
#define WHO_AM_I_VALUE      0x68
#define PWR_MGMT_1_DEFAULT  0x40    /* Sleep mode by default */

/* ---------- Message type ---------- */
#define HILS_MSG_TYPE_I2C_SENSOR  0x30

/* ---------- Sensor data structure (from USB CDC) ---------- */
typedef struct __attribute__((packed)) {
    uint8_t  msg_type;      /* 0x30 */
    int16_t  accel_x;       /* big-endian raw, already scaled */
    int16_t  accel_y;
    int16_t  accel_z;
    int16_t  gyro_x;
    int16_t  gyro_y;
    int16_t  gyro_z;
    int16_t  temp;
} hils_i2c_sensor_data_t;

/* ---------- Register map ---------- */
#define REG_MAP_SIZE  128
static volatile uint8_t reg_map[REG_MAP_SIZE];
static volatile uint8_t reg_ptr = 0;   /* Current register pointer */

/* ---------- Fault injection state (docs 9.5 / Phase 5) ---------- */
static volatile bool     fault_nack = false;      /* IC_ENABLE held low */
static volatile uint32_t fault_delay_us = 0;      /* clock stretch */
static volatile bool     fault_reg_freeze = false;
/* Clock-stretch only the first read byte of each transaction, not all
 * 14 burst bytes: set on STOP/RESTART, consumed by the next RD_REQ. */
static volatile bool     stretch_next_rd = true;

/* Flag: true when master is writing (first byte after address is register pointer) */
static volatile bool reg_ptr_written = false;

/* ---------- CDC receive buffer ---------- */
#define RX_BUF_SIZE     64
static uint8_t rx_payload[RX_BUF_SIZE];

#define CDC_BUF_SIZE    256
static uint8_t cdc_buf[CDC_BUF_SIZE];

/* ---------- Frame protocol state machine ---------- */
static hils_rx_state_t rx_state = HILS_RX_WAIT_SYNC0;
static uint8_t  length_buf[4];
static uint32_t length_idx = 0;
static uint32_t payload_length = 0;
static uint32_t payload_idx = 0;
static uint8_t  running_checksum = 0;

static uint32_t frame_count = 0;

static void reset_state_machine(void) {
    rx_state = HILS_RX_WAIT_SYNC0;
    length_idx = 0;
    payload_length = 0;
    payload_idx = 0;
    running_checksum = 0;
}

static bool process_byte(uint8_t byte) {
    switch (rx_state) {
    case HILS_RX_WAIT_SYNC0:
        if (byte == HILS_FRAME_SYNC_0) {
            rx_state = HILS_RX_WAIT_SYNC1;
        }
        break;

    case HILS_RX_WAIT_SYNC1:
        if (byte == HILS_FRAME_SYNC_1) {
            rx_state = HILS_RX_READ_LENGTH;
            length_idx = 0;
        } else if (byte == HILS_FRAME_SYNC_0) {
            /* stay */
        } else {
            reset_state_machine();
        }
        break;

    case HILS_RX_READ_LENGTH:
        length_buf[length_idx++] = byte;
        if (length_idx == 4) {
            payload_length = (uint32_t)length_buf[0]
                           | ((uint32_t)length_buf[1] << 8)
                           | ((uint32_t)length_buf[2] << 16)
                           | ((uint32_t)length_buf[3] << 24);
            if (payload_length == 0 || payload_length > RX_BUF_SIZE) {
                reset_state_machine();
            } else {
                payload_idx = 0;
                running_checksum = 0;
                rx_state = HILS_RX_READ_PAYLOAD;
            }
        }
        break;

    case HILS_RX_READ_PAYLOAD:
        rx_payload[payload_idx++] = byte;
        running_checksum ^= byte;
        if (payload_idx >= payload_length) {
            rx_state = HILS_RX_READ_CHECKSUM;
        }
        break;

    case HILS_RX_READ_CHECKSUM:
        if (byte == running_checksum) {
            rx_state = HILS_RX_WAIT_SYNC0;
            return true;
        }
        reset_state_machine();
        break;
    }
    return false;
}

/* ---------- Register map initialization ---------- */
static void init_register_map(void) {
    memset((void *)reg_map, 0, REG_MAP_SIZE);

    /* WHO_AM_I register: always 0x68 */
    reg_map[REG_WHO_AM_I] = WHO_AM_I_VALUE;

    /* PWR_MGMT_1: default is sleep mode (0x40) */
    reg_map[REG_PWR_MGMT_1] = PWR_MGMT_1_DEFAULT;

    /* PWR_MGMT_2: default 0x00 */
    reg_map[REG_PWR_MGMT_2] = 0x00;

    /* Default temperature: 25 deg C
     * MPU-6050 formula: Temperature = (TEMP_OUT / 340) + 36.53
     * raw = (25 - 36.53) * 340 = -3920.2 => -3920
     */
    int16_t temp_raw = -3920;
    reg_map[REG_TEMP_OUT_H] = (uint8_t)(temp_raw >> 8);
    reg_map[REG_TEMP_OUT_L] = (uint8_t)(temp_raw & 0xFF);
}

/* ---------- Fault command handling (docs 9.5 / Phase 5) ---------- */

/* Framed reverse-channel write over CDC (same pattern as Pico#1) */
static void send_ack(uint8_t fault_code, uint8_t status) {
    hils_fault_ack_t ack = {
        .msg_type   = HILS_MSG_TYPE_FAULT_ACK,
        .fault_code = fault_code,
        .status     = status,
    };
    hils_frame_header_t hdr;
    hils_build_header(&hdr, sizeof(ack));
    uint8_t checksum = hils_compute_checksum((const uint8_t *)&ack,
                                             sizeof(ack));
    const uint8_t *parts[] = {
        (const uint8_t *)&hdr, (const uint8_t *)&ack, &checksum
    };
    uint32_t sizes[] = { sizeof(hdr), sizeof(ack), 1 };
    for (int p = 0; p < 3; p++) {
        for (uint32_t j = 0; j < sizes[p]; j++) {
            putchar_raw(parts[p][j]);
        }
    }
    stdio_flush();
}

static void set_nack(bool active) {
    /* IC_ENABLE=0 makes the peripheral stop ACKing its address - the
     * device vanishes from the bus without touching the pins. */
    i2c0->hw->enable = active ? 0 : 1;
    fault_nack = active;
}

static void handle_fault_set(const uint8_t *payload, uint32_t len) {
    if (len < sizeof(hils_fault_set_t)) {
        send_ack(0, HILS_FAULT_ACK_BAD_ARG);
        return;
    }
    hils_fault_set_t cmd;
    memcpy(&cmd, payload, sizeof(cmd));

    switch (cmd.fault_code) {
    case HILS_FAULT_I2C_NACK:
        set_nack(true);
        break;
    case HILS_FAULT_I2C_RESP_DELAY:
        if (cmd.arg0 < 1 || cmd.arg0 > 50000) {
            send_ack(cmd.fault_code, HILS_FAULT_ACK_BAD_ARG);
            return;
        }
        fault_delay_us = cmd.arg0;
        break;
    case HILS_FAULT_I2C_REG_FREEZE:
        fault_reg_freeze = true;
        break;
    case HILS_FAULT_I2C_WHO_AM_I:
        if (cmd.arg0 > 0xFF) {
            send_ack(cmd.fault_code, HILS_FAULT_ACK_BAD_ARG);
            return;
        }
        reg_map[REG_WHO_AM_I] = (uint8_t)cmd.arg0;
        break;
    default:
        send_ack(cmd.fault_code, HILS_FAULT_ACK_UNKNOWN_CODE);
        return;
    }
    send_ack(cmd.fault_code, HILS_FAULT_ACK_OK);
}

static void handle_fault_clear(const uint8_t *payload, uint32_t len) {
    if (len < sizeof(hils_fault_clear_t)) {
        send_ack(0, HILS_FAULT_ACK_BAD_ARG);
        return;
    }
    uint8_t code = payload[1];
    if (code == 0 || code == HILS_FAULT_I2C_NACK) {
        set_nack(false);
    }
    if (code == 0 || code == HILS_FAULT_I2C_RESP_DELAY) {
        fault_delay_us = 0;
    }
    if (code == 0 || code == HILS_FAULT_I2C_REG_FREEZE) {
        fault_reg_freeze = false;
    }
    if (code == 0 || code == HILS_FAULT_I2C_WHO_AM_I) {
        reg_map[REG_WHO_AM_I] = WHO_AM_I_VALUE;
    }
    send_ack(code, HILS_FAULT_ACK_OK);
}

static void fault_process(const uint8_t *payload, uint32_t len) {
    switch (payload[0]) {
    case HILS_MSG_TYPE_FAULT_SET:
        handle_fault_set(payload, len);
        break;
    case HILS_MSG_TYPE_FAULT_CLEAR:
        handle_fault_clear(payload, len);
        break;
    case HILS_MSG_TYPE_RESET_BOOTSEL: {
        hils_reset_bootsel_t cmd;
        if (len >= sizeof(cmd)) {
            memcpy(&cmd, payload, sizeof(cmd));
            if (cmd.magic == HILS_RESET_BOOTSEL_MAGIC) {
                reset_usb_boot(0, 0);   /* does not return */
            }
        }
        break;
    }
    default:
        break;
    }
}

/* ---------- Update sensor registers from CDC data ---------- */
static void update_sensor_data(const uint8_t *payload, uint32_t len) {
    if (fault_reg_freeze) {
        return;   /* REG_FREEZE fault: registers keep their last values */
    }
    if (len < sizeof(hils_i2c_sensor_data_t)) {
        return;
    }

    const hils_i2c_sensor_data_t *data = (const hils_i2c_sensor_data_t *)payload;
    if (data->msg_type != HILS_MSG_TYPE_I2C_SENSOR) {
        return;
    }

    /*
     * Data arrives as little-endian int16 from USB CDC (host byte order).
     * MPU-6050 registers are big-endian (MSB first).
     * Convert to big-endian when storing in register map.
     */

    /* Accelerometer X/Y/Z */
    reg_map[REG_ACCEL_XOUT_H] = (uint8_t)(data->accel_x >> 8);
    reg_map[REG_ACCEL_XOUT_L] = (uint8_t)(data->accel_x & 0xFF);
    reg_map[REG_ACCEL_YOUT_H] = (uint8_t)(data->accel_y >> 8);
    reg_map[REG_ACCEL_YOUT_L] = (uint8_t)(data->accel_y & 0xFF);
    reg_map[REG_ACCEL_ZOUT_H] = (uint8_t)(data->accel_z >> 8);
    reg_map[REG_ACCEL_ZOUT_L] = (uint8_t)(data->accel_z & 0xFF);

    /* Gyroscope X/Y/Z */
    reg_map[REG_GYRO_XOUT_H] = (uint8_t)(data->gyro_x >> 8);
    reg_map[REG_GYRO_XOUT_L] = (uint8_t)(data->gyro_x & 0xFF);
    reg_map[REG_GYRO_YOUT_H] = (uint8_t)(data->gyro_y >> 8);
    reg_map[REG_GYRO_YOUT_L] = (uint8_t)(data->gyro_y & 0xFF);
    reg_map[REG_GYRO_ZOUT_H] = (uint8_t)(data->gyro_z >> 8);
    reg_map[REG_GYRO_ZOUT_L] = (uint8_t)(data->gyro_z & 0xFF);

    /* Temperature */
    reg_map[REG_TEMP_OUT_H] = (uint8_t)(data->temp >> 8);
    reg_map[REG_TEMP_OUT_L] = (uint8_t)(data->temp & 0xFF);
}

/* ---------- I2C Slave IRQ Handler ---------- */

/*
 * I2C slave interrupt handler.
 *
 * Handles two events:
 * 1. RD_REQ: Master wants to read data. Send byte from register map
 *            at current register pointer, then auto-increment pointer.
 * 2. RX_FULL: Master wrote data. First byte after address sets the
 *             register pointer. Subsequent bytes are written to registers
 *             (for writable registers like PWR_MGMT_1/2).
 *
 * This runs in ISR context - keep it minimal and fast.
 */
static void i2c_slave_irq_handler(void) {
    uint32_t status = i2c0->hw->intr_stat;

    /* Master requests a read: send data from register map */
    if (status & I2C_IC_INTR_STAT_R_RD_REQ_BITS) {
        /* RESP_DELAY fault: hold off the first byte of the transaction.
         * The bus clock-stretches while data_cmd stays empty, exactly
         * how a slow real sensor looks to the master. Busy-waiting in
         * the ISR is fine here - serving the bus IS this device's job. */
        if (fault_delay_us && stretch_next_rd) {
            busy_wait_us_32(fault_delay_us);
        }
        stretch_next_rd = false;

        /* Write the byte at current register pointer to TX buffer */
        uint8_t val = reg_map[reg_ptr & (REG_MAP_SIZE - 1)];
        i2c0->hw->data_cmd = (uint32_t)val;

        /* Auto-increment register pointer (MPU-6050 behavior) */
        reg_ptr++;

        /* Clear the RD_REQ interrupt */
        (void)i2c0->hw->clr_rd_req;
    }

    /* Master wrote data: update register pointer or register value */
    if (status & I2C_IC_INTR_STAT_R_RX_FULL_BITS) {
        uint8_t byte = (uint8_t)i2c0->hw->data_cmd;

        if (!reg_ptr_written) {
            /* First byte after START+address: this is the register pointer */
            reg_ptr = byte;
            reg_ptr_written = true;
        } else {
            /* Subsequent bytes: write to register map (for writable regs) */
            uint8_t addr = reg_ptr & (REG_MAP_SIZE - 1);

            /* Only allow writes to writable registers */
            switch (addr) {
            case REG_PWR_MGMT_1:
            case REG_PWR_MGMT_2:
                reg_map[addr] = byte;
                break;
            default:
                /* Ignore writes to read-only registers */
                break;
            }

            /* Auto-increment pointer */
            reg_ptr++;
        }
    }

    /* STOP or RESTART condition: reset write state for next transaction */
    if (status & I2C_IC_INTR_STAT_R_STOP_DET_BITS) {
        reg_ptr_written = false;
        stretch_next_rd = true;
        (void)i2c0->hw->clr_stop_det;
    }

    if (status & I2C_IC_INTR_STAT_R_RESTART_DET_BITS) {
        reg_ptr_written = false;
        stretch_next_rd = true;
        (void)i2c0->hw->clr_restart_det;
    }
}

/* ---------- I2C Slave Initialization ---------- */
static void i2c_slave_init(void) {
    /* 3.3V rail for the external pull-up resistors */
    gpio_init(PULLUP_RAIL_PIN);
    gpio_set_dir(PULLUP_RAIL_PIN, GPIO_OUT);
    gpio_set_drive_strength(PULLUP_RAIL_PIN, GPIO_DRIVE_STRENGTH_12MA);
    gpio_put(PULLUP_RAIL_PIN, 1);

    /* Initialize I2C0 peripheral */
    i2c_init(i2c0, I2C_BAUDRATE);

    /* Configure GPIO pins for I2C */
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);

    /* Switch to slave mode at MPU-6050 address */
    i2c_set_slave_mode(i2c0, true, MPU6050_ADDR);

    /* Enable relevant interrupts:
     * - RD_REQ:      Master wants to read
     * - RX_FULL:     Master wrote a byte
     * - STOP_DET:    STOP condition detected
     * - RESTART_DET: Repeated START detected
     */
    i2c0->hw->intr_mask = I2C_IC_INTR_MASK_M_RD_REQ_BITS
                        | I2C_IC_INTR_MASK_M_RX_FULL_BITS
                        | I2C_IC_INTR_MASK_M_STOP_DET_BITS
                        | I2C_IC_INTR_MASK_M_RESTART_DET_BITS;

    /* Set up IRQ handler */
    irq_set_exclusive_handler(I2C0_IRQ, i2c_slave_irq_handler);
    irq_set_enabled(I2C0_IRQ, true);
}

/* ---------- Main ---------- */
int main(void) {
    stdio_init_all();

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_put(LED_PIN, 0);

    /* Initialize register map with MPU-6050 defaults */
    init_register_map();

    /* Initialize I2C slave */
    i2c_slave_init();

    reset_state_machine();

    printf("\n[HILS] I2C IMU Sensor Emulator (MPU-6050) ready\n");
    printf("[HILS] I2C address: 0x%02X, SDA: GPIO%d, SCL: GPIO%d\n",
           MPU6050_ADDR, SDA_PIN, SCL_PIN);

    uint32_t last_led_toggle = 0;

    while (1) {
        /* Poll USB CDC for incoming frames */
        int count = stdio_usb_in_chars((char *)cdc_buf, CDC_BUF_SIZE);
        if (count > 0) {
            for (int i = 0; i < count; i++) {
                if (process_byte(cdc_buf[i])) {
                    uint8_t msg_type = rx_payload[0];
                    if (msg_type >= HILS_MSG_TYPE_FAULT_SET &&
                        msg_type <= HILS_MSG_TYPE_RESET_BOOTSEL) {
                        fault_process(rx_payload, payload_length);
                    } else {
                        update_sensor_data(rx_payload, payload_length);
                    }
                    frame_count++;

                    /* Toggle LED on each received frame */
                    gpio_xor_mask(1u << LED_PIN);
                }
            }
        }

        /* Slow blink LED when idle (no recent frames) */
        uint32_t now = time_us_32();
        if (frame_count == 0 && (now - last_led_toggle) >= 1000000) {
            gpio_xor_mask(1u << LED_PIN);
            last_led_toggle = now;
        }
    }

    return 0;
}
