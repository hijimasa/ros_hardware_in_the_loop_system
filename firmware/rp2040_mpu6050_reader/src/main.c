/*
 * MPU-6050 I2C master reader - test-harness "robot controller".
 *
 * Plays the real-driver role against the rp2040_imu_invensense_mpu6050
 * slave emulator (docs 9.5 / Phase 5): probes WHO_AM_I, wakes the
 * device, then burst-reads ACCEL..GYRO (0x3B-0x48) at 100 Hz exactly
 * like common MPU-6050 drivers.
 *
 * Every outcome is reported over USB CDC as parseable text lines so a
 * host-side script can act as the oracle:
 *   [BOOT]   ...                        once at startup
 *   [WHO]    who=0x68 ok=1              WHO_AM_I probe result
 *   [ERR]    kind=nack|timeout op=...   individual failure (rate-limited)
 *   [REINIT] consec_err=N               reader re-probing after errors
 *   [STAT]   t_ms=... reads=... ok=... nack=... timeout=... who=0x68
 *            ax=.. ay=.. az=.. gx=.. gy=.. gz=.. temp=..   (1 Hz)
 *
 * Wiring (both sides 3.3V, external 4.7k pull-ups on the slave rail):
 *   GPIO 4 (SDA) - slave GPIO 4, GPIO 5 (SCL) - slave GPIO 5, GND-GND.
 */

#include <stdio.h>
#include <string.h>

#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"

#define SDA_PIN          4
#define SCL_PIN          5
#define LED_PIN          25

#define MPU6050_ADDR     0x68
#define I2C_BAUDRATE     400000
#define REG_ACCEL_XOUT_H 0x3B
#define REG_PWR_MGMT_1   0x6B
#define REG_WHO_AM_I     0x75
#define WHO_AM_I_VALUE   0x68

#define BURST_LEN        14          /* 0x3B..0x48 */
#define READ_PERIOD_US   10000       /* 100 Hz */
#define IO_TIMEOUT_US    10000       /* classify slower responses as timeout */
#define REINIT_THRESHOLD 100         /* consecutive errors before re-probe */
#define ERR_LOG_PER_SEC  5

/* Counters (reset never; STAT reports running totals) */
static uint32_t reads = 0, ok_count = 0, nack_count = 0, timeout_count = 0;
static uint32_t consec_err = 0;
static uint8_t last_who = 0x00;

typedef enum { IO_OK = 0, IO_NACK, IO_TIMEOUT } io_result_t;

static io_result_t classify(int ret, int expected) {
    if (ret == expected) {
        return IO_OK;
    }
    return (ret == PICO_ERROR_TIMEOUT) ? IO_TIMEOUT : IO_NACK;
}

/* Register read: write pointer (no STOP), repeated-START read */
static io_result_t reg_read(uint8_t reg, uint8_t *dst, size_t len) {
    int ret = i2c_write_timeout_us(i2c0, MPU6050_ADDR, &reg, 1, true,
                                   IO_TIMEOUT_US);
    io_result_t r = classify(ret, 1);
    if (r != IO_OK) {
        return r;
    }
    ret = i2c_read_timeout_us(i2c0, MPU6050_ADDR, dst, len, false,
                              IO_TIMEOUT_US);
    return classify(ret, (int)len);
}

static io_result_t reg_write(uint8_t reg, uint8_t val) {
    uint8_t buf[2] = {reg, val};
    int ret = i2c_write_timeout_us(i2c0, MPU6050_ADDR, buf, 2, false,
                                   IO_TIMEOUT_US);
    return classify(ret, 2);
}

static void count_error(io_result_t r, const char *op,
                        uint32_t *err_logged_this_sec) {
    consec_err++;
    if (r == IO_TIMEOUT) {
        timeout_count++;
    } else {
        nack_count++;
    }
    if (*err_logged_this_sec < ERR_LOG_PER_SEC) {
        printf("[ERR] kind=%s op=%s consec=%lu\n",
               r == IO_TIMEOUT ? "timeout" : "nack", op,
               (unsigned long)consec_err);
        (*err_logged_this_sec)++;
    }
}

/* Bus + controller recovery: nine SCL pulses release a slave stuck
 * driving SDA, and the deinit/init clears the DW controller state
 * that stays wedged after an aborted (timed-out) transaction. */
static void i2c_recover(void) {
    i2c_deinit(i2c0);
    gpio_set_function(SCL_PIN, GPIO_FUNC_SIO);
    gpio_set_function(SDA_PIN, GPIO_FUNC_SIO);
    gpio_set_dir(SCL_PIN, GPIO_OUT);
    gpio_set_dir(SDA_PIN, GPIO_IN);
    for (int i = 0; i < 9; i++) {
        gpio_put(SCL_PIN, 0);
        busy_wait_us(5);
        gpio_put(SCL_PIN, 1);
        busy_wait_us(5);
    }
    i2c_init(i2c0, I2C_BAUDRATE);
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);
}

/* WHO_AM_I probe + wake, as a real driver's init sequence */
static void probe_and_wake(void) {
    uint8_t who = 0;
    io_result_t r = reg_read(REG_WHO_AM_I, &who, 1);
    last_who = (r == IO_OK) ? who : 0x00;
    printf("[WHO] who=0x%02X ok=%d\n", last_who,
           (r == IO_OK && who == WHO_AM_I_VALUE) ? 1 : 0);
    if (r == IO_OK) {
        reg_write(REG_PWR_MGMT_1, 0x00);   /* clear sleep bit */
    }
}

int main(void) {
    stdio_init_all();

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    i2c_init(i2c0, I2C_BAUDRATE);
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    /* External 4.7k pull-ups on the slave's GPIO3 rail; the internal
     * ones are enabled as a harmless extra. */
    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);

    /* Give USB CDC a moment so early lines are not lost */
    sleep_ms(2000);
    printf("\n[BOOT] MPU-6050 I2C master reader (SDA=GP%d SCL=GP%d %d kHz)\n",
           SDA_PIN, SCL_PIN, I2C_BAUDRATE / 1000);

    probe_and_wake();

    uint8_t burst[BURST_LEN];
    uint32_t next_read_us = time_us_32();
    uint32_t next_stat_us = time_us_32() + 1000000;
    uint32_t next_who_us = time_us_32() + 10000000;
    uint32_t err_logged_this_sec = 0;
    int16_t ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0, temp = 0;

    while (1) {
        uint32_t now = time_us_32();

        if ((int32_t)(now - next_read_us) >= 0) {
            next_read_us += READ_PERIOD_US;
            reads++;
            io_result_t r = reg_read(REG_ACCEL_XOUT_H, burst, BURST_LEN);
            if (r == IO_OK) {
                ok_count++;
                consec_err = 0;
                ax = (int16_t)((burst[0] << 8) | burst[1]);
                ay = (int16_t)((burst[2] << 8) | burst[3]);
                az = (int16_t)((burst[4] << 8) | burst[5]);
                temp = (int16_t)((burst[6] << 8) | burst[7]);
                gx = (int16_t)((burst[8] << 8) | burst[9]);
                gy = (int16_t)((burst[10] << 8) | burst[11]);
                gz = (int16_t)((burst[12] << 8) | burst[13]);
                gpio_xor_mask(1u << LED_PIN);
            } else {
                count_error(r, "burst_read", &err_logged_this_sec);
                if (consec_err % REINIT_THRESHOLD == 0) {
                    printf("[REINIT] consec_err=%lu\n",
                           (unsigned long)consec_err);
                    i2c_recover();
                    probe_and_wake();
                }
            }
        }

        /* Periodic identification check (health-monitor style): makes
         * the WHO_AM_I fault observable without restarting the reader. */
        if ((int32_t)(now - next_who_us) >= 0) {
            next_who_us += 10000000;
            uint8_t who = 0;
            io_result_t r = reg_read(REG_WHO_AM_I, &who, 1);
            last_who = (r == IO_OK) ? who : 0x00;
            printf("[WHO] who=0x%02X ok=%d\n", last_who,
                   (r == IO_OK && who == WHO_AM_I_VALUE) ? 1 : 0);
        }

        if ((int32_t)(now - next_stat_us) >= 0) {
            next_stat_us += 1000000;
            err_logged_this_sec = 0;
            printf("[STAT] t_ms=%lu reads=%lu ok=%lu nack=%lu timeout=%lu "
                   "who=0x%02X ax=%d ay=%d az=%d gx=%d gy=%d gz=%d temp=%d\n",
                   (unsigned long)(now / 1000), (unsigned long)reads,
                   (unsigned long)ok_count, (unsigned long)nack_count,
                   (unsigned long)timeout_count, last_who,
                   ax, ay, az, gx, gy, gz, temp);
        }
    }

    return 0;
}
