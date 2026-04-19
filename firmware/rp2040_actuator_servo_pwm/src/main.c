/*
 * HILS RC Servo PWM Capture - RP2040 Firmware
 *
 * Captures the PWM signals that a robot controller outputs toward its
 * RC servos and reports the measured pulse widths to a ROS 2 node via
 * USB CDC. This allows the simulator / test harness to evaluate the
 * commands the controller is issuing, instead of driving real servos.
 *
 * Direction:
 *   Robot controller -> GPIO 2-5 (inputs) -> PIO pulse measurement
 *                    -> USB CDC -> ROS 2 (/servo_pwm/measurements)
 *
 * The reverse-direction implementation (ROS -> PIO PWM output) lived
 * here previously but has been replaced — HILS needs to evaluate what
 * the controller commands, not substitute for it.
 *
 * Encoder pulse emulation (feedback direction, motor -> controller)
 * still lives in the separate firmware / ROS package
 * (rp2040_encoder_quadrature / hils_bridge_encoder_quadrature).
 */

#include <stdio.h>
#include <string.h>

#include "pico/stdlib.h"
#include "pico/stdio_usb.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"

#include "hils_frame_protocol.h"
#include "pwm_capture.pio.h"

/* ---------- Pin assignments ---------- */
#define SERVO_BASE_PIN      2       /* GPIO 2-5: servo PWM inputs */
#define LED_PIN             25

/* ---------- Configuration ---------- */
#define MAX_SERVO_CHANNELS  4

/* Report cadence toward host (microseconds). 20 ms = 50 Hz. */
#define REPORT_INTERVAL_US  20000u

/* A channel that has not produced a new measurement within this
 * many microseconds is reported as valid=0. 100 ms = 5 periods at 50 Hz. */
#define STALE_TIMEOUT_US    100000u

/* ---------- Per-channel capture state ---------- */
typedef struct {
    uint16_t last_pulse_us;      /* Most recent HIGH pulse width (us) */
    uint16_t last_period_us;     /* Time between last two rising edges (us) */
    uint32_t last_rise_us;       /* time_us_32() when last measurement was produced */
    uint32_t prev_rise_us;       /* Previous sample's timestamp, for period calc */
    bool     has_sample;         /* At least one measurement seen */
} capture_channel_t;

static capture_channel_t channels[MAX_SERVO_CHANNELS];

/* ---------- PIO handles ---------- */
static PIO capture_pio;
static uint capture_sm[MAX_SERVO_CHANNELS];
static uint capture_offset;

/* ---------- TX buffer for outgoing frames ---------- */
/* Header(6) + payload(max: 2 + 4 * 6 = 26) + checksum(1) = 33 bytes */
#define TX_BUF_SIZE     64
static uint8_t tx_buf[TX_BUF_SIZE];

/* ---------- Capture helpers ---------- */

static void capture_pio_init(void) {
    capture_pio = pio0;
    capture_offset = pio_add_program(capture_pio, &pwm_capture_program);

    for (int i = 0; i < MAX_SERVO_CHANNELS; i++) {
        capture_sm[i] = pio_claim_unused_sm(capture_pio, true);
        pwm_capture_program_init(capture_pio, capture_sm[i], capture_offset,
                                 SERVO_BASE_PIN + i);

        channels[i].last_pulse_us  = 0;
        channels[i].last_period_us = 0;
        channels[i].last_rise_us   = 0;
        channels[i].prev_rise_us   = 0;
        channels[i].has_sample     = false;
    }
}

/* Drain every channel's RX FIFO and update per-channel state.
 * Called frequently from the main loop.
 */
static void capture_poll(void) {
    uint32_t now = time_us_32();

    for (int i = 0; i < MAX_SERVO_CHANNELS; i++) {
        while (!pio_sm_is_rx_fifo_empty(capture_pio, capture_sm[i])) {
            uint32_t count = pio_sm_get(capture_pio, capture_sm[i]);

            /* Guard against absurd values before truncating. */
            uint16_t pulse_us = (count > 0xFFFFu) ? 0xFFFFu : (uint16_t)count;

            uint16_t period_us = 0;
            if (channels[i].has_sample) {
                uint32_t dt = now - channels[i].last_rise_us;
                period_us = (dt > 0xFFFFu) ? 0xFFFFu : (uint16_t)dt;
            }

            channels[i].prev_rise_us   = channels[i].last_rise_us;
            channels[i].last_rise_us   = now;
            channels[i].last_pulse_us  = pulse_us;
            channels[i].last_period_us = period_us;
            channels[i].has_sample     = true;
        }
    }
}

/* ---------- Framed report to host ---------- */

static void send_measurement_report(void) {
    uint32_t now = time_us_32();

    uint8_t *p = tx_buf;

    /* Frame sync bytes */
    *p++ = HILS_FRAME_SYNC_0;
    *p++ = HILS_FRAME_SYNC_1;

    /* Payload length placeholder (written after we know the size) */
    uint8_t *len_ptr = p;
    p += 4;

    uint8_t *payload_start = p;

    /* Payload header */
    *p++ = HILS_MSG_TYPE_SERVO_MEASURED;
    *p++ = MAX_SERVO_CHANNELS;

    /* One entry per channel */
    for (uint8_t i = 0; i < MAX_SERVO_CHANNELS; i++) {
        bool stale = !channels[i].has_sample ||
                     ((now - channels[i].last_rise_us) > STALE_TIMEOUT_US);

        uint16_t pulse  = stale ? 0 : channels[i].last_pulse_us;
        uint16_t period = stale ? 0 : channels[i].last_period_us;

        *p++ = i;                                /* channel */
        *p++ = stale ? 0u : 1u;                  /* valid flag */
        *p++ = (uint8_t)(pulse & 0xFF);          /* pulse_us LE */
        *p++ = (uint8_t)((pulse >> 8) & 0xFF);
        *p++ = (uint8_t)(period & 0xFF);         /* period_us LE */
        *p++ = (uint8_t)((period >> 8) & 0xFF);
    }

    uint32_t payload_len = (uint32_t)(p - payload_start);

    /* Write the length field (little-endian). */
    len_ptr[0] = (uint8_t)(payload_len & 0xFF);
    len_ptr[1] = (uint8_t)((payload_len >> 8) & 0xFF);
    len_ptr[2] = (uint8_t)((payload_len >> 16) & 0xFF);
    len_ptr[3] = (uint8_t)((payload_len >> 24) & 0xFF);

    /* Append XOR checksum */
    *p++ = hils_compute_checksum(payload_start, payload_len);

    uint32_t frame_len = (uint32_t)(p - tx_buf);
    for (uint32_t i = 0; i < frame_len; i++) {
        putchar_raw(tx_buf[i]);
    }
    stdio_flush();
}

/* ---------- Main ---------- */
int main(void) {
    stdio_init_all();

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_put(LED_PIN, 0);

    capture_pio_init();

    printf("\n[HILS] RC Servo PWM capture ready\n");
    printf("[HILS] Inputs: GPIO %d-%d (PWM pulse width measurement)\n",
           SERVO_BASE_PIN, SERVO_BASE_PIN + MAX_SERVO_CHANNELS - 1);

    uint32_t last_report_us = 0;
    uint32_t last_led_toggle_us = 0;

    while (1) {
        capture_poll();

        uint32_t now = time_us_32();

        if ((now - last_report_us) >= REPORT_INTERVAL_US) {
            last_report_us = now;
            send_measurement_report();
        }

        /* LED: fast blink when at least one channel is active,
         * slow blink otherwise. */
        bool any_active = false;
        for (int i = 0; i < MAX_SERVO_CHANNELS; i++) {
            if (channels[i].has_sample &&
                (now - channels[i].last_rise_us) <= STALE_TIMEOUT_US) {
                any_active = true;
                break;
            }
        }

        uint32_t led_period_us = any_active ? 100000u : 1000000u;
        if ((now - last_led_toggle_us) >= led_period_us) {
            gpio_xor_mask(1u << LED_PIN);
            last_led_toggle_us = now;
        }
    }

    return 0;
}
