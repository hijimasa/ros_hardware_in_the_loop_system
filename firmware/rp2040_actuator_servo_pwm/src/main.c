/*
 * HILS RC Servo PWM Emulator - RP2040 Firmware
 *
 * Receives servo angle commands from a ROS 2 node via USB CDC and outputs
 * 50 Hz RC servo PWM signals on GPIO 2-5 via PIO.
 *
 * Encoder feedback emulation is handled by a separate firmware /
 * ROS package (rp2040_encoder_quadrature / hils_bridge_encoder_quadrature)
 * because PWM (command direction, controller -> servo) and encoder pulses
 * (feedback direction, motor -> controller) play different roles in the
 * HILS loop and should not share a payload.
 *
 * Data flow:
 *   ROS 2 (JointState) -> USB CDC -> this firmware -> PIO PWM -> oscilloscope / receiver
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "pico/stdlib.h"
#include "pico/stdio_usb.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"

#include "hils_frame_protocol.h"
#include "pwm_servo.pio.h"

/* ---------- Pin assignments ---------- */
#define SERVO_BASE_PIN      2       /* GPIO 2-5: servo PWM outputs */
#define LED_PIN             25

/* ---------- Configuration ---------- */
#define MAX_SERVO_CHANNELS  4
#define SERVO_PERIOD_US     20000   /* 50Hz = 20ms period */
#define DEFAULT_PULSE_US    1500    /* Center position */

/* ---------- Message type ---------- */
#define HILS_MSG_TYPE_SERVO_CMD  0x20

/* ---------- Servo command structures ---------- */
typedef struct __attribute__((packed)) {
    uint8_t  msg_type;          /* 0x20 */
    uint8_t  channel_count;     /* Number of channels */
} hils_servo_cmd_header_t;

typedef struct __attribute__((packed)) {
    uint8_t  channel;           /* Channel index (0-3) */
    uint16_t pulse_us;          /* Pulse width in microseconds (500-2500) */
} hils_servo_channel_t;

/* ---------- Servo state ---------- */
static uint16_t servo_pulse_us[MAX_SERVO_CHANNELS];
static bool servo_updated[MAX_SERVO_CHANNELS];

/* ---------- PIO handles ---------- */
static PIO servo_pio;
static uint servo_sm[MAX_SERVO_CHANNELS];
static uint servo_offset;

/* ---------- CDC receive buffer ---------- */
#define RX_BUF_SIZE     256
static uint8_t rx_payload[RX_BUF_SIZE];

#define CDC_BUF_SIZE    512
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

/* ---------- Command processing ---------- */
static void process_servo_command(const uint8_t *payload, uint32_t len) {
    if (len < sizeof(hils_servo_cmd_header_t)) {
        return;
    }

    const hils_servo_cmd_header_t *hdr = (const hils_servo_cmd_header_t *)payload;
    if (hdr->msg_type != HILS_MSG_TYPE_SERVO_CMD) {
        return;
    }

    uint8_t ch_count = hdr->channel_count;
    uint32_t expected_len = sizeof(hils_servo_cmd_header_t)
                          + ch_count * sizeof(hils_servo_channel_t);
    if (len < expected_len) {
        return;
    }

    const hils_servo_channel_t *channels =
        (const hils_servo_channel_t *)(payload + sizeof(hils_servo_cmd_header_t));

    for (uint8_t i = 0; i < ch_count; i++) {
        uint8_t ch = channels[i].channel;
        if (ch < MAX_SERVO_CHANNELS) {
            uint16_t pulse = channels[i].pulse_us;
            if (pulse < 500) pulse = 500;
            if (pulse > 2500) pulse = 2500;
            servo_pulse_us[ch] = pulse;
            servo_updated[ch] = true;
        }
    }
}

/* ---------- PIO initialization ---------- */
static void servo_pio_init(void) {
    servo_pio = pio0;
    servo_offset = pio_add_program(servo_pio, &pwm_servo_program);

    for (int i = 0; i < MAX_SERVO_CHANNELS; i++) {
        servo_sm[i] = pio_claim_unused_sm(servo_pio, true);
        pwm_servo_program_init(servo_pio, servo_sm[i], servo_offset,
                               SERVO_BASE_PIN + i);
        servo_pulse_us[i] = DEFAULT_PULSE_US;
        servo_updated[i] = true;  /* Send initial position */
    }
}

/* ---------- Servo update ---------- */
static void servo_update(void) {
    for (int i = 0; i < MAX_SERVO_CHANNELS; i++) {
        if (servo_updated[i]) {
            servo_updated[i] = false;
            pwm_servo_set_pulse(servo_pio, servo_sm[i], servo_pulse_us[i]);
        }
    }
}

/* ---------- Main ---------- */
int main(void) {
    stdio_init_all();

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_put(LED_PIN, 0);

    servo_pio_init();
    reset_state_machine();

    printf("\n[HILS] RC Servo PWM emulator ready\n");
    printf("[HILS] Servo: GPIO %d-%d (50Hz, 500-2500us pulse width)\n",
           SERVO_BASE_PIN, SERVO_BASE_PIN + MAX_SERVO_CHANNELS - 1);

    uint32_t last_led_toggle = 0;

    while (1) {
        int count = stdio_usb_in_chars((char *)cdc_buf, CDC_BUF_SIZE);
        if (count > 0) {
            for (int i = 0; i < count; i++) {
                if (process_byte(cdc_buf[i])) {
                    process_servo_command(rx_payload, payload_length);
                    frame_count++;
                    gpio_xor_mask(1u << LED_PIN);
                }
            }
        }

        servo_update();

        uint32_t now = time_us_32();
        if (frame_count == 0 && (now - last_led_toggle) >= 1000000) {
            gpio_xor_mask(1u << LED_PIN);
            last_led_toggle = now;
        }
    }

    return 0;
}
