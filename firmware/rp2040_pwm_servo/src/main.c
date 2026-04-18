/*
 * HILS PWM Servo + Encoder Emulator - RP2040 Firmware
 *
 * Receives servo angle commands from ROS2 node via USB CDC,
 * outputs PWM servo signals and quadrature encoder pulses via PIO.
 *
 * PWM servo output: GPIO 2-5 (up to 4 channels, 50Hz RC servo PWM)
 * Encoder output:   GPIO 6-9 (A0,B0 on 6-7, A1,B1 on 8-9)
 *
 * Data flow:
 *   ROS2 (JointState) -> USB CDC -> this firmware -> PIO PWM + Encoder
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
#include "encoder_output.pio.h"

/* ---------- Pin assignments ---------- */
#define SERVO_BASE_PIN      2       /* GPIO 2-5: servo PWM outputs */
#define ENCODER_BASE_PIN    6       /* GPIO 6-9: encoder A/B outputs */
#define LED_PIN             25

/* ---------- Configuration ---------- */
#define MAX_SERVO_CHANNELS  4
#define MAX_ENCODER_CHANNELS 2
#define SERVO_PERIOD_US     20000   /* 50Hz = 20ms period */
#define DEFAULT_PULSE_US    1500    /* Center position */

/* Encoder step frequency (max quadrature steps per second) */
#define ENCODER_STEP_FREQ   100000.0f  /* 100kHz max step rate */

/* ---------- Message type ---------- */
#define HILS_MSG_TYPE_SERVO_CMD  0x20

/* ---------- Servo command structures ---------- */
typedef struct __attribute__((packed)) {
    uint8_t  msg_type;          /* 0x20 */
    uint8_t  channel_count;     /* Number of channels (1-8) */
} hils_servo_cmd_header_t;

typedef struct __attribute__((packed)) {
    uint8_t  channel;           /* Channel index (0-7) */
    uint16_t pulse_us;          /* Pulse width in microseconds (500-2500) */
    int32_t  encoder_count;     /* Target encoder count */
} hils_servo_channel_t;

/* ---------- Servo state ---------- */
static uint16_t servo_pulse_us[MAX_SERVO_CHANNELS];
static bool servo_updated[MAX_SERVO_CHANNELS];

/* ---------- Encoder state ---------- */
/* Quadrature A/B lookup table (phase 0-3) */
/* Phase: 0->1->2->3->0 = forward, 0->3->2->1->0 = reverse */
/* Bits: [1]=B, [0]=A */
static const uint8_t quad_table[4] = {
    0b00,   /* Phase 0: A=0, B=0 */
    0b01,   /* Phase 1: A=1, B=0 */
    0b11,   /* Phase 2: A=1, B=1 */
    0b10,   /* Phase 3: A=0, B=1 */
};

typedef struct {
    int32_t current_count;      /* Current encoder position */
    int32_t target_count;       /* Target encoder position */
    uint8_t phase;              /* Current quadrature phase (0-3) */
} encoder_state_t;

static encoder_state_t encoder_state[MAX_ENCODER_CHANNELS];

/* ---------- PIO handles ---------- */
static PIO servo_pio;
static uint servo_sm[MAX_SERVO_CHANNELS];
static uint servo_offset;

static PIO encoder_pio;
static uint encoder_sm[MAX_ENCODER_CHANNELS];
static uint encoder_offset;

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

        /* Update servo PWM */
        if (ch < MAX_SERVO_CHANNELS) {
            uint16_t pulse = channels[i].pulse_us;
            if (pulse < 500) pulse = 500;
            if (pulse > 2500) pulse = 2500;
            servo_pulse_us[ch] = pulse;
            servo_updated[ch] = true;
        }

        /* Update encoder target */
        if (ch < MAX_ENCODER_CHANNELS) {
            encoder_state[ch].target_count = channels[i].encoder_count;
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

static void encoder_pio_init(void) {
    encoder_pio = pio1;
    encoder_offset = pio_add_program(encoder_pio, &encoder_output_program);

    for (int i = 0; i < MAX_ENCODER_CHANNELS; i++) {
        encoder_sm[i] = pio_claim_unused_sm(encoder_pio, true);
        encoder_output_program_init(encoder_pio, encoder_sm[i], encoder_offset,
                                     ENCODER_BASE_PIN + i * 2,
                                     ENCODER_STEP_FREQ);
        encoder_state[i].current_count = 0;
        encoder_state[i].target_count = 0;
        encoder_state[i].phase = 0;

        /* Send initial state (A=0, B=0) */
        pio_sm_put_blocking(encoder_pio, encoder_sm[i], quad_table[0]);
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

/* ---------- Encoder update ---------- */

/*
 * Feed encoder steps to PIO.
 * Called from main loop. Generates one quadrature step at a time
 * toward the target position, pushing A/B states to the PIO FIFO.
 * The PIO's clock divider controls the step rate.
 */
static void encoder_update(void) {
    for (int i = 0; i < MAX_ENCODER_CHANNELS; i++) {
        int32_t diff = encoder_state[i].target_count - encoder_state[i].current_count;
        if (diff == 0) {
            continue;
        }

        /* Only push if FIFO has space (non-blocking) */
        if (pio_sm_is_tx_fifo_full(encoder_pio, encoder_sm[i])) {
            continue;
        }

        /* Determine number of steps to push (fill FIFO up to capacity) */
        int steps_to_push = abs(diff);
        /* Limit to available FIFO space (4 entries without FIFO join) */
        int fifo_level = pio_sm_get_tx_fifo_level(encoder_pio, encoder_sm[i]);
        int fifo_free = 4 - fifo_level;
        if (steps_to_push > fifo_free) {
            steps_to_push = fifo_free;
        }

        for (int s = 0; s < steps_to_push; s++) {
            if (diff > 0) {
                /* Forward: advance phase 0->1->2->3->0 */
                encoder_state[i].phase = (encoder_state[i].phase + 1) & 0x03;
                encoder_state[i].current_count++;
                diff--;
            } else {
                /* Reverse: retreat phase 0->3->2->1->0 */
                encoder_state[i].phase = (encoder_state[i].phase - 1) & 0x03;
                encoder_state[i].current_count--;
                diff++;
            }
            pio_sm_put(encoder_pio, encoder_sm[i],
                       quad_table[encoder_state[i].phase]);
        }
    }
}

/* ---------- Main ---------- */
int main(void) {
    stdio_init_all();

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_put(LED_PIN, 0);

    /* Initialize PIO programs */
    servo_pio_init();
    encoder_pio_init();

    reset_state_machine();

    printf("\n[HILS] PWM Servo + Encoder emulator ready\n");
    printf("[HILS] Servo: GPIO %d-%d, Encoder: GPIO %d-%d\n",
           SERVO_BASE_PIN, SERVO_BASE_PIN + MAX_SERVO_CHANNELS - 1,
           ENCODER_BASE_PIN, ENCODER_BASE_PIN + MAX_ENCODER_CHANNELS * 2 - 1);

    uint32_t last_led_toggle = 0;

    while (1) {
        /* Poll USB CDC for incoming frames */
        int count = stdio_usb_in_chars((char *)cdc_buf, CDC_BUF_SIZE);
        if (count > 0) {
            for (int i = 0; i < count; i++) {
                if (process_byte(cdc_buf[i])) {
                    process_servo_command(rx_payload, payload_length);
                    frame_count++;

                    /* Toggle LED on each received frame */
                    gpio_xor_mask(1u << LED_PIN);
                }
            }
        }

        /* Update servo PWM outputs */
        servo_update();

        /* Feed encoder steps toward target positions */
        encoder_update();

        /* Slow blink LED when idle (no recent frames) */
        uint32_t now = time_us_32();
        if (frame_count == 0 && (now - last_led_toggle) >= 1000000) {
            gpio_xor_mask(1u << LED_PIN);
            last_led_toggle = now;
        }
    }

    return 0;
}
