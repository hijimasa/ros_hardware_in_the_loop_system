/*
 * HILS Quadrature Encoder Emulator - RP2040 Firmware
 *
 * Receives target encoder count commands from a ROS 2 node via USB CDC and
 * emits quadrature A/B pulses on GPIO 6-9 via PIO. Each channel uses two
 * adjacent GPIOs (A on N, B on N+1).
 *
 * This emulates the feedback signal a real motor's encoder would produce,
 * so the real PC's controller code (which reads the A/B pulses to estimate
 * motor position) can be tested in a HILS loop.
 *
 * Data flow:
 *   ROS 2 (JointState) -> USB CDC -> this firmware -> PIO A/B pulses -> real PC
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "pico/stdlib.h"
#include "pico/stdio_usb.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"

#include "hils_frame_protocol.h"
#include "encoder_output.pio.h"

/* ---------- Pin assignments ---------- */
#define ENCODER_BASE_PIN    6       /* GPIO 6-9: encoder A/B outputs */
#define LED_PIN             25

/* ---------- Configuration ---------- */
#define MAX_ENCODER_CHANNELS 2

/* Encoder step frequency (max quadrature steps per second) */
#define ENCODER_STEP_FREQ   100000.0f  /* 100kHz max step rate */

/* ---------- Message type ---------- */
#define HILS_MSG_TYPE_ENCODER_CMD  0x40

/* ---------- Encoder command structures ---------- */
typedef struct __attribute__((packed)) {
    uint8_t  msg_type;          /* 0x40 */
    uint8_t  channel_count;     /* Number of channels */
} hils_encoder_cmd_header_t;

typedef struct __attribute__((packed)) {
    uint8_t  channel;           /* Channel index (0-1) */
    int32_t  target_count;      /* Target encoder count (signed) */
} hils_encoder_channel_t;

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
    int32_t current_count;      /* Current emulated position */
    int32_t target_count;       /* Target position (latest command) */
    uint8_t phase;              /* Current quadrature phase (0-3) */
} encoder_state_t;

static encoder_state_t encoder_state[MAX_ENCODER_CHANNELS];

/* ---------- PIO handles ---------- */
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
static void process_encoder_command(const uint8_t *payload, uint32_t len) {
    if (len < sizeof(hils_encoder_cmd_header_t)) {
        return;
    }

    const hils_encoder_cmd_header_t *hdr = (const hils_encoder_cmd_header_t *)payload;
    if (hdr->msg_type != HILS_MSG_TYPE_ENCODER_CMD) {
        return;
    }

    uint8_t ch_count = hdr->channel_count;
    uint32_t expected_len = sizeof(hils_encoder_cmd_header_t)
                          + ch_count * sizeof(hils_encoder_channel_t);
    if (len < expected_len) {
        return;
    }

    const hils_encoder_channel_t *channels =
        (const hils_encoder_channel_t *)(payload + sizeof(hils_encoder_cmd_header_t));

    for (uint8_t i = 0; i < ch_count; i++) {
        uint8_t ch = channels[i].channel;
        if (ch < MAX_ENCODER_CHANNELS) {
            encoder_state[ch].target_count = channels[i].target_count;
        }
    }
}

/* ---------- PIO initialization ---------- */
static void encoder_pio_init(void) {
    encoder_pio = pio0;
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

/* ---------- Encoder update ----------
 * Push quadrature steps into the PIO TX FIFO toward each channel's target.
 * Non-blocking; runs every main loop iteration. The PIO clock divider sets
 * the actual pulse output rate.
 */
static void encoder_update(void) {
    for (int i = 0; i < MAX_ENCODER_CHANNELS; i++) {
        int32_t diff = encoder_state[i].target_count - encoder_state[i].current_count;
        if (diff == 0) {
            continue;
        }

        if (pio_sm_is_tx_fifo_full(encoder_pio, encoder_sm[i])) {
            continue;
        }

        int steps_to_push = abs(diff);
        int fifo_level = pio_sm_get_tx_fifo_level(encoder_pio, encoder_sm[i]);
        int fifo_free = 4 - fifo_level;
        if (steps_to_push > fifo_free) {
            steps_to_push = fifo_free;
        }

        for (int s = 0; s < steps_to_push; s++) {
            if (diff > 0) {
                encoder_state[i].phase = (encoder_state[i].phase + 1) & 0x03;
                encoder_state[i].current_count++;
                diff--;
            } else {
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

    encoder_pio_init();
    reset_state_machine();

    printf("\n[HILS] Quadrature encoder emulator ready\n");
    printf("[HILS] Encoder: GPIO %d-%d (A,B per channel, %d channels)\n",
           ENCODER_BASE_PIN,
           ENCODER_BASE_PIN + MAX_ENCODER_CHANNELS * 2 - 1,
           MAX_ENCODER_CHANNELS);

    uint32_t last_led_toggle = 0;

    while (1) {
        int count = stdio_usb_in_chars((char *)cdc_buf, CDC_BUF_SIZE);
        if (count > 0) {
            for (int i = 0; i < count; i++) {
                if (process_byte(cdc_buf[i])) {
                    process_encoder_command(rx_payload, payload_length);
                    frame_count++;
                    gpio_xor_mask(1u << LED_PIN);
                }
            }
        }

        encoder_update();

        uint32_t now = time_us_32();
        if (frame_count == 0 && (now - last_led_toggle) >= 1000000) {
            gpio_xor_mask(1u << LED_PIN);
            last_led_toggle = now;
        }
    }

    return 0;
}
