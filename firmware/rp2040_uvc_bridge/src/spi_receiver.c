/*
 * UART Receiver for HILS UVC Bridge (Pico#2)
 *
 * Receives MJPEG frames from Pico#1 via UART0.
 * Uses DMA ring buffer to capture UART RX data.
 */

#include "spi_receiver.h"
#include "frame_buffer.h"
#include "uvc_device.h"
#include "hils_frame_protocol.h"

#include "hardware/uart.h"
#include "hardware/dma.h"
#include "hardware/gpio.h"
#include <string.h>

#define LED_PIN 25

#define UART_DMA_BUF_BITS  14
#define UART_DMA_BUF_SIZE  (1 << UART_DMA_BUF_BITS)  /* 16384 */
static uint8_t __attribute__((aligned(UART_DMA_BUF_SIZE))) dma_ring_buf[UART_DMA_BUF_SIZE];
static uint32_t sw_read_pos = 0;
static int dma_chan = -1;

#define MAX_BYTES_PER_POLL  2048

/* Framing state machine */
static hils_rx_state_t rx_state = HILS_RX_WAIT_SYNC0;
static uint8_t  length_buf[4];
static uint32_t length_idx = 0;
static uint32_t payload_length = 0;
static uint32_t payload_idx = 0;
static uint8_t  running_checksum = 0;

static void reset_state_machine(void) {
    rx_state = HILS_RX_WAIT_SYNC0;
    length_idx = 0;
    payload_length = 0;
    payload_idx = 0;
    running_checksum = 0;
}

void spi_receiver_init(void) {
    /* Initialize UART */
    uart_init(HILS_UART_INSTANCE, HILS_UART_BAUDRATE);
    gpio_set_function(HILS_UART_PIN_RX, GPIO_FUNC_UART);
    /* TX pin not needed on receiver side */

    /* Disable UART FIFO to get byte-by-byte DMA triggers */
    uart_set_fifo_enabled(HILS_UART_INSTANCE, false);

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_put(LED_PIN, 0);

    /* Clear any residual data */
    while (uart_is_readable(HILS_UART_INSTANCE)) {
        uart_getc(HILS_UART_INSTANCE);
    }

    memset(dma_ring_buf, 0, sizeof(dma_ring_buf));

    /* Setup DMA for UART RX into ring buffer */
    dma_chan = dma_claim_unused_channel(true);
    dma_channel_config cfg = dma_channel_get_default_config(dma_chan);
    channel_config_set_transfer_data_size(&cfg, DMA_SIZE_8);
    channel_config_set_read_increment(&cfg, false);
    channel_config_set_write_increment(&cfg, true);
    channel_config_set_ring(&cfg, true, UART_DMA_BUF_BITS);
    channel_config_set_dreq(&cfg, uart_get_dreq(HILS_UART_INSTANCE, false));

    dma_channel_configure(
        dma_chan, &cfg,
        dma_ring_buf,
        &uart_get_hw(HILS_UART_INSTANCE)->dr,
        UINT32_MAX,
        true
    );

    sw_read_pos = 0;
    reset_state_machine();
}

void spi_receiver_debug_check(void) {
    /* Find first 0xAA and check next byte */
    for (uint32_t i = 0; i < UART_DMA_BUF_SIZE - 1; i++) {
        if (dma_ring_buf[i] == 0xAA) {
            uint8_t next = dma_ring_buf[i + 1];
            if (next == 0x55) {
                gpio_put(LED_PIN, 1);  /* steady ON = correct sync pair */
            } else {
                /* blink 3 times = 0xAA found but wrong next byte */
                for (int j = 0; j < 6; j++) {
                    gpio_xor_mask(1u << LED_PIN);
                    busy_wait_ms(200);
                }
            }
            return;
        }
    }
    /* No 0xAA: stay off */
}

bool spi_receiver_poll(void) {
    uint32_t hw_write_pos = ((uint32_t)dma_channel_hw_addr(dma_chan)->write_addr
                             - (uint32_t)dma_ring_buf) % UART_DMA_BUF_SIZE;

    bool frame_complete = false;
    frame_buf_t *wbuf = frame_buffer_get_write_buf();
    uint32_t bytes_processed = 0;

    while (sw_read_pos != hw_write_pos && !frame_complete
           && bytes_processed < MAX_BYTES_PER_POLL) {
        uint8_t byte = dma_ring_buf[sw_read_pos];
        sw_read_pos = (sw_read_pos + 1) % UART_DMA_BUF_SIZE;
        bytes_processed++;

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
                if (payload_length == 0 || payload_length > HILS_FRAME_MAX_PAYLOAD) {
                    reset_state_machine();
                } else {
                    payload_idx = 0;
                    running_checksum = 0;
                    rx_state = HILS_RX_READ_PAYLOAD;
                }
            }
            break;

        case HILS_RX_READ_PAYLOAD:
            if (payload_idx < FRAME_BUF_SIZE) {
                wbuf->data[payload_idx] = byte;
            }
            running_checksum ^= byte;
            payload_idx++;
            if (payload_idx >= payload_length) {
                rx_state = HILS_RX_READ_CHECKSUM;
            }
            break;

        case HILS_RX_READ_CHECKSUM:
            if (byte == running_checksum && payload_length <= FRAME_BUF_SIZE) {
                wbuf->length = payload_length;
                wbuf->ready = true;
                frame_buffer_swap();
                gpio_put(LED_PIN, 1);
                frame_complete = true;
            }
            reset_state_machine();
            break;
        }
    }

    return frame_complete;
}
