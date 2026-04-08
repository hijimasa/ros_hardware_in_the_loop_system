/*
 * HILS UVC Camera Bridge - Pico#2 (UVC output + SPI receiver)
 *
 * Receives MJPEG frames from Pico#1 via SPI slave and outputs
 * them as a UVC (USB Video Class) camera device.
 */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "bsp/board_api.h"
#include "tusb.h"
#include "usb_descriptors.h"
#include "spi_receiver.h"
#include "uvc_device.h"
#include "frame_buffer.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"

#define LED_PIN 25

/*
 * LED diagnostic patterns (no stdio available):
 *   fast blink (100ms)  = USB not mounted
 *   slow blink (500ms)  = mounted but not streaming
 *   solid ON            = streaming active
 */
static void led_diagnostic(void) {
    static uint32_t last_toggle_us = 0;
    uint32_t now = time_us_32();

    if (tud_video_n_streaming(0, 0)) {
        gpio_put(LED_PIN, 1);
        return;
    }

    uint32_t interval_us = tud_mounted() ? 500000 : 100000;
    if (now - last_toggle_us >= interval_us) {
        gpio_xor_mask(1u << LED_PIN);
        last_toggle_us = now;
    }
}

int main(void) {
    board_init();

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    /* Initialize frame double buffers */
    frame_buffer_init();

    /* Initialize TinyUSB first (before SPI to avoid DMA conflicts) */
    tusb_rhport_init_t dev_init = {
        .role = TUSB_ROLE_DEVICE,
        .speed = TUSB_SPEED_AUTO,
    };
    tusb_init(BOARD_TUD_RHPORT, &dev_init);

    if (board_init_after_tusb) {
        board_init_after_tusb();
    }

    /* Initialize SPI slave receiver after USB is ready */
    spi_receiver_init();

    printf("\n[HILS] UVC Bridge ready (UART1 debug @ GP4)\n");
    printf("[HILS] EP bufsize=%d\n", CFG_TUD_VIDEO_STREAMING_EP_BUFSIZE);

    /* Main loop */
    while (1) {
        tud_task();
        spi_receiver_poll();
        video_task();
        led_diagnostic();
    }

    return 0;
}
