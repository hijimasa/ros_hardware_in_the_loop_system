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

int main(void) {
    board_init();

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

    /* Main loop */
    while (1) {
        tud_task();
        spi_receiver_poll();
        video_task();
    }

    return 0;
}
