/*
 * Firmware-level fault injection for the UVC bridge (docs 9.3 / Phase 5).
 *
 * Commands arrive as framed 0x50-series payloads on the same UART as
 * the MJPEG stream (Pico#1 forwards them verbatim). Every set/clear is
 * acknowledged on the reverse channel with a 0x52 frame so the ROS
 * node can log delivery.
 */

#include "fault_handler.h"
#include "hils_frame_protocol.h"

#include <stdio.h>
#include <string.h>

#include "tusb.h"
#include "hardware/timer.h"
#include "hardware/uart.h"
#include "pico/bootrom.h"

/* --- active fault state --- */
static bool     detach_active = false;
static uint32_t reconnect_deadline_us = 0;   /* 0 = no timed reconnect */
static uint8_t  drop_percent = 0;            /* 0 = fault inactive */
static uint32_t drop_accumulator = 0;
static uint8_t  keep_percent = 100;          /* 100 = fault inactive */

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
    uart_write_blocking(HILS_UART_INSTANCE, (const uint8_t *)&hdr,
                        sizeof(hdr));
    uart_write_blocking(HILS_UART_INSTANCE, (const uint8_t *)&ack,
                        sizeof(ack));
    uart_write_blocking(HILS_UART_INSTANCE, &checksum, 1);
}

static void usb_detach(uint32_t reconnect_after_ms) {
    if (!detach_active) {
        tud_disconnect();
        detach_active = true;
    }
    reconnect_deadline_us = reconnect_after_ms
        ? time_us_32() + reconnect_after_ms * 1000u
        : 0;
    printf("[FAULT] usb detach (reconnect_after=%lu ms)\n",
           (unsigned long)reconnect_after_ms);
}

static void usb_reattach(void) {
    if (detach_active) {
        tud_connect();
        detach_active = false;
        reconnect_deadline_us = 0;
        printf("[FAULT] usb reconnect\n");
    }
}

static void handle_set(const uint8_t *payload, uint32_t len) {
    if (len < sizeof(hils_fault_set_t)) {
        send_ack(0, HILS_FAULT_ACK_BAD_ARG);
        return;
    }
    hils_fault_set_t cmd;
    memcpy(&cmd, payload, sizeof(cmd));

    switch (cmd.fault_code) {
    case HILS_FAULT_UVC_USB_DETACH:
        if (cmd.arg0 > 600000u) {
            send_ack(cmd.fault_code, HILS_FAULT_ACK_BAD_ARG);
            return;
        }
        /* Ack BEFORE detaching: the reverse path is unaffected, but
         * ack first keeps ordering obvious in the node's log. */
        send_ack(cmd.fault_code, HILS_FAULT_ACK_OK);
        usb_detach(cmd.arg0);
        return;

    case HILS_FAULT_UVC_FRAME_DROP:
        if (cmd.arg0 < 1 || cmd.arg0 > 100) {
            send_ack(cmd.fault_code, HILS_FAULT_ACK_BAD_ARG);
            return;
        }
        drop_percent = (uint8_t)cmd.arg0;
        drop_accumulator = 0;
        printf("[FAULT] frame drop %u%%\n", drop_percent);
        send_ack(cmd.fault_code, HILS_FAULT_ACK_OK);
        return;

    case HILS_FAULT_UVC_PARTIAL_FRAME:
        if (cmd.arg0 < 1 || cmd.arg0 > 99) {
            send_ack(cmd.fault_code, HILS_FAULT_ACK_BAD_ARG);
            return;
        }
        keep_percent = (uint8_t)cmd.arg0;
        printf("[FAULT] partial frame keep %u%%\n", keep_percent);
        send_ack(cmd.fault_code, HILS_FAULT_ACK_OK);
        return;

    default:
        send_ack(cmd.fault_code, HILS_FAULT_ACK_UNKNOWN_CODE);
        return;
    }
}

static void handle_clear(const uint8_t *payload, uint32_t len) {
    if (len < sizeof(hils_fault_clear_t)) {
        send_ack(0, HILS_FAULT_ACK_BAD_ARG);
        return;
    }
    uint8_t code = payload[1];

    if (code == 0 || code == HILS_FAULT_UVC_USB_DETACH) {
        usb_reattach();
    }
    if (code == 0 || code == HILS_FAULT_UVC_FRAME_DROP) {
        drop_percent = 0;
        drop_accumulator = 0;
    }
    if (code == 0 || code == HILS_FAULT_UVC_PARTIAL_FRAME) {
        keep_percent = 100;
    }
    printf("[FAULT] clear code=0x%02X\n", code);
    send_ack(code, HILS_FAULT_ACK_OK);
}

void fault_handler_process(const uint8_t *payload, uint32_t len) {
    if (len == 0) {
        return;
    }
    switch (payload[0]) {
    case HILS_MSG_TYPE_FAULT_SET:
        handle_set(payload, len);
        break;

    case HILS_MSG_TYPE_FAULT_CLEAR:
        handle_clear(payload, len);
        break;

    case HILS_MSG_TYPE_RESET_BOOTSEL: {
        hils_reset_bootsel_t cmd;
        if (len < sizeof(cmd)) {
            return;
        }
        memcpy(&cmd, payload, sizeof(cmd));
        if (cmd.magic == HILS_RESET_BOOTSEL_MAGIC) {
            printf("[FAULT] rebooting into BOOTSEL\n");
            reset_usb_boot(0, 0);   /* does not return */
        }
        break;
    }

    default:
        break;
    }
}

void fault_handler_task(void) {
    if (detach_active && reconnect_deadline_us != 0 &&
        (int32_t)(time_us_32() - reconnect_deadline_us) >= 0) {
        usb_reattach();
    }
}

bool fault_should_drop_frame(void) {
    if (drop_percent == 0) {
        return false;
    }
    /* Deterministic thinning: drop exactly drop_percent of frames. */
    drop_accumulator += drop_percent;
    if (drop_accumulator >= 100) {
        drop_accumulator -= 100;
        return true;
    }
    return false;
}

uint32_t fault_effective_frame_len(uint32_t len) {
    if (keep_percent >= 100) {
        return len;
    }
    uint32_t kept = (len * keep_percent) / 100;
    return kept < 2 ? 2 : kept;   /* keep at least the JPEG SOI marker */
}
