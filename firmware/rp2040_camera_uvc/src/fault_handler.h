/*
 * Firmware-level fault injection for the UVC bridge (docs 9.3 / Phase 5).
 *
 * Executes 0x50-series commands received over the inter-Pico UART:
 *   USB_DETACH    - tud_disconnect(), optional timed reconnect
 *   FRAME_DROP    - deterministic thinning of received frames
 *   PARTIAL_FRAME - truncate frames before UVC transmission
 *   RESET_BOOTSEL - magic-guarded reboot into the UF2 bootloader
 */

#ifndef FAULT_HANDLER_H
#define FAULT_HANDLER_H

#include <stdint.h>
#include <stdbool.h>

/* Dispatch a 0x50-0x5F payload from the frame receiver. */
void fault_handler_process(const uint8_t *payload, uint32_t len);

/* Periodic work (timed USB reconnect). Call from the main loop. */
void fault_handler_task(void);

/* true if the frame arriving now must be discarded (FRAME_DROP). */
bool fault_should_drop_frame(void);

/* Frame length to actually transmit (PARTIAL_FRAME truncation). */
uint32_t fault_effective_frame_len(uint32_t len);

#endif /* FAULT_HANDLER_H */
