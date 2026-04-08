#ifndef TUSB_CONFIG_H
#define TUSB_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

/* ---- Board ---- */
#ifndef BOARD_TUD_RHPORT
#define BOARD_TUD_RHPORT      0
#endif

#ifndef BOARD_TUD_MAX_SPEED
#define BOARD_TUD_MAX_SPEED   OPT_MODE_DEFAULT_SPEED
#endif

/* ---- Common ---- */
#ifndef CFG_TUSB_OS
#define CFG_TUSB_OS           OPT_OS_NONE
#endif

#ifndef CFG_TUSB_DEBUG
#define CFG_TUSB_DEBUG        0
#endif

#define CFG_TUD_ENABLED       1
#define CFG_TUD_MAX_SPEED     BOARD_TUD_MAX_SPEED

#ifndef CFG_TUSB_MEM_SECTION
#define CFG_TUSB_MEM_SECTION
#endif

#ifndef CFG_TUSB_MEM_ALIGN
#define CFG_TUSB_MEM_ALIGN    __attribute__((aligned(4)))
#endif

/* ---- Device ---- */
#ifndef CFG_TUD_ENDPOINT0_SIZE
#define CFG_TUD_ENDPOINT0_SIZE  64
#endif

/* ---- Video ---- */
#define CFG_TUD_VIDEO              1
#define CFG_TUD_VIDEO_STREAMING    1
#define CFG_TUD_VIDEO_STREAMING_EP_BUFSIZE  1023  /* FS isochronous max */
#define CFG_TUD_VIDEO_STREAMING_BULK  0  /* Use isochronous */

#ifdef __cplusplus
}
#endif

#endif /* TUSB_CONFIG_H */
