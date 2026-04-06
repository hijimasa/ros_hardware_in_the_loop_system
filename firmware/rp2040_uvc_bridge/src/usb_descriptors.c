/*
 * USB Descriptors for HILS UVC Camera (Pico#2)
 *
 * Based on TinyUSB examples/device/video_capture (struct-based approach).
 * Configured for MJPEG 640x480 streaming over USB Full-Speed isochronous.
 */

#include "bsp/board_api.h"
#include "tusb.h"
#include "usb_descriptors.h"

/* ---- Constants ---- */
#define UVC_CLOCK_FREQUENCY    27000000

#define UVC_ENTITY_CAP_INPUT_TERMINAL  0x01
#define UVC_ENTITY_CAP_OUTPUT_TERMINAL 0x02

enum {
    ITF_NUM_VIDEO_CONTROL = 0,
    ITF_NUM_VIDEO_STREAMING,
    ITF_NUM_TOTAL
};

#define EPNUM_VIDEO_IN  0x81

/* ---- String Descriptors ---- */
enum {
    STRID_LANGID = 0,
    STRID_MANUFACTURER,
    STRID_PRODUCT,
    STRID_SERIAL,
    STRID_UVC_CONTROL,
    STRID_UVC_STREAMING,
};

static char const *string_desc_arr[] = {
    [STRID_LANGID]        = (const char[]){0x09, 0x04},
    [STRID_MANUFACTURER]  = "HILS Project",
    [STRID_PRODUCT]       = "HILS UVC Camera",
    [STRID_SERIAL]        = NULL, /* use unique ID */
    [STRID_UVC_CONTROL]   = "UVC Control",
    [STRID_UVC_STREAMING] = "UVC Streaming",
};

/* ---- Device Descriptor ---- */
tusb_desc_device_t const desc_device = {
    .bLength            = sizeof(tusb_desc_device_t),
    .bDescriptorType    = TUSB_DESC_DEVICE,
    .bcdUSB             = 0x0200,
    .bDeviceClass       = TUSB_CLASS_MISC,
    .bDeviceSubClass    = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol    = MISC_PROTOCOL_IAD,
    .bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,
    .idVendor           = 0xCafe,
    .idProduct          = 0x4020,
    .bcdDevice          = 0x0100,
    .iManufacturer      = STRID_MANUFACTURER,
    .iProduct           = STRID_PRODUCT,
    .iSerialNumber      = STRID_SERIAL,
    .bNumConfigurations = 1,
};

uint8_t const *tud_descriptor_device_cb(void) {
    return (uint8_t const *)&desc_device;
}

/* ---- Configuration Descriptor (struct-based, matching TinyUSB example) ---- */

typedef struct TU_ATTR_PACKED {
    tusb_desc_interface_t                         itf;
    tusb_desc_video_control_header_1itf_t         header;
    tusb_desc_video_control_camera_terminal_t     camera_terminal;
    tusb_desc_video_control_output_terminal_t     output_terminal;
} uvc_control_desc_t;

typedef struct TU_ATTR_PACKED {
    tusb_desc_interface_t                         itf;      /* Alt 0: zero-bandwidth */
    tusb_desc_video_streaming_input_header_1byte_t header;
    tusb_desc_video_format_mjpeg_t                format;
    tusb_desc_video_frame_mjpeg_continuous_t       frame;
    tusb_desc_video_streaming_color_matching_t     color;
    tusb_desc_interface_t                         itf_alt;  /* Alt 1: operational */
    tusb_desc_endpoint_t                          ep;
} uvc_streaming_desc_t;

typedef struct TU_ATTR_PACKED {
    tusb_desc_configuration_t  config;
    tusb_desc_interface_assoc_t iad;
    uvc_control_desc_t         video_control;
    uvc_streaming_desc_t       video_streaming;
} uvc_cfg_desc_t;

static const uvc_cfg_desc_t desc_fs_configuration = {
    .config = {
        .bLength             = sizeof(tusb_desc_configuration_t),
        .bDescriptorType     = TUSB_DESC_CONFIGURATION,
        .wTotalLength        = sizeof(uvc_cfg_desc_t),
        .bNumInterfaces      = ITF_NUM_TOTAL,
        .bConfigurationValue = 1,
        .iConfiguration      = 0,
        .bmAttributes        = TU_BIT(7),
        .bMaxPower           = 250, /* 500mA */
    },
    .iad = {
        .bLength             = sizeof(tusb_desc_interface_assoc_t),
        .bDescriptorType     = TUSB_DESC_INTERFACE_ASSOCIATION,
        .bFirstInterface     = ITF_NUM_VIDEO_CONTROL,
        .bInterfaceCount     = 2,
        .bFunctionClass      = TUSB_CLASS_VIDEO,
        .bFunctionSubClass   = VIDEO_SUBCLASS_INTERFACE_COLLECTION,
        .bFunctionProtocol   = VIDEO_ITF_PROTOCOL_UNDEFINED,
        .iFunction           = 0,
    },
    .video_control = {
        .itf = {
            .bLength            = sizeof(tusb_desc_interface_t),
            .bDescriptorType    = TUSB_DESC_INTERFACE,
            .bInterfaceNumber   = ITF_NUM_VIDEO_CONTROL,
            .bAlternateSetting  = 0,
            .bNumEndpoints      = 0,
            .bInterfaceClass    = TUSB_CLASS_VIDEO,
            .bInterfaceSubClass = VIDEO_SUBCLASS_CONTROL,
            .bInterfaceProtocol = VIDEO_ITF_PROTOCOL_15,
            .iInterface         = STRID_UVC_CONTROL,
        },
        .header = {
            .bLength            = sizeof(tusb_desc_video_control_header_1itf_t),
            .bDescriptorType    = TUSB_DESC_CS_INTERFACE,
            .bDescriptorSubType = VIDEO_CS_ITF_VC_HEADER,
            .bcdUVC             = VIDEO_BCD_1_50,
            .wTotalLength       = sizeof(uvc_control_desc_t) - sizeof(tusb_desc_interface_t),
            .dwClockFrequency   = UVC_CLOCK_FREQUENCY,
            .bInCollection      = 1,
            .baInterfaceNr      = { ITF_NUM_VIDEO_STREAMING },
        },
        .camera_terminal = {
            .bLength                  = sizeof(tusb_desc_video_control_camera_terminal_t),
            .bDescriptorType          = TUSB_DESC_CS_INTERFACE,
            .bDescriptorSubType       = VIDEO_CS_ITF_VC_INPUT_TERMINAL,
            .bTerminalID              = UVC_ENTITY_CAP_INPUT_TERMINAL,
            .wTerminalType            = VIDEO_ITT_CAMERA,
            .bAssocTerminal           = 0,
            .iTerminal                = 0,
            .wObjectiveFocalLengthMin = 0,
            .wObjectiveFocalLengthMax = 0,
            .wOcularFocalLength       = 0,
            .bControlSize             = 3,
            .bmControls               = { 0, 0, 0 },
        },
        .output_terminal = {
            .bLength            = sizeof(tusb_desc_video_control_output_terminal_t),
            .bDescriptorType    = TUSB_DESC_CS_INTERFACE,
            .bDescriptorSubType = VIDEO_CS_ITF_VC_OUTPUT_TERMINAL,
            .bTerminalID        = UVC_ENTITY_CAP_OUTPUT_TERMINAL,
            .wTerminalType      = VIDEO_TT_STREAMING,
            .bAssocTerminal     = 0,
            .bSourceID          = UVC_ENTITY_CAP_INPUT_TERMINAL,
            .iTerminal          = 0,
        },
    },
    .video_streaming = {
        .itf = {
            .bLength            = sizeof(tusb_desc_interface_t),
            .bDescriptorType    = TUSB_DESC_INTERFACE,
            .bInterfaceNumber   = ITF_NUM_VIDEO_STREAMING,
            .bAlternateSetting  = 0,
            .bNumEndpoints      = 0, /* zero-bandwidth alt for ISO */
            .bInterfaceClass    = TUSB_CLASS_VIDEO,
            .bInterfaceSubClass = VIDEO_SUBCLASS_STREAMING,
            .bInterfaceProtocol = VIDEO_ITF_PROTOCOL_15,
            .iInterface         = STRID_UVC_STREAMING,
        },
        .header = {
            .bLength            = sizeof(tusb_desc_video_streaming_input_header_1byte_t),
            .bDescriptorType    = TUSB_DESC_CS_INTERFACE,
            .bDescriptorSubType = VIDEO_CS_ITF_VS_INPUT_HEADER,
            .bNumFormats        = 1,
            .wTotalLength       = sizeof(uvc_streaming_desc_t)
                                  - sizeof(tusb_desc_interface_t)   /* itf Alt 0 */
                                  - sizeof(tusb_desc_endpoint_t)    /* ep */
                                  - sizeof(tusb_desc_interface_t),  /* itf Alt 1 */
            .bEndpointAddress   = EPNUM_VIDEO_IN,
            .bmInfo             = 0,
            .bTerminalLink      = UVC_ENTITY_CAP_OUTPUT_TERMINAL,
            .bStillCaptureMethod = 0,
            .bTriggerSupport    = 0,
            .bTriggerUsage      = 0,
            .bControlSize       = 1,
            .bmaControls        = { 0 },
        },
        .format = {
            .bLength               = sizeof(tusb_desc_video_format_mjpeg_t),
            .bDescriptorType       = TUSB_DESC_CS_INTERFACE,
            .bDescriptorSubType    = VIDEO_CS_ITF_VS_FORMAT_MJPEG,
            .bFormatIndex          = 1,
            .bNumFrameDescriptors  = 1,
            .bmFlags               = 0,
            .bDefaultFrameIndex    = 1,
            .bAspectRatioX         = 0,
            .bAspectRatioY         = 0,
            .bmInterlaceFlags      = 0,
            .bCopyProtect          = 0,
        },
        .frame = {
            .bLength                  = sizeof(tusb_desc_video_frame_mjpeg_continuous_t),
            .bDescriptorType          = TUSB_DESC_CS_INTERFACE,
            .bDescriptorSubType       = VIDEO_CS_ITF_VS_FRAME_MJPEG,
            .bFrameIndex              = 1,
            .bmCapabilities           = 0,
            .wWidth                   = FRAME_WIDTH,
            .wHeight                  = FRAME_HEIGHT,
            .dwMinBitRate             = 30000 * 8 * 1,    /* ~30KB/frame * 1fps */
            .dwMaxBitRate             = 30000 * 8 * FRAME_RATE, /* ~30KB/frame * 15fps */
            .dwMaxVideoFrameBufferSize = 65536,           /* max MJPEG frame size */
            .dwDefaultFrameInterval   = 10000000 / FRAME_RATE,
            .bFrameIntervalType       = 0, /* continuous */
            .dwFrameInterval          = {
                10000000 / FRAME_RATE,               /* min interval */
                10000000,                             /* max interval (1 fps) */
                10000000 / FRAME_RATE,               /* step */
            },
        },
        .color = {
            .bLength            = sizeof(tusb_desc_video_streaming_color_matching_t),
            .bDescriptorType    = TUSB_DESC_CS_INTERFACE,
            .bDescriptorSubType = VIDEO_CS_ITF_VS_COLORFORMAT,
            .bColorPrimaries           = VIDEO_COLOR_PRIMARIES_BT709,
            .bTransferCharacteristics  = VIDEO_COLOR_XFER_CH_BT709,
            .bMatrixCoefficients       = VIDEO_COLOR_COEF_SMPTE170M,
        },
        .itf_alt = {
            .bLength            = sizeof(tusb_desc_interface_t),
            .bDescriptorType    = TUSB_DESC_INTERFACE,
            .bInterfaceNumber   = ITF_NUM_VIDEO_STREAMING,
            .bAlternateSetting  = 1,
            .bNumEndpoints      = 1,
            .bInterfaceClass    = TUSB_CLASS_VIDEO,
            .bInterfaceSubClass = VIDEO_SUBCLASS_STREAMING,
            .bInterfaceProtocol = VIDEO_ITF_PROTOCOL_15,
            .iInterface         = STRID_UVC_STREAMING,
        },
        .ep = {
            .bLength          = sizeof(tusb_desc_endpoint_t),
            .bDescriptorType  = TUSB_DESC_ENDPOINT,
            .bEndpointAddress = EPNUM_VIDEO_IN,
            .bmAttributes     = {
                .xfer = TUSB_XFER_ISOCHRONOUS,
                .sync = 1,
            },
            .wMaxPacketSize   = CFG_TUD_VIDEO_STREAMING_EP_BUFSIZE,
            .bInterval        = 1,
        },
    },
};

uint8_t const *tud_descriptor_configuration_cb(uint8_t index) {
    (void)index;
    return (uint8_t const *)&desc_fs_configuration;
}

/* ---- String Descriptors ---- */
static uint16_t _desc_str[32 + 1];

uint16_t const *tud_descriptor_string_cb(uint8_t index, uint16_t langid) {
    (void)langid;
    size_t chr_count;

    switch (index) {
    case STRID_LANGID:
        memcpy(&_desc_str[1], string_desc_arr[0], 2);
        chr_count = 1;
        break;

    case STRID_SERIAL:
        chr_count = board_usb_get_serial(_desc_str + 1, 32);
        break;

    default:
        if (index >= sizeof(string_desc_arr) / sizeof(string_desc_arr[0])) return NULL;
        const char *str = string_desc_arr[index];
        chr_count = strlen(str);
        if (chr_count > 31) chr_count = 31;
        for (size_t i = 0; i < chr_count; i++) {
            _desc_str[1 + i] = str[i];
        }
        break;
    }

    _desc_str[0] = (uint16_t)((TUSB_DESC_STRING << 8) | (2 * chr_count + 2));
    return _desc_str;
}
