#ifndef USB_DESCRIPTORS_H
#define USB_DESCRIPTORS_H

/* UVC frame parameters */
#define FRAME_RATE    15

/* Supported resolutions (bFrameIndex is 1-based) */
#define FRAME_INDEX_320x240   1
#define FRAME_INDEX_640x480   2
#define FRAME_INDEX_1280x720  3
#define NUM_FRAME_SIZES       3
#define DEFAULT_FRAME_INDEX   FRAME_INDEX_640x480

#endif /* USB_DESCRIPTORS_H */
