#ifndef UVC_DEVICE_H
#define UVC_DEVICE_H

#include <stdbool.h>

void video_task(void);

/* Returns true if UVC is currently transmitting a frame */
bool video_is_busy(void);

#endif /* UVC_DEVICE_H */
