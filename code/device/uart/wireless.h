#ifndef _WIRELESS_H
#define _WIRELESS_H

#include "zf_common_headfile.h"

#define WIRELESS_DEVICE SEEKFREE_ASSISTANT_BLE6A20
#define WIRELESS_SEND_IMG_TYPE SEEKFREE_ASSISTANT_MT9V03X
void wireless_assistant_init();
void wireless_assistant_send_image(uint8 *img);

#endif