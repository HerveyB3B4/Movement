#include "wireless.h"

uint8 wireless_image_tmp[MT9V03X_H][MT9V03X_W];

void wireless_assistant_init()
{
    seekfree_assistant_interface_init(WIRELESS_DEVICE);
    seekfree_assistant_camera_information_config(WIRELESS_SEND_IMG_TYPE, wireless_image_tmp[0], MT9V03X_W, MT9V03X_H);
}

void wireless_assistant_send_image(uint8 *img)
{
    memcpy(wireless_image_tmp[0], img, MT9V03X_IMAGE_SIZE);
    seekfree_assistant_camera_send();
}
