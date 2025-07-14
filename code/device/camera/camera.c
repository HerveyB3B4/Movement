#include "camera.h"

uint32 g_camera_exp_time_def = 212;
uint32 g_camera_gain_def = 63;
uint32 g_binary_threshold_def = 110;

// void camera_init()
// {
//     mt9v03x_init();
//     // mt9v03x2_init(); // 如果使用双摄像头，可以取消注释
// }

void camera_config()
{
    unsigned char mt9v03x_set_exposure_time_sccb(unsigned short int g_camera_gain_def);
    mt9v03x_set_exposure_time_sccb(g_camera_exp_time_def);
}