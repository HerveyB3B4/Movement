#ifndef _CAMERA_H_
#define _CAMERA_H_

#include "zf_common_headfile.h"
#include "zf_device_mt9v03x.h"

extern uint32 g_camera_exp_time_def;
extern uint32 g_camera_gain_def;
extern uint32 g_binary_threshold_def;

void camera_config(void);

#endif