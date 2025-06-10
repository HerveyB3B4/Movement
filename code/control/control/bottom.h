#ifndef _CONTROL_BOTTOM_H
#define _CONTROL_BOTTOM_H

#include "zf_common_headfile.h"

void control_bottom_balance(struct Control_Target *control_target,
                            struct Control_Flag *control_flag,
                            struct Velocity_Motor *vel_motor,
                            struct EulerAngle *euler_angle_bias,
                            struct Control_Motion_Manual_Parmas *control_motion_params);
int32 get_bottom_duty();
#endif