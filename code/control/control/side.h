#ifndef _CONTROL_SIDE_H
#define _CONTROL_SIDE_H

#include "zf_common_headfile.h"

void control_side_balance(
    struct Control_Target *control_target,
    struct Control_Flag *control_flag,
    struct Control_Turn_Manual_Params *control_turn_params,
    struct Velocity_Motor *vel_motor,
    struct EulerAngle *euler_angle_bias,
    struct Control_Motion_Manual_Parmas *control_motion_params);

int32 get_side_duty();
#endif