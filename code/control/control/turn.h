#ifndef _CONTROL_TURN_H
#define _CONTROL_TURN_H
#include "zf_common_headfile.h"
// #include "velocity.h"
// struct Velocity_Motor;

void control_turn(struct Control_Target *control_target,
                  struct Control_Flag *control_flag,
                  struct Control_Turn_Manual_Params *control_turn_params,
                  struct Control_Motion_Manual_Parmas *control_motion_params,
                  //   struct EulerAngle *euler_angle_bias,
                  struct Velocity_Motor *vel_motor);
void control_buckling(struct Control_Target *control_target,
                      struct Control_Turn_Manual_Params *control_turn_params,
                      struct Velocity_Motor *vel_motor);

int32 get_momentum_diff();

#endif