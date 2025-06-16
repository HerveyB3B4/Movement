#ifndef _SYSTEM_H_
#define _SYSTEM_H_

#include "zf_common_headfile.h"

#define PIT_KEY_T 5      // 更新频率
#define PIT_VELOCITY_T 1 // 更新周期(ms)
#define PIT_CONTROL_T 1  // 控制周期(ms)

typedef enum
{
    CAR_STOP,
    CAR_READY,
    CAR_STABLE,
    CAR_RUNNING,
    CAR_TESTING,
} RunState_t;

struct Control_Time;
struct Control_Flag;
struct Control_Target;
struct Control_Turn_Manual_Params;
struct Velocity_Motor;
struct EulerAngle;
struct Control_Motion_Manual_Parmas;

void system_init();
void system_attitude_timer(
    struct Control_Turn_Manual_Params *control_turn_params,
    struct Control_Target *control_target,
    struct Velocity_Motor *vel_motor,
    struct EulerAngle *euler_angle);
void bottom_control_timer(struct Control_Time *control_time,
                          struct Control_Flag *control_flag,
                          struct Control_Target *control_target,
                          struct Velocity_Motor *vel_motor,
                          struct EulerAngle *euler_angle_bias,
                          struct Control_Motion_Manual_Parmas *control_motion_params);
void side_control_timer(struct Control_Time *control_time,
                        struct Control_Flag *control_flag,
                        struct Control_Target *control_target,
                        struct Control_Turn_Manual_Params *control_turn_params,
                        struct Velocity_Motor *vel_motor,
                        struct EulerAngle *euler_angle_bias,
                        struct Control_Motion_Manual_Parmas *control_motion_params);
void turn_control_timer(struct Control_Time *control_time,
                        struct Control_Flag *control_flag,
                        struct Control_Target *control_target,
                        struct Control_Turn_Manual_Params *control_turn_params,
                        struct Velocity_Motor *vel_motor,
                        int error);

void system_set_runstate(RunState_t state);

extern RunState_t runState;

#endif