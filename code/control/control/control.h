#ifndef _CONTROL_H_
#define _CONTROL_H_

#include "pid.h"
#include "zf_common_headfile.h"

struct Control_Turn_Manual_Params
{
    float buckling_side_coefficient;
    float buckling_front_coefficient;
};

struct Control_Motion_Manual_Parmas
{
    // 这是底轮速度
    int32 bottom_velocity;

    // PID
    uint32 bottom_velocity_parameter[3];
    uint32 bottom_angle_velocity_parameter[3];
    uint32 bottom_angle_parameter[3];
    uint32 bottom_position_parameter[3];

    uint32 side_angle_velocity_parameter[3];
    uint32 side_angle_parameter[3];
    uint32 side_velocity_parameter[3];

    uint32 turn_angle_parameter[3];
    uint32 turn_velocity_parameter[3];
    uint32 turn_error_parameter[3];
    uint32 turn_angle_velocity_parameter[3];

    // PID极性参数
    int32 bottom_velocity_polarity;
    int32 bottom_angle_velocity_polarity;
    int32 bottom_angle_polarity;
    int32 side_angle_velocity_polarity;
    int32 side_angle_polarity;
    int32 side_velocity_polarity;
    int32 turn_angle_polarity;
    int32 turn_angle_velocity_polarity;
    int32 turn_error_polarity;
    int32 turn_velocity_polarity;
};

struct Control_Time
{
    // 串级pid各个环的时间，时间由长到短，由外到内
    uint32 turn[4];
    uint32 bottom[3];
    uint32 side[3];
};

struct Control_Target
{
    // 控制目标状态

    // front side
    float bottom_angle;
    float bottom_angle_vel;
    float bottom_vel;

    // side
    //  float side_vel;
    float side_angle;
    float side_angle_vel;

    // turn
    float buckling_side;
    float buckling_front;

    float turn_angle;
    float turn_angle_vel;
    float turn_err;
};

struct Control_Flag
{
    // 控制更新标志
    uint8 bottom_angle;
    uint8 bottom_angle_vel;
    uint8 bottom_vel;

    uint8 side_angle;
    uint8 side_angle_vel;
    uint8 side_vel;

    uint8 turn;
    uint8 turn_angle;
    uint8 turn_angle_vel;
    uint8 turn_vel;
    uint8 turnAngleDiffVelocity;
    uint8 turn_err;

    uint8 turn_curve;
    uint8 buckling_side;

    uint8 bottom_angle_cnt;
    uint8 bottom_angle_vel_cnt;
    uint8 bottom_vel_cnt;

    uint8 side_angle_cnt;
    uint8 side_angle_vel_cnt;
    uint8 side_vel_cnt;

    uint8 turn_angle_cnt;
    uint8 turn_angle_vel_cnt;
    uint8 turn_vel_cnt;
    uint8 turn_err_cnt;

    uint8 turn_curve_cnt;

    uint8 bucking_cnt;
};

struct EulerAngle;
struct Velocity_Motor;
struct EulerAngle;
struct Menu_Manual_Param;

extern uint8 g_turn_start_flag;
extern int32 g_control_shutdown_flag;
extern uint32 g_control_bottom_flag;
extern uint32 g_control_side_flag;
extern uint32 g_control_turn_flag;

extern uint32 g_control_output_sav_flag;
extern uint32 g_control_output_sv_flag;
extern uint32 g_control_output_sa_flag;

extern uint32 g_control_output_fa_flag;
extern uint32 g_control_output_fv_flag;
extern uint32 g_control_output_fav_flag;

extern struct Control_Turn_Manual_Params g_control_turn_manual_params;
extern struct Control_Target g_control_target;
extern struct Control_Flag g_control_flag;
extern struct Control_Time g_control_time;
extern struct Control_Motion_Manual_Parmas g_control_motion_params;

extern pid_type_def bottom_angle_velocity_PID;

extern uint32 control_time;
extern float g_angle_limit;

void control_shutdown(struct Control_Target *control_target,
                      struct EulerAngle *euler_angle_bias,
                      struct Velocity_Motor *vel_motor);
void control_init(struct Control_Motion_Manual_Parmas *control_motion_params);
void control_manual_param_init();

void control_reset(struct Control_Target *control_target);

extern pid_type_def turn_angle_PID;
extern pid_type_def turn_angle_velocity_PID;
extern pid_type_def turn_velocity_PID;
extern pid_type_def turn_error_PID;

extern pid_type_def bottom_angle_PID;
extern pid_type_def bottom_angle_velocity_PID;
extern pid_type_def bottom_velocity_PID;
extern pid_type_def bottom_position_PID;

extern pid_type_def side_angle_velocity_PID;
extern pid_type_def side_angle_PID;
extern pid_type_def side_velocity_PID;

#endif