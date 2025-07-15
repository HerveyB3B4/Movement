#include "control.h"
#include "turn.h"
#include "Attitude.h"
#include "velocity.h"

static int32 s_momentum_diff = 0;

static void control_turn_velocity(struct Control_Target *control_target,
                                  struct Control_Motion_Manual_Parmas *control_motion_params,
                                  struct Velocity_Motor *vel_motor);
static void control_turn_angle_velocity(struct Control_Target *control_target,
                                        struct Control_Motion_Manual_Parmas *control_motion_params);
// static void control_turn_angle(struct Control_Target *control_target);
static void control_turn_error(struct Control_Target *control_target,
                               struct Control_Motion_Manual_Parmas *control_motion_params);

int32 get_momentum_diff()
{
    return s_momentum_diff;
}

void control_buckling(struct Control_Target *control_target,
                      struct Control_Turn_Manual_Params *control_turn_params,
                      struct Velocity_Motor *vel_motor,
                      float distance, float sin_yaw)
{
    // 逐飞公式
    // 静态项
    // float state = control_turn_params->buckling_side_state * control_target->turn_err;

    // // 动态项 - 可省略
    // float dynamic = control_turn_params->buckling_side_dynamic * vel_motor->bottom_real * vel_motor->bottom_real * control_target->turn_err;

    // // printf("%f,%f\n", state, dynamic);
    // control_target->buckling_side = state + dynamic;
    // restrictValueF(&control_target->buckling_side, 10.0f, -10.0f);

    // 物理公式
    // 此处 turn err 表示转弯半径
    // float theta = atan2f(vel_motor->bottom_real * vel_motor->bottom_real, GravityAcc * 0.20);
    // printf("%f,%f\n", theta, vel_motor->bottom_real * vel_motor->bottom_real);
    // control_target->buckling_side = control_turn_params->buckling_side_state * theta;

    // 引入图像偏移
    // float r = distance / (2.0f * sin_yaw);
    // theta = atan2f(vel_motor->bottom_real * vel_motor->bottom_real, GravityAcc * r);

    // test
    float v = vel_motor->bottom_filtered / V_KALMAN_MULTIPLE;
    if (v < 0)
        v = 0; // 这个需检查方向
    float x = (float)control_target->turn_angle_vel * 0.1f * logf(v + 2);
    control_target->buckling_side = control_turn_params->buckling_side_state * x;
    restrictValueF(&control_target->buckling_side, 15.0f, -15.0f);
}

void control_turn(struct Control_Target *control_target,
                  struct Control_Flag *control_flag,
                  struct Control_Turn_Manual_Params *control_turn_params,
                  struct Control_Motion_Manual_Parmas *control_motion_params,
                  //   struct EulerAngle *euler_angle_bias,
                  struct Velocity_Motor *vel_motor)
{
    // if (control_flag->turn)
    // {
    //     control_flag->turn = 0;
    //     control_buckling(control_target, control_turn_params, vel_motor, 0, 0);
    // }
    // if (control_flag->turn_vel)
    // {
    //     control_flag->turn_vel = 0;
    //     control_turn_velocity(control_target, vel_motor, control_motion_params);
    // }

    if (control_flag->turn_err)
    {
        control_flag->turn_err = 0;
        control_turn_error(control_target, control_motion_params);
        // control_buckling(control_target, control_turn_params, vel_motor, 0, 0);
    }
    if (control_flag->turn_angle_vel)
    {
        control_flag->turn_angle_vel = 0;
        control_turn_angle_velocity(control_target, control_motion_params);
        control_buckling(control_target, control_turn_params, vel_motor, 0, 0);
    }

    // diff 滤波
    static int32 last_diff = 0;
    lowPassFilterI(&s_momentum_diff, &last_diff, 0.2f);
    last_diff = s_momentum_diff;

    restrictValueI(&s_momentum_diff, 3000, -3000);

    // control_target->buckling_side = control_turn_params->buckling_side_state * control_target->turn_err;
}

static void control_turn_angle_velocity(struct Control_Target *control_target,
                                        struct Control_Motion_Manual_Parmas *control_motion_params)
{
    // restrictValueF(&control_target->turn_angle_vel, 250, -250);
    static float turn_angle_vel_filter[2] = {0};
    turn_angle_vel_filter[1] = turn_angle_vel_filter[0];
    turn_angle_vel_filter[0] = YAW_VEL;

    lowPassFilterF(&turn_angle_vel_filter[0], &turn_angle_vel_filter[1], 0.3f);

    s_momentum_diff = control_motion_params->turn_angle_velocity_polarity *
                      (int32)PID_calc_Position_LowPassD(
                          &turn_angle_velocity_PID,
                          turn_angle_vel_filter[0],
                          control_target->turn_angle_vel);
}

static void control_turn_error(struct Control_Target *control_target,
                               struct Control_Motion_Manual_Parmas *control_motion_params)
{
    // printf("%f,%f\n",
    //        g_control_target.turn_err,
    //        g_control_target.buckling_side);

    static float turn_err_filter[2] = {0}; // err 滤波
    turn_err_filter[1] = turn_err_filter[0];
    turn_err_filter[0] = control_target->turn_err;
    // noiseFilter(momentumAngleFilter[0], 0.02f);
    // lowPassFilterF(&turn_err_filter[0], &turn_err_filter[1], 0.3f);

    // s_momentum_diff = control_motion_params->turn_error_polarity *
    //                   PID_calc_Position(
    //                       &turn_error_PID,
    //                       turn_err_filter[0],
    //                       0);
    control_target->turn_angle_vel = control_motion_params->turn_error_polarity *
                                     PID_calc_Position(
                                         &turn_error_PID,
                                         turn_err_filter[0],
                                         0);
}

// 不知道需不需要加个速度环
static void control_turn_velocity(struct Control_Target *control_target,
                                  struct Control_Motion_Manual_Parmas *control_motion_params,
                                  struct Velocity_Motor *vel_motor)
{
    // static float turnVelocityFilter = 0;
    // turnVelocityFilter = (float)vel_motor->bottom_real;
    // control_target->turn_angle_vel =
    //     PID_calc_Position(&turn_velocity_PID, turnVelocityFilter,
    //                       control_target->turn_vel);
    control_target->turn_err = control_motion_params->turn_velocity_polarity *
                               PID_calc_Position(
                                   &turn_velocity_PID, (float)(vel_motor->momentum_front - vel_motor->momentum_back) / 2.0f,
                                   0.0f);
    // if (g_control_output_tv_flag != 0)
    // {
    //     printf("%f\n", control_target->turn_angle_vel);
    // }
}
