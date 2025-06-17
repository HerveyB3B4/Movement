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

int32 get_momentum_diff() { return s_momentum_diff; }

void control_turn(struct Control_Target *control_target,
                  struct Control_Flag *control_flag,
                  struct Control_Turn_Manual_Params *control_turn_params,
                  struct Control_Motion_Manual_Parmas *control_motion_params,
                  //   struct EulerAngle *euler_angle_bias,
                  struct Velocity_Motor *vel_motor)
{
    // if (control_flag->turn) {
    //     control_flag->turn = 0;
    //     TurnCurvatureControl();
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
    }
    // if (control_flag->turn_angle_vel)
    // {
    //     control_flag->turn_angle_vel = 0;
    //     control_turn_angle_velocity(control_target, control_motion_params);
    // }
    // if (control_flag->bucking) {
    //     control_flag->bucking = 0;
    //     float v = (float)fabsf(vel_motor->bottomReal);
    //     if (v < 0) {
    //         v = 0;
    //     }

    //     // float x = (float) control_target.turn_angle_velocity * 0.01f *
    //     // fabsf(motor_velocity.bottom_filtered * 0.01f);
    //     float x = (float)control_target.turn_angle_velocity * 0.1f *
    //               logf(v + 2);  // 使用ln函数，降低速度对压弯的影响
    //     control_target.bucking = bucking_k * x;
    //     RestrictValueF(&control_target.bucking, 10.5f, -10.5f);
    // }
}

static void control_turn_angle_velocity(struct Control_Target *control_target,
                                        struct Control_Motion_Manual_Parmas *control_motion_params)
{
    static float preTurnAngleVelocity = 0;
    restrictValueF(&control_target->turn_angle_vel, 250, -250);
    preTurnAngleVelocity = control_target->turn_angle_vel;
    static int32 preMomentumDiff = 0;
    s_momentum_diff = control_motion_params->turn_angle_velocity_polarity *
                      (int32)PID_calc_Position_LowPassD(
                          &turn_angle_velocity_PID, yawAngleVelocity,
                          control_target->turn_angle_vel);

    s_momentum_diff = (int32)(0.8f * s_momentum_diff + 0.2f * preMomentumDiff);
    restrictValueI(&s_momentum_diff, 1000, -1000);
    preMomentumDiff = s_momentum_diff;
}

static void control_turn_error(struct Control_Target *control_target,
                               struct Control_Motion_Manual_Parmas *control_motion_params)
{
    static float turn_err_filter[2] = {0}; // 角度滤波
    turn_err_filter[1] = turn_err_filter[0];
    turn_err_filter[0] = s_momentum_diff;
    // noiseFilter(momentumAngleFilter[0], 0.02f);
    lowPassFilterF(&turn_err_filter[0], &turn_err_filter[1], 0.3f);

    s_momentum_diff = control_motion_params->turn_error_polarity *
                      PID_calc_Position(
                          &turn_error_PID, control_target->turn_err, 0);
}

// 不知道需不需要加个速度环
static void control_turn_velocity(struct Control_Target *control_target,
                                  struct Control_Motion_Manual_Parmas *control_motion_params,
                                  struct Velocity_Motor *vel_motor)
{
    // static float turnVelocityFilter = 0;
    // turnVelocityFilter = (float)vel_motor->bottomReal;
    // control_target->turn_angle_vel =
    //     PID_calc_Position(&turn_velocity_PID, turnVelocityFilter,
    //                       control_target->turn_vel);
    control_target->turn_err = control_motion_params->turn_velocity_polarity *
                               PID_calc_Position(
                                   &turn_velocity_PID, (float)(vel_motor->momentumFront - vel_motor->momentumBack) / 2.0f,
                                   0.0f);
    // if (g_control_output_tv_flag != 0)
    // {
    //     printf("%f\n", control_target->turn_angle_vel);
    // }
}