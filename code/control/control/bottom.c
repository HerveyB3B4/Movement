#include "bottom.h"
#include "control.h"
#include "Attitude.h"
#include "velocity.h"

uint32 g_control_output_fa_flag = 0;
uint32 g_control_output_fv_flag = 0;
uint32 g_control_output_fav_flag = 0;

static int32 s_bottom_balance_duty = 0;

// bottom
static void control_bottom_velocity(struct Velocity_Motor *vel_motor,
                                    struct Control_Target *control_target,
                                    struct Control_Motion_Manual_Parmas *control_motion_params);
static void control_bottom_angle(struct EulerAngle *euler_angle_bias,
                                 struct Control_Target *control_target,
                                 struct Control_Motion_Manual_Parmas *control_motion_params);
static void control_bottom_angle_velocity(
    struct Control_Target *control_target,
    struct Control_Motion_Manual_Parmas *control_motion_params);

int32 get_bottom_duty() { return s_bottom_balance_duty; }

void control_bottom_balance(struct Control_Target *control_target,
                            struct Control_Flag *control_flag,
                            struct Velocity_Motor *vel_motor,
                            struct EulerAngle *euler_angle_bias,
                            struct Control_Motion_Manual_Parmas *control_motion_params)
{
    if (control_flag->bottom_vel)
    {
        control_flag->bottom_vel = 0;
        control_bottom_velocity(vel_motor, control_target, control_motion_params);
    }
    if (control_flag->bottom_angle)
    {
        control_flag->bottom_angle = 0;
        control_bottom_angle(euler_angle_bias, control_target, control_motion_params);
    }
    if (control_flag->bottom_angle_vel)
    {
        control_flag->bottom_angle_vel = 0;
        control_bottom_angle_velocity(control_target, control_motion_params);
    }
    // restrictValueI(&s_bottom_balance_duty,-3000,3000);
    // fuzzyBottomMotorHertz();
    // if (bottom_motor_deadzone != 0) {  // apply feedforward if enabled
    //     int32 vel_motorDeadV = vel_motor->bottom;
    //     int8 d = 80;
    //     restrictValueI(&vel_motorDeadV, -d, d);
    //     vel_motorDeadV = (abs(d - abs(vel_motorDeadV)) /
    //                       d);  // normalize to 0~1, 0 is the max deadzone pwm
    //     if (s_bottom_balance_duty > 0) {
    //         s_bottom_balance_duty +=
    //             bottom_motor_deadzone *
    //             (vel_motorDeadV +
    //              0.1f);  // 0.1f is the minimum pwm to avoid shaking
    //     } else {
    //         s_bottom_balance_duty -=
    //             bottom_motor_deadzone * (vel_motorDeadV + 0.1f);
    //     }
    // }
    // s_bottom_balance_duty = control_target->bottom_angle_vel * 10;
    if (s_bottom_balance_duty > 0)
    {
        s_bottom_balance_duty += bottom_motor_deadzone_forward;
    }
    else if (s_bottom_balance_duty < 0)
    {
        s_bottom_balance_duty -= bottom_motor_deadzone_backword;
    }

    restrictValueI(&s_bottom_balance_duty, -9999, 9999);

    set_bottom_motor_pwn(
        (int32)(s_bottom_balance_duty)); // set bottom motor pwm to
                                         // keep front balance
}

static void control_bottom_velocity(struct Velocity_Motor *vel_motor,
                                    struct Control_Target *control_target,
                                    struct Control_Motion_Manual_Parmas *control_motion_params)
{
#ifdef VELOCITY_KALMAN_FILTER
    control_target->bottom_angle = control_motion_params->bottom_velocity_polarity *
                                   PID_calc_Position(
                                       &bottom_velocity_PID,
                                       (float)vel_motor->bottomFiltered,
                                       control_target->bottom_vel);
#endif
#ifndef VELOCITY_KALMAN_FILTER
    control_target->bottom_angle =
        PID_calc_Position(&bottom_velocity_PID, (float)vel_motor->bottom,
                          control_target->bottom_vel);
#endif
    // control_target->bottom_angle = g_euler_angle_bias->roll - 0.1f *
    // PID_calc_Position_DynamicI(&bottom_velocity_PID,(float)vel_motor->bottom,control_target->bottom_vel,
    // 80, 1.5f);
    // TODO:tune the parameter
    // restrictValueF(&control_target->bottom_angle, 15.0f, -15.0f);
    // {
    //     control_target->bottom_angle = g_euler_angle_bias->roll;
    // }
    if (g_control_output_fv_flag != 0)
    {
        printf("%f\n", control_target->bottom_angle);
    }
}

static void control_bottom_angle(struct EulerAngle *euler_angle_bias,
                                 struct Control_Target *control_target,
                                 struct Control_Motion_Manual_Parmas *control_motion_params)
{
    static float angleControlFilter[2] = {0};
    angleControlFilter[1] = angleControlFilter[0];
    angleControlFilter[0] = PITCH;
    // lowPassFilter(&angleControlFilter[0],&angleControlFilter[1],0.1f);
    // noiseFilter(angleControlFilter[0],0.002f);

    // simpleFuzzyProcess(&frontBalanceSimpleFuzzy,angleControlFilter[0],control_target->bottom_angle,&bottom_angle_PID);
    control_target->bottom_angle_vel = control_motion_params->bottom_angle_polarity * (PID_calc_Position(
                                                                                          &bottom_angle_PID, (angleControlFilter[0] - euler_angle_bias->pitch),
                                                                                          control_target->bottom_angle));

    if (g_control_output_fa_flag != 0)
    {
        printf("%f,%f\n", control_target->bottom_angle_vel, PITCH);
    }
}

static void control_bottom_angle_velocity(
    struct Control_Target *control_target,
    struct Control_Motion_Manual_Parmas *control_motion_params)
{
    // imu963raPushingSensorData();
    static float angleVelocityControlFilter[2] = {0};
    angleVelocityControlFilter[1] = angleVelocityControlFilter[0];
    angleVelocityControlFilter[0] = PITCH_VEL;

    s_bottom_balance_duty = control_motion_params->bottom_angle_velocity_polarity * (PID_calc_Position(
                                                                                        &bottom_angle_velocity_PID, angleVelocityControlFilter[0],
                                                                                        control_target->bottom_angle_vel));
    if (g_control_output_fav_flag != 0)
    {
        printf("%d\n", s_bottom_balance_duty);
    }
}
