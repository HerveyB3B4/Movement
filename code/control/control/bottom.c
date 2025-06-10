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
    if (control_flag->frontVelocity)
    {
        control_flag->frontVelocity = 0;
        control_bottom_velocity(vel_motor, control_target, control_motion_params);
    }
    if (control_flag->frontAngle)
    {
        control_flag->frontAngle = 0;
        control_bottom_angle(euler_angle_bias, control_target, control_motion_params);
    }
    if (control_flag->frontAngleVelocity)
    {
        control_flag->frontAngleVelocity = 0;
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
    // s_bottom_balance_duty = control_target->frontAngleVelocity * 10;
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
    control_target->frontAngle = control_motion_params->bottom_velocity_polarity * PID_calc_Position(
                                                                                       &bottom_velocity_PID, (float)vel_motor->bottomFiltered, control_target->frontVelocity);
#endif
#ifndef VELOCITY_KALMAN_FILTER
    control_target->frontAngle =
        PID_calc_Position(&bottom_velocity_PID, (float)vel_motor->bottom,
                          control_target->frontVelocity);
#endif
    // control_target->frontAngle = g_euler_angle_bias->roll - 0.1f *
    // PID_calc_Position_DynamicI(&bottom_velocity_PID,(float)vel_motor->bottom,control_target->frontVelocity,
    // 80, 1.5f);
    // TODO:tune the parameter
    // restrictValueF(&control_target->frontAngle, 15.0f, -15.0f);
    // {
    //     control_target->frontAngle = g_euler_angle_bias->roll;
    // }
    if (g_control_output_fv_flag != 0)
    {
        printf("%f\n", control_target->frontAngle);
    }
}

static void control_bottom_angle(struct EulerAngle *euler_angle_bias,
                                 struct Control_Target *control_target,
                                 struct Control_Motion_Manual_Parmas *control_motion_params)
{
    static float angleControlFilter[2] = {0};
    angleControlFilter[1] = angleControlFilter[0];
    angleControlFilter[0] = currentFrontAngle;
    // lowPassFilter(&angleControlFilter[0],&angleControlFilter[1],0.1f);
    // noiseFilter(angleControlFilter[0],0.002f);

    // simpleFuzzyProcess(&frontBalanceSimpleFuzzy,angleControlFilter[0],control_target->frontAngle,&bottom_angle_PID);
    control_target->frontAngleVelocity = control_motion_params->bottom_angle_polarity * (PID_calc_Position(
                                                                                            &bottom_angle_PID, (angleControlFilter[0] - euler_angle_bias->pitch),
                                                                                            control_target->frontAngle));

    if (g_control_output_fa_flag != 0)
    {
        printf("%f,%f\n", control_target->frontAngleVelocity, currentFrontAngle);
    }
}

static void control_bottom_angle_velocity(
    struct Control_Target *control_target,
    struct Control_Motion_Manual_Parmas *control_motion_params)
{
    // imu963raPushingSensorData();
    static float angleVelocityControlFilter[2] = {0};
    angleVelocityControlFilter[1] = angleVelocityControlFilter[0];
    angleVelocityControlFilter[0] = currentFrontAngleVelocity;

    s_bottom_balance_duty = control_motion_params->bottom_angle_velocity_polarity * (PID_calc_Position(
                                                                                        &bottom_angle_velocity_PID, angleVelocityControlFilter[0],
                                                                                        control_target->frontAngleVelocity));
    if (g_control_output_fav_flag != 0)
    {
        printf("%d\n", s_bottom_balance_duty);
    }
}
