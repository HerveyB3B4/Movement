#include "control.h"
#include "menu_input.h"
#include "motor.h"
#include "pid.h"
#include "system.h"
#include "velocity.h"
#include "zf_common_headfile.h"

uint32 control_time = 0;
// global
uint8 g_turn_start_flag = 0;
int32 g_control_shutdown_flag = 0;
uint32 g_control_bottom_flag = 0;
uint32 g_control_side_flag = 0;

uint32 g_control_output_sav_flag = 0;
uint32 g_control_output_sv_flag = 0;
uint32 g_control_output_sa_flag = 0;

uint32 side_front_deadzone = 172;
uint32 side_back_deadzone = 169;

struct Control_Turn_Manual_Params g_control_turn_manual_params;
struct Control_Target g_control_target;
struct Control_Flag g_control_flag;
struct Control_Time g_control_time;
struct Control_Motion_Manual_Parmas g_control_motion_params;

// static
static int32 s_bottom_balance_duty = 0;
static int32 s_side_balance_duty = 0;
static int32 s_momentum_diff = 0;

static void control_param_init(pid_type_def* pid,
                               const uint32 para[3],
                               float coefficient,
                               float maxOut,
                               float maxIOut);
// static float control_bottom_feedforward();
// static void control_shutdown(struct Control_Target* control_target,
//                              struct EulerAngle* euler_angle_bias);

// bottom
static void control_bottom_velocity(struct Velocity_Motor* vel_motor,
                                    struct Control_Target* control_target);
static void control_bottom_angle(struct EulerAngle* euler_angle_bias,
                                 struct Control_Target* control_target);
static void control_bottom_angle_velocity(
    struct Control_Target* control_target);

// side
static void control_side_velocity(
    struct Velocity_Motor* vel_motor,
    struct Control_Target* control_target,
    struct Control_Turn_Manual_Params* control_turn_params);
static void control_side_angle(struct EulerAngle* euler_angle_bias,
                               struct Control_Target* control_target);
static void control_side_angle_velocity(struct Control_Target* control_target);

// PID
// bottom wheel
pid_type_def bottom_angle_velocity_PID;
pid_type_def bottom_angle_PID;
pid_type_def bottom_velocity_PID;

// momentum wheel pid
pid_type_def side_angle_velocity_PID;
pid_type_def side_angle_PID;
pid_type_def side_velocity_PID;

// turn pid
pid_type_def turn_angle_PID;
pid_type_def turn_angle_velocity_PID;

int32 get_bottom_duty() {
    return s_bottom_balance_duty;
}

int32 get_side_duty() {
    return s_side_balance_duty;
}

int get_momentum_diff() {
    return s_momentum_diff;
}

void control_bottom_balance(struct Control_Target* control_target,
                            struct Control_Flag* control_flag,
                            struct Velocity_Motor* vel_motor,
                            struct EulerAngle* euler_angle_bias) {
    if (control_flag->frontVelocity) {
        control_flag->frontVelocity = 0;
        control_bottom_velocity(vel_motor, control_target);
    }
    if (control_flag->frontAngle) {
        control_flag->frontAngle = 0;
        control_bottom_angle(euler_angle_bias, control_target);
    }
    if (control_flag->frontAngleVelocity) {
        control_flag->frontAngleVelocity = 0;
        control_bottom_angle_velocity(control_target);
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
    // s_bottom_balance_duty = control_target->frontAngle * 100;
    if (s_bottom_balance_duty > 0) {
        s_bottom_balance_duty += bottom_motor_deadzone;
    } else if (s_bottom_balance_duty < 0) {
        s_bottom_balance_duty -= bottom_motor_deadzone;
    }

    restrictValueI(&s_bottom_balance_duty, -10000, 10000);

    set_bottom_motor_pwn(
        (int32)(s_bottom_balance_duty));  // set bottom motor pwm to
                                          // keep front balance
}

static void control_bottom_velocity(struct Velocity_Motor* vel_motor,
                                    struct Control_Target* control_target) {
#ifdef VELOCITY_KALMAN_FILTER
    control_target->frontAngle = -PID_calc_Position(
        &bottom_velocity_PID, (float)vel_motor->bottomFiltered, 0.0f);
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
}

static void control_bottom_angle(struct EulerAngle* euler_angle_bias,
                                 struct Control_Target* control_target) {
    static float angleControlFilter[2] = {0};
    angleControlFilter[1] = angleControlFilter[0];
    angleControlFilter[0] = currentFrontAngle;
    // lowPassFilter(&angleControlFilter[0],&angleControlFilter[1],0.1f);
    // noiseFilter(angleControlFilter[0],0.002f);

    // simpleFuzzyProcess(&frontBalanceSimpleFuzzy,angleControlFilter[0],control_target->frontAngle,&bottom_angle_PID);
    control_target->frontAngleVelocity = -(PID_calc_Position(
        &bottom_angle_PID, (angleControlFilter[0] - euler_angle_bias->pitch),
        control_target->frontAngle));
}

static void control_bottom_angle_velocity(
    struct Control_Target* control_target) {
    // imu963raPushingSensorData();
    static float angleVelocityControlFilter[2] = {0};
    angleVelocityControlFilter[1] = angleVelocityControlFilter[0];
    angleVelocityControlFilter[0] = currentFrontAngleVelocity;

    s_bottom_balance_duty = (int32)(PID_calc_Position(
        &bottom_angle_velocity_PID, angleVelocityControlFilter[0],
        control_target->frontAngleVelocity));
}

void control_side_balance(
    struct Control_Target* control_target,
    struct Control_Flag* control_flag,
    struct Control_Turn_Manual_Params* control_turn_params,
    struct Velocity_Motor* vel_motor,
    struct EulerAngle* euler_angle_bias) {
    if (control_flag->sideVelocity) {
        control_flag->sideVelocity = 0;
        control_side_velocity(vel_motor, control_target, control_turn_params);
    }

    if (control_flag->sideAngle) {
        control_flag->sideAngle = 0;
        control_side_angle(euler_angle_bias, control_target);
    }

    if (control_flag->sideAngleVelocity) {
        control_flag->sideAngleVelocity = 0;
        control_side_angle_velocity(control_target);
    }

    // turnControl();
    int32 left_motor_duty, right_motor_duty;
    left_motor_duty = -s_side_balance_duty - s_momentum_diff;
    right_motor_duty = s_side_balance_duty - s_momentum_diff;

    restrictValueI(&s_side_balance_duty, -8000, 8000);

    // 添加死区补偿
    if (left_motor_duty > 0) {
        left_motor_duty += side_front_deadzone;
    } else if (left_motor_duty < 0) {
        left_motor_duty -= side_front_deadzone;
    }

    if (right_motor_duty > 0) {
        right_motor_duty += side_back_deadzone;
    } else if (right_motor_duty < 0) {
        right_motor_duty -= side_back_deadzone;
    }

    // printf("%d,%d\n", left_motor_duty, right_motor_duty);

    set_momentum_motor_pwm(left_motor_duty, right_motor_duty);
    // set momentum motor pwm to keep side balance
}

static void control_side_velocity(
    struct Velocity_Motor* vel_motor,
    struct Control_Target* control_target,
    struct Control_Turn_Manual_Params* control_turn_params) {
    // static float momentumVelocityFilter = 0;
    // momentumVelocityFilter =
    //     (float)(vel_motor->momentumFront - vel_motor->momentumBack);

    control_target->sideAngle = (float)PID_calc_Position(
        &side_velocity_PID,
        (float)(vel_motor->momentumFront - vel_motor->momentumBack) / 2.0f,
        0.0f);  // 速度是正反馈，因此set和ref要反过来

    // 输出pid信息：error，输出，实际值，目标值
    if (g_control_output_sv_flag != 0) {
        printf("%f,%f\n", currentSideAngle, control_target->sideAngle * 100);
    }
}

static void control_side_angle(struct EulerAngle* euler_angle_bias,
                               struct Control_Target* control_target) {
    static float momentumAngleFilter[2] = {0};  // 角度滤波
    momentumAngleFilter[1] = momentumAngleFilter[0];
    momentumAngleFilter[0] = -currentSideAngle;
    // noiseFilter(momentumAngleFilter[0],0.02f);
    // lowPassFilterF(&momentumAngleFilter[0], &momentumAngleFilter[1], 0.1f);
    control_target->sideAngleVelocity = PID_calc_Position(
        &side_angle_PID, (momentumAngleFilter[0] - euler_angle_bias->roll),
        control_target->sideAngle);

    // 输出pid信息：error，输出，实际值，目标值
    if (g_control_output_sa_flag != 0) {
        printf("%f,%f\n", currentSideAngle,
               control_target->sideAngleVelocity / 100);
    }
}

static void control_side_angle_velocity(struct Control_Target* control_target) {
    static float momentumGyroFilter[2] = {0};  // 角度速度滤波
    momentumGyroFilter[1] = momentumGyroFilter[0];
    momentumGyroFilter[0] = currentSideAngleVelocity;

    // lowPassFilterF(&momentumGyroFilter[0], &momentumGyroFilter[1], 0.1f);
    // 修改为位置式
    s_side_balance_duty =
        (int32)(PID_calc_DELTA(&side_angle_velocity_PID, momentumGyroFilter[0],
                               control_target->sideAngleVelocity));

    // 输出pid信息：error，输出，实际值，目标值
    if (g_control_output_sav_flag != 0) {
        printf("%f,%f\n", -currentSideAngleVelocity,
               s_side_balance_duty / 100.0f);
    }
}

void control_shutdown(struct Control_Target* control_target,
                      struct EulerAngle* euler_angle_bias) {
    if (g_control_shutdown_flag != 0) {
        if (fabsf(currentSideAngle - euler_angle_bias->roll -
                  control_target->sideAngle) > 28) {
            stop_bottom_motor();
            stop_momentum_motor();

            printf("control_shutdown 1\n");
            char tmp[10];
            snprintf(tmp, 10, "%.3f", euler_angle_bias->roll);
            zf_log(0, tmp);
            runState = CAR_STOP;
        }
        if (fabsf(currentFrontAngle) > 35) {
            stop_bottom_motor();
            stop_momentum_motor();

            printf("control_shutdown 2\n");
            char tmp[10];
            snprintf(tmp, 10, "%.3f", euler_angle_bias->pitch);
            zf_log(0, tmp);
            runState = CAR_STOP;
        }
    }
}

void control_init(struct Control_Motion_Manual_Parmas* control_motion_params) {
    control_param_init(&bottom_angle_velocity_PID,
                       control_motion_params->bottom_angle_velocity_parameter,
                       10, MOTOR_PWM_MAX, 9999);
    control_param_init(&bottom_angle_PID,
                       control_motion_params->bottom_angle_parameter, 1, 9999,
                       10);
    control_param_init(&bottom_velocity_PID,
                       control_motion_params->bottom_velocity_parameter, 10000,
                       50, 2.5f);

    // momentum wheel pid
    control_param_init(&side_angle_velocity_PID,
                       control_motion_params->side_angle_velocity_parameter, 10,
                       MOMENTUM_MOTOR_PWM_MAX, 8000);
    control_param_init(&side_angle_PID,
                       control_motion_params->side_angle_parameter, 10, 9999,
                       2.5);
    control_param_init(&side_velocity_PID,
                       control_motion_params->side_velocity_parameter, 10000,
                       10, 10);

    control_param_init(&turn_angle_PID,
                       control_motion_params->turn_angle_parameter, 100, 9999,
                       500);
    control_param_init(&turn_angle_velocity_PID,
                       control_motion_params->turn_velocity_parameter, 1, 9999,
                       500);
}

/// @brief init the control parameter in the menu
void control_manual_param_init() {
    bottom_angle_velocity_PID.Kp = 0;
    bottom_angle_velocity_PID.Ki = 0;
    bottom_angle_velocity_PID.Kd = 0;
    bottom_angle_PID.Kp = 0;
    bottom_angle_PID.Ki = 0;
    bottom_angle_PID.Kd = 0;
    bottom_velocity_PID.Kp = 0;
    bottom_velocity_PID.Ki = 0;
    bottom_velocity_PID.Kd = 0;
    side_angle_velocity_PID.Kp = 0;
    side_angle_velocity_PID.Ki = 0;
    side_angle_velocity_PID.Kd = 0;
    side_angle_PID.Kp = 0;
    side_angle_PID.Ki = 0;
    side_angle_PID.Kd = 0;
    side_velocity_PID.Kp = 0;
    side_velocity_PID.Ki = 0;
    side_velocity_PID.Kd = 0;
    turn_angle_PID.Kp = 0;
    turn_angle_PID.Ki = 0;
    turn_angle_PID.Kd = 0;
    turn_angle_velocity_PID.Kp = 0;
    turn_angle_velocity_PID.Ki = 0;
    turn_angle_velocity_PID.Kd = 0;
}

static void control_param_init(pid_type_def* pid,
                               const uint32 para[3],
                               float coefficient,
                               float maxOut,
                               float maxIOut) {
    float temp_pid[3];
    temp_pid[0] = (float)para[0] / coefficient;
    temp_pid[1] = (float)para[1] / coefficient;
    temp_pid[2] = (float)para[2] / coefficient;
    PID_init_Position(pid, temp_pid, maxOut, maxIOut);
}