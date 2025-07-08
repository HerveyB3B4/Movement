#include "control.h"
#include "menu_input.h"
#include "motor.h"
#include "pid.h"
#include "system.h"
#include "velocity.h"
#include "zf_common_headfile.h"

uint32 control_params_from = 0; // 0: menu, 1: preset
uint32 control_time = 0;

// global
uint8 g_turn_start_flag = 0;
int32 g_control_shutdown_flag = 0;
uint32 g_control_bottom_flag = 0;
uint32 g_control_side_flag = 0;
uint32 g_control_turn_flag = 0;

float g_angle_limit = 50.0f; // 菜单上的角度限制

struct Control_Turn_Manual_Params g_control_turn_manual_params;
struct Control_Target g_control_target;
struct Control_Flag g_control_flag;
struct Control_Time g_control_time;
struct Control_Motion_Manual_Parmas g_control_motion_params;

// static
static void control_param_init(pid_type_def *pid,
                               const uint32 para[3],
                               float kp_coefficient,
                               float ki_coefficient,
                               float kd_coefficient,
                               float maxOut,
                               float maxIOut);
static void control_init_preset(struct Control_Motion_Manual_Parmas *control_motion_params);
static void control_init_menu(struct Control_Motion_Manual_Parmas *control_motion_params);

void control_reset(struct Control_Target *control_target);

// PID
// bottom wheel
pid_type_def bottom_angle_velocity_PID;
pid_type_def bottom_angle_PID;
pid_type_def bottom_velocity_PID;
pid_type_def bottom_position_PID;

// momentum wheel pid
pid_type_def side_angle_velocity_PID;
pid_type_def side_angle_PID;
pid_type_def side_velocity_PID;

// turn pid
pid_type_def turn_angle_PID;
pid_type_def turn_angle_velocity_PID;
pid_type_def turn_velocity_PID;
pid_type_def turn_error_PID;

void control_shutdown(struct Control_Target *control_target,
                      struct EulerAngle *euler_angle_bias,
                      struct Velocity_Motor *vel_motor)
{
    if (g_control_shutdown_flag != 0)
    {
        if (fabsf(ROLL - euler_angle_bias->roll) > 28)
        {
            system_set_runstate(CAR_STOP);
        }
        if (fabsf(PITCH - euler_angle_bias->pitch) > 35)
        {
            system_set_runstate(CAR_STOP);
        }
        // if (abs(get_side_duty()) >= 8000 || abs(get_bottom_duty()) >= 8000)
        // {
        //     system_set_runstate(CAR_STOP);
        // }
    }
}

void control_polarity_init(struct Control_Motion_Manual_Parmas *control_motion_params)
{
    control_motion_params->bottom_angle_velocity_polarity = -1;
    control_motion_params->bottom_angle_polarity = -1;
    control_motion_params->bottom_velocity_polarity = -1;

    control_motion_params->side_angle_velocity_polarity = -1;
    control_motion_params->side_angle_polarity = 1;
    control_motion_params->side_velocity_polarity = -1;

    control_motion_params->turn_angle_velocity_polarity = 1;
    control_motion_params->turn_error_polarity = 1;
    control_motion_params->turn_velocity_polarity = 1;
    control_motion_params->turn_angle_polarity = 1;
}

void control_init(struct Control_Motion_Manual_Parmas *control_motion_params)
{
    control_polarity_init(control_motion_params);

    if (control_params_from == 0)
    {
        // 从菜单中获取参数
        control_init_menu(control_motion_params);
    }
    else if (control_params_from == 1)
    {
        // 从预设中获取参数
        control_init_preset(control_motion_params);
    }
}

void control_manual_param_init()
{
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

void control_reset(struct Control_Target *control_target)
{
    // 重置控制目标值
    control_target->bottom_angle = 0;
    control_target->bottom_vel = 0;
    control_target->side_angle = 0;
    control_target->turn_angle = 0;
    control_target->turn_angle_vel = 0;
    control_target->buckling_side = 0;

    // 重置控制标志
    g_control_flag.bottom_angle = 0;
    g_control_flag.bottom_vel = 0;
    g_control_flag.side_angle = 0;
    g_control_flag.side_vel = 0;
    g_control_flag.turn_angle = 0;
    g_control_flag.turn_vel = 0;
    g_control_flag.turn_err = 0;
    // g_control_flag.buckling_side = 0;

    control_init(&g_control_motion_params);
}

static void control_init_preset(struct Control_Motion_Manual_Parmas *control_motion_params)
{
    // bottom wheel

    // float bottom_angle_velocity_pid[3] = {15, 0.20, 0};
    // float bottom_angle_pid[3] = {4, 0.0, 1};
    // float bottom_velocity_pid[3] = {0.001, 0.000000, 0.00};

    // float bottom_angle_velocity_pid[3] = {20, 0.20, 0};
    // float bottom_angle_pid[3] = {3.5, 0.0, 1};
    // float bottom_velocity_pid[3] = {0.0007, 0.0000035, 0.00};

    // float bottom_angle_velocity_pid[3] = {25, 0.15, 8};
    // float bottom_angle_pid[3] = {6, 0.0, 2};
    // float bottom_velocity_pid[3] = {0.00065, 0.0000035, 0.00};

    float bottom_angle_velocity_pid[3] = {20, 0.4, 0};
    float bottom_angle_pid[3] = {0, 0.0, 0};
    float bottom_velocity_pid[3] = {1, 0, 0};

    PID_init_Position(&bottom_angle_velocity_PID, bottom_angle_velocity_pid,
                      9999, 9999);
    PID_init_Position(&bottom_angle_PID, bottom_angle_pid, 9999, 9999);
    PID_init_Position(&bottom_velocity_PID, bottom_velocity_pid, 9999, 10);

    // printf("fav: kp: %f, ki: %f, kd: %f\n", bottom_angle_velocity_PID.Kp,
    //        bottom_angle_velocity_PID.Ki, bottom_angle_velocity_PID.Kd);
    // printf("fa: kp: %f, ki: %f, kd: %f\n", bottom_angle_PID.Kp,
    //        bottom_angle_PID.Ki, bottom_angle_PID.Kd);
    // printf("fv: kp: %f, ki: %f, kd: %f\n", bottom_velocity_PID.Kp,
    //        bottom_velocity_PID.Ki, bottom_velocity_PID.Kd);

    float side_angle_velocity_pid[3] = {58, 8, 1};
    float side_angle_pid[3] = {2.5, 0, 5};
    float side_velocity_pid[3] = {0.0058, 0.0, 0.000};

    PID_init_Position(&side_angle_velocity_PID, side_angle_velocity_pid, MOMENTUM_MOTOR_PWM_MAX, 8000);
    PID_init_Position(&side_angle_PID, side_angle_pid, 9999, 2.5f);
    PID_init_Position(&side_velocity_PID, side_velocity_pid, 9999, 10);

    float turn_angle_velocity_pid[3] = {0.0, 0.0, 0.0};
    float turn_angle_pid[3] = {0.0, 0.0, 0.0};
    float turn_velocity_pid[3] = {0.0, 0.0, 0.0};
    float turn_error_pid[3] = {0.0, 0.0, 0.0};

    PID_init_Position(&turn_angle_velocity_PID, turn_angle_velocity_pid, 9999, 500);
    PID_init_Position(&turn_angle_PID, turn_angle_pid, 9999, 500);
    PID_init_Position(&turn_velocity_PID, turn_velocity_pid, 9999, 500);
    PID_init_Position(&turn_error_PID, turn_error_pid, 9999, 500);
}

static void control_param_init(pid_type_def *pid,
                               const uint32 para[3],
                               float kp_coefficient,
                               float ki_coefficient,
                               float kd_coefficient,
                               float maxOut,
                               float maxIOut)
{
    float temp_pid[3];
    temp_pid[0] = (float)para[0] / kp_coefficient;
    temp_pid[1] = (float)para[1] / ki_coefficient;
    temp_pid[2] = (float)para[2] / kd_coefficient;
    PID_init_Position(pid, temp_pid, maxOut, maxIOut);
}

static void control_init_menu(struct Control_Motion_Manual_Parmas *control_motion_params)
{
    control_param_init(&bottom_angle_velocity_PID,
                       control_motion_params->bottom_angle_velocity_parameter,
                       1, 10, 1, MOTOR_PWM_MAX, 9999);
    // control_param_init(&bottom_angle_PID,
    //                    control_motion_params->bottom_angle_parameter, 10,
    //                    100, 10);
    control_param_init(&bottom_angle_PID,
                       control_motion_params->bottom_angle_parameter,
                       10, 1, 100, 9999, 10);
    control_param_init(&bottom_velocity_PID,
                       control_motion_params->bottom_velocity_parameter,
                       1000, 10000, 1000, 5000, 10.0f);

    control_param_init(&bottom_position_PID,
                       control_motion_params->bottom_position_parameter,
                       100, 10000, 1000, 10, 10.0f);

    // momentum wheel pid
    control_param_init(&side_angle_velocity_PID,
                       control_motion_params->side_angle_velocity_parameter,
                       1, 10, 10, MOMENTUM_MOTOR_PWM_MAX, 8000);
    control_param_init(&side_angle_PID,
                       control_motion_params->side_angle_parameter,
                       10, 10, 10, 9999, 2.5f);
    control_param_init(&side_velocity_PID,
                       control_motion_params->side_velocity_parameter,
                       10000, 10000000, 10000, 9999, 10);

    // turn pid
    control_param_init(&turn_angle_velocity_PID,
                       control_motion_params->turn_angle_velocity_parameter, 10, 10, 10,
                       2000, 2000);
    control_param_init(&turn_error_PID,
                       control_motion_params->turn_error_parameter, 1, 1, 1,
                       5000, 500);
    control_param_init(&turn_angle_PID,
                       control_motion_params->turn_angle_parameter, 100, 100, 100, 9999,
                       500);
    control_param_init(&turn_velocity_PID,
                       control_motion_params->turn_velocity_parameter, 1000, 1000, 1000,
                       9999, 500);
}