#include "side.h"
#include "velocity.h"
#include "Attitude.h"
#include "control.h"

uint32 side_front_deadzone = 172;
uint32 side_back_deadzone = 169;

uint32 g_control_output_sav_flag = 0;
uint32 g_control_output_sv_flag = 0;
uint32 g_control_output_sa_flag = 0;

static int32 s_side_balance_duty = 0;
static int32 s_side_internal_diff = 0;

// side
static void control_side_velocity(
    struct Velocity_Motor *vel_motor,
    struct Control_Target *control_target,
    struct Control_Turn_Manual_Params *control_turn_params,
    struct Control_Motion_Manual_Parmas *control_motion_params);
static void control_side_angle(struct EulerAngle *euler_angle_bias,
                               struct Control_Target *control_target,
                               struct Control_Motion_Manual_Parmas *control_motion_params);
static void control_side_angle_velocity(struct Control_Target *control_target,
                                        struct Control_Motion_Manual_Parmas *control_motion_params);

int32 get_side_duty()
{
    return s_side_balance_duty;
}

void side_set_internal_diff(int32 diff)
{
    // 设置内部差值
    s_side_internal_diff = diff;
}

void control_side_balance(
    struct Control_Target *control_target,
    struct Control_Flag *control_flag,
    struct Control_Turn_Manual_Params *control_turn_params,
    struct Velocity_Motor *vel_motor,
    struct EulerAngle *euler_angle_bias,
    struct Control_Motion_Manual_Parmas *control_motion_params)
{
    if (control_flag->side_vel)
    {
        control_flag->side_vel = 0;
        velocity_update_side(vel_motor);
        control_side_velocity(vel_motor, control_target, control_turn_params, control_motion_params);
    }

    if (control_flag->side_angle)
    {
        control_flag->side_angle = 0;
        control_side_angle(euler_angle_bias, control_target, control_motion_params);
    }

    if (control_flag->side_angle_vel)
    {
        control_flag->side_angle_vel = 0;
        control_side_angle_velocity(control_target, control_motion_params);
    }

    static int side_duty_filter[2] = {0}; // 角度滤波
    side_duty_filter[1] = side_duty_filter[0];
    side_duty_filter[0] = s_side_balance_duty;
    // noiseFilter(momentumAngleFilter[0], 0.02f);
    lowPassFilterI(&side_duty_filter[0], &side_duty_filter[1], 0.2f);

    // s_side_balance_duty = control_target->side_angle * 100;
    // turnControl();
    int32 left_motor_duty, right_motor_duty;

    int32 turn_diff_left = (int32_t)(get_momentum_diff() *
                                     (float)(3.3f -
                                             logf(0.2f * abs(vel_motor->momentumFront) + 8)));
    int32 turn_diff_right = (int32_t)(get_momentum_diff() *
                                      (float)(3.3f -
                                              logf(0.2f * abs(vel_motor->momentumBack) + 8)));

    // left_motor_duty = -(s_side_balance_duty + s_side_internal_diff + turn_diff_left);
    // right_motor_duty = s_side_balance_duty + s_side_internal_diff - turn_diff_right;

    left_motor_duty = -(s_side_balance_duty + s_side_internal_diff + get_momentum_diff());
    right_motor_duty = s_side_balance_duty + s_side_internal_diff - get_momentum_diff();

    // left_motor_duty = -get_momentum_diff();
    // right_motor_duty = -get_momentum_diff();

    // left_motor_duty =
    //     -s_side_balance_duty -
    //     (int32_t)(get_momentum_diff() *
    //               (float)(3.3f -
    //                       logf(0.2f * abs(vel_motor->momentumFront) + 8)));
    // right_motor_duty =
    //     s_side_balance_duty -
    //     (int32_t)(get_momentum_diff() *
    //               (float)(3.3f -
    //                       logf(0.2f * abs(vel_motor->momentumBack) + 8)));

    // 添加死区补偿
    if (left_motor_duty > 0)
    {
        left_motor_duty += side_front_deadzone;
    }
    else if (left_motor_duty < 0)
    {
        left_motor_duty -= side_front_deadzone;
    }

    if (right_motor_duty > 0)
    {
        right_motor_duty += side_back_deadzone;
    }
    else if (right_motor_duty < 0)
    {
        right_motor_duty -= side_back_deadzone;
    }

    // printf("%d,%d\n", left_motor_duty, right_motor_duty);
    restrictValueI(&left_motor_duty, -8000, 8000);
    restrictValueI(&right_motor_duty, -8000, 8000);
    set_momentum_motor_pwm(left_motor_duty, right_motor_duty);
    // set momentum motor pwm to keep side balance
}

static void control_side_velocity(
    struct Velocity_Motor *vel_motor,
    struct Control_Target *control_target,
    struct Control_Turn_Manual_Params *control_turn_params,
    struct Control_Motion_Manual_Parmas *control_motion_params)
{
    static float side_vel_filter[2] = {0};
    side_vel_filter[1] = side_vel_filter[0];
    side_vel_filter[0] = (float)(vel_motor->momentumFront - vel_motor->momentumBack);
    // noiseFilter(momentumAngleFilter[0], 0.02f);
    // lowPassFilterF(&side_vel_filter[0], &side_vel_filter[1], 0.2f);

    control_target->side_angle = control_motion_params->side_velocity_polarity *
                                 PID_calc_Position(&side_velocity_PID,
                                                   side_vel_filter[0],
                                                   0.0f);

    // static float side_tar_angle_filter[2] = {0};
    // side_tar_angle_filter[1] = side_tar_angle_filter[0];
    // side_tar_angle_filter[0] = control_target->side_angle;
    // lowPassFilterF(&side_tar_angle_filter[0], &side_tar_angle_filter[1], 0.2f);

    if (g_control_output_sv_flag != 0)
    {
        printf("%f,%f\n", control_target->side_angle, side_vel_filter[0]);
    }
}

static void control_side_angle(struct EulerAngle *euler_angle_bias,
                               struct Control_Target *control_target,
                               struct Control_Motion_Manual_Parmas *control_motion_params)
{
    static uint32 last = 0;
    uint32 curr = system_getval_us();
    printf("%d ", curr - last);
    last = curr;

    static float momentumAngleFilter[2] = {0}; // 角度滤波
    momentumAngleFilter[1] = momentumAngleFilter[0];
    momentumAngleFilter[0] = -ROLL_VEL;
    // noiseFilter(momentumAngleFilter[0],0.02f);
    // lowPassFilterF(&momentumAngleFilter[0], &momentumAngleFilter[1], 0.3f);

    // control_target->side_angle_vel = control_motion_params->side_angle_polarity *
    //                                  PID_calc_Position_Gyro_D(
    //                                      &side_angle_PID,
    //                                      (ROLL - euler_angle_bias->roll),
    //                                      //  control_target->side_angle + control_target->buckling_side // 压弯
    //                                      control_target->side_angle,
    //                                      momentumAngleFilter[0]);

    control_target->side_angle_vel = control_motion_params->side_angle_polarity *
                                     PID_calc_Position(
                                         &side_angle_PID,
                                         (ROLL - euler_angle_bias->roll),
                                         //  control_target->side_angle + control_target->buckling_side // 压弯
                                         control_target->side_angle);

    if (g_control_output_sa_flag != 0)
    {
        printf("%f, %f\n", ROLL, control_target->side_angle_vel);
    }
    uint32 curr_cal = system_getval_us();
    printf("%d\n", curr_cal - curr);
}

static void control_side_angle_velocity(struct Control_Target *control_target,
                                        struct Control_Motion_Manual_Parmas *control_motion_params)
{
    static float side_angle_vel_filter[2] = {0}; // 角速度滤波
    side_angle_vel_filter[1] = side_angle_vel_filter[0];
    side_angle_vel_filter[0] = ROLL_VEL;
    // lowPassFilterF(&side_angle_vel_filter[0], &side_angle_vel_filter[1], 0.5f);

    s_side_balance_duty =
        control_motion_params->side_angle_velocity_polarity *
        (int32)(PID_calc_DELTA(&side_angle_velocity_PID,
                               side_angle_vel_filter[0],
                               control_target->side_angle_vel));

    if (g_control_output_sav_flag != 0)
    {
        printf("%f,%f\n", -ROLL_VEL,
               s_side_balance_duty / 100.0f);
    }
}
