#include "attitude.h"
#include "QuaternionEKF.h"
#include "control.h"
#include "velocity.h"
#include "zf_common_headfile.h"
#include "Madgwick.h"
#include "Mahony.h"
#include "system.h"
#include "YawIntegral.h"

struct EulerAngle g_euler_angle;
struct EulerAngle g_euler_angle_bias;

uint8 g_attitude_cal_flag = 0;
uint8 attitude_time = 0;

static Attitude_algorithm current_algorithm;

void attitude_init(Attitude_algorithm algo)
{
    current_algorithm = algo;

    switch (current_algorithm)
    {
    case ATTITUDE_EKF:
        IMU_QuaternionEKF_Init(10, 0.001, 10000000, 1, 0.001f, 0);
        break;
    case ATTITUDE_MADGWICK:
        MadgwickAHRS_init();
        break;
    case ATTITUDE_MAHONY:
        MahonyAHRS_init();
        imu_get_data(&g_imu_data); // 获取一个初始的acc信息加速收敛
        MahonyAHRS_calibrate(g_imu_data);
        break;
    default:
        MahonyAHRS_init();
        imu_get_data(&g_imu_data);
        MahonyAHRS_calibrate(g_imu_data);
        break;
    }

    imu_init_offset(); // 初始化零飘
}

void attitude_cal(struct IMU_DATA *data)
{
    imu_get_data(data);
    imu_remove_offset(data);

    switch (current_algorithm)
    {
    case ATTITUDE_EKF:
        IMU_QuaternionEKF_Update(*data);
        break;
    case ATTITUDE_MADGWICK:
        MadgwickAHRS_update(*data);
        break;
    case ATTITUDE_MAHONY:
        MahonyAHRS_update(*data);
        break;
    default:
        MahonyAHRS_update(*data);
        break;
    }
}

void attitude_cal_amend(struct Control_Turn_Manual_Params *turn_param,
                        struct Control_Target *control_target,
                        struct Velocity_Motor *velocity_motor,
                        struct EulerAngle *euler_angle,
                        struct IMU_DATA *data)
{
    // if (g_turn_start_flag)
    // {
    //     float x = (float)control_target->turn_angle_vel * 0.01f *
    //               fabsf((float)velocity_motor->bottom_filtered * 0.01f);
    //     control_target->buckling_side = turn_param->buckling_side_state * x;
    //     restrictValueF(&control_target->buckling_side, 10.5f, -10.5f);
    //     control_target->buckling_front = x * turn_param->buckling_front_coefficientT;
    //     restrictValueF(&control_target->buckling_front, 5.0f, -5.0f);
    // }

    attitude_cal(data);

    switch (current_algorithm)
    {
    case ATTITUDE_EKF:
        euler_angle->roll = EKF_get_roll();
        euler_angle->pitch = EKF_get_pitch();
        euler_angle->yaw = EKF_get_yaw();
        break;
    case ATTITUDE_MADGWICK:
        euler_angle->roll = MadgwickAHRS_get_roll();
        euler_angle->pitch = MadgwickAHRS_get_pitch();
        euler_angle->yaw = MadgwickAHRS_get_yaw();
        break;
    case ATTITUDE_MAHONY:
        euler_angle->roll = MahonyAHRS_get_roll();
        euler_angle->pitch = MahonyAHRS_get_pitch();
        euler_angle->yaw = MahonyAHRS_get_yaw();
        break;
    default:
        euler_angle->roll = EKF_get_roll();
        euler_angle->pitch = EKF_get_pitch();
        euler_angle->yaw = EKF_get_yaw();
        break;
    }

    euler_angle->yaw = 360.0f - euler_angle->yaw; // 0~360
    euler_angle->yaw > 360 ? (euler_angle->yaw -= 360)
                           : euler_angle->yaw; // 0~360
}
