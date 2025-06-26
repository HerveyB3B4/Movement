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

void attitude_init()
{
    // 参数顺序依次为 Q1, Q2, R, lambda,
    // dt(这个已经和计时器中断一致)，根据波形图调整
    // IMU_QuaternionEKF_Init(10000, 100000, 1000000, 0.9996, 0.001f, 0);  //
    // ekf初始化
    // IMU_QuaternionEKF_Init(5, 20, 100, 0.9, 0.001f, 0); // ekf初始化 (原始参数)
    IMU_QuaternionEKF_Init(5, 20, 100, 0.9, 0.001f, 0); // ekf初始化 - 优化Yaw轴旋转
    imu_init_offset();                                  // 初始化零飘
    // YawIntegral_Init();                                 // 初始化Yaw积分
}

void attitude_cal_ekf()
{
    // EKF 姿态解算
    imu_get_data(&g_imu_data);
    imu_remove_offset(&g_imu_data);
    // // LowPassFilter
    // LowPassFilter(&Accelerometer[0], IMUdata.acc.x);
    // LowPassFilter(&Accelerometer[1], IMUdata.acc.y);
    // LowPassFilter(&Accelerometer[2], IMUdata.acc.z);

    IMU_QuaternionEKF_Update(&g_imu_data);
}

void attitude_init_Madgwick()
{
    uint8 flag = 0;
    while (!flag)
    {
        imu_get_data(&g_imu_data);
        imu_remove_offset(&g_imu_data);
        flag = MadgwickAHRS_calibrate(&g_imu_data);
    }
}

void attitude_cal_Madgwick()
{
    imu_get_data(&g_imu_data);
    imu_remove_offset(&g_imu_data);

    MadgwickAHRS_update(&g_imu_data);
}

static void attitude_cal_Mahony()
{
    imu_get_data(&g_imu_data);
    imu_remove_offset(&g_imu_data);

    MahonyAHRS_update(&g_imu_data);
}

void attitude_cal_amend(struct Control_Turn_Manual_Params *turn_param,
                        struct Control_Target *control_target,
                        struct Velocity_Motor *velocity_motor,
                        struct EulerAngle *euler_angle)
{
    // 修正姿态计算
    if (g_attitude_cal_flag == 0)
    {
        return;
    }
    else
    {
        g_attitude_cal_flag = 0;
    }

    if (g_turn_start_flag)
    {
        float x = (float)control_target->turn_angle_vel * 0.01f *
                  fabsf((float)velocity_motor->bottomFiltered * 0.01f);
        control_target->bucking = turn_param->buckling_turn_coefficient * x;
        restrictValueF(&control_target->bucking, 10.5f, -10.5f);
        control_target->Fbucking = x * turn_param->buckling_front_coefficientT;
        restrictValueF(&control_target->Fbucking, 5.0f, -5.0f);
    }
#ifdef USE_MAHONY
    attitude_cal_Mahony();
    euler_angle->roll = MahonyAHRS_get_roll() + control_target->bucking;
    euler_angle->pitch = MahonyAHRS_get_pitch() + control_target->Fbucking;
    euler_angle->yaw = MahonyAHRS_get_yaw() - 180 + YAWCorrection;
    ;
#endif
#ifdef USE_EKF
    attitude_cal_ekf();
    euler_angle->roll =
        ekf_get_roll() + control_target->bucking; // + convergenceGain;
    euler_angle->pitch = ekf_get_pitch() + control_target->Fbucking;
    euler_angle->yaw = ekf_get_yaw();
#endif
    // g_euler_angle->yaw += 180; // transfer to the same direction
    euler_angle->yaw = 360.0f - euler_angle->yaw; // opposite direction
    // g_euler_angle->yaw += YAWCorrection;
    euler_angle->yaw > 360 ? (euler_angle->yaw -= 360)
                           : euler_angle->yaw; // 0~360
    // update module state

    // moduleState.attitude = 1;

    attitude_time = system_getval();
}
