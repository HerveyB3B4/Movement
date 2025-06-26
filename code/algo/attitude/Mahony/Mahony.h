#ifndef _ATTITUDE_MAHONY_H
#define _ATTITUDE_MAHONY_H

#include "zf_common_headfile.h"

#define PI_2 1.57079632679489661923f // π/2

typedef struct
{
    // 四元数状态
    float q0, q1, q2, q3;

    // 滤波器参数
    float beta;         // 基础融合系数
    float beta_static;  // 静止时的融合系数
    float beta_dynamic; // 运动时的融合系数

    // 传感器校准参数
    float g_bias_x, g_bias_y, g_bias_z;       // 陀螺仪零偏
    float acc_bias_x, acc_bias_y, acc_bias_z; // 加速度计零偏

    // 运动状态检测
    float gravity_ref;          // 参考重力值 (m/s²)
    float acc_threshold_static; // 静止检测阈值
    float acc_threshold_motion; // 运动检测阈值

    // 时间相关
    float sample_rate;     // 采样频率 (Hz)
    float inv_sample_rate; // 采样时间 (1/s)

    // 滤波器状态
    unsigned long last_update; // 最后更新时间 (us)
    int is_initialized;        // 初始化标志
    uint16 init_count;
} MahonyAHRS;

// 函数声明
void MahonyAHRS_init(float sample_rate);
int MahonyAHRS_calibrate(IMU_DATA *imu_data);
void MahonyAHRS_update(IMU_DATA *imu_data);

float MahonyAHRS_get_pitch(void);
float MahonyAHRS_get_roll(void);
float MahonyAHRS_get_yaw(void);

#endif // _ATTITUDE_MAHONY_H