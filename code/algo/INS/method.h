#ifndef TRACE_H
#define TRACE_H

#include <stdint.h>
#include "../attitude/Attitude.h"  // 引入已有的姿态解算头文件

/**
 * @brief 三维向量结构体
 */
typedef struct {
    float x;
    float y;
    float z;
} Vector3f;

/**
 * @brief 惯性导航系统模式枚举
 */
typedef enum {
    INS_MODE_BASIC = 0,    // 基本模式 - 简单欧拉积分
    INS_MODE_RUNGE_KUTTA,  // 龙格库塔积分模式 - 更高精度
    INS_MODE_KALMAN,       // 卡尔曼滤波模式 - 更高精度、抗干扰
    INS_MODE_ADAPTIVE      // 自适应模式 - 根据运动状态自动选择算法
} INS_Mode;

/**
 * @brief 零速度检测方法枚举
 */
typedef enum {
    ZUPT_BASIC = 0,  // 基本阈值检测
    ZUPT_ADVANCED,   // 高级检测 - 考虑加速度和角速度
    ZUPT_ADAPTIVE    // 自适应检测 - 动态调整阈值
} ZUPT_Method;

/**
 * @brief 卡尔曼滤波器状态
 */
typedef struct {
    float P[6][6];  // 协方差矩阵 (位置和速度)
    float Q[6];     // 过程噪声协方差
    float R[3];     // 测量噪声协方差
    float K[6][3];  // 卡尔曼增益
} KalmanFilter;

/**
 * @brief 惯性导航系统数据结构
 */
typedef struct {
    /* 传感器数据 */
    Vector3f accel_raw;  // 加速度计原始数据 (m/s^2)
    Vector3f accel;      // 校准后的加速度数据
    Vector3f gyro;       // 角速度数据，用于高级ZUPT

    /* 校准参数 */
    Vector3f accel_bias;   // 加速度计零偏
    Vector3f accel_scale;  // 加速度计比例因子校准

    /* 位置与速度估计 */
    Vector3f position;          // 位置 (m)
    Vector3f position_last;     // 上一次位置 (m)，用于计算速度
    Vector3f position_k1;       // 上上次位置，用于高阶积分
    Vector3f velocity;          // 速度 (m/s)
    Vector3f velocity_last;     // 上一次速度，用于高阶积分
    Vector3f accel_world;       // 世界坐标系下的加速度
    Vector3f accel_world_last;  // 上一次世界坐标系加速度，用于高阶积分

    /* 龙格库塔积分中间值 */
    Vector3f k1, k2, k3, k4;  // 龙格库塔中间值 - 速度
    Vector3f l1, l2, l3, l4;  // 龙格库塔中间值 - 位置

    /* 卡尔曼滤波器 */
    KalmanFilter kf;  // 卡尔曼滤波器状态

    /* 系统参数 */
    float dt;                // 采样时间间隔 (s)
    float accel_threshold;   // 加速度阈值，用于静止检测
    uint8_t is_initialized;  // 初始化标志
    uint8_t zero_velocity;   // 零速度更新标志

    /* 导航模式和方法配置 */
    INS_Mode mode;            // 当前导航模式
    ZUPT_Method zupt_method;  // 零速度检测方法

    /* 自适应模式参数 */
    float motion_intensity;  // 运动强度指标
    uint8_t high_dynamic;    // 高动态运动标志
} INS_State;

/**
 * @brief 初始化惯性导航系统
 *
 * @param ins 惯性导航系统状态指针
 * @param sample_time 采样时间间隔(s)
 * @param mode 导航模式
 * @param zupt_method 零速度检测方法
 */
void INS_Init(INS_State* ins,
              float sample_time,
              INS_Mode mode,
              ZUPT_Method zupt_method);

/**
 * @brief 更新传感器数据
 *
 * @param ins 惯性导航系统状态指针
 * @param accel_raw 加速度计原始数据
 * @param gyro 陀螺仪原始数据，用于高级ZUPT
 */
void INS_UpdateSensorData(INS_State* ins,
                          const Vector3f* accel_raw,
                          const Vector3f* gyro);

/**
 * @brief 将加速度从传感器坐标系转换到世界坐标系
 *
 * @param ins 惯性导航系统状态指针
 * @param euler_angle 欧拉角指针
 */
void INS_ConvertAccelToWorldFrame(INS_State* ins,
                                  const struct EulerAngle* euler_angle);

/**
 * @brief 执行惯性导航系统的位置和速度更新 (基本欧拉积分)
 *
 * @param ins 惯性导航系统状态指针
 */
void INS_UpdatePositionVelocityEuler(INS_State* ins);

/**
 * @brief 执行惯性导航系统的位置和速度更新 (龙格库塔积分)
 *
 * @param ins 惯性导航系统状态指针
 * @param euler_angle 欧拉角指针 (用于中间步骤的坐标转换)
 */
void INS_UpdatePositionVelocityRK4(INS_State* ins,
                                   const struct EulerAngle* euler_angle);

/**
 * @brief 执行惯性导航系统的位置和速度更新 (卡尔曼滤波)
 *
 * @param ins 惯性导航系统状态指针
 */
void INS_UpdatePositionVelocityKalman(INS_State* ins);

/**
 * @brief 更新自适应模式下的运动强度指标
 *
 * @param ins 惯性导航系统状态指针
 */
void INS_UpdateMotionIntensity(INS_State* ins);

/**
 * @brief 执行零偏校准
 *
 * @param ins 惯性导航系统状态指针
 * @param samples 采样次数
 */
void INS_Calibrate(INS_State* ins, uint16_t samples);

/**
 * @brief 执行多位置静止校准 - 进阶校准方法
 *
 * @param ins 惯性导航系统状态指针
 * @param num_positions 校准位置数
 * @param samples_per_position 每个位置的采样数
 */
void INS_CalibrateAdvanced(INS_State* ins,
                           uint8_t num_positions,
                           uint16_t samples_per_position);

/**
 * @brief 复位惯性导航系统
 *
 * @param ins 惯性导航系统状态指针
 */
void INS_Reset(INS_State* ins);

/**
 * @brief 主更新函数，处理所有惯性导航计算
 *
 * @param ins 惯性导航系统状态指针
 * @param euler_angle 欧拉角指针
 */
void INS_Update(INS_State* ins, const struct EulerAngle* euler_angle);

/**
 * @brief 零速度更新，用于修正速度漂移
 *
 * @param ins 惯性导航系统状态指针
 * @param is_stationary 是否处于静止状态
 */
void INS_ZeroVelocityUpdate(INS_State* ins, uint8_t is_stationary);

/**
 * @brief 高级零速度检测
 *
 * @param ins 惯性导航系统状态指针
 * @return uint8_t 检测结果，1表示静止，0表示运动
 */
uint8_t INS_ZeroVelocityDetectionAdvanced(INS_State* ins);

/**
 * @brief 设置导航模式
 *
 * @param ins 惯性导航系统状态指针
 * @param mode 导航模式
 */
void INS_SetMode(INS_State* ins, INS_Mode mode);

/**
 * @brief 设置零速度检测方法
 *
 * @param ins 惯性导航系统状态指针
 * @param method 检测方法
 */
void INS_SetZuptMethod(INS_State* ins, ZUPT_Method method);

/**
 * @brief 初始化卡尔曼滤波器
 *
 * @param ins 惯性导航系统状态指针
 */
void INS_InitKalmanFilter(INS_State* ins);

/* 全局导航系统状态 */
extern INS_State g_ins_state;

#endif /* TRACE_H */