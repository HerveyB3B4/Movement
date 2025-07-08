#ifndef INS_CORE_H
#define INS_CORE_H

#include "imu.h"

/**
 * @file ins_core.h
 * @brief 惯性导航解算核心库 - 头文件
 * @details 负责根据姿态角和加速度计算速度和位置。
 */

// --- 常量定义 ---
#define GRAVITY_MSS GravityAcc // 重力加速度 (m/s^2), 您可以根据当地精确值修改

// --- 数据结构定义 ---

/**
 * @brief 惯性导航系统状态结构体
 * @details 存储速度和位置信息，所有数据都在导航坐标系(NED: 北-东-地)下。
 */
typedef struct
{
    // 速度 (Velocity) - 在导航坐标系(NED)下, 单位: m/s
    float vel_n; // 北向速度
    float vel_e; // 东向速度
    float vel_d; // 地向(向下为正)速度

    // 位置 (Position) - 在导航坐标系(NED)下的相对位置, 单位: m
    float pos_n; // 北向相对位置
    float pos_e; // 东向相对位置
    float pos_d; // 地向相对位置
} ins_state_t;

extern ins_state_t g_ins_state;

// --- 函数接口声明 ---

/**
 * @brief 初始化惯性导航系统状态
 * @param state 指向要初始化的INS状态结构体的指针
 */
void ins_init(ins_state_t *state);

/**
 * @brief 使用已知的姿态角来更新INS状态 (核心解算函数)
 *
 * @param state        指向INS状态结构体的指针
 * @param accel_mps2   一个包含3个元素的数组，存放机体坐标系的加速度 [ax, ay, az], 单位: m/s^2
 * @param roll_rad     当前测量的翻滚角, 单位: 弧度 (rad)
 * @param pitch_rad    当前测量的俯仰角, 单位: 弧度 (rad)
 * @param yaw_rad      当前测量的航向角, 单位: 弧度 (rad)
 * @param dt           两次更新之间的时间差, 单位: 秒 (s)
 */
void ins_update(ins_state_t *state, float acc_x, float acc_y, float acc_z,
                float roll_rad, float pitch_rad, float yaw_rad, float dt);

#endif // INS_CORE_H