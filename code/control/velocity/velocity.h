#ifndef _VELOCITY_H_
#define _VELOCITY_H_

#include "zf_common_headfile.h"

#define V_KALMAN_MULTIPLE 100

// 脉冲当量：1m / 编码器脉冲数
#define ENCODER_PULSE (1.0f / 5978.0f)
#define ENCODER2VELOCITY (ENCODER_PULSE / PIT_VELOCITY_T * 1000.0f)

struct Velocity_Motor
{
    int32 momentum_front;
    int32 momentum_back;
    int32 momentum_sum;
    int32 momentum_diff;

    int32 bottom;          // 编码器数据
    float bottom_real;     // 线速度
    float bottom_filtered; // 滤波后的速度
};

extern struct Velocity_Motor g_vel_motor;

void velocity_init(struct Velocity_Motor *vel_motor);
void velocity_update_bottom(struct Velocity_Motor *vel_motor);
void velocity_update_side(struct Velocity_Motor *vel_motor);

#endif