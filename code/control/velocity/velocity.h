#ifndef _VELOCITY_H_
#define _VELOCITY_H_

#include "zf_common_headfile.h"

#define V_KALMAN_MULTIPLE 100
#define VELOCITY_KALMAN_FILTER

// 一块砖长度(m)/编码器脉冲数/速度更新周期(ms)
#define ENCODER_TO_VELOCITY ((0.6f) / (7200.0f) / (PIT_VELOCITY_T) * 1000.0f)
#define BOTTOM_VELOCITY_PARAMS 100.0f

extern uint32 vel_time;
struct Velocity_Motor
{
    int32 momentumFront;
    int32 momentumBack;
    int32 bottom;
    float bottom_real;
    float bottom_filtered;
    int32 velocityDiff;
};

extern struct Velocity_Motor g_vel_motor;

void velocity_init(struct Velocity_Motor *vel_motor);
void velocity_update(struct Velocity_Motor *vel_motor);
void velocity_update_bottom(struct Velocity_Motor *vel_motor);
void velocity_update_side(struct Velocity_Motor *vel_motor);

#endif