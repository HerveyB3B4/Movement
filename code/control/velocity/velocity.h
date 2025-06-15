#ifndef _VELOCITY_H_
#define _VELOCITY_H_

#include "zf_common_headfile.h"

#define V_KALMAN_MULTIPLE 5000
#define VELOCITY_KALMAN_FILTER

#define ENCODER_TO_VELOCITY ((0.6f) / (3600.0f) / (PIT_VELOCITY_T) * 1000.0f)

extern uint32 vel_time;
struct Velocity_Motor
{
    int32 momentumFront;
    int32 momentumBack;
    int32 bottom;
    float bottomReal;
    float bottomFiltered;
    int32 bottomSum;
    int32 velocityDiff;
};

extern struct Velocity_Motor g_vel_motor;

void velocity_init(struct Velocity_Motor *vel_motor);
void velocity_update(struct Velocity_Motor *vel_motor);
#endif