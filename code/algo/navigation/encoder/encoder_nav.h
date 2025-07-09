#ifndef ENCODER_NAV_H
#define ENCODER_NAV_H

#include "zf_common_headfile.h"

typedef struct
{
    float x;
    float y;
    float x_encoder;
    float y_encoder;
    float space;
} encoder_nav_pos;

void encoder_nav_init(float sample_time);
void encoder_nav_update(struct Velocity_Motor *vel_motor);

#endif