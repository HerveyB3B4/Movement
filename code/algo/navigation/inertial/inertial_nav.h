#ifndef _inertial_nav_H
#define _inertial_nav_H

#include "zf_common_headfile.h"

typedef struct
{
    float x_vel;
    float y_vel;
    float x;
    float y;
} inertial_nav_pos;

void inertial_nav_init(float sample_time);
void inertial_nav_update(void);

void inertial_nav_get_pos(float *x, float *y);
void inertial_nav_get_vel(float *x_vel, float *y_vel);

#endif