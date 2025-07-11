#include "inertial_nav.h"
#include "Attitude.h"

static inertial_nav_pos pos;
static float inertial_nav_sample_time = 0.01f; // Default sample time
static uint8 inertial_nav_enable_flag = 0;

void inertial_nav_init(float sample_time)
{
    pos.x_vel = 0.0f;
    pos.y_vel = 0.0f;
    pos.x = 0.0f;
    pos.y = 0.0f;

    inertial_nav_sample_time = sample_time;

    // for (uint16 i = 0; i < 100000; i++)
    // {
    //     if (fabsf(ROLL_ACC) < 0.1f && fabsf(PITCH_ACC) < 0.1f)
    //     {
    //         inertial_nav_enable_flag = 1;
    //         break;
    //     }
    // }
    // 这里需要等待陀螺仪稳定
}

void inertial_nav_update()
{
    // if (!inertial_nav_enable_flag)
    // {
    //     return;
    // }

    float ax = ROLL_ACC * cosf(ANGLE_TO_RAD(YAW)) - PITCH_ACC * sinf(ANGLE_TO_RAD(YAW));
    float ay = ROLL_ACC * sinf(ANGLE_TO_RAD(YAW)) + PITCH_ACC * cosf(ANGLE_TO_RAD(YAW));
    pos.x_vel += ax * inertial_nav_sample_time;
    pos.y_vel += ay * inertial_nav_sample_time;
    float delta_x = pos.x_vel * inertial_nav_sample_time;
    float delta_y = pos.y_vel * inertial_nav_sample_time;
    // float delta_x = -currentSideAcceleration * inertial_nav_sample_time * inertial_nav_sample_time;
    // float delta_y = -currentFrontAcceleration * inertial_nav_sample_time * inertial_nav_sample_time;
    pos.x += delta_x;
    pos.y += delta_y;
}

PointF inertial_nav_get_pos(void)
{
    PointF p;
    p.x = pos.x;
    p.y = pos.y;
    return p;
}

void inertial_nav_get_vel(float *x_vel, float *y_vel)
{
    if (x_vel != NULL)
    {
        *x_vel = pos.x_vel;
    }
    if (y_vel != NULL)
    {
        *y_vel = pos.y_vel;
    }
}