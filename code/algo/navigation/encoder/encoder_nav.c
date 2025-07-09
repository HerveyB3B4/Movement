#include "encoder_nav.h"
#include "Attitude.h"
#include "system.h"
#include "velocity.h"

static encoder_nav_pos pos;
static float encoder_nav_sample_time = 0.01f; // Default sample time

void encoder_nav_init(float sample_time)
{
    pos.x = 0;
    pos.y = 0;
    pos.x_encoder = 0;
    pos.y_encoder = 0;
    encoder_nav_sample_time = sample_time;
}

void encoder_nav_update(struct Velocity_Motor *vel_motor)
{
    // 这里的YAW是机体坐标系下的角度，需要想办法转换成世界坐标系下的角度
    double correct_yaw = 0;
    correct_yaw = (double)YAW + (double)ROLL * sin(ANGLE_TO_RAD((double)PITCH));
    // correct_v = -vel_motor->bottom_real * (ANGLE_TO_RAD(PITCH - imu963raBias.roll))
    if (fabsf(vel_motor->bottom_filtered / V_KALMAN_MULTIPLE) < 0.001f)
    {
        vel_motor->bottom_filtered = 0;
    }
    float delta_x = -vel_motor->bottom_filtered / V_KALMAN_MULTIPLE * encoder_nav_sample_time * cos(ANGLE_TO_RAD(correct_yaw));
    float delta_y = -vel_motor->bottom_filtered / V_KALMAN_MULTIPLE * encoder_nav_sample_time * sin(ANGLE_TO_RAD(correct_yaw));
    float delta_x_encoder = -vel_motor->bottom_real * encoder_nav_sample_time * cos(ANGLE_TO_RAD(correct_yaw));
    float delta_y_encoder = -vel_motor->bottom_real * encoder_nav_sample_time * sin(ANGLE_TO_RAD(correct_yaw));
    // if(-vel_motor->bottom_encoder < 0 && -vel_motor->bottom_encoder > -0.05f){ //encoder is not reliable when it is too small
    //     delta_x_encoder = 0;
    //     delta_y_encoder = 0;
    // }
    if (abs(vel_motor->bottom) < 10 && run_state != CAR_RUNNING)
    {
        delta_x = 0;
        delta_y = 0;
        delta_x_encoder = 0;
        delta_y_encoder = 0;
    }
    else
    {
        pos.space += -vel_motor->bottom_filtered / V_KALMAN_MULTIPLE * encoder_nav_sample_time;
    }
    pos.x += delta_x;
    pos.y += delta_y;
    pos.x_encoder += delta_x_encoder;
    pos.y_encoder += delta_y_encoder;
}