#include "velocity.h"
#include "attitude.h"
#include "encoder.h"
#include "kalman_filter_velocity.h"

struct Velocity_Motor g_vel_motor;
static kalman_filter_velocity_t s_kf;
static uint8 s_vel_init_flag = 0;

void velocity_init(struct Velocity_Motor *vel_motor)
{
    vel_motor->momentum_front = 0;
    vel_motor->momentum_back = 0;
    vel_motor->bottom = 0;
    vel_motor->bottom_real = 0;
    vel_motor->bottom_filtered = 0;
    vel_motor->momentum_diff = 0;
    kalman_filter_velocity_init(&s_kf);
    s_vel_init_flag = 1;
}

void velocity_update_bottom(struct Velocity_Motor *vel_motor)
{
    vel_motor->bottom = encoder_get_bottom(vel_motor);
    vel_motor->bottom_real = (float)vel_motor->bottom * ENCODER2VELOCITY;

    kalman_filter_velocity_predict(&s_kf);

    float z_1 = (float)vel_motor->bottom_real;          // m/s
    float z_2 = -PITCH_ACC / cosf(ANGLE_TO_RAD(PITCH)); // m/s^2
    float z[MEASUREMENT_SIZE] = {z_1, z_2};
    kalman_filter_velocity_update(&s_kf, z);

    vel_motor->bottom_filtered = s_kf.x[0] * V_KALMAN_MULTIPLE;
}

void velocity_update_side(struct Velocity_Motor *vel_motor)
{
    encoder_get_momentum(vel_motor);
}