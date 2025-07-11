#include "encoder.h"
#include "single_driver.h"
#include "velocity.h"
#include "zf_common_headfile.h"

void encoder_init()
{
    // bottom encoder
    encoder_dir_init(ENCODER_BOTTOM, BOTTOM_ENCODER_PIN0, BOTTOM_ENCODER_PIN1);
}

void encoder_get_momentum(struct Velocity_Motor *vel_motor)
{
    int32 front_vel = motor_value.receive_left_speed_data;
    int32 back_vel = motor_value.receive_right_speed_data;

    vel_motor->momentum_front = front_vel;
    vel_motor->momentum_back = back_vel;
    vel_motor->momentum_diff = vel_motor->momentum_front + vel_motor->momentum_back;
    vel_motor->momentum_sum = vel_motor->momentum_front - vel_motor->momentum_back;
}

int16 encoder_get_bottom()
{
    int16 cnt = encoder_get_count(ENCODER_BOTTOM);
    encoder_clear_count(ENCODER_BOTTOM);

    return cnt;
}
