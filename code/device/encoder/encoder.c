#include "encoder.h"
#include "single_driver.h"
#include "velocity.h"
#include "zf_common_headfile.h"

void encoder_init()
{
    // bottom encoder
    encoder_dir_init(ENCODER_BOTTOM, ENCODER_PIN0_BOTTOM, ENCODER_PIN1_BOTTOM);
}

void encoder_get_momentum(struct Velocity_Motor *vel_motor)
{
    int32 front_vel = motor_value.receive_left_speed_data;
    int32 back_vel = motor_value.receive_right_speed_data;

    vel_motor->momentumFront = front_vel;
    vel_motor->momentumBack = back_vel;
    vel_motor->velocityDiff =
        vel_motor->momentumFront + vel_motor->momentumBack;
}

int16 encoder_get_bottom()
{
    int16 cnt = encoder_get_count(ENCODER_BOTTOM);
    encoder_clear_count(ENCODER_BOTTOM);

    return cnt;
}
