#include "YawIntegral.h"
#include "control.h"

static Integral_info s_integral;

void integral_init(float dt)
{
    s_integral.yaw = 0.0f;
    s_integral.dt = dt;
}

void integral_update(IMU_DATA *data)
{

    s_integral.yaw += data->gyro.z * s_integral.dt;

    // 限制积分值的范围，防止溢出或累积过大的误差
    restrictValueF(&s_integral.yaw, 1000.0f, -1000.0f);
}

float integral_get_yaw(void)
{
    return s_integral.yaw;
}

void integral_reset(void)
{
    s_integral.yaw = 0.0f;
}