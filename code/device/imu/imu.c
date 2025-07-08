#include "imu.h"

struct IMU_DATA g_imu_data;
float gyroOffset[3] = {0.0f, 0.0f, 0.0f};
uint32 imu_time;

void imu_init()
{
    bool tmp_init_flag = imu660rb_init();
    if (tmp_init_flag == 1)
    {
        printf("imu660rb init Failed\n");
        return;
    }
}

void imu_get_data(struct IMU_DATA *data)
{
    imu660rb_get_acc();
    imu660rb_get_gyro();

    data->gyro.x = imu660rb_gyro_transition(imu660rb_gyro_y) * DEG2RAD;
    data->gyro.y = -imu660rb_gyro_transition(imu660rb_gyro_x) * DEG2RAD;
    data->gyro.z = imu660rb_gyro_transition(imu660rb_gyro_z) * DEG2RAD;

    // data->gyro.x = imu660rb_gyro_transition(imu660rb_gyro_x);
    // data->gyro.y = -imu660rb_gyro_transition(imu660rb_gyro_y);
    // data->gyro.z = imu660rb_gyro_transition(imu660rb_gyro_z);

    // data->acc.x = imu660rb_acc_transition(imu660rb_acc_x) * GravityAcc;
    // data->acc.y = -imu660rb_acc_transition(imu660rb_acc_y) * GravityAcc;
    // data->acc.z = imu660rb_acc_transition(imu660rb_acc_z) * GravityAcc;

    data->acc.x = imu660rb_acc_transition(imu660rb_acc_y) * GravityAcc;
    data->acc.y = -imu660rb_acc_transition(imu660rb_acc_x) * GravityAcc;
    data->acc.z = imu660rb_acc_transition(imu660rb_acc_z) * GravityAcc;

    // data->gyro.x = imu660rb_gyro_transition(imu660rb_gyro_x);
    // data->gyro.y = imu660rb_gyro_transition(imu660rb_gyro_z);
    // data->gyro.z = -imu660rb_gyro_transition(imu660rb_gyro_y);

    // data->acc.x = imu660rb_acc_transition(imu660rb_acc_x) * GravityAcc;
    // data->acc.y = imu660rb_acc_transition(imu660rb_acc_z) * GravityAcc;
    // data->acc.z = -imu660rb_acc_transition(imu660rb_acc_y) * GravityAcc;

    imu_time = system_getval();
}

void imu_init_offset()
{
    struct IMU_DATA data;
    for (int i = 0; i < 2000; i++)
    {
        imu_get_data(&data);
        if (fabsf(data.gyro.x) + fabsf(data.gyro.y) + fabsf(data.gyro.z) >
            gyroscope_threshold)
        {
            i--;
            continue;
        }
        gyroOffset[0] += data.gyro.x;
        gyroOffset[1] += data.gyro.y;
        gyroOffset[2] += data.gyro.z;
        system_delay_ms(1);
    }
    gyroOffset[0] *= 0.0005f;
    gyroOffset[1] *= 0.0005f;
    gyroOffset[2] *= 0.0005f;
}

void imu_remove_offset(struct IMU_DATA *data)
{
    data->gyro.x -= gyroOffset[0];
    data->gyro.y -= gyroOffset[1];
    data->gyro.z -= gyroOffset[2];
}
