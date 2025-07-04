#ifndef _IMU_H_
#define _IMU_H_

#include "zf_common_headfile.h"

#define GravityAcc 9.7936f
#define cheat_define 0.0016f
#define gyroscope_threshold 5
#define DEG2RAD 0.0174533f

typedef struct
{
    float x;
    float y;
    float z;
} Axis3f;

struct IMU_DATA
{
    Axis3f gyro;
    Axis3f acc;
};

void imu_init();
void imu_get_data(struct IMU_DATA *data);

void imu_init_offset();
void imu_remove_offset(struct IMU_DATA *data);

extern struct IMU_DATA g_imu_data;
extern float gyroOffset[3]; // gyroOffset
extern uint32 imu_time;
#endif