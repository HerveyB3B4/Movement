#ifndef _ATTITUDE_MAHONY_H
#define _ATTITUDE_MAHONY_H

#include "zf_common_headfile.h"

typedef struct
{
    // Filter gains
    float twoKp;         // 2 * proportional gain (Kp)
    float twoKi;         // 2 * integral gain (Ki)
    float invSampleFreq; // Inverse of sample frequency

    // Quaternion of sensor frame relative to auxiliary frame
    float q0, q1, q2, q3;

    // Integral error terms scaled by Ki
    float integralFBx, integralFBy, integralFBz;

    // Calculated Euler angles
    float roll, pitch, yaw;

    // Flag to indicate if Euler angles have been computed from the latest quaternion
    volatile char anglesComputed;
} MahonyAHRS_info_t;

static MahonyAHRS_info_t s_MahonyAHRS_info;

void MahonyAHRS_init(float sample_rate);
int MahonyAHRS_calibrate(struct IMU_DATA *imu_data);
void MahonyAHRS_update(struct IMU_DATA *imu_data);

float MahonyAHRS_get_pitch(void);
float MahonyAHRS_get_roll(void);
float MahonyAHRS_get_yaw(void);

#endif // _ATTITUDE_MAHONY_H