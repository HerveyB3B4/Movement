#ifndef _ATTITUDE_MAHONY_H
#define _ATTITUDE_MAHONY_H

#include "zf_common_headfile.h"
#include "imu.h"

#define twoKpDef (2.0f * 0.5f) // 2 * proportional gain
#define twoKiDef (2.0f * 0.0f) // 2 * integral gain

typedef struct
{
    float twoKi;                                 // 2 * integral gain (Ki)
    float q0, q1, q2, q3;                        // quaternion of sensor frame relative to auxiliary frame
    float integralFBx, integralFBy, integralFBz; // integral error terms scaled by Ki
    float invSampleFreq;
    float roll, pitch, yaw;
} MahonyAHRS_INFO;

struct IMU_DATA;

void MahonyAHRS_calibrate(struct IMU_DATA *imu_data);
void MahonyAHRS_init(float sampleFreq);
void MahonyAHRS_update(struct IMU_DATA *imu_data);
float MahonyAHRS_get_roll(void);
float MahonyAHRS_get_pitch(void);
float MahonyAHRS_get_yaw(void);

extern uint32 mahony_cnt;
#endif