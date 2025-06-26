#ifndef Mahony_ATTITUDE_H
#define Mahony_ATTITUDE_H

#include "zf_common_headfile.h"
#include "YawIntegral.h"

#define USE_EKF

// front direction
#define PITCH g_euler_angle.pitch
#define PITCH_VEL (g_imu_data.gyro.y / (0.0174533f))
// #define PITCH_VEL g_imu_data.gyro.y
#define PITCH_ACC g_imu_data.acc.x
// side direction
#define ROLL g_euler_angle.roll
#define ROLL_VEL (g_imu_data.gyro.x / (0.0174533f))
// #define ROLL_VEL g_imu_data.gyro.x
#define ROLL_ACC g_imu_data.acc.y
// yaw direction
#define YAW g_euler_angle.yaw
#define YAW_VEL g_imu_data.gyro.z
#define YAW_ACC g_imu_data.acc.z

struct EulerAngle
{
    float roll;
    float pitch;
    float yaw;
};

struct Control_Turn_Manual_Params;
struct Control_Target;
struct Velocity_Motor;

typedef enum {
    ATTITUDE_EKF,
    ATTITUDE_MADGWICK,
    ATTITUDE_MAHONY
} Attitude_algorithm;

void attitude_init(Attitude_algorithm algo);
void attitude_cal(struct IMU_DATA* data);

void attitude_cal_amend(struct Control_Turn_Manual_Params *turn_param,
                        struct Control_Target *control_target,
                        struct Velocity_Motor *velocity_motor,
                        struct EulerAngle *euler_angle,
                        struct IMU_DATA* data);

extern struct EulerAngle g_euler_angle;
extern struct EulerAngle g_euler_angle_bias;
extern uint8 g_attitude_cal_flag;
extern uint8 attitude_time;

#endif
