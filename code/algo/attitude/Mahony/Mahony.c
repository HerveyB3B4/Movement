#include "Mahony.h"
#include <math.h>

//----------------------------------------------------------------------------------------------------
// Definitions

#ifndef M_PI
#define M_PI (3.14159265358979323846f)
#endif

#define DEG_TO_RAD (M_PI / 180.0f)
#define RAD_TO_DEG (180.0f / M_PI)

// Default proportional and integral gains
#define TWO_KP_DEF (2.0f * 0.5f) // 2 * proportional gain
#define TWO_KI_DEF (2.0f * 0.0f) // 2 * integral gain

//----------------------------------------------------------------------------------------------------
// Static variables

//----------------------------------------------------------------------------------------------------
// Helper functions

/**
 * @brief Fast inverse square-root.
 * @param x Input value.
 * @return Inverse square root of x.
 */
static float invSqrt(float x)
{
    // This is the famous fast inverse square root algorithm
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long *)&y;
    i = 0x5f3759df - (i >> 1);
    y = *(float *)&i;
    y = y * (1.5f - (halfx * y * y));
    y = y * (1.5f - (halfx * y * y)); // Second iteration for more accuracy
    return y;
}

/**
 * @brief Computes Euler angles (roll, pitch, yaw) from the quaternion.
 */
static void compute_angles(void)
{
    // Pre-calculate products for efficiency
    float q0q1 = s_MahonyAHRS_info.q0 * s_MahonyAHRS_info.q1;
    float q0q2 = s_MahonyAHRS_info.q0 * s_MahonyAHRS_info.q2;
    float q0q3 = s_MahonyAHRS_info.q0 * s_MahonyAHRS_info.q3;
    float q1q1 = s_MahonyAHRS_info.q1 * s_MahonyAHRS_info.q1;
    float q1q3 = s_MahonyAHRS_info.q1 * s_MahonyAHRS_info.q3;
    float q2q2 = s_MahonyAHRS_info.q2 * s_MahonyAHRS_info.q2;
    float q2q3 = s_MahonyAHRS_info.q2 * s_MahonyAHRS_info.q3;
    float q3q3 = s_MahonyAHRS_info.q3 * s_MahonyAHRS_info.q3;

    // Roll (x-axis rotation)
    s_MahonyAHRS_info.roll = atan2f(2.0f * (q0q1 + q2q3), 1.0f - 2.0f * (q1q1 + q2q2));

    // Pitch (y-axis rotation)
    // Use asinf to handle potential floating point inaccuracies at +/-90 degrees
    float pitch_arg = -2.0f * (q1q3 - q0q2);
    if (pitch_arg > 1.0f)
        pitch_arg = 1.0f;
    if (pitch_arg < -1.0f)
        pitch_arg = -1.0f;
    s_MahonyAHRS_info.pitch = asinf(pitch_arg);

    // Yaw (z-axis rotation)
    s_MahonyAHRS_info.yaw = atan2f(2.0f * (q0q1 + q2q3), 1.0f - 2.0f * (q1q1 + q2q2));

    s_MahonyAHRS_info.anglesComputed = 1;
}

//----------------------------------------------------------------------------------------------------
// Public API functions

/**
 * @brief Initializes the Mahony AHRS algorithm.
 * @param sample_rate The sample frequency in Hz.
 */
void MahonyAHRS_init(float sample_rate)
{
    s_MahonyAHRS_info.twoKp = TWO_KP_DEF;
    s_MahonyAHRS_info.twoKi = TWO_KI_DEF;

    s_MahonyAHRS_info.q0 = 1.0f;
    s_MahonyAHRS_info.q1 = 0.0f;
    s_MahonyAHRS_info.q2 = 0.0f;
    s_MahonyAHRS_info.q3 = 0.0f;

    s_MahonyAHRS_info.integralFBx = 0.0f;
    s_MahonyAHRS_info.integralFBy = 0.0f;
    s_MahonyAHRS_info.integralFBz = 0.0f;

    s_MahonyAHRS_info.invSampleFreq = 1.0f / sample_rate;
    s_MahonyAHRS_info.anglesComputed = 0;
}

/**
 * @brief Calibrates the initial orientation using accelerometer and magnetometer data.
 * This helps to establish a correct initial yaw angle.
 * @param imu_data Pointer to the IMU data structure.
 * @return 0 on success, -1 on failure (e.g., zero-magnitude vectors).
 */
int MahonyAHRS_calibrate(struct IMU_DATA *imu_data)
{
    float ax = imu_data->acc.x;
    float ay = imu_data->acc.y;
    float az = imu_data->acc.z;
    float mx = 0;
    float my = 0;
    float mz = 0;

    // Check for valid accelerometer data
    if ((ax * ax + ay * ay + az * az) == 0.0f)
    {
        return -1;
    }

    // Roll and Pitch from accelerometer
    float init_roll = atan2f(ay, az);
    float init_pitch = atan2f(-ax, sqrtf(ay * ay + az * az));

    // Yaw from magnetometer (tilt-compensated)
    // Check for valid magnetometer data
    float init_yaw;
    if ((mx * mx + my * my + mz * mz) == 0.0f)
    {
        init_yaw = 0.0f; // Cannot determine yaw, default to 0
    }
    else
    {
        float mag_x_comp = mx * cosf(init_pitch) + my * sinf(init_pitch) * sinf(init_roll) + mz * sinf(init_pitch) * cosf(init_roll);
        float mag_y_comp = my * cosf(init_roll) - mz * sinf(init_roll);
        init_yaw = atan2f(-mag_y_comp, mag_x_comp);
    }

    // Convert Euler angles to quaternion
    float cy = cosf(init_yaw * 0.5f);
    float sy = sinf(init_yaw * 0.5f);
    float cp = cosf(init_pitch * 0.5f);
    float sp = sinf(init_pitch * 0.5f);
    float cr = cosf(init_roll * 0.5f);
    float sr = sinf(init_roll * 0.5f);

    s_MahonyAHRS_info.q0 = cr * cp * cy + sr * sp * sy;
    s_MahonyAHRS_info.q1 = sr * cp * cy - cr * sp * sy;
    s_MahonyAHRS_info.q2 = cr * sp * cy + sr * cp * sy;
    s_MahonyAHRS_info.q3 = cr * cp * sy - sr * sp * cy;

    s_MahonyAHRS_info.anglesComputed = 0; // Mark angles as needing re-computation
    return 0;
}

/**
 * @brief Updates the attitude and heading reference system.
 * @param imu_data Pointer to the IMU data structure containing gyro, accel, and (optionally) mag data.
 */
void MahonyAHRS_update(struct IMU_DATA *imu_data)
{
    float gx = imu_data->gyro.x;
    float gy = imu_data->gyro.y;
    float gz = imu_data->gyro.z;
    float ax = imu_data->acc.x;
    float ay = imu_data->acc.y;
    float az = imu_data->acc.z;
    float mx = 0;
    float my = 0;
    float mz = 0;

    float recipNorm;
    float halfvx, halfvy, halfvz;
    float halfex, halfey, halfez;
    float qa, qb, qc;

    // Convert gyroscope degrees/sec to radians/sec
    gx *= DEG_TO_RAD;
    gy *= DEG_TO_RAD;
    gz *= DEG_TO_RAD;

    // Use 6-axis algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
    if ((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f))
    {
        // Normalise accelerometer measurement
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // Estimated direction of gravity
        halfvx = s_MahonyAHRS_info.q1 * s_MahonyAHRS_info.q3 - s_MahonyAHRS_info.q0 * s_MahonyAHRS_info.q2;
        halfvy = s_MahonyAHRS_info.q0 * s_MahonyAHRS_info.q1 + s_MahonyAHRS_info.q2 * s_MahonyAHRS_info.q3;
        halfvz = s_MahonyAHRS_info.q0 * s_MahonyAHRS_info.q0 - 0.5f + s_MahonyAHRS_info.q3 * s_MahonyAHRS_info.q3;

        // Error is sum of cross product between estimated and measured direction of gravity
        halfex = (ay * halfvz - az * halfvy);
        halfey = (az * halfvx - ax * halfvz);
        halfez = (ax * halfvy - ay * halfvx);
    }
    else // Use 9-axis (MARG) algorithm if magnetometer data is valid
    {
        float q0q0 = s_MahonyAHRS_info.q0 * s_MahonyAHRS_info.q0;
        float q0q1 = s_MahonyAHRS_info.q0 * s_MahonyAHRS_info.q1;
        float q0q2 = s_MahonyAHRS_info.q0 * s_MahonyAHRS_info.q2;
        float q0q3 = s_MahonyAHRS_info.q0 * s_MahonyAHRS_info.q3;
        float q1q1 = s_MahonyAHRS_info.q1 * s_MahonyAHRS_info.q1;
        float q1q2 = s_MahonyAHRS_info.q1 * s_MahonyAHRS_info.q2;
        float q1q3 = s_MahonyAHRS_info.q1 * s_MahonyAHRS_info.q3;
        float q2q2 = s_MahonyAHRS_info.q2 * s_MahonyAHRS_info.q2;
        float q2q3 = s_MahonyAHRS_info.q2 * s_MahonyAHRS_info.q3;
        float q3q3 = s_MahonyAHRS_info.q3 * s_MahonyAHRS_info.q3;
        float hx, hy, bx, bz;
        float halfwx, halfwy, halfwz;

        // Normalise accelerometer measurement
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // Normalise magnetometer measurement
        recipNorm = invSqrt(mx * mx + my * my + mz * mz);
        mx *= recipNorm;
        my *= recipNorm;
        mz *= recipNorm;

        // Reference direction of Earth's magnetic field
        hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
        hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
        bx = sqrtf(hx * hx + hy * hy);
        bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));

        // Estimated direction of gravity and magnetic field
        halfvx = s_MahonyAHRS_info.q1 * s_MahonyAHRS_info.q3 - s_MahonyAHRS_info.q0 * s_MahonyAHRS_info.q2;
        halfvy = s_MahonyAHRS_info.q0 * s_MahonyAHRS_info.q1 + s_MahonyAHRS_info.q2 * s_MahonyAHRS_info.q3;
        halfvz = s_MahonyAHRS_info.q0 * s_MahonyAHRS_info.q0 - 0.5f + s_MahonyAHRS_info.q3 * s_MahonyAHRS_info.q3;
        halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
        halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
        halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);
        
        // Error is sum of cross product between estimated and measured direction of fields
        halfex = (ay * halfvz - az * halfvy) + (my * halfwz - mz * halfwy);
        halfey = (az * halfvx - ax * halfvz) + (mz * halfwx - mx * halfwz);
        halfez = (ax * halfvy - ay * halfvx) + (mx * halfwy - my * halfwx);
    }

    // Compute and apply integral feedback if enabled
    if (s_MahonyAHRS_info.twoKi > 0.0f)
    {
        s_MahonyAHRS_info.integralFBx += s_MahonyAHRS_info.twoKi * halfex * s_MahonyAHRS_info.invSampleFreq; // integral error scaled by Ki
        s_MahonyAHRS_info.integralFBy += s_MahonyAHRS_info.twoKi * halfey * s_MahonyAHRS_info.invSampleFreq;
        s_MahonyAHRS_info.integralFBz += s_MahonyAHRS_info.twoKi * halfez * s_MahonyAHRS_info.invSampleFreq;
        gx += s_MahonyAHRS_info.integralFBx; // apply integral feedback
        gy += s_MahonyAHRS_info.integralFBy;
        gz += s_MahonyAHRS_info.integralFBz;
    }
    else
    {
        s_MahonyAHRS_info.integralFBx = 0.0f; // prevent integral windup
        s_MahonyAHRS_info.integralFBy = 0.0f;
        s_MahonyAHRS_info.integralFBz = 0.0f;
    }

    // Apply proportional feedback
    gx += s_MahonyAHRS_info.twoKp * halfex;
    gy += s_MahonyAHRS_info.twoKp * halfey;
    gz += s_MahonyAHRS_info.twoKp * halfez;

    // Integrate rate of change of quaternion
    gx *= (0.5f * s_MahonyAHRS_info.invSampleFreq); // pre-multiply common factors
    gy *= (0.5f * s_MahonyAHRS_info.invSampleFreq);
    gz *= (0.5f * s_MahonyAHRS_info.invSampleFreq);
    qa = s_MahonyAHRS_info.q0;
    qb = s_MahonyAHRS_info.q1;
    qc = s_MahonyAHRS_info.q2;
    s_MahonyAHRS_info.q0 += (-qb * gx - qc * gy - s_MahonyAHRS_info.q3 * gz);
    s_MahonyAHRS_info.q1 += (qa * gx + qc * gz - s_MahonyAHRS_info.q3 * gy);
    s_MahonyAHRS_info.q2 += (qa * gy - qb * gz + s_MahonyAHRS_info.q3 * gx);
    s_MahonyAHRS_info.q3 += (qa * gz + qb * gy - qc * gx);

    // Normalise quaternion
    recipNorm = invSqrt(s_MahonyAHRS_info.q0 * s_MahonyAHRS_info.q0 + s_MahonyAHRS_info.q1 * s_MahonyAHRS_info.q1 + s_MahonyAHRS_info.q2 * s_MahonyAHRS_info.q2 + s_MahonyAHRS_info.q3 * s_MahonyAHRS_info.q3);
    s_MahonyAHRS_info.q0 *= recipNorm;
    s_MahonyAHRS_info.q1 *= recipNorm;
    s_MahonyAHRS_info.q2 *= recipNorm;
    s_MahonyAHRS_info.q3 *= recipNorm;

    // Mark angles as dirty
    s_MahonyAHRS_info.anglesComputed = 0;
}

/**
 * @brief Gets the roll angle in degrees.
 * @return Roll angle in degrees [-180, 180].
 */
float MahonyAHRS_get_roll(void)
{   
    if (!s_MahonyAHRS_info.anglesComputed)
    {
        compute_angles();
    }
    return s_MahonyAHRS_info.roll * RAD_TO_DEG;
}

/**
 * @brief Gets the pitch angle in degrees.
 * @return Pitch angle in degrees [-90, 90].
 */
float MahonyAHRS_get_pitch(void)
{   
    if (!s_MahonyAHRS_info.anglesComputed)
    {
        compute_angles();
    }
    return s_MahonyAHRS_info.pitch * RAD_TO_DEG;
}

/**
 * @brief Gets the yaw angle in degrees.
 * @return Yaw angle in degrees [-180, 180].
 */
float MahonyAHRS_get_yaw(void)
{   
    if (!s_MahonyAHRS_info.anglesComputed)
    {
        compute_angles();
    }
    return s_MahonyAHRS_info.yaw * RAD_TO_DEG;
}