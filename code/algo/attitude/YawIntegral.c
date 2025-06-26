#include "YawIntegral.h"
#include "control.h" // For restrictValueF

/**
 * @brief Structure to hold the state of the yaw integral.
 */
typedef struct
{
    float integralValue; // yaw 角速度积分值 (单位: 度)
} YawIntegral_t;

// Static instance of the yaw integral data, encapsulated within this file.
static YawIntegral_t s_yawIntegral;

/**
 * @brief Initializes the yaw integral module.
 */
void YawIntegral_Init(void)
{
    s_yawIntegral.integralValue = 0.0f;
}

/**
 * @brief Updates the yaw integral with the current yaw velocity.
 */
void YawIntegral_Update(float yaw_velocity, float dt)
{
    s_yawIntegral.integralValue += yaw_velocity * dt;
    // 限制积分值的范围，防止溢出或累积过大的误差
    // The range is arbitrary, can be adjusted if needed.
    restrictValueF(&s_yawIntegral.integralValue, 1000.0f, -1000.0f);
}

/**
 * @brief Gets the current integrated yaw value.
 */
float YawIntegral_GetValue(void)
{
    return s_yawIntegral.integralValue;
}

/**
 * @brief Resets the yaw integral value to zero.
 */
void YawIntegral_Reset(void)
{
    s_yawIntegral.integralValue = 0.0f;
}