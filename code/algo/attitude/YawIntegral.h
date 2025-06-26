#ifndef YAW_INTEGRAL_H
#define YAW_INTEGRAL_H

#include "zf_common_headfile.h"

/**
 * @brief Initializes the yaw integral module.
 * @note This should be called once at startup.
 */
void YawIntegral_Init(void);

/**
 * @brief Updates the yaw integral with the current yaw velocity.
 *
 * @param yaw_velocity The current yaw angular velocity in degrees/second.
 * @param dt The time delta since the last update in seconds.
 */
void YawIntegral_Update(float yaw_velocity, float dt);

/**
 * @brief Gets the current integrated yaw value.
 *
 * @return The integrated yaw value in degrees.
 */
float YawIntegral_GetValue(void);

/**
 * @brief Resets the yaw integral value to zero.
 */
void YawIntegral_Reset(void);

#endif // YAW_INTEGRAL_H