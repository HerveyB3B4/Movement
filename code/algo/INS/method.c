#include "method.h"
#include <math.h>
#include <string.h>
#include "zf_common_headfile.h"

/* 定义常量 */
#define PI 3.14159265358979323846f
#define DEG_TO_RAD (PI / 180.0f)
#define RAD_TO_DEG (180.0f / PI)
#define GRAVITY 9.80665f

/* 矩阵运算辅助宏 */
#define MAT_ELEM_3X3(mat, row, col) ((mat)[(row) * 3 + (col)])
#define MAT_ELEM_6X6(mat, row, col) ((mat)[(row) * 6 + (col)])

/* 全局导航系统状态 */
INS_State g_ins_state;

/* 内部辅助函数声明 */
static void Vector3f_Add(const Vector3f* a,
                         const Vector3f* b,
                         Vector3f* result);
static void Vector3f_Subtract(const Vector3f* a,
                              const Vector3f* b,
                              Vector3f* result);
static void Vector3f_Multiply(const Vector3f* v,
                              float scalar,
                              Vector3f* result);
static void Vector3f_ScaleAdd(const Vector3f* v1,
                              float scale1,
                              const Vector3f* v2,
                              float scale2,
                              Vector3f* result);
static float Vector3f_Magnitude(const Vector3f* v);
static void Matrix_Multiply3x3(const float* A, const float* B, float* C);

void INS_Init(INS_State* ins,
              float sample_time,
              INS_Mode mode,
              ZUPT_Method zupt_method) {
    /* 清零所有数据 */
    memset(ins, 0, sizeof(INS_State));

    /* 设置采样时间 */
    ins->dt = sample_time;

    /* 设置加速度阈值 */
    ins->accel_threshold = 0.05f * GRAVITY;  // 静止状态默认为0.05g阈值

    /* 设置导航模式和ZUPT方法 */
    ins->mode = mode;
    ins->zupt_method = zupt_method;

    /* 初始化比例因子为单位增益 */
    ins->accel_scale.x = ins->accel_scale.y = ins->accel_scale.z = 1.0f;

    /* 如果使用卡尔曼滤波模式，初始化滤波器 */
    if (mode == INS_MODE_KALMAN) {
        INS_InitKalmanFilter(ins);
    }

    ins->is_initialized = 0;  // 标记为未初始化，需要校准
}

void INS_UpdateSensorData(INS_State* ins,
                          const Vector3f* accel_raw,
                          const Vector3f* gyro) {
    /* 保存原始数据 */
    ins->accel_raw = *accel_raw;

    /* 如果提供了陀螺仪数据，保存它用于高级ZUPT */
    if (gyro != NULL) {
        ins->gyro = *gyro;
    }

    /* 应用校准 (零偏和比例因子) */
    ins->accel.x = (accel_raw->x - ins->accel_bias.x) * ins->accel_scale.x;
    ins->accel.y = (accel_raw->y - ins->accel_bias.y) * ins->accel_scale.y;
    ins->accel.z = (accel_raw->z - ins->accel_bias.z) * ins->accel_scale.z;
}

void INS_ConvertAccelToWorldFrame(INS_State* ins,
                                  const struct EulerAngle* euler_angle) {
    /* 保存上一次的世界坐标系加速度，用于高阶积分 */
    ins->accel_world_last = ins->accel_world;

    /* 从欧拉角计算旋转矩阵 */
    float roll = euler_angle->roll * DEG_TO_RAD;    // 横滚角转弧度
    float pitch = euler_angle->pitch * DEG_TO_RAD;  // 俯仰角转弧度
    float yaw = euler_angle->yaw * DEG_TO_RAD;      // 偏航角转弧度

    float cos_roll = cosf(roll);
    float sin_roll = sinf(roll);
    float cos_pitch = cosf(pitch);
    float sin_pitch = sinf(pitch);
    float cos_yaw = cosf(yaw);
    float sin_yaw = sinf(yaw);

    /* 旋转矩阵 - 从体坐标系转换到世界坐标系的旋转 */
    /*
     * R = Rz(yaw) * Ry(pitch) * Rx(roll)
     */
    float R11 = cos_yaw * cos_pitch;
    float R12 = cos_yaw * sin_pitch * sin_roll - sin_yaw * cos_roll;
    float R13 = cos_yaw * sin_pitch * cos_roll + sin_yaw * sin_roll;

    float R21 = sin_yaw * cos_pitch;
    float R22 = sin_yaw * sin_pitch * sin_roll + cos_yaw * cos_roll;
    float R23 = sin_yaw * sin_pitch * cos_roll - cos_yaw * sin_roll;

    float R31 = -sin_pitch;
    float R32 = cos_pitch * sin_roll;
    float R33 = cos_pitch * cos_roll;

    /* 转换加速度从体坐标系到世界坐标系 */
    ins->accel_world.x =
        R11 * ins->accel.x + R12 * ins->accel.y + R13 * ins->accel.z;
    ins->accel_world.y =
        R21 * ins->accel.x + R22 * ins->accel.y + R23 * ins->accel.z;
    ins->accel_world.z =
        R31 * ins->accel.x + R32 * ins->accel.y + R33 * ins->accel.z;

    /* 移除重力分量 */
    ins->accel_world.z -= GRAVITY;
}

void INS_UpdatePositionVelocityEuler(INS_State* ins) {
    if (!ins->is_initialized)
        return;

    /* 保存上一次状态 */
    ins->position_k1 = ins->position_last;  // 上上次位置
    ins->position_last = ins->position;     // 上次位置
    ins->velocity_last = ins->velocity;     // 上次速度

    /* 简单的一阶欧拉积分 */
    /* 速度更新 (一阶积分) */
    ins->velocity.x += ins->accel_world.x * ins->dt;
    ins->velocity.y += ins->accel_world.y * ins->dt;
    ins->velocity.z += ins->accel_world.z * ins->dt;

    /* 位置更新 (一阶积分) */
    ins->position.x += ins->velocity.x * ins->dt;
    ins->position.y += ins->velocity.y * ins->dt;
    ins->position.z += ins->velocity.z * ins->dt;
}

void INS_UpdatePositionVelocityRK4(INS_State* ins,
                                   const struct EulerAngle* euler_angle) {
    if (!ins->is_initialized)
        return;

    float dt = ins->dt;
    float dt_2 = dt * 0.5f;

    /* 保存上一次状态 */
    ins->position_k1 = ins->position_last;
    ins->position_last = ins->position;
    ins->velocity_last = ins->velocity;

    /* 第一阶段 - 使用当前状态和加速度 */
    Vector3f temp_velocity, temp_position;

    /* k1 = a(t) - 当前加速度 */
    ins->k1 = ins->accel_world;

    /* l1 = v(t) - 当前速度 */
    ins->l1 = ins->velocity;

    /* 第二阶段 - 使用k1计算中间速度，并更新加速度 */
    /* v(t + dt/2) = v(t) + k1*dt/2 */
    Vector3f_ScaleAdd(&ins->velocity, 1.0f, &ins->k1, dt_2, &temp_velocity);

    /* x(t + dt/2) = x(t) + l1*dt/2 */
    Vector3f_ScaleAdd(&ins->position, 1.0f, &ins->l1, dt_2, &temp_position);

    /* 需要计算t+dt/2时刻的加速度，这需要姿态数据 */
    /* 简化处理：使用当前加速度和上一时刻加速度的平均值作为中间点加速度 */
    Vector3f accel_mid;
    Vector3f_ScaleAdd(&ins->accel_world, 0.5f, &ins->accel_world_last, 0.5f,
                      &accel_mid);

    /* k2 = a(t + dt/2) */
    ins->k2 = accel_mid;

    /* l2 = v(t + dt/2) */
    ins->l2 = temp_velocity;

    /* 第三阶段 */
    /* v(t + dt/2) = v(t) + k2*dt/2 */
    Vector3f_ScaleAdd(&ins->velocity, 1.0f, &ins->k2, dt_2, &temp_velocity);

    /* x(t + dt/2) = x(t) + l2*dt/2 */
    Vector3f_ScaleAdd(&ins->position, 1.0f, &ins->l2, dt_2, &temp_position);

    /* k3 = a(t + dt/2) - 再次使用中间点加速度 */
    ins->k3 = accel_mid;

    /* l3 = v(t + dt/2) - 更新的中间点速度 */
    ins->l3 = temp_velocity;

    /* 第四阶段 */
    /* v(t + dt) = v(t) + k3*dt */
    Vector3f_ScaleAdd(&ins->velocity, 1.0f, &ins->k3, dt, &temp_velocity);

    /* x(t + dt) = x(t) + l3*dt */
    Vector3f_ScaleAdd(&ins->position, 1.0f, &ins->l3, dt, &temp_position);

    /* k4 = a(t + dt) - 估计下一时刻加速度 (简化：假设加速度变化率保持不变) */
    Vector3f accel_next;
    float accel_delta_x = ins->accel_world.x - ins->accel_world_last.x;
    float accel_delta_y = ins->accel_world.y - ins->accel_world_last.y;
    float accel_delta_z = ins->accel_world.z - ins->accel_world_last.z;

    accel_next.x = ins->accel_world.x + accel_delta_x;
    accel_next.y = ins->accel_world.y + accel_delta_y;
    accel_next.z = ins->accel_world.z + accel_delta_z;

    ins->k4 = accel_next;

    /* l4 = v(t + dt) - 估计下一时刻速度 */
    ins->l4 = temp_velocity;

    /* 计算最终结果 - RK4公式 */
    /* v(t+dt) = v(t) + (dt/6)*(k1 + 2*k2 + 2*k3 + k4) */
    ins->velocity.x =
        ins->velocity_last.x + (dt / 6.0f) * (ins->k1.x + 2.0f * ins->k2.x +
                                              2.0f * ins->k3.x + ins->k4.x);
    ins->velocity.y =
        ins->velocity_last.y + (dt / 6.0f) * (ins->k1.y + 2.0f * ins->k2.y +
                                              2.0f * ins->k3.y + ins->k4.y);
    ins->velocity.z =
        ins->velocity_last.z + (dt / 6.0f) * (ins->k1.z + 2.0f * ins->k2.z +
                                              2.0f * ins->k3.z + ins->k4.z);

    /* x(t+dt) = x(t) + (dt/6)*(l1 + 2*l2 + 2*l3 + l4) */
    ins->position.x =
        ins->position_last.x + (dt / 6.0f) * (ins->l1.x + 2.0f * ins->l2.x +
                                              2.0f * ins->l3.x + ins->l4.x);
    ins->position.y =
        ins->position_last.y + (dt / 6.0f) * (ins->l1.y + 2.0f * ins->l2.y +
                                              2.0f * ins->l3.y + ins->l4.y);
    ins->position.z =
        ins->position_last.z + (dt / 6.0f) * (ins->l1.z + 2.0f * ins->l2.z +
                                              2.0f * ins->l3.z + ins->l4.z);
}

void INS_InitKalmanFilter(INS_State* ins) {
    int i, j;

    /* 初始化状态协方差矩阵 P */
    for (i = 0; i < 6; i++) {
        for (j = 0; j < 6; j++) {
            ins->kf.P[i][j] = 0.0f;
        }
        /* 对角线元素设置初始不确定性 */
        ins->kf.P[i][i] = (i < 3) ? 0.01f : 1.0f;  // 位置和速度不确定性
    }

    /* 设置过程噪声 Q */
    for (i = 0; i < 3; i++) {
        /* 位置噪声 */
        ins->kf.Q[i] = 0.001f;
        /* 速度噪声 */
        ins->kf.Q[i + 3] = 0.01f;
    }

    /* 设置测量噪声 R */
    for (i = 0; i < 3; i++) {
        ins->kf.R[i] = 0.1f;  // 加速度测量噪声
    }
}

void INS_UpdatePositionVelocityKalman(INS_State* ins) {
    if (!ins->is_initialized)
        return;

    int i, j, k;
    float dt = ins->dt;
    float dt2_2 = dt * dt * 0.5f;

    /* 保存上一次状态 */
    ins->position_last = ins->position;
    ins->velocity_last = ins->velocity;

    /* 步骤1: 预测状态 - 使用物理模型 */
    /* X_k = F * X_{k-1} + B * u_{k-1} */
    /* 位置更新 = 上一时刻位置 + 速度*dt + 0.5*加速度*dt^2 */
    ins->position.x += ins->velocity.x * dt + ins->accel_world.x * dt2_2;
    ins->position.y += ins->velocity.y * dt + ins->accel_world.y * dt2_2;
    ins->position.z += ins->velocity.z * dt + ins->accel_world.z * dt2_2;

    /* 速度更新 = 上一时刻速度 + 加速度*dt */
    ins->velocity.x += ins->accel_world.x * dt;
    ins->velocity.y += ins->accel_world.y * dt;
    ins->velocity.z += ins->accel_world.z * dt;

    /* 步骤2: 预测协方差 */
    /* P_k = F * P_{k-1} * F^T + Q */
    /* 构建状态转移矩阵 F */
    float F[36] = {0};  // 6x6矩阵，线性化

    /* 对角线元素为1 */
    for (i = 0; i < 6; i++) {
        F[i * 6 + i] = 1.0f;
    }

    /* 位置关于速度的偏导数 */
    F[0 * 6 + 3] = dt;  // x关于vx
    F[1 * 6 + 4] = dt;  // y关于vy
    F[2 * 6 + 5] = dt;  // z关于vz

    /* 计算 F * P * F^T + Q */
    float temp[36] = {0};   // 临时矩阵
    float temp2[36] = {0};  // 临时矩阵2

    /* temp = F * P */
    for (i = 0; i < 6; i++) {
        for (j = 0; j < 6; j++) {
            temp[i * 6 + j] = 0;
            for (k = 0; k < 6; k++) {
                temp[i * 6 + j] += F[i * 6 + k] * ins->kf.P[k][j];
            }
        }
    }

    /* temp2 = temp * F^T = F * P * F^T */
    for (i = 0; i < 6; i++) {
        for (j = 0; j < 6; j++) {
            temp2[i * 6 + j] = 0;
            for (k = 0; k < 6; k++) {
                temp2[i * 6 + j] +=
                    temp[i * 6 + k] * F[j * 6 + k];  // F^T[k,j] = F[j,k]
            }
        }
    }

    /* P = temp2 + Q */
    for (i = 0; i < 6; i++) {
        for (j = 0; j < 6; j++) {
            ins->kf.P[i][j] = temp2[i * 6 + j];
        }
        /* 添加过程噪声 (只在对角线) */
        ins->kf.P[i][i] += ins->kf.Q[i];
    }

    /* 步骤3: 如果有外部测量，计算卡尔曼增益并更新状态 */
    /* 这里简化处理，我们不使用外部测量，但有ZUPT */
    /* 如果需要融合GPS或其他位置传感器，可以在此处添加代码 */
}

void INS_UpdateMotionIntensity(INS_State* ins) {
    /* 计算加速度和角速度的强度 */
    float accel_magnitude = Vector3f_Magnitude(&ins->accel);
    float gyro_magnitude = Vector3f_Magnitude(&ins->gyro);

    /* 计算运动强度指标 - 结合加速度和角速度 */
    float accel_intensity = fabsf(accel_magnitude - GRAVITY) / GRAVITY;
    float gyro_intensity = gyro_magnitude * RAD_TO_DEG * 0.01f;  // 归一化处理

    /* 结合加速度和角速度计算综合运动强度 */
    ins->motion_intensity = 0.7f * accel_intensity + 0.3f * gyro_intensity;

    /* 确定是否处于高动态运动状态 */
    ins->high_dynamic = (ins->motion_intensity > 0.5f) ? 1 : 0;
}

void INS_Calibrate(INS_State* ins, uint16_t samples) {
    if (samples == 0)
        return;

    /* 临时存储累加值 */
    Vector3f accel_sum = {0.0f, 0.0f, 0.0f};

    /* 保存当前零偏 */
    Vector3f accel_bias_old = ins->accel_bias;

    /* 重置零偏 */
    ins->accel_bias.x = ins->accel_bias.y = ins->accel_bias.z = 0.0f;

    /* 累加多个采样 */
    for (uint16_t i = 0; i < samples; i++) {
        accel_sum.x += ins->accel_raw.x;
        accel_sum.y += ins->accel_raw.y;
        accel_sum.z += ins->accel_raw.z;

        /* 等待下一个采样（系统应提供延迟） */
        /* 注意：实际应用中应当在调用者端确保有延迟和采样 */
    }

    /* 计算平均值作为零偏 */
    ins->accel_bias.x = accel_sum.x / samples;
    ins->accel_bias.y = accel_sum.y / samples;
    ins->accel_bias.z = accel_sum.z / samples - GRAVITY;  // 去除重力加速度

    /* 可以考虑与先前零偏进行融合 */
    float alpha = 0.3f;  // 新数据的权重
    ins->accel_bias.x =
        alpha * ins->accel_bias.x + (1.0f - alpha) * accel_bias_old.x;
    ins->accel_bias.y =
        alpha * ins->accel_bias.y + (1.0f - alpha) * accel_bias_old.y;
    ins->accel_bias.z =
        alpha * ins->accel_bias.z + (1.0f - alpha) * accel_bias_old.z;

    /* 标记系统为已初始化 */
    ins->is_initialized = 1;
}

void INS_CalibrateAdvanced(INS_State* ins,
                           uint8_t num_positions,
                           uint16_t samples_per_position) {
    if (num_positions < 4 || samples_per_position == 0)
        return;

    /* 高级校准需要至少4个不同位置的数据：
     * 1. 水平放置（Z轴向上）
     * 2. 倒置放置（Z轴向下）
     * 3. X轴向上放置
     * 4. Y轴向上放置
     */

    /* 储存每个位置的平均加速度值 */
    Vector3f positions[6];  // 最多支持6个位置
    uint8_t actual_positions = (num_positions > 6) ? 6 : num_positions;

    for (uint8_t pos = 0; pos < actual_positions; pos++) {
        /* 临时存储累加值 */
        Vector3f accel_sum = {0.0f, 0.0f, 0.0f};

        /* 累加该位置的多个采样 */
        for (uint16_t i = 0; i < samples_per_position; i++) {
            accel_sum.x += ins->accel_raw.x;
            accel_sum.y += ins->accel_raw.y;
            accel_sum.z += ins->accel_raw.z;

            /* 等待下一个采样（系统应提供延迟） */
            /* 注意：实际应用中应当在调用者端确保有延迟和采样 */
        }

        /* 计算该位置的平均值 */
        positions[pos].x = accel_sum.x / samples_per_position;
        positions[pos].y = accel_sum.y / samples_per_position;
        positions[pos].z = accel_sum.z / samples_per_position;

        /* 等待切换到下一个位置（系统应提供延迟） */
    }

    /* 使用多个位置数据计算加速度计零偏和比例因子 */
    if (actual_positions >= 4) {
        /* 假设前两个位置是Z轴上下，后两个位置是X和Y轴 */

        /* 计算零偏 - 取相反方向的平均值 */
        ins->accel_bias.x = (positions[0].x + positions[1].x) * 0.5f;
        ins->accel_bias.y = (positions[0].y + positions[1].y) * 0.5f;
        ins->accel_bias.z = (positions[0].z + positions[1].z) * 0.5f;

        /* 计算比例因子 */
        float z_range = fabsf(positions[0].z - positions[1].z);
        float x_range = fabsf(positions[2].x - positions[0].x);
        float y_range = fabsf(positions[3].y - positions[0].y);

        /* 理论上，从正到负应该是2g的范围 */
        float expected_range = 2.0f * GRAVITY;

        ins->accel_scale.x = (x_range > 0.1f) ? expected_range / x_range : 1.0f;
        ins->accel_scale.y = (y_range > 0.1f) ? expected_range / y_range : 1.0f;
        ins->accel_scale.z = (z_range > 0.1f) ? expected_range / z_range : 1.0f;

        /* 限制比例因子在合理范围内 */
        const float min_scale = 0.8f;
        const float max_scale = 1.2f;

        ins->accel_scale.x = (ins->accel_scale.x < min_scale) ? min_scale
                             : (ins->accel_scale.x > max_scale)
                                 ? max_scale
                                 : ins->accel_scale.x;
        ins->accel_scale.y = (ins->accel_scale.y < min_scale) ? min_scale
                             : (ins->accel_scale.y > max_scale)
                                 ? max_scale
                                 : ins->accel_scale.y;
        ins->accel_scale.z = (ins->accel_scale.z < min_scale) ? min_scale
                             : (ins->accel_scale.z > max_scale)
                                 ? max_scale
                                 : ins->accel_scale.z;
    } else {
        /* 如果位置不足，则使用基本校准方法 */
        INS_Calibrate(ins, samples_per_position);
        return;
    }

    /* 标记系统为已初始化 */
    ins->is_initialized = 1;
}

void INS_Reset(INS_State* ins) {
    /* 保留校准参数和采样时间 */
    float dt = ins->dt;
    Vector3f accel_bias = ins->accel_bias;
    Vector3f accel_scale = ins->accel_scale;
    uint8_t is_initialized = ins->is_initialized;
    float accel_threshold = ins->accel_threshold;
    INS_Mode mode = ins->mode;
    ZUPT_Method zupt_method = ins->zupt_method;
    KalmanFilter kf = ins->kf;

    /* 清零所有状态 */
    memset(ins, 0, sizeof(INS_State));

    /* 恢复保存的参数 */
    ins->dt = dt;
    ins->accel_bias = accel_bias;
    ins->accel_scale = accel_scale;
    ins->is_initialized = is_initialized;
    ins->accel_threshold = accel_threshold;
    ins->mode = mode;
    ins->zupt_method = zupt_method;
    ins->kf = kf;
}

void INS_Update(INS_State* ins, const struct EulerAngle* euler_angle) {
    if (!ins->is_initialized)
        return;

    /* 将加速度从体坐标系转换到世界坐标系 */
    INS_ConvertAccelToWorldFrame(ins, euler_angle);

    /* 更新运动强度指标 (用于自适应模式) */
    if (ins->mode == INS_MODE_ADAPTIVE) {
        INS_UpdateMotionIntensity(ins);
    }

    /* 根据当前模式选择合适的更新方法 */
    switch (ins->mode) {
        case INS_MODE_BASIC:
            INS_UpdatePositionVelocityEuler(ins);
            break;

        case INS_MODE_RUNGE_KUTTA:
            INS_UpdatePositionVelocityRK4(ins, euler_angle);
            break;

        case INS_MODE_KALMAN:
            INS_UpdatePositionVelocityKalman(ins);
            break;

        case INS_MODE_ADAPTIVE:
            /* 根据运动强度选择合适的算法 */
            if (ins->high_dynamic) {
                /* 高动态运动情况下使用龙格库塔 */
                INS_UpdatePositionVelocityRK4(ins, euler_angle);
            } else {
                /* 低动态运动情况下使用卡尔曼滤波 */
                INS_UpdatePositionVelocityKalman(ins);
            }
            break;

        default:
            INS_UpdatePositionVelocityEuler(ins);
            break;
    }

    /* 进行零速度检测和更新 */
    uint8_t is_stationary = 0;

    switch (ins->zupt_method) {
        case ZUPT_BASIC:
            /* 基本阈值检测 */
            {
                float accel_magnitude = Vector3f_Magnitude(&ins->accel);
                is_stationary =
                    (fabsf(accel_magnitude - GRAVITY) < ins->accel_threshold)
                        ? 1
                        : 0;
            }
            break;

        case ZUPT_ADVANCED:
            /* 高级检测 - 同时考虑加速度和角速度 */
            is_stationary = INS_ZeroVelocityDetectionAdvanced(ins);
            break;

        case ZUPT_ADAPTIVE:
            /* 自适应检测 - 根据运动强度动态调整阈值 */
            {
                float adaptive_threshold = ins->accel_threshold;

                /* 根据运动强度调整阈值 */
                if (ins->motion_intensity > 0.3f) {
                    adaptive_threshold *= (1.0f + ins->motion_intensity);
                }

                float accel_magnitude = Vector3f_Magnitude(&ins->accel);
                float gyro_magnitude = Vector3f_Magnitude(&ins->gyro);

                /* 结合加速度和角速度判断静止状态 */
                is_stationary =
                    (fabsf(accel_magnitude - GRAVITY) < adaptive_threshold &&
                     gyro_magnitude < 0.05f)
                        ? 1
                        : 0;
            }
            break;

        default: {
            float accel_magnitude = Vector3f_Magnitude(&ins->accel);
            is_stationary =
                (fabsf(accel_magnitude - GRAVITY) < ins->accel_threshold) ? 1
                                                                          : 0;
        } break;
    }

    /* 应用零速度更新 */
    INS_ZeroVelocityUpdate(ins, is_stationary);
}

uint8_t INS_ZeroVelocityDetectionAdvanced(INS_State* ins) {
    /* 计算加速度和角速度幅值 */
    float accel_magnitude = Vector3f_Magnitude(&ins->accel);
    float gyro_magnitude = Vector3f_Magnitude(&ins->gyro);

    /* 加速度检测：近似为1g */
    uint8_t accel_stationary =
        (fabsf(accel_magnitude - GRAVITY) < ins->accel_threshold) ? 1 : 0;

    /* 角速度检测：接近零 */
    uint8_t gyro_stationary =
        (gyro_magnitude < 0.05f) ? 1 : 0;  // 阈值为0.05弧度/秒

    /* 同时满足加速度和角速度条件才判断为静止 */
    return accel_stationary && gyro_stationary;
}

void INS_ZeroVelocityUpdate(INS_State* ins, uint8_t is_stationary) {
    if (!is_stationary) {
        ins->zero_velocity = 0;
        return;
    }

    /* 零速度更新：如果检测到静止状态，缓慢衰减速度以减轻漂移影响 */
    float damping_factor = 0.95f;

    if (ins->zero_velocity < 100) {
        ins->zero_velocity++;
    }

    /* 速度积累更多的零速度检测样本，阻尼系数增大 */
    if (ins->zero_velocity > 20) {
        damping_factor = 0.8f;
    }

    if (ins->zero_velocity > 50) {
        damping_factor = 0.5f;
    }

    /* 应用阻尼衰减速度 */
    ins->velocity.x *= damping_factor;
    ins->velocity.y *= damping_factor;
    ins->velocity.z *= damping_factor;

    /* 完全静止状态下直接清零速度 */
    if (ins->zero_velocity > 75) {
        ins->velocity.x = 0.0f;
        ins->velocity.y = 0.0f;
        ins->velocity.z = 0.0f;
    }
}

void INS_SetMode(INS_State* ins, INS_Mode mode) {
    if (ins->mode != mode) {
        ins->mode = mode;

        /* 如果切换到卡尔曼滤波模式，确保初始化滤波器 */
        if (mode == INS_MODE_KALMAN && ins->kf.P[0][0] == 0.0f) {
            INS_InitKalmanFilter(ins);
        }
    }
}

void INS_SetZuptMethod(INS_State* ins, ZUPT_Method method) {
    ins->zupt_method = method;
}

/* 向量操作辅助函数 */
static void Vector3f_Add(const Vector3f* a,
                         const Vector3f* b,
                         Vector3f* result) {
    result->x = a->x + b->x;
    result->y = a->y + b->y;
    result->z = a->z + b->z;
}

static void Vector3f_Subtract(const Vector3f* a,
                              const Vector3f* b,
                              Vector3f* result) {
    result->x = a->x - b->x;
    result->y = a->y - b->y;
    result->z = a->z - b->z;
}

static void Vector3f_Multiply(const Vector3f* v,
                              float scalar,
                              Vector3f* result) {
    result->x = v->x * scalar;
    result->y = v->y * scalar;
    result->z = v->z * scalar;
}

static void Vector3f_ScaleAdd(const Vector3f* v1,
                              float scale1,
                              const Vector3f* v2,
                              float scale2,
                              Vector3f* result) {
    result->x = v1->x * scale1 + v2->x * scale2;
    result->y = v1->y * scale1 + v2->y * scale2;
    result->z = v1->z * scale1 + v2->z * scale2;
}

static float Vector3f_Magnitude(const Vector3f* v) {
    return sqrtf(v->x * v->x + v->y * v->y + v->z * v->z);
}

static void Matrix_Multiply3x3(const float* A, const float* B, float* C) {
    int i, j, k;
    float temp[9] = {0};

    for (i = 0; i < 3; i++) {
        for (j = 0; j < 3; j++) {
            temp[i * 3 + j] = 0;
            for (k = 0; k < 3; k++) {
                temp[i * 3 + j] += A[i * 3 + k] * B[k * 3 + j];
            }
        }
    }

    /* 复制结果 */
    for (i = 0; i < 9; i++) {
        C[i] = temp[i];
    }
}