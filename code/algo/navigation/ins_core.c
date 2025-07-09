#include "ins_core.h"
#include "math.h"

ins_state_t g_ins_state;

void ins_init(ins_state_t *state)
{
    state->pos_d = 0.0f;
    state->pos_e = 0.0f;
    state->pos_n = 0.0f;
    state->vel_d = 0.0f;
    state->vel_e = 0.0f;
    state->vel_n = 0.0f;
}

void ins_update(ins_state_t *state, float acc_x, float acc_y, float acc_z,
                float roll_rad, float pitch_rad, float yaw_rad, float dt)
{
    // 检查时间间隔的有效性
    if (dt <= 0.0f)
    {
        return;
    }

    // --- 1. 根据欧拉角计算从机体坐标系(body)到导航坐标系(NED)的旋转矩阵 C_b^n ---
    // 这是惯性导航算法的核心步骤，用于将加速度计的读数从载体方向转换到地理方向。
    // 使用 ZYX 顺序旋转: Yaw(z) -> Pitch(y) -> Roll(x)
    float cr = cosf(roll_rad);
    float sr = sinf(roll_rad);
    float cp = cosf(pitch_rad);
    float sp = sinf(pitch_rad);
    float cy = cosf(yaw_rad);
    float sy = sinf(yaw_rad);

    // 方向余弦矩阵 (DCM)
    float Cbn[3][3];
    Cbn[0][0] = cp * cy;
    Cbn[0][1] = sr * sp * cy - cr * sy;
    Cbn[0][2] = cr * sp * cy + sr * sy;
    Cbn[1][0] = cp * sy;
    Cbn[1][1] = sr * sp * sy + cr * cy;
    Cbn[1][2] = cr * sp * sy - sr * cy;
    Cbn[2][0] = -sp;
    Cbn[2][1] = sr * cp;
    Cbn[2][2] = cr * cp;

    // --- 2. 将机体坐标系的加速度转换到导航坐标系 ---
    // an = C_b^n * ab

    float acc_n = Cbn[0][0] * acc_x + Cbn[0][1] * acc_y + Cbn[0][2] * acc_z;
    float acc_e = Cbn[1][0] * acc_x + Cbn[1][1] * acc_y + Cbn[1][2] * acc_z;
    float acc_d = Cbn[2][0] * acc_x + Cbn[2][1] * acc_y + Cbn[2][2] * acc_z;

    // --- 3. 在导航坐标系下，移除重力加速度的影响 ---
    // 由于导航坐标系是NED（北-东-地），重力方向沿着D轴（地）为正
    float free_acc_d = acc_d - GRAVITY_MSS;

    // --- 4. 对无重力的加速度进行积分，更新速度 ---
    // 使用简单的一阶欧拉积分: v_new = v_old + a * dt
    state->vel_n += acc_n * dt;
    state->vel_e += acc_e * dt;
    state->vel_d += free_acc_d * dt;

    // --- 5. 对速度进行积分，更新位置 ---
    // 使用简单的一阶欧拉积分: p_new = p_old + v * dt
    state->pos_n += state->vel_n * dt;
    state->pos_e += state->vel_e * dt;
    state->pos_d += state->vel_d * dt;
}