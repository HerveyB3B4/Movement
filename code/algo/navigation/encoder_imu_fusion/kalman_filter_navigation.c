#include "kalman_filter_navigation.h"
#include "Ifx_LutAtan2F32.h"
#include "Ifx_LutAtan2F32.h"

static void mat_add_6_6(float a[STATE_SIZE][STATE_SIZE], float b[STATE_SIZE][STATE_SIZE], float c[STATE_SIZE][STATE_SIZE]);
static void mat_add_4_4(float a[MEASUREMENT_SIZE][MEASUREMENT_SIZE], float b[MEASUREMENT_SIZE][MEASUREMENT_SIZE], float c[MEASUREMENT_SIZE][MEASUREMENT_SIZE]);
static void mat_sub_6_6(float a[STATE_SIZE][STATE_SIZE], float b[STATE_SIZE][STATE_SIZE], float c[STATE_SIZE][STATE_SIZE]);
static void mat_mult_6x6_6x6(float a[STATE_SIZE][STATE_SIZE], float b[STATE_SIZE][STATE_SIZE], float c[STATE_SIZE][STATE_SIZE]);
static void mat_mult_6x6_6x4(float a[STATE_SIZE][STATE_SIZE], float b[STATE_SIZE][MEASUREMENT_SIZE], float c[STATE_SIZE][MEASUREMENT_SIZE]);
static void mat_mult_4x6_6x6(float a[MEASUREMENT_SIZE][STATE_SIZE], float b[STATE_SIZE][STATE_SIZE], float c[MEASUREMENT_SIZE][STATE_SIZE]);
static void mat_mult_4x6_6x4(float a[MEASUREMENT_SIZE][STATE_SIZE], float b[STATE_SIZE][MEASUREMENT_SIZE], float c[MEASUREMENT_SIZE][MEASUREMENT_SIZE]);
static void mat_mult_6x4_4x6(float a[STATE_SIZE][MEASUREMENT_SIZE], float b[MEASUREMENT_SIZE][STATE_SIZE], float c[STATE_SIZE][STATE_SIZE]);
static void mat_mult_6x4_4x4(float a[STATE_SIZE][MEASUREMENT_SIZE], float b[MEASUREMENT_SIZE][MEASUREMENT_SIZE], float c[STATE_SIZE][MEASUREMENT_SIZE]);
static void mat_mult_vec_6x6(float a[STATE_SIZE][STATE_SIZE], float b[STATE_SIZE], float c[STATE_SIZE]);
static void mat_mult_vec_4x6(float a[MEASUREMENT_SIZE][STATE_SIZE], float b[STATE_SIZE], float c[MEASUREMENT_SIZE]);
static void mat_mult_vec_6x4(float a[STATE_SIZE][MEASUREMENT_SIZE], float b[MEASUREMENT_SIZE], float c[STATE_SIZE]);
static void mat_transpose_6x6(float a[STATE_SIZE][STATE_SIZE], float b[STATE_SIZE][STATE_SIZE]);
static void mat_transpose_4x6(float a[MEASUREMENT_SIZE][STATE_SIZE], float b[STATE_SIZE][MEASUREMENT_SIZE]);
static void vec_add_6_6(float a[STATE_SIZE], float b[STATE_SIZE], float c[STATE_SIZE]);
static void vec_sub_4_4(float a[MEASUREMENT_SIZE], float b[MEASUREMENT_SIZE], float c[MEASUREMENT_SIZE]);
static int8 mat_inv_4_4(float a[MEASUREMENT_SIZE][MEASUREMENT_SIZE], float b[MEASUREMENT_SIZE][MEASUREMENT_SIZE]);

void kalman_filter_navigation_init(kalman_filter_navigation_t *kf)
{
    // Initialize the state vector
    /**
     * x = [0; px
     *     0; py
     *     0; vx
     *     0; vy
     *     0; ax
     *     0] ay
     */
    for (uint8_t i = 0; i < STATE_SIZE; i++)
    {
        kf->x[i] = 0;
    }

    // Initialize the state covariance matrix
    /**
     * P = [1, 0, 0, 0, 0, 0;
     *     0, 1, 0, 0, 0, 0;
     *     0, 0, 1, 0, 0, 0;
     *     0, 0, 0, 1, 0, 0;
     *     0, 0, 0, 0, 1, 0;
     *     0, 0, 0, 0, 0, 1]
     */
    for (uint8_t i = 0; i < STATE_SIZE; i++)
    {
        for (uint8_t j = 0; j < STATE_SIZE; j++)
        {
            if (i == j)
            {
                kf->P[i][j] = 1.0f;
            }
            else
            {
                kf->P[i][j] = 0;
            }
        }
    }

    // Initialize the state transition matrix
    /**
     * A = [1, 0, dt, 0, 0.5*dt*dt, 0;
     *     0, 1, 0, dt, 0, 0.5*dt*dt;
     *     0, 0, 1, 0, dt, 0;
     *     0, 0, 0, 1, 0, dt;
     *     0, 0, 0, 0, 1, 0;
     *     0, 0, 0, 0, 0, 1]
     */
    float dt = 0.01f;
    for (uint8_t i = 0; i < STATE_SIZE; i++)
    {
        for (uint8_t j = 0; j < STATE_SIZE; j++)
        {
            if (i == j)
            {
                kf->A[i][j] = 1.0f;
            }
            else if (i == j - 2)
            {
                kf->A[i][j] = dt;
            }
            else if (i == j - 4)
            {
                kf->A[i][j] = 0.5f * dt * dt;
            }
            else
            {
                kf->A[i][j] = 0;
            }
        }
    }

    // Initialize the control matrix (not used)
    for (uint8_t i = 0; i < STATE_SIZE; i++)
    {
        kf->B[i] = 0;
    }

    // Initialize the measurement matrix
    /** we can measure px, py, ax, ay
     * H = [1, 0, 0, 0, 0, 0;
     *     0, 1, 0, 0, 0, 0;
     *     0, 0, 0, 0, 1, 0;
     *     0, 0, 0, 0, 0, 1]
     */
    memset(kf->H, 0, sizeof(kf->H));
    kf->H[0][0] = 1.0f;
    kf->H[1][1] = 1.0f;
    kf->H[2][4] = 1.0f;
    kf->H[3][5] = 1.0f;

    // Initialize the process noise covariance matrix
    /**
     * Q = [0.1, 0, 0, 0, 0, 0;
     *     0, 0.1, 0, 0, 0, 0;
     *     0, 0, 0.1, 0, 0, 0;
     *     0, 0, 0, 0.1, 0, 0;
     *     0, 0, 0, 0, 0.1, 0;
     *     0, 0, 0, 0, 0, 0.1]
     */
    for (uint8_t i = 0; i < STATE_SIZE; i++)
    {
        for (uint8_t j = 0; j < STATE_SIZE; j++)
        {
            if (i == j)
            {
                kf->Q[i][j] = 0.1f;
            }
            else
            {
                kf->Q[i][j] = 0;
            }
        }
    }

    // Initialize the measurement noise covariance matrix
    /**
     * R = [0.6, 0, 0, 0;
     *     0, 0.6, 0, 0;
     *     0, 0, 0.01, 0;
     *     0, 0, 0, 0.01]
     */
    memset(kf->R, 0, sizeof(kf->R));
    kf->R[0][0] = 0.6f;
    kf->R[1][1] = 0.6f;
    kf->R[2][2] = 0.01f;
    kf->R[3][3] = 0.01f;

    // Initialize the control vector
    kf->u[0] = 0;
}
void kalman_filter_navigation_predict(kalman_filter_navigation_t *kf)
{
    // Predict the state vector
    /**
     * x = A * x + B * u
     */
    float x_temp[STATE_SIZE];
    mat_mult_vec_6x6(kf->A, kf->x, x_temp);
    vec_add_6_6(x_temp, kf->B, kf->x); // B * u = 0

    // Predict the state covariance matrix
    /**
     * P = A * P * A' + Q
     */
    float AP_temp[STATE_SIZE][STATE_SIZE];
    mat_mult_6x6_6x6(kf->A, kf->P, AP_temp);
    float At[STATE_SIZE][STATE_SIZE];
    mat_transpose_6x6(kf->A, At);
    float APAt[STATE_SIZE][STATE_SIZE];
    mat_mult_6x6_6x6(AP_temp, At, APAt);
    mat_add_6_6(APAt, kf->Q, kf->P);
}
void kalman_filter_navigation_update(kalman_filter_navigation_t *kf, float z[MEASUREMENT_SIZE])
{
    // Calculate the measurement residual
    /**
     * y = z - H * x
     */
    float Hx[MEASUREMENT_SIZE];
    mat_mult_vec_4x6(kf->H, kf->x, Hx);
    float y[MEASUREMENT_SIZE];
    vec_sub_4_4(z, Hx, y);

    // Calculate the residual covariance
    /**
     * S = H * P * H' + R
     */
    float HP_temp[MEASUREMENT_SIZE][STATE_SIZE];
    mat_mult_4x6_6x6(kf->H, kf->P, HP_temp);
    float Ht[STATE_SIZE][MEASUREMENT_SIZE];
    mat_transpose_4x6(kf->H, Ht);
    float HPHt[MEASUREMENT_SIZE][MEASUREMENT_SIZE];
    mat_mult_4x6_6x4(HP_temp, Ht, HPHt);
    float S[MEASUREMENT_SIZE][MEASUREMENT_SIZE]; // residual covariance
    mat_add_4_4(HPHt, kf->R, S);

    // Calculate the Kalman gain
    /**
     * K = P * H' * S^-1
     */
    float S_inv[MEASUREMENT_SIZE][MEASUREMENT_SIZE];
    if (!mat_inv_4_4(S, S_inv))
    {
        return;
    }
    float PHt[STATE_SIZE][MEASUREMENT_SIZE];
    mat_mult_6x6_6x4(kf->P, Ht, PHt);
    float K[STATE_SIZE][MEASUREMENT_SIZE]; // Kalman gain
    mat_mult_6x4_4x4(PHt, S_inv, K);

    // Update the state vector
    /**
     * x = x + K * y
     */
    float Ky[STATE_SIZE];
    mat_mult_vec_6x4(K, y, Ky);
    vec_add_6_6(kf->x, Ky, kf->x);

    // Update the state covariance matrix
    /**
     * P = (I - K * H) * P
     */
    float KH[STATE_SIZE][STATE_SIZE];
    mat_mult_6x4_4x6(K, kf->H, KH);
    float I[STATE_SIZE][STATE_SIZE] = {{1, 0}, {0, 1}};
    float IKH[STATE_SIZE][STATE_SIZE];
    mat_sub_6_6(I, KH, IKH);
    float P_temp[STATE_SIZE][STATE_SIZE];
    mat_mult_6x6_6x6(IKH, kf->P, P_temp);

    // Copy P_temp to P
    for (uint8_t i = 0; i < STATE_SIZE; i++)
    {
        for (uint8_t j = 0; j < STATE_SIZE; j++)
        {
            kf->P[i][j] = P_temp[i][j];
        }
    }
}

static void mat_add_6_6(float a[STATE_SIZE][STATE_SIZE], float b[STATE_SIZE][STATE_SIZE], float c[STATE_SIZE][STATE_SIZE])
{
    for (uint8_t i = 0; i < STATE_SIZE; i++)
    {
        for (uint8_t j = 0; j < STATE_SIZE; j++)
        {
            c[i][j] = a[i][j] + b[i][j];
        }
    }
}
static void mat_add_4_4(float a[MEASUREMENT_SIZE][MEASUREMENT_SIZE], float b[MEASUREMENT_SIZE][MEASUREMENT_SIZE], float c[MEASUREMENT_SIZE][MEASUREMENT_SIZE])
{
    for (uint8_t i = 0; i < MEASUREMENT_SIZE; i++)
    {
        for (uint8_t j = 0; j < MEASUREMENT_SIZE; j++)
        {
            c[i][j] = a[i][j] + b[i][j];
        }
    }
}
static void mat_sub_6_6(float a[STATE_SIZE][STATE_SIZE], float b[STATE_SIZE][STATE_SIZE], float c[STATE_SIZE][STATE_SIZE])
{
    for (uint8_t i = 0; i < STATE_SIZE; i++)
    {
        for (uint8_t j = 0; j < STATE_SIZE; j++)
        {
            c[i][j] = a[i][j] - b[i][j];
        }
    }
}
static void mat_mult_6x6_6x6(float a[STATE_SIZE][STATE_SIZE], float b[STATE_SIZE][STATE_SIZE], float c[STATE_SIZE][STATE_SIZE])
{
    for (uint8_t i = 0; i < STATE_SIZE; i++)
    {
        for (uint8_t j = 0; j < STATE_SIZE; j++)
        {
            c[i][j] = 0;
            for (uint8_t k = 0; k < STATE_SIZE; k++)
            {
                c[i][j] += a[i][k] * b[k][j];
            }
        }
    }
}
static void mat_mult_6x6_6x4(float a[STATE_SIZE][STATE_SIZE], float b[STATE_SIZE][MEASUREMENT_SIZE], float c[STATE_SIZE][MEASUREMENT_SIZE])
{
    for (uint8_t i = 0; i < STATE_SIZE; i++)
    {
        for (uint8_t j = 0; j < MEASUREMENT_SIZE; j++)
        {
            c[i][j] = 0;
            for (uint8_t k = 0; k < STATE_SIZE; k++)
            {
                c[i][j] += a[i][k] * b[k][j];
            }
        }
    }
}
static void mat_mult_4x6_6x6(float a[MEASUREMENT_SIZE][STATE_SIZE], float b[STATE_SIZE][STATE_SIZE], float c[MEASUREMENT_SIZE][STATE_SIZE])
{
    for (uint8_t i = 0; i < MEASUREMENT_SIZE; i++)
    {
        for (uint8_t j = 0; j < STATE_SIZE; j++)
        {
            c[i][j] = 0;
            for (uint8_t k = 0; k < STATE_SIZE; k++)
            {
                c[i][j] += a[i][k] * b[k][j];
            }
        }
    }
}
static void mat_mult_4x6_6x4(float a[MEASUREMENT_SIZE][STATE_SIZE], float b[STATE_SIZE][MEASUREMENT_SIZE], float c[MEASUREMENT_SIZE][MEASUREMENT_SIZE])
{
    for (uint8_t i = 0; i < MEASUREMENT_SIZE; i++)
    {
        for (uint8_t j = 0; j < MEASUREMENT_SIZE; j++)
        {
            c[i][j] = 0;
            for (uint8_t k = 0; k < STATE_SIZE; k++)
            {
                c[i][j] += a[i][k] * b[k][j];
            }
        }
    }
}
static void mat_mult_6x4_4x6(float a[STATE_SIZE][MEASUREMENT_SIZE], float b[MEASUREMENT_SIZE][STATE_SIZE], float c[STATE_SIZE][STATE_SIZE])
{
    for (uint8_t i = 0; i < STATE_SIZE; i++)
    {
        for (uint8_t j = 0; j < STATE_SIZE; j++)
        {
            c[i][j] = 0;
            for (uint8_t k = 0; k < MEASUREMENT_SIZE; k++)
            {
                c[i][j] += a[i][k] * b[k][j];
            }
        }
    }
}
static void mat_mult_6x4_4x4(float a[STATE_SIZE][MEASUREMENT_SIZE], float b[MEASUREMENT_SIZE][MEASUREMENT_SIZE], float c[STATE_SIZE][MEASUREMENT_SIZE])
{
    for (uint8_t i = 0; i < STATE_SIZE; i++)
    {
        for (uint8_t j = 0; j < MEASUREMENT_SIZE; j++)
        {
            c[i][j] = 0;
            for (uint8_t k = 0; k < MEASUREMENT_SIZE; k++)
            {
                c[i][j] += a[i][k] * b[k][j];
            }
        }
    }
}
static void mat_mult_vec_6x6(float a[STATE_SIZE][STATE_SIZE], float b[STATE_SIZE], float c[STATE_SIZE])
{
    for (uint8_t i = 0; i < STATE_SIZE; i++)
    {
        c[i] = 0;
        for (uint8_t j = 0; j < STATE_SIZE; j++)
        {
            c[i] += a[i][j] * b[j];
        }
    }
}
static void mat_mult_vec_4x6(float a[MEASUREMENT_SIZE][STATE_SIZE], float b[STATE_SIZE], float c[MEASUREMENT_SIZE])
{
    for (uint8_t i = 0; i < MEASUREMENT_SIZE; i++)
    {
        c[i] = 0;
        for (uint8_t j = 0; j < STATE_SIZE; j++)
        {
            c[i] += a[i][j] * b[j];
        }
    }
}
static void mat_mult_vec_6x4(float a[STATE_SIZE][MEASUREMENT_SIZE], float b[MEASUREMENT_SIZE], float c[STATE_SIZE])
{
    for (uint8_t i = 0; i < STATE_SIZE; i++)
    {
        c[i] = 0;
        for (uint8_t j = 0; j < MEASUREMENT_SIZE; j++)
        {
            c[i] += a[i][j] * b[j];
        }
    }
}
static void mat_transpose_6x6(float a[STATE_SIZE][STATE_SIZE], float b[STATE_SIZE][STATE_SIZE])
{
    for (uint8_t i = 0; i < STATE_SIZE; i++)
    {
        for (uint8_t j = 0; j < STATE_SIZE; j++)
        {
            b[j][i] = a[i][j];
        }
    }
}
static void mat_transpose_4x6(float a[MEASUREMENT_SIZE][STATE_SIZE], float b[STATE_SIZE][MEASUREMENT_SIZE])
{
    for (uint8_t i = 0; i < MEASUREMENT_SIZE; i++)
    {
        for (uint8_t j = 0; j < STATE_SIZE; j++)
        {
            b[j][i] = a[i][j];
        }
    }
}
static void vec_add_6_6(float a[STATE_SIZE], float b[STATE_SIZE], float c[STATE_SIZE])
{
    for (uint8_t i = 0; i < STATE_SIZE; i++)
    {
        c[i] = a[i] + b[i];
    }
}
static void vec_sub_4_4(float a[MEASUREMENT_SIZE], float b[MEASUREMENT_SIZE], float c[MEASUREMENT_SIZE])
{
    for (uint8_t i = 0; i < MEASUREMENT_SIZE; i++)
    {
        c[i] = a[i] - b[i];
    }
}
// static void mat_inv_4_4(float a[MEASUREMENT_SIZE][MEASUREMENT_SIZE], float b[MEASUREMENT_SIZE][MEASUREMENT_SIZE]){
//     // inverse of a 4x4 matrix using gauss-jordan elimination
//     // only for 4x4 matrix
//     float A[MEASUREMENT_SIZE][2*MEASUREMENT_SIZE];
//     for(uint8_t i = 0; i < MEASUREMENT_SIZE; i++){
//         for(uint8_t j = 0; j < MEASUREMENT_SIZE; j++){
//             A[i][j] = a[i][j];
//         }
//     }
//     for(uint8_t i = 0; i < MEASUREMENT_SIZE; i++){
//         for(uint8_t j = MEASUREMENT_SIZE; j < 2*MEASUREMENT_SIZE; j++){
//             if(i == j - MEASUREMENT_SIZE){
//                 A[i][j] = 1;
//             }else{
//                 A[i][j] = 0;
//             }
//         }
//     }
//     for(uint8_t i = 0; i < MEASUREMENT_SIZE; i++){
//         float temp = A[i][i];
//         for(uint8_t j = 0; j < 2*MEASUREMENT_SIZE; j++){
//             A[i][j] /= temp;
//         }
//         for(uint8_t j = 0; j < MEASUREMENT_SIZE; j++){
//             if(i != j){
//                 temp = A[j][i];
//                 for(uint8_t k = 0; k < 2*MEASUREMENT_SIZE; k++){
//                     A[j][k] -= temp * A[i][k];
//                 }
//             }
//         }
//     }
//     for(uint8_t i = 0; i < MEASUREMENT_SIZE; i++){
//         for(uint8_t j = 0; j < MEASUREMENT_SIZE; j++){
//             b[i][j] = A[i][j + MEASUREMENT_SIZE];
//         }
//     }
// }

static int8 mat_inv_4_4(float a[MEASUREMENT_SIZE][MEASUREMENT_SIZE], float b[MEASUREMENT_SIZE][MEASUREMENT_SIZE])
{
    float det;

    b[0][0] = a[1][1] * a[2][2] * a[3][3] + a[1][2] * a[2][3] * a[3][1] + a[1][3] * a[2][1] * a[3][2] - a[1][3] * a[2][2] * a[3][1] - a[1][2] * a[2][1] * a[3][3] - a[1][1] * a[2][3] * a[3][2];
    b[0][1] = a[0][3] * a[2][2] * a[3][1] + a[0][2] * a[2][1] * a[3][3] + a[0][1] * a[2][3] * a[3][2] - a[0][1] * a[2][2] * a[3][3] - a[0][2] * a[2][3] * a[3][1] - a[0][3] * a[2][1] * a[3][2];
    b[0][2] = a[0][1] * a[1][2] * a[3][3] + a[0][2] * a[1][3] * a[3][1] + a[0][3] * a[1][1] * a[3][2] - a[0][3] * a[1][2] * a[3][1] - a[0][2] * a[1][1] * a[3][3] - a[0][1] * a[1][3] * a[3][2];
    b[0][3] = a[0][2] * a[1][3] * a[2][1] + a[0][3] * a[1][1] * a[2][2] + a[0][1] * a[1][2] * a[2][3] - a[0][1] * a[1][3] * a[2][2] - a[0][3] * a[1][2] * a[2][1] - a[0][2] * a[1][1] * a[2][3];
    b[1][0] = a[1][3] * a[2][2] * a[3][0] + a[1][2] * a[2][0] * a[3][3] + a[1][0] * a[2][3] * a[3][2] - a[1][0] * a[2][2] * a[3][3] - a[1][2] * a[2][3] * a[3][0] - a[1][3] * a[2][0] * a[3][2];
    b[1][1] = a[0][0] * a[2][2] * a[3][3] + a[0][2] * a[2][3] * a[3][0] + a[0][3] * a[2][0] * a[3][2] - a[0][3] * a[2][2] * a[3][0] - a[0][2] * a[2][0] * a[3][3] - a[0][0] * a[2][3] * a[3][2];
    b[1][2] = a[0][3] * a[1][2] * a[3][0] + a[0][2] * a[1][0] * a[3][3] + a[0][0] * a[1][3] * a[3][2] - a[0][0] * a[1][2] * a[3][3] - a[0][2] * a[1][3] * a[3][0] - a[0][3] * a[1][0] * a[3][2];
    b[1][3] = a[0][0] * a[1][3] * a[2][2] + a[0][3] * a[1][2] * a[2][0] + a[0][2] * a[1][0] * a[2][3] - a[0][2] * a[1][3] * a[2][0] - a[0][3] * a[1][0] * a[2][2] - a[0][0] * a[1][2] * a[2][3];
    b[2][0] = a[1][0] * a[2][1] * a[3][3] + a[1][1] * a[2][3] * a[3][0] + a[1][3] * a[2][0] * a[3][1] - a[1][3] * a[2][1] * a[3][0] - a[1][1] * a[2][0] * a[3][3] - a[1][0] * a[2][3] * a[3][1];
    b[2][1] = a[0][3] * a[2][1] * a[3][0] + a[0][1] * a[2][0] * a[3][3] + a[0][0] * a[2][3] * a[3][1] - a[0][0] * a[2][1] * a[3][3] - a[0][1] * a[2][3] * a[3][0] - a[0][3] * a[2][0] * a[3][1];
    b[2][2] = a[0][0] * a[1][1] * a[3][3] + a[0][1] * a[1][3] * a[3][0] + a[0][3] * a[1][0] * a[3][1] - a[0][3] * a[1][1] * a[3][0] - a[0][1] * a[1][0] * a[3][3] - a[0][0] * a[1][3] * a[3][1];
    b[2][3] = a[0][3] * a[1][1] * a[2][0] + a[0][1] * a[1][0] * a[2][3] + a[0][0] * a[1][3] * a[2][1] - a[0][0] * a[1][1] * a[2][3] - a[0][1] * a[1][3] * a[2][0] - a[0][3] * a[1][0] * a[2][1];
    b[3][0] = a[1][2] * a[2][1] * a[3][0] + a[1][1] * a[2][0] * a[3][2] + a[1][0] * a[2][2] * a[3][1] - a[1][0] * a[2][1] * a[3][2] - a[1][1] * a[2][2] * a[3][0] - a[1][2] * a[2][0] * a[3][1];
    b[3][1] = a[0][0] * a[2][1] * a[3][2] + a[0][1] * a[2][2] * a[3][0] + a[0][2] * a[2][0] * a[3][1] - a[0][2] * a[2][1] * a[3][0] - a[0][1] * a[2][0] * a[3][2] - a[0][0] * a[2][2] * a[3][1];
    b[3][2] = a[0][2] * a[1][1] * a[3][0] + a[0][1] * a[1][0] * a[3][2] + a[0][0] * a[1][2] * a[3][1] - a[0][0] * a[1][1] * a[3][2] - a[0][1] * a[1][2] * a[3][0] - a[0][2] * a[1][0] * a[3][1];
    b[3][3] = a[0][0] * a[1][1] * a[2][2] + a[0][1] * a[1][2] * a[2][0] + a[0][2] * a[1][0] * a[2][1] - a[0][2] * a[1][1] * a[2][0] - a[0][1] * a[1][0] * a[2][2] - a[0][0] * a[1][2] * a[2][1];

    det = a[0][0] * b[0][0] + a[0][1] * b[1][0] + a[0][2] * b[2][0] + a[0][3] * b[3][0];

    if (det == 0)
    {
        return 0;
    }

    det = 1.0 / det;
    for (int i = 0; i < MEASUREMENT_SIZE; i++)
    {
        for (int j = 0; j < MEASUREMENT_SIZE; j++)
        {
            b[i][j] *= det;
        }
    }
    return 1;
}