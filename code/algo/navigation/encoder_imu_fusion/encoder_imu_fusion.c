#include "encoder_imu_fusion.h"
#include "kalman_filter_navigation.h"
#include "Attitude.h"

static fusion_nav_pos pos;
static kalman_filter_navigation_t kf;

void fusion_nav_init()
{
    kalman_filter_navigation_init(&kf);

    pos.x = 0;
    pos.y = 0;
}

void fusion_nav_update(void)
{
    kalman_filter_navigation_predict(&kf);

    float z[MEASUREMENT_SIZE];
    z[0] = pos.y;
    z[1] = pos.x;
    z[2] = PITCH_ACC;
    z[3] = -ROLL_ACC;

    kalman_filter_navigation_update(&kf, z);

    pos.x = kf.x[1];
    pos.y = kf.x[0];
}

PointF fusion_nav_get_pos(void)
{
    PointF p;
    p.x = pos.x;
    p.y = pos.y;
    return p;
}