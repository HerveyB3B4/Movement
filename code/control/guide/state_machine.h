#ifndef _STATE_MACHINE_H
#define _STATE_MACHINE_H

#include "zf_common_headfile.h"
#include "image.h"

typedef enum
{
    STATE_SEARCHING,   // 状态：寻找目标
    STATE_APPROACHING, // 状态：接近目标
    STATE_CIRCLING,    // 状态：环绕目标
} Run_State;

typedef enum
{
    SINGLE_CAMERA, // 单摄像头模式
    DUAL_CAMERA,   // 双摄像头模式
} Camera_Mode;

static inline int8 compare_components(const void *a, const void *b)
{
    const Component_Info *comp_a = (const Component_Info *)a;
    const Component_Info *comp_b = (const Component_Info *)b;

    const int16 center_x = IMG_WIDTH / 2;
    const int16 center_y = IMG_HEIGHT / 2;

    int32 dx_a = comp_a->center.x - center_x;
    int32 dy_a = comp_a->center.y - center_y;
    uint32 dist_sq_a = dx_a * dx_a + dy_a * dy_a;

    int32 dx_b = comp_b->center.x - center_x;
    int32 dy_b = comp_b->center.y - center_y;
    uint32 dist_sq_b = dx_b * dx_b + dy_b * dy_b;

    if (dist_sq_a < dist_sq_b)
        return -1;
    if (dist_sq_a > dist_sq_b)
        return 1;

    if (comp_a->bbox.area > comp_b->bbox.area)
        return -1;
    if (comp_a->bbox.area < comp_b->bbox.area)
        return 1;

    return 0;
}

#endif
