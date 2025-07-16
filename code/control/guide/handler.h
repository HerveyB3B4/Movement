#ifndef HANDLER_H
#define HANDLER_H

#include "zf_common_headfile.h"
#include "image.h"
#include "detection.h"

#define AREA_TOLERANCE 10
#define ERROR_TOLERANCE 5

typedef enum
{
    STATE_SEARCHING, // 状态：寻找目标
    STATE_TURNING,   // 低速转向
    STATE_TRACKING,  // 状态：加速靠近目标
    STATE_CLOSE      // 接近目标减速
} Run_State;

static inline int8 compare_components(const void *a, const void *b)
{
    const Component_Info *comp_a = (const Component_Info *)a;
    const Component_Info *comp_b = (const Component_Info *)b;

    // 第一关键字：面积（降序），带容差
    int32 area_diff = (int32)comp_a->bbox.area - (int32)comp_b->bbox.area;
    if (abs(area_diff) > AREA_TOLERANCE)
    {
        return (area_diff > 0) ? -1 : 1;
    }

    // 第二关键字：到中心的距离（升序），带容差
    const int16 center_x = IMG_WIDTH / 2;
    const int16 center_y = IMG_HEIGHT / 2;

    int32 dx_a = comp_a->center.x - center_x;
    int32 dx_b = comp_b->center.x - center_x;

    int32 dist_sq_diff = (int32)dx_a - (int32)dx_b;
    if (abs(dist_sq_diff) > ERROR_TOLERANCE)
    {
        return (dist_sq_diff < 0) ? -1 : 1;
    }

    // 第三关键字：前置摄像头优先于后置摄像头
    return (comp_a->camera_id < comp_b->camera_id) ? -1 : 1;
}

void handler_main(void);
void handler_single_camera(void);
void handler_target(void);
void handler_execute(void);
void handler_searching(void);
void handler_tracking(void);
void handler_close(void);

#endif