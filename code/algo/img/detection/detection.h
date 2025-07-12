#ifndef _IMG_CONNECTED_COMPONENT_H
#define _IMG_CONNECTED_COMPONENT_H

#include "zf_common_headfile.h"
#include "distance.h"
#include "image.h"

// 连通域检测相关参数宏定义（可根据需要调整）
// #define MIN_AREA_THRESHOLD 40    // 最小连通域面积阈值
// #define MAX_AREA_THRESHOLD 5000  // 最大连通域面积阈值（避免过大噪声）
// #define MIN_WIDTH_THRESHOLD 5    // 最小宽度阈值
// #define MIN_HEIGHT_THRESHOLD 5   // 最小高度阈值
// #define MAX_WIDTH_THRESHOLD 200  // 最大宽度阈值
// #define MAX_HEIGHT_THRESHOLD 200 // 最大高度阈值
// #define MIN_ASPECT_RATIO 0.2f    // 最小宽高比（防止过细长条）
// #define MAX_ASPECT_RATIO 5.0f    // 最大宽高比
// #define MIN_COMPACTNESS 0.1f     // 最小紧凑度（面积/外接矩形面积）
#define MAX_REGIONS 128 // 最大连通域数量
#define STACK_SIZE 1024 // 种子填充栈大小

// 连通域检测算法类型
typedef enum
{
    ALGORITHM_TWO_PASS,  // 两遍扫描算法（精确，适合复杂场景）
    ALGORITHM_FLOOD_FILL // 种子填充算法（快速，适合简单检测）
} Component_AlgorithmEnum;

typedef struct
{
    uint32 area;         // 面积
    uint16 min_x, max_x; // 边界框
    uint16 min_y, max_y;
} Component_Box;

typedef struct
{
    Point center;       // 中心点
    Component_Box bbox; // 边界框和面积信息
} Component_Info;

// 检测配置
typedef struct
{
    Component_AlgorithmEnum algorithm;
    Component_Info *sorted_components_array;
    uint8 max_components;
} detection_config_t;

void detection_init(Component_AlgorithmEnum algo, Component_Info *output);
void detection_find_components(uint8 *binary_image);
Component_Info *detection_get_results(void);

static inline bool is_valid_target(const Component_Info *component)
{
    if (!component || component->bbox.area == 0)
        return false;

    // 获取当前的地平线位置
    int16 horizon_y = get_image_horizon();

    // 检查连通域是否完全在地平线以上
    if (component->bbox.max_y < horizon_y)
    {
        return false;
    }

    // // 计算宽高
    // uint16 width = component->bbox.max_x - component->bbox.min_x + 1;
    // uint16 height = component->bbox.max_y - component->bbox.min_y + 1;

    // // 面积限制
    // if (component->bbox.area < MIN_AREA_THRESHOLD || component->bbox.area > MAX_AREA_THRESHOLD)
    //     return false;

    // // 尺寸限制
    // if (width < MIN_WIDTH_THRESHOLD || width > MAX_WIDTH_THRESHOLD ||
    //     height < MIN_HEIGHT_THRESHOLD || height > MAX_HEIGHT_THRESHOLD)
    //     return false;

    // // 宽高比限制
    // float aspect_ratio = (float)width / height;
    // if (aspect_ratio < MIN_ASPECT_RATIO || aspect_ratio > MAX_ASPECT_RATIO)
    //     return false;

    // // 紧凑度限制（防止过于分散的连通域）
    // float compactness = (float)component->bbox.area / (width * height);
    // if (compactness < MIN_COMPACTNESS)
    //     return false;

    return true;
}

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