#ifndef _IMG_CONNECTED_COMPONENT_H
#define _IMG_CONNECTED_COMPONENT_H

#include "zf_common_headfile.h"
#include "image.h"

// 连通域检测相关参数宏定义（可根据需要调整）
#define MIN_AREA_THRESHOLD 40   // 最小连通域面积阈值
#define MAX_AREA_THRESHOLD 5000 // 最大连通域面积阈值（避免过大噪声）
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
} connected_component_algorithm_enum;

// 连通域信息
typedef struct
{
    Point center;        // 中心点
    uint32 area;         // 面积
    uint16 min_x, max_x; // 边界框
    uint16 min_y, max_y;
    bool is_valid; // 是否为有效连通域
} connected_component_info;

// 分析结果
typedef struct
{
    connected_component_info largest_component; // 最大连通域
    uint16 total_components;                    // 总连通域数量
    uint32 total_white_pixels;                  // 总白色像素数
} component_analysis_result;

// 主要接口函数
Point find_white_center(uint8 *binary_image, connected_component_algorithm_enum algorithm);

// 获取所有目标点坐标的接口
uint8 find_all_white_centers(uint8 *binary_image, Point *points_array, uint8 max_points, connected_component_algorithm_enum algorithm);

// 获取所有连通域详细信息的接口
uint8 get_all_components_info(connected_component_info *components_array, uint8 max_components);

// 获取连通域总数
uint8 get_total_components_count(void);

// 获取排序后的连通域
uint8 find_and_sort_components_by_proximity(
    uint8 *binary_image,
    connected_component_algorithm_enum algorithm,
    connected_component_info *sorted_components_array,
    uint8 max_components);

// 工具函数
uint16 find_root(uint16 x);
void union_sets(uint16 x, uint16 y);

// 检查连通域是否为有效目标
static inline bool is_valid_target(const connected_component_info *component)
{
    if (!component || component->area == 0)
        return false;

    // 计算宽高
    // uint16 width = component->max_x - component->min_x + 1;
    // uint16 height = component->max_y - component->min_y + 1;

    // 面积限制
    if (component->area < MIN_AREA_THRESHOLD || component->area > MAX_AREA_THRESHOLD)
        return false;

    // // 尺寸限制
    // if (width < MIN_WIDTH_THRESHOLD || width > MAX_WIDTH_THRESHOLD ||
    //     height < MIN_HEIGHT_THRESHOLD || height > MAX_HEIGHT_THRESHOLD)
    //     return false;

    // // 宽高比限制
    // float aspect_ratio = (float)width / height;
    // if (aspect_ratio < MIN_ASPECT_RATIO || aspect_ratio > MAX_ASPECT_RATIO)
    //     return false;

    // // 紧凑度限制（防止过于分散的连通域）
    // float compactness = (float)component->area / (width * height);
    // if (compactness < MIN_COMPACTNESS)
    //     return false;

    return true;
}

static inline int8 compare_components(const void *a, const void *b)
{
    const connected_component_info *comp_a = (const connected_component_info *)a;
    const connected_component_info *comp_b = (const connected_component_info *)b;

    const int16 center_x = IMG_WIDTH / 2;
    const int16 center_y = IMG_HEIGHT / 2;

    int32 dx_a = comp_a->center.x - center_x;
    int32 dy_a = comp_a->center.y - center_y;
    uint32 dist_sq_a = dx_a * dx_a + dy_a * dy_a;

    int32 dx_b = comp_b->center.x - center_x;
    int32 dy_b = comp_b->center.y - center_y;
    uint32 dist_sq_b = dx_b * dx_b + dy_b * dy_b;

    if (dist_sq_a < dist_sq_b) return -1;
    if (dist_sq_a > dist_sq_b) return 1;

    if (comp_a->area > comp_b->area) return -1;
    if (comp_a->area < comp_b->area) return 1;

    return 0;
}

#endif