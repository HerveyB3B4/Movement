#ifndef _IMG_CONNECTED_COMPONENT_H
#define _IMG_CONNECTED_COMPONENT_H

#include "zf_common_headfile.h"

// 连通域检测相关参数宏定义（可根据需要调整）
#define MIN_AREA_THRESHOLD 5 // 最小连通域面积阈值
#define MAX_REGIONS 512      // 最大连通域数量
#define STACK_SIZE 1024      // 种子填充栈大小

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
component_analysis_result find_largest_connected_component_with_algo(
    uint8 *binary_image,
    connected_component_algorithm_enum algorithm);

Point find_white_center(uint8 *binary_image); // 默认使用种子填充

// 工具函数
uint16 find_root(uint16 x);
void union_sets(uint16 x, uint16 y);

#endif