#ifndef _IMG_TWOPASS_H
#define _IMG_TWOPASS_H

#include "zf_common_headfile.h"

// 连通域处理策略
typedef enum
{
    STRATEGY_IGNORE_SMALL,      // 忽略过小的连通域
    STRATEGY_MERGE_NEARBY,      // 合并邻近的连通域
    STRATEGY_EARLY_TERMINATION, // 提前终止，返回已找到的最大连通域
    STRATEGY_DYNAMIC_THRESHOLD  // 动态调整面积阈值
} region_overflow_strategy_enum;

// 连通域分析配置
typedef struct
{
    uint32 min_area_threshold;                       // 最小面积阈值
    uint16 max_regions_limit;                        // 最大区域数量限制
    region_overflow_strategy_enum overflow_strategy; // 溢出处理策略
    uint8 merge_distance;                            // 合并距离阈值
} connected_component_config;

// 连通域信息
typedef struct
{
    Point center;        // 中心点
    uint32 area;         // 面积
    uint16 min_x, max_x; // 边界框
    uint16 min_y, max_y;
    uint16 label; // 标签号
} connected_component_info;

// 分析结果
typedef struct
{
    connected_component_info largest_component; // 最大连通域
    uint16 total_components;                    // 总连通域数量
    uint16 processed_components;                // 实际处理的连通域数量
    bool overflow_occurred;                     // 是否发生溢出
    uint32 total_white_pixels;                  // 总白色像素数
} component_analysis_result;

// 函数声明
component_analysis_result find_largest_connected_component_advanced(
    uint8 *binary_image,
    connected_component_config *config);

Point find_largest_connected_component(uint8 *binary_image);
uint16 find_root(uint16 x);
void union_sets(uint16 x, uint16 y);

// 配置初始化
void init_component_config(connected_component_config *config);

#endif