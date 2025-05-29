#ifndef _NAVIGATION_H_
#define _NAVIGATION_H_

#include "Attitude.h"
#include "trace.h"
#include "zf_common_headfile.h"

// 初始化导航系统 - 基本校准
void navigation_init(void);

// 使用高级校准方法初始化导航系统
void navigation_init_advanced(void);

// 记录当前位置为途径点
void record_waypoint(const char* name);

// 设置起始点（重置位置为原点）
void set_start_point(void);

// 显示所有记录的途径点
void display_waypoints(void);

// 计算两点间的距离
float calculate_distance(Vector3f* pos1, Vector3f* pos2);

// 计算当前位置到指定途径点的距离和方向
void calculate_navigation_info(int target_index, float* distance, float* angle);

// 开始导航到指定的途径点
void start_navigation(int target_index);

// 停止导航
void stop_navigation(void);

// 更新导航状态和显示 - 需要在主循环中调用
void update_navigation(void);

#endif /* _NAVIGATION_H_ */