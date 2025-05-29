#ifndef _NAVIGATION_H_
#define _NAVIGATION_H_

#include "Attitude.h"
#include "trace.h"
#include "zf_common_headfile.h"

// 轨迹记录相关定义
#define MAX_TRAJECTORY_POINTS 1000      // 最大轨迹点数量
#define TRAJECTORY_RECORD_INTERVAL 100  // 记录间隔(毫秒)

// 轨迹点结构体
typedef struct {
    Vector3f position;   // 位置
    Vector3f velocity;   // 速度
    uint32_t timestamp;  // 时间戳(毫秒)
} TrajectoryPoint;

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

// 导航菜单，提供导航功能的用户界面
void navigation_menu(void);

// 轨迹记录功能
// 开始记录轨迹
void start_trajectory_recording(void);

// 停止记录轨迹
void stop_trajectory_recording(void);

// 保存轨迹到Flash或其他存储介质
uint8_t save_trajectory(const char* name);

// 加载已保存的轨迹
uint8_t load_trajectory(const char* name);

// 显示当前记录的轨迹信息
void display_trajectory_info(void);

// 导出轨迹数据到串口，可以在电脑上绘制
void export_trajectory_data(void);

#endif /* _NAVIGATION_H_ */