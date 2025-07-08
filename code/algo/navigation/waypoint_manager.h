#ifndef WAYPOINT_MANAGER_H
#define WAYPOINT_MANAGER_H

#include "ins_core.h" // 需要 ins_state_t 来获取位置

/**
 * @file waypoint_manager.h
 * @brief 航点管理库 - 头文件
 * @details 负责保存、检索和管理导航路径点。
 */

// --- 常量定义 ---
#define MAX_WAYPOINTS 10 // 定义最多可以保存的航点数量

// --- 数据结构定义 ---

/**
 * @brief 单个航点的数据结构
 */
typedef struct
{
    int id;      // 航点的唯一ID (从0开始)
    float pos_n; // 保存时的北向位置 (m)
    float pos_e; // 保存时的东向位置 (m)
    float pos_d; // 保存时的地向位置 (m)
} waypoint_t;

// --- 函数接口声明 ---

/**
 * @brief 初始化航点管理器，清空所有已存航点
 */
void waypoint_manager_init();

/**
 * @brief 保存当前位置为一个新的航点
 * @param current_state 指向当前INS状态的指针，函数会从中提取位置信息
 * @return 返回新航点的ID。如果航点列表已满, 则返回 -1
 */
int waypoint_save_current_point(const ins_state_t *current_state);

/**
 * @brief 获取已保存的航点数量
 * @return 返回当前已保存的航点总数
 */
int waypoint_get_count();

/**
 * @brief 通过ID读取指定航点的信息
 * @param id 要读取的航点的ID
 * @param out_waypoint 指向一个 waypoint_t 结构体的指针，用于存放读取到的数据
 * @return 成功返回 0, 如果ID无效则返回 -1
 */
int waypoint_get_by_id(int id, waypoint_t *out_waypoint);

/**
 * @brief 获取所有航点的只读指针
 * @return 返回一个指向航点数组的常量指针，可用于遍历所有点
 */
const waypoint_t *waypoint_get_all();

extern waypoint_t g_waypoints[MAX_WAYPOINTS];
extern int g_waypoint_count;

#endif // WAYPOINT_MANAGER_H