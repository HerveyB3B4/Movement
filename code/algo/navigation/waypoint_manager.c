#include "waypoint_manager.h"
#include "zf_common_typedef.h"

// 使用静态全局变量来存储航点数据和计数，对外部文件隐藏
waypoint_t g_waypoints[MAX_WAYPOINTS];
int g_waypoint_count = 0;

void waypoint_manager_init()
{
    g_waypoint_count = 0;
}

int waypoint_save_current_point(const ins_state_t *current_state)
{
    if (current_state == NULL || g_waypoint_count >= MAX_WAYPOINTS)
    {
        return -1; // 错误：指针为空或列表已满
    }

    // 创建新的航点
    int new_id = g_waypoint_count;
    g_waypoints[new_id].id = new_id;
    g_waypoints[new_id].pos_n = current_state->pos_n;
    g_waypoints[new_id].pos_e = current_state->pos_e;
    g_waypoints[new_id].pos_d = current_state->pos_d;

    // 航点计数增加
    g_waypoint_count++;

    return new_id; // 返回新航点的ID
}

int waypoint_get_count()
{
    return g_waypoint_count;
}

int waypoint_get_by_id(int id, waypoint_t *out_waypoint)
{
    if (out_waypoint == NULL || id < 0 || id >= g_waypoint_count)
    {
        return -1; // 错误：指针为空或ID无效
    }

    *out_waypoint = g_waypoints[id];

    return 0; // 成功
}

const waypoint_t *waypoint_get_all()
{
    return g_waypoints;
}