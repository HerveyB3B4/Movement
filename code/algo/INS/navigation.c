#include "navigation.h"
#include "method.h"

// 定义一个结构体来保存目标点
typedef struct {
    Vector3f position;
    char name[32];
} Waypoint;

// 定义一些全局变量来管理途径点
#define MAX_WAYPOINTS 20
Waypoint g_waypoints[MAX_WAYPOINTS];
int g_waypoint_count = 0;
int g_current_target_index = 0;   // 当前导航目标点索引
uint8_t g_navigation_active = 0;  // 导航状态标志

// 记录数据的函数
void collect_position_data(uint8_t position_index) {
    lcd_clear();
    lcd_show_string(0, 0, "Collecting data...");

    // 临时存储累加值
    Vector3f accel_sum = {0.0f, 0.0f, 0.0f};

    // 采集2000个样本
    for (int i = 0; i < 2000; i++) {
        // 获取IMU数据
        imu_get_data(&g_imu_data);

        // 累加加速度值
        accel_sum.x += g_imu_data.acc.x;
        accel_sum.y += g_imu_data.acc.y;
        accel_sum.z += g_imu_data.acc.z;

        Vector3f accel_raw = {g_imu_data.acc.x, g_imu_data.acc.y,
                              g_imu_data.acc.z};
        Vector3f gyro_raw = {g_imu_data.gyro.x, g_imu_data.gyro.y,
                             g_imu_data.gyro.z};

        // 更新传感器数据
        INS_UpdateSensorData(&g_ins_state, &accel_raw, &gyro_raw);

        system_delay_ms(1);
    }

    lcd_clear();
    lcd_show_string(0, 0, "Position recorded!");
    system_delay_ms(500);
}

void navigation_init() {
    lcd_clear();
    while (keymsg.key != KEY_L) {
        INS_Init(&g_ins_state, 0.001f, INS_MODE_ADAPTIVE, ZUPT_ADVANCED);
        lcd_show_string(0, 0, "keep calm");
        system_delay_ms(1000);
        for (int i = 0; i < 2000; i++) {
            // 获取IMU数据
            imu_get_data(&g_imu_data);

            // 转换为Vector3f格式
            Vector3f accel_raw = {g_imu_data.acc.x, g_imu_data.acc.y,
                                  g_imu_data.acc.z};
            Vector3f gyro_raw = {g_imu_data.gyro.x, g_imu_data.gyro.y,
                                 g_imu_data.gyro.z};

            // 更新传感器数据
            INS_UpdateSensorData(&g_ins_state, &accel_raw, &gyro_raw);

            system_delay_ms(1);
        }
        INS_Calibrate(&g_ins_state, 2000);
        lcd_clear();
        lcd_show_string(0, 0, "INS initialized");
    }
    lcd_clear();

    // 初始化途径点计数和导航状态
    g_waypoint_count = 0;
    g_current_target_index = 0;
    g_navigation_active = 0;
}

void navigation_init_advanced() {
    lcd_clear();
    while (keymsg.key != KEY_L) {
        // 四个位置，随便重新设置就行
        lcd_show_string(0, 0, "Press KEY_B to next");
        while (keymsg.key != KEY_B) {
            lcd_show_string(0, 1, "Z axis up");
        }
        collect_position_data(0);
        while (keymsg.key != KEY_B) {
            lcd_show_string(0, 1, "Z axis down");
        }
        collect_position_data(1);
        while (keymsg.key != KEY_B) {
            lcd_show_string(0, 1, "X axis up");
        }
        collect_position_data(2);
        while (keymsg.key != KEY_B) {
            lcd_show_string(0, 1, "Y axis up");
        }
        collect_position_data(3);
        lcd_clear();
        lcd_show_string(0, 0, "Calibrating...");
        INS_CalibrateAdvanced(&g_ins_state, 4, 2000);
    }
    lcd_clear();

    // 初始化途径点计数和导航状态
    g_waypoint_count = 0;
    g_current_target_index = 0;
    g_navigation_active = 0;
}

// 记录当前位置为途径点
void record_waypoint(const char* name) {
    if (g_waypoint_count >= MAX_WAYPOINTS) {
        lcd_clear();
        lcd_show_string(0, 0, "Too many waypoints");
        system_delay_ms(1000);
        return;
    }

    // 复制当前位置
    g_waypoints[g_waypoint_count].position = g_ins_state.position;

    // 复制名称
    strncpy(g_waypoints[g_waypoint_count].name, name, 31);
    g_waypoints[g_waypoint_count].name[31] = '\0';  // 确保字符串结束

    g_waypoint_count++;

    lcd_clear();
    lcd_show_string(0, 0, "Waypoint recorded!");
    lcd_show_string(0, 1, name);

    char pos_str[32];
    sprintf(pos_str, "X:%.2f Y:%.2f", g_ins_state.position.x,
            g_ins_state.position.y);
    lcd_show_string(0, 2, pos_str);

    system_delay_ms(1000);
}

// 记录起始点
void set_start_point() {
    // 重置位置为原点
    g_ins_state.position.x = 0.0f;
    g_ins_state.position.y = 0.0f;
    g_ins_state.position.z = 0.0f;

    // 重置速度
    g_ins_state.velocity.x = 0.0f;
    g_ins_state.velocity.y = 0.0f;
    g_ins_state.velocity.z = 0.0f;

    // 记录起始点
    record_waypoint("Start Point");

    lcd_clear();
    lcd_show_string(0, 0, "Start point set");
    lcd_show_string(0, 1, "Position reset to 0");
    system_delay_ms(1000);
}

// 显示所有记录的途径点
void display_waypoints() {
    lcd_clear();

    if (g_waypoint_count == 0) {
        lcd_show_string(0, 0, "No waypoints");
        system_delay_ms(1000);
        return;
    }

    int page = 0;
    int page_count = (g_waypoint_count + 3) / 4;  // 每页最多显示4个点

    while (1) {
        lcd_clear();
        char page_info[32];
        sprintf(page_info, "Page %d/%d", page + 1, page_count);
        lcd_show_string(0, 0, page_info);

        for (int i = 0; i < 4; i++) {
            int idx = page * 4 + i;
            if (idx < g_waypoint_count) {
                char wp_str[32];
                sprintf(wp_str, "%d.%.10s: %.1f,%.1f", idx + 1,
                        g_waypoints[idx].name, g_waypoints[idx].position.x,
                        g_waypoints[idx].position.y);
                lcd_show_string(0, i + 1, wp_str);
            }
        }

        // 等待按键
        lcd_show_string(0, 6, "KEY_L:Exit KEY_B:Next");

        while (1) {
            if (keymsg.key == KEY_L) {
                return;  // 退出
            }
            if (keymsg.key == KEY_B) {
                page = (page + 1) % page_count;  // 下一页
                break;
            }
            system_delay_ms(10);
        }
    }
}

// 计算两点间的距离
float calculate_distance(Vector3f* pos1, Vector3f* pos2) {
    float dx = pos1->x - pos2->x;
    float dy = pos1->y - pos2->y;
    float dz = pos1->z - pos2->z;

    return sqrtf(dx * dx + dy * dy + dz * dz);
}

// 计算当前位置到指定途径点的距离和方向
void calculate_navigation_info(int target_index,
                               float* distance,
                               float* angle) {
    if (target_index < 0 || target_index >= g_waypoint_count) {
        *distance = 0.0f;
        *angle = 0.0f;
        return;
    }

    Vector3f* target = &g_waypoints[target_index].position;
    Vector3f* current = &g_ins_state.position;

    // 计算距离
    float dx = target->x - current->x;
    float dy = target->y - current->y;
    *distance = sqrtf(dx * dx + dy * dy);

    // 计算角度 (相对于正北方向)
    *angle = atan2f(dy, dx) * 180.0f / PI;
    if (*angle < 0) {
        *angle += 360.0f;
    }
}

// 开始导航到指定的途径点
void start_navigation(int target_index) {
    if (target_index < 0 || target_index >= g_waypoint_count) {
        lcd_clear();
        lcd_show_string(0, 0, "Invalid target!");
        system_delay_ms(1000);
        return;
    }

    g_current_target_index = target_index;
    g_navigation_active = 1;

    // todo
    lcd_clear();
    lcd_show_string(0, 0, "Navigation started");
    lcd_show_string(0, 1, g_waypoints[target_index].name);
    system_delay_ms(1000);
}

// 停止导航
void stop_navigation() {
    g_navigation_active = 0;

    lcd_clear();
    lcd_show_string(0, 0, "Navigation stopped");
    system_delay_ms(1000);
}

// 更新导航状态和显示 - 在主循环中调用
void update_navigation() {
    // 获取IMU数据和更新传感器数据
    imu_get_data(&g_imu_data);
    Vector3f accel_raw = {g_imu_data.acc.x, g_imu_data.acc.y, g_imu_data.acc.z};
    Vector3f gyro_raw = {g_imu_data.gyro.x, g_imu_data.gyro.y,
                         g_imu_data.gyro.z};
    INS_UpdateSensorData(&g_ins_state, &accel_raw, &gyro_raw);

    // 更新惯性导航
    INS_Update(&g_ins_state, &g_euler_angle);

    // 如果导航处于活动状态，更新导航信息
    if (g_navigation_active) {
        static uint32_t display_timer = 0;
        display_timer++;

        // 每100ms更新显示 (假设此函数以1kHz频率调用)
        if (display_timer % 100 == 0) {
            float distance, angle;
            calculate_navigation_info(g_current_target_index, &distance,
                                      &angle);

            lcd_clear();
            lcd_show_string(0, 0, "Navigating to:");
            lcd_show_string(0, 1, g_waypoints[g_current_target_index].name);

            char dist_str[32];
            sprintf(dist_str, "Dist: %.2f m", distance);
            lcd_show_string(0, 2, dist_str);

            char angle_str[32];
            sprintf(angle_str, "Dir: %.1f deg", angle);
            lcd_show_string(0, 3, angle_str);

            char pos_str[32];
            sprintf(pos_str, "X:%.2f Y:%.2f", g_ins_state.position.x,
                    g_ins_state.position.y);
            lcd_show_string(0, 4, pos_str);

            // 检查是否到达目标点 (距离小于0.5米)
            if (distance < 0.5f) {
                lcd_clear();
                lcd_show_string(0, 0, "Target reached!");
                lcd_show_string(0, 1, g_waypoints[g_current_target_index].name);
                system_delay_ms(2000);

                g_navigation_active = 0;  // 停止导航
            }
        }
    }
}
