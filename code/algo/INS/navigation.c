#include "navigation.h"
#include "method.h"

// 轨迹记录相关全局变量
TrajectoryPoint g_trajectory[MAX_TRAJECTORY_POINTS];
int g_trajectory_count = 0;
uint8_t g_trajectory_recording = 0;
uint32_t g_last_record_time = 0;
uint32_t g_trajectory_start_time = 0;

// 记录数据的函数
void collect_position_data(uint8_t position_index)
{
    lcd_clear();
    lcd_show_string(0, 0, "Collecting data...");

    // 临时存储累加值
    Vector3f accel_sum = {0.0f, 0.0f, 0.0f};

    // 采集2000个样本
    for (int i = 0; i < 2000; i++)
    {
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

void navigation_init()
{
    lcd_clear();
    while (keymsg.key != KEY_L)
    {
        INS_Init(&g_ins_state, 0.001f, INS_MODE_ADAPTIVE, ZUPT_ADVANCED);
        lcd_show_string(0, 0, "keep calm");
        system_delay_ms(1000);
        for (int i = 0; i < 2000; i++)
        {
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

    // 初始化轨迹变量
    g_trajectory_recording = 0;
    g_trajectory_count = 0;
}

void navigation_init_advanced()
{
    lcd_clear();
    while (keymsg.key != KEY_L)
    {
        // 四个位置，随便重新设置就行
        lcd_show_string(0, 0, "Press KEY_B to next");
        while (keymsg.key != KEY_B)
        {
            lcd_show_string(0, 1, "Z axis up");
        }
        collect_position_data(0);
        while (keymsg.key != KEY_B)
        {
            lcd_show_string(0, 1, "Z axis down");
        }
        collect_position_data(1);
        while (keymsg.key != KEY_B)
        {
            lcd_show_string(0, 1, "X axis up");
        }
        collect_position_data(2);
        while (keymsg.key != KEY_B)
        {
            lcd_show_string(0, 1, "Y axis up");
        }
        collect_position_data(3);
        lcd_clear();
        lcd_show_string(0, 0, "Calibrating...");
        INS_CalibrateAdvanced(&g_ins_state, 4, 2000);
    }
    lcd_clear();

    // 初始化轨迹变量
    g_trajectory_recording = 0;
    g_trajectory_count = 0;
}

// 设置起始点（重置位置为原点）
void set_start_point(void)
{
    lcd_clear();
    lcd_show_string(0, 0, "Setting start point...");

    // 重置位置为原点
    g_ins_state.position.x = 0.0f;
    g_ins_state.position.y = 0.0f;
    g_ins_state.position.z = 0.0f;

    lcd_clear();
    lcd_show_string(0, 0, "Start point set");
    lcd_show_string(0, 1, "Position reset to 0,0");
    system_delay_ms(1000);
}

// 计算两点间的距离
float calculate_distance(Vector3f *pos1, Vector3f *pos2)
{
    float dx = pos1->x - pos2->x;
    float dy = pos1->y - pos2->y;
    float dz = pos1->z - pos2->z;

    return sqrtf(dx * dx + dy * dy + dz * dz);
}

// 开始记录轨迹
void start_trajectory_recording()
{
    // 确保不会重复启动记录
    if (g_trajectory_recording)
    {
        lcd_clear();
        lcd_show_string(0, 0, "Already recording!");
        system_delay_ms(1000);
        return;
    }

    // 重置轨迹计数
    g_trajectory_count = 0;
    g_trajectory_recording = 1;

    // 记录开始时间
    g_trajectory_start_time = system_getval();
    g_last_record_time = g_trajectory_start_time;

    // 记录第一个点（当前位置）
    if (g_trajectory_count < MAX_TRAJECTORY_POINTS)
    {
        g_trajectory[g_trajectory_count].position = g_ins_state.position;
        g_trajectory[g_trajectory_count].velocity = g_ins_state.velocity;
        g_trajectory[g_trajectory_count].timestamp = 0; // 初始时间戳为0
        g_trajectory_count++;
    }

    lcd_clear();
    lcd_show_string(0, 0, "Trajectory recording");
    lcd_show_string(0, 1, "started!");
    system_delay_ms(1000);
}

// 停止记录轨迹
void stop_trajectory_recording()
{
    if (!g_trajectory_recording)
    {
        lcd_clear();
        lcd_show_string(0, 0, "Not recording!");
        system_delay_ms(1000);
        return;
    }

    g_trajectory_recording = 0;

    lcd_clear();
    lcd_show_string(0, 0, "Recording stopped!");
    char count_str[32];
    sprintf(count_str, "%d points recorded", g_trajectory_count);
    lcd_show_string(0, 1, count_str);
    system_delay_ms(1000);
}

// 保存轨迹到Flash（示例实现，实际需要根据硬件调整）
uint8_t save_trajectory(const char *name)
{
    // 这里只是一个简化示例，实际项目中应该保存到Flash或SD卡
    lcd_clear();
    lcd_show_string(0, 0, "Saving trajectory...");
    lcd_show_string(0, 1, name);

    // TODO: 实现实际的保存功能，例如写入到Flash或SD卡

    system_delay_ms(1000);
    lcd_show_string(0, 2, "Save completed!");
    system_delay_ms(1000);

    return 1; // 成功返回1
}

// 加载已保存的轨迹
uint8_t load_trajectory(const char *name)
{
    // 这里只是一个简化示例，实际项目中应该从Flash或SD卡加载
    lcd_clear();
    lcd_show_string(0, 0, "Loading trajectory...");
    lcd_show_string(0, 1, name);

    // TODO: 实现实际的加载功能，例如从Flash或SD卡读取

    system_delay_ms(1000);
    lcd_show_string(0, 2, "Load not implemented");
    system_delay_ms(1000);

    return 0; // 未实现返回0
}

// 显示当前记录的轨迹信息
void display_trajectory_info()
{
    lcd_clear();

    if (g_trajectory_count == 0)
    {
        lcd_show_string(0, 0, "No trajectory data");
        system_delay_ms(1000);
        return;
    }

    // 计算轨迹总长度
    float total_distance = 0.0f;
    for (int i = 1; i < g_trajectory_count; i++)
    {
        total_distance += calculate_distance(&g_trajectory[i - 1].position,
                                             &g_trajectory[i].position);
    }

    // 计算总时间（毫秒）
    uint32_t total_time_ms = g_trajectory[g_trajectory_count - 1].timestamp;

    lcd_show_string(0, 0, "Trajectory info:");

    char points_str[32];
    sprintf(points_str, "Points: %d", g_trajectory_count);
    lcd_show_string(0, 1, points_str);

    char dist_str[32];
    sprintf(dist_str, "Distance: %.2f m", total_distance);
    lcd_show_string(0, 2, dist_str);

    char time_str[32];
    sprintf(time_str, "Time: %.1f s", total_time_ms / 1000.0f);
    lcd_show_string(0, 3, time_str);

    // 等待按键返回
    lcd_show_string(0, 6, "Press any key...");
    while (keymsg.key == 0)
    {
        system_delay_ms(10);
    }
}

// 导出轨迹数据到串口，可以在电脑上绘制
void export_trajectory_data()
{
    lcd_clear();
    lcd_show_string(0, 0, "Exporting data...");

    if (g_trajectory_count == 0)
    {
        lcd_show_string(0, 1, "No data to export");
        system_delay_ms(1000);
        return;
    }

    // 通过串口发送CSV格式的数据头
    printf("timestamp,x,y,z,vx,vy,vz\n");

    // 发送每个点的数据
    for (int i = 0; i < g_trajectory_count; i++)
    {
        printf("%d,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n", g_trajectory[i].timestamp,
               g_trajectory[i].position.x, g_trajectory[i].position.y,
               g_trajectory[i].position.z, g_trajectory[i].velocity.x,
               g_trajectory[i].velocity.y, g_trajectory[i].velocity.z);

        // 每发送10个点延迟一下，避免缓冲区溢出
        if (i % 10 == 0)
        {
            system_delay_ms(5);
        }
    }

    lcd_show_string(0, 1, "Export completed!");
    system_delay_ms(1000);
}

// 显示当前位置信息
void display_position_info()
{
    lcd_clear();
    lcd_show_string(0, 0, "Current Position:");

    while (1)
    {
        // 格式化位置和速度
        char pos_str[32];
        sprintf(pos_str, "X:%.2f Y:%.2f Z:%.2f",
                g_ins_state.position.x, g_ins_state.position.y, g_ins_state.position.z);
        lcd_show_string(0, 1, pos_str);

        char vel_str[32];
        sprintf(vel_str, "VX:%.2f VY:%.2f VZ:%.2f",
                g_ins_state.velocity.x, g_ins_state.velocity.y, g_ins_state.velocity.z);
        lcd_show_string(0, 2, vel_str);

        // 显示欧拉角
        char euler_str[32];
        sprintf(euler_str, "Yaw:%.1f P:%.1f R:%.1f",
                g_euler_angle.yaw, g_euler_angle.pitch, g_euler_angle.roll);
        lcd_show_string(0, 3, euler_str);

        // 实时刷新，按L键返回
        lcd_show_string(0, 6, "Press KEY_L to return");

        if (keymsg.key == KEY_L)
        {
            return;
        }

        system_delay_ms(100); // 刷新间隔

        // 清除旧数据，但保留标题和提示
        lcd_show_string(0, 1, "                    ");
        lcd_show_string(0, 2, "                    ");
        lcd_show_string(0, 3, "                    ");
    }
}

// 更新导航状态和显示 - 在主循环中调用
void update_navigation()
{
    // 获取IMU数据和更新传感器数据
    imu_get_data(&g_imu_data);
    Vector3f accel_raw = {g_imu_data.acc.x, g_imu_data.acc.y, g_imu_data.acc.z};
    Vector3f gyro_raw = {g_imu_data.gyro.x, g_imu_data.gyro.y,
                         g_imu_data.gyro.z};
    INS_UpdateSensorData(&g_ins_state, &accel_raw, &gyro_raw);

    // 更新惯性导航
    INS_Update(&g_ins_state, &g_euler_angle);

    // 如果轨迹记录处于活动状态，记录轨迹点
    if (g_trajectory_recording)
    {
        uint32_t current_time = system_getval();

        // 根据设定的间隔记录轨迹点
        if ((current_time - g_last_record_time) >= TRAJECTORY_RECORD_INTERVAL)
        {
            if (g_trajectory_count < MAX_TRAJECTORY_POINTS)
            {
                g_trajectory[g_trajectory_count].position =
                    g_ins_state.position;
                g_trajectory[g_trajectory_count].velocity =
                    g_ins_state.velocity;
                g_trajectory[g_trajectory_count].timestamp =
                    current_time - g_trajectory_start_time;
                g_trajectory_count++;
                g_last_record_time = current_time;

                // 如果轨迹点数量达到最大值，自动停止记录
                if (g_trajectory_count >= MAX_TRAJECTORY_POINTS)
                {
                    stop_trajectory_recording();
                    lcd_clear();
                    lcd_show_string(0, 0, "Trajectory full!");
                    lcd_show_string(0, 1, "Recording stopped");
                    system_delay_ms(1000);
                }
            }
        }
    }
}

// 轨迹选项子菜单
void trajectory_submenu()
{
    const char *submenu_items[] = {
        "1. Start Recording",   // 0
        "2. Stop Recording",    // 1
        "3. Show Traj Info",    // 2
        "4. Export Trajectory", // 3
        "5. Back"               // 4
    };
    const int submenu_count = sizeof(submenu_items) / sizeof(submenu_items[0]);
    int submenu_index = 0;

    while (1)
    {
        lcd_clear();
        lcd_show_string(0, 0, "Trajectory Options");

        for (int i = 0; i < 5 && i < submenu_count; i++)
        {
            lcd_show_string(0, i + 1, submenu_items[i]);
        }

        lcd_show_string(0, 6, "KEY_U/D:Move B:Sel L:Back");

        // 等待用户选择
        while (1)
        {
            if (keymsg.key == KEY_L)
            {
                return; // 返回上一级菜单
            }

            if (keymsg.key == KEY_U && submenu_index > 0)
            {
                submenu_index--;
                break;
            }

            if (keymsg.key == KEY_D && submenu_index < submenu_count - 1)
            {
                submenu_index++;
                break;
            }

            if (keymsg.key == KEY_B)
            {
                // 执行选中的子菜单项
                switch (submenu_index)
                {
                case 0: // 开始轨迹记录
                    start_trajectory_recording();
                    break;

                case 1: // 停止轨迹记录
                    stop_trajectory_recording();
                    break;

                case 2: // 显示轨迹信息
                    display_trajectory_info();
                    break;

                case 3: // 导出轨迹数据
                    export_trajectory_data();
                    break;

                case 4: // 返回
                    return;
                }
                break;
            }

            system_delay_ms(10);
        }
    }
}

// 导航菜单，提供导航功能的用户界面
void navigation_menu()
{
    const char *menu_items[] = {
        "1. Set start point",    // 0
        "2. Trajectory Options", // 1
        "3. View Position Info", // 2
        "4. Exit Menu"           // 3
    };
    const int menu_count = sizeof(menu_items) / sizeof(menu_items[0]);
    int menu_index = 0;

    while (1)
    {
        lcd_clear();
        lcd_show_string(0, 0, "Navigation Menu");

        for (int i = 0; i < 5 && i < menu_count; i++)
        {
            lcd_show_string(0, i + 1, menu_items[i]);
        }

        lcd_show_string(0, 6, "KEY_U/D:Move B:Sel L:Exit");

        // 等待用户选择
        while (1)
        {
            if (keymsg.key == KEY_L)
            {
                lcd_clear();
                return; // 退出菜单
            }

            if (keymsg.key == KEY_U && menu_index > 0)
            {
                menu_index--;
                break;
            }

            if (keymsg.key == KEY_D && menu_index < menu_count - 1)
            {
                menu_index++;
                break;
            }

            if (keymsg.key == KEY_B)
            {
                // 执行选中的菜单项
                switch (menu_index)
                {
                case 0: // 设置起始点
                    set_start_point();
                    break;

                case 1: // 轨迹选项子菜单
                    trajectory_submenu();
                    break;

                case 2: // 查看位置信息
                    display_position_info();
                    break;

                case 3: // 退出菜单
                    lcd_clear();
                    return;
                }
                break;
            }

            system_delay_ms(10);
        }
    }
}
